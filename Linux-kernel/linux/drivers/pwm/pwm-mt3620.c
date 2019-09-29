// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 PWM driver
 *
 * Copyright (c) 2019 Microsoft Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>

#define FREQ_32KHZ                              32000
#define FREQ_2MHZ                               2000000
#define FREQ_26MHZ                              26000000

#define PWM_GLO_CTRL                            0x0
#define PWM_N_OFFSET                            0x10
#define PWM_CTRL_N                              0x100
#define PWM_PARAM_S0_N                          0x104
#define PWM_PARAM_S1_N                          0x108

#define PWM_GLO_CTRL_GLOBAL_KICK                BIT(0)
#define PWM_GLO_CTRL_TICK_CLOCK_SEL_MASK        GENMASK(2, 1)
#define PWM_GLO_CTRL_TICK_CLOCK_SEL_32KHZ       0x0
// PWM_GLO_CTRL_TICK_CLOCK_SEL_XTALDIV = XTAL / 13 by default = 2MHz
#define PWM_GLO_CTRL_TICK_CLOCK_SEL_XTALDIV     0x1
// PWM_GLO_CTRL_TICK_CLOCK_SEL_XTAL = 26MHz on MT3620 Dev Board
#define PWM_GLO_CTRL_TICK_CLOCK_SEL_XTAL        0x2
#define PWM_GLO_CTRL_GLOBAL_RESET               BIT(3)
#define PWM_GLO_CTRL_DP_SEL_PWM0                GENMASK(9, 8)
#define PWM_GLO_CTRL_DP_SEL_PWM1                GENMASK(11, 10)
#define PWM_GLO_CTRL_DP_SEL_PWM2                GENMASK(13, 12)
#define PWM_GLO_CTRL_DP_SEL_PWM3                GENMASK(15, 14)

#define PWM_CTRL_KICK                           BIT(0)
#define PWM_CTRL_REPLAY                         BIT(1)
#define PWM_CTRL_POLARITY                       BIT(2)
#define PWM_CTRL_IO_CTRL                        BIT(3)
#define PWM_CTRL_CLOCK_EN                       BIT(4)
#define PWM_CTRL_GLOBAL_KICK_EN                 BIT(5)
#define PWM_CTRL_S0_STAY_CYCLE_MASK             GENMASK(19, 8)
#define PWM_CTRL_S1_STAY_CYCLE_MASK             GENMASK(31, 20)

#define PWM_PARAM_ON_TIME_MASK                  GENMASK(15, 0)
#define PWM_PARAM_OFF_TIME_MASK                 GENMASK(31, 16)

#define GPIO_PWM_GRP_GLOBAL_CTRL                0x0
#define GPIO_PWM_GRP_GLOBAL_SW_RST              BIT(2)

#define MT3620_PWM_EXTENDED_STATE_MAGIC         0x3620

#define MT3620_PWM_GLOBAL_KICK                  _IO(0xf7, 0x10)

struct mt3620_pwm {
	struct pwm_chip chip;
	struct device *dev;
	struct clk *pwm_clk;
	struct regmap *gpio_regmap;
	void __iomem *base;
};

struct mt3620_per_pwm_private {
	// Update after each kick
	bool stuck_state;
};

enum mt3620_output_mode {
	MT3620_PWM_OUTPUT_PUSH_PULL,
	MT3620_PWM_OUTPUT_OPEN_DRAIN
};

struct mt3620_extended_state {
	int magic;
	unsigned int s0_stay_cycles;
	unsigned int s1_period;
	unsigned int s1_duty_cycle;
	unsigned int s1_stay_cycles;
	bool replay;
	unsigned int differential_output;
	enum mt3620_output_mode output_mode;
	bool global_kick_enable;
};

static void mt3620_extended_state_init(struct mt3620_extended_state *ext,
				       unsigned int pwm_index)
{
	memset(ext, 0, sizeof(*ext));
	ext->magic = MT3620_PWM_EXTENDED_STATE_MAGIC;
	ext->s0_stay_cycles = 1;
	ext->output_mode = MT3620_PWM_OUTPUT_OPEN_DRAIN;
	ext->differential_output = pwm_index;
	// Other default values are 0
}

static inline struct mt3620_pwm *to_mt3620_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct mt3620_pwm, chip);
}

static bool check_stuck_config(struct mt3620_pwm *pwm_mt3620, unsigned int hwpwm)
{
	u32 param_s1 = readl(pwm_mt3620->base + PWM_PARAM_S1_N +
			     (PWM_N_OFFSET * hwpwm));
	u32 pwm_ctrl = readl(pwm_mt3620->base + PWM_CTRL_N +
			     (PWM_N_OFFSET * hwpwm));
	u16 ticks_on_s1 = FIELD_GET(PWM_PARAM_ON_TIME_MASK, param_s1);
	u16 ticks_off_s1 = FIELD_GET(PWM_PARAM_OFF_TIME_MASK, param_s1);
	unsigned int s1_stay_cycles =
		FIELD_GET(PWM_CTRL_S1_STAY_CYCLE_MASK, pwm_ctrl);

	return (ticks_on_s1 == 0 || ticks_off_s1 == 0) && s1_stay_cycles > 0;
}

static void global_kick_force_unstick(struct mt3620_pwm *pwm_mt3620, unsigned int hwpwm, unsigned long freq_ns)
{
	u32 glo_ctrl = readl(pwm_mt3620->base + PWM_GLO_CTRL);
	u32 pwm_ctrls[4] = {0};
	int i = 0;

	// Save all outputs' original global kick settings
	for (i = 0; i < pwm_mt3620->chip.npwm; ++i) {
		pwm_ctrls[i] = readl(pwm_mt3620->base + PWM_CTRL_N +
			     (PWM_N_OFFSET * i));

		// Temporarily disable global kick
		writel(pwm_ctrls[i] & ~PWM_CTRL_GLOBAL_KICK_EN,
			pwm_mt3620->base + PWM_CTRL_N + (PWM_N_OFFSET * i));
	}

	// Temporarily enable global kick for just the stuck output
	writel(pwm_ctrls[hwpwm] | PWM_CTRL_GLOBAL_KICK_EN,
		pwm_mt3620->base + PWM_CTRL_N + (PWM_N_OFFSET * hwpwm));

	// Kick the stuck output
	glo_ctrl |= PWM_GLO_CTRL_GLOBAL_KICK;
	writel(glo_ctrl, pwm_mt3620->base + PWM_GLO_CTRL);

	// Need to sleep before restoring original global kick settings or the hardware
	// will just read the restored settings
	// The datasheet states that hardware takes 2~3 ticks to sync and recommends
	// waiting 4 ticks of the PWM clock
	ndelay(freq_ns * 4);

	// Restore all outputs' original global kick settings
	for (i = 0; i < pwm_mt3620->chip.npwm; ++i) {
		writel(pwm_ctrls[i],
			pwm_mt3620->base + PWM_CTRL_N + (PWM_N_OFFSET * i));
	}
}

static int mt3620_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    struct pwm_state *state)
{
	struct mt3620_pwm *pwm_mt3620 = to_mt3620_pwm(chip);

	unsigned long clk_rate = clk_get_rate(pwm_mt3620->pwm_clk);
	unsigned long freq_ns = NSEC_PER_SEC / clk_rate;
	unsigned long ticks_on_s0 = state->duty_cycle / freq_ns;
	unsigned long ticks_off_s0 = (state->period / freq_ns) - ticks_on_s0;
	unsigned long ticks_on_s1;
	unsigned long ticks_off_s1;
	unsigned long max_ticks_on =
		PWM_PARAM_ON_TIME_MASK >> __bf_shf(PWM_PARAM_ON_TIME_MASK);
	unsigned long max_ticks_off =
		PWM_PARAM_OFF_TIME_MASK >> __bf_shf(PWM_PARAM_OFF_TIME_MASK);
	unsigned long max_stay_cycle_s0 = PWM_CTRL_S0_STAY_CYCLE_MASK >>
					  __bf_shf(PWM_CTRL_S0_STAY_CYCLE_MASK);
	unsigned long max_stay_cycle_s1 = PWM_CTRL_S0_STAY_CYCLE_MASK >>
					  __bf_shf(PWM_CTRL_S0_STAY_CYCLE_MASK);
	struct mt3620_per_pwm_private *pwm_priv;

	u32 param_s0 = 0;
	u32 param_s1 = 0;
	u32 pwm_ctrl = 0;
	u32 glo_ctrl = 0;

	struct mt3620_extended_state extended_state;
	// Fills the struct with defaults that can be used if extended state
	// isn't supplied
	mt3620_extended_state_init(&extended_state, pwm->hwpwm);

	pwm_priv = pwm_get_chip_data(pwm);

	if (state->extended_state) {
		if (state->extended_state_size != sizeof(extended_state)) {
			dev_err(chip->dev, "invalid size of extended_state\n");
			return -EINVAL;
		}

		extended_state =
			*(struct mt3620_extended_state *)state->extended_state;

		if (extended_state.magic != MT3620_PWM_EXTENDED_STATE_MAGIC) {
			dev_err(chip->dev,
				"invalid magic value in extended_state\n");
			return -EINVAL;
		}
	}

	ticks_on_s1 = extended_state.s1_duty_cycle / freq_ns;
	ticks_off_s1 = (extended_state.s1_period / freq_ns) - ticks_on_s1;

	if (ticks_on_s0 > max_ticks_on || ticks_off_s0 > max_ticks_off ||
	    ticks_on_s1 > max_ticks_on || ticks_off_s1 > max_ticks_off) {
		dev_err(chip->dev,
			"time overflow - ticks_on_s0: %lu ticks_off_s0: %lu ticks_on_s1: %lu ticks_off_s1: %lu\n",
			ticks_on_s0, ticks_off_s0, ticks_on_s1, ticks_off_s1);
		return -EINVAL;
	}

	if (extended_state.s0_stay_cycles > max_stay_cycle_s0 ||
	    extended_state.s1_stay_cycles > max_stay_cycle_s1) {
		dev_err(chip->dev,
			"stay cycle overflow - s0_stay_cycles: %u s1_stay_cycles: %u\n",
			extended_state.s0_stay_cycles,
			extended_state.s1_stay_cycles);
		return -EINVAL;
	}

	// Differential output is set in PWM_GLO_CTRL
	glo_ctrl = readl(pwm_mt3620->base + PWM_GLO_CTRL);
	switch (pwm->hwpwm) {
	case 0:
		glo_ctrl &= ~PWM_GLO_CTRL_DP_SEL_PWM0;
		glo_ctrl |= FIELD_PREP(PWM_GLO_CTRL_DP_SEL_PWM0,
				       extended_state.differential_output);
		break;
	case 1:
		glo_ctrl &= ~PWM_GLO_CTRL_DP_SEL_PWM1;
		glo_ctrl |= FIELD_PREP(PWM_GLO_CTRL_DP_SEL_PWM1,
				       extended_state.differential_output);
		break;
	case 2:
		glo_ctrl &= ~PWM_GLO_CTRL_DP_SEL_PWM2;
		glo_ctrl |= FIELD_PREP(PWM_GLO_CTRL_DP_SEL_PWM2,
				       extended_state.differential_output);
		break;
	case 3:
		glo_ctrl &= ~PWM_GLO_CTRL_DP_SEL_PWM3;
		glo_ctrl |= FIELD_PREP(PWM_GLO_CTRL_DP_SEL_PWM3,
				       extended_state.differential_output);
		break;
	default:
		dev_err(chip->dev, "bad pwm index (%d)\n", pwm->hwpwm);
		return -EINVAL;
	}
	writel(glo_ctrl, pwm_mt3620->base + PWM_GLO_CTRL);

	// Set time_on and time_off in ticks
	param_s0 |= FIELD_PREP(PWM_PARAM_ON_TIME_MASK, ticks_on_s0);
	param_s0 |= FIELD_PREP(PWM_PARAM_OFF_TIME_MASK, ticks_off_s0);
	writel(param_s0,
	       pwm_mt3620->base + PWM_PARAM_S0_N + (PWM_N_OFFSET * pwm->hwpwm));

	param_s1 |= FIELD_PREP(PWM_PARAM_ON_TIME_MASK, ticks_on_s1);
	param_s1 |= FIELD_PREP(PWM_PARAM_OFF_TIME_MASK, ticks_off_s1);
	writel(param_s1,
	       pwm_mt3620->base + PWM_PARAM_S1_N + (PWM_N_OFFSET * pwm->hwpwm));

	pwm_ctrl |= FIELD_PREP(PWM_CTRL_S0_STAY_CYCLE_MASK,
			       extended_state.s0_stay_cycles);
	pwm_ctrl |= FIELD_PREP(PWM_CTRL_S1_STAY_CYCLE_MASK,
			       extended_state.s1_stay_cycles);

	pwm_ctrl |=
		extended_state.global_kick_enable ? PWM_CTRL_GLOBAL_KICK_EN : 0;

	pwm_ctrl |= state->enabled ? PWM_CTRL_CLOCK_EN : 0;

	switch (extended_state.output_mode) {
	case MT3620_PWM_OUTPUT_PUSH_PULL:
		pwm_ctrl &= ~PWM_CTRL_IO_CTRL;
		break;
	case MT3620_PWM_OUTPUT_OPEN_DRAIN:
		pwm_ctrl |= PWM_CTRL_IO_CTRL;
		break;
	default:
		dev_err(chip->dev, "bad output mode (%d)\n",
			extended_state.output_mode);
		return -EINVAL;
	}

	switch (state->polarity) {
	case PWM_POLARITY_NORMAL:
		pwm_ctrl &= ~PWM_CTRL_POLARITY;
		break;
	case PWM_POLARITY_INVERSED:
		pwm_ctrl |= PWM_CTRL_POLARITY;
		break;
	default:
		dev_err(chip->dev, "bad polarity (%d)\n", state->polarity);
		return -EINVAL;
	}

	pwm_ctrl |= extended_state.replay ? PWM_CTRL_REPLAY : 0;

	writel(pwm_ctrl,
	       pwm_mt3620->base + PWM_CTRL_N + (PWM_N_OFFSET * pwm->hwpwm));

	// Read settings and start, unless we're waiting for global kick
	if (!extended_state.global_kick_enable) {
		if (pwm_priv->stuck_state) {
			dev_err(chip->dev, "Detected that PWM output is in stuck state. Unsticking it. (output:%d)\n", pwm->hwpwm);
			global_kick_force_unstick(pwm_mt3620, pwm->hwpwm, freq_ns);
			// This also kicks the PWM, so a normal kick isn't necessary
		} else {
			pwm_ctrl |= PWM_CTRL_KICK;
			writel(pwm_ctrl, pwm_mt3620->base + PWM_CTRL_N +
						(PWM_N_OFFSET * pwm->hwpwm));
		}

		// Update stuck state after kicking
		pwm_priv->stuck_state = check_stuck_config(pwm_mt3620, pwm->hwpwm);
	}

	return 0;
}

static void mt3620_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct mt3620_pwm *pwm_mt3620 = to_mt3620_pwm(chip);

	unsigned long clk_rate = clk_get_rate(pwm_mt3620->pwm_clk);
	unsigned long freq_ns = NSEC_PER_SEC / clk_rate;
	struct mt3620_extended_state *ext;

	u32 glo_ctrl = readl(pwm_mt3620->base + PWM_GLO_CTRL);
	u32 param_s0 = readl(pwm_mt3620->base + PWM_PARAM_S0_N +
			     (PWM_N_OFFSET * pwm->hwpwm));
	u32 param_s1 = readl(pwm_mt3620->base + PWM_PARAM_S1_N +
			     (PWM_N_OFFSET * pwm->hwpwm));
	u32 pwm_ctrl = readl(pwm_mt3620->base + PWM_CTRL_N +
			     (PWM_N_OFFSET * pwm->hwpwm));
	u16 ticks_on_s0 = FIELD_GET(PWM_PARAM_ON_TIME_MASK, param_s0);
	u16 ticks_off_s0 = FIELD_GET(PWM_PARAM_OFF_TIME_MASK, param_s0);
	u16 ticks_on_s1 = FIELD_GET(PWM_PARAM_ON_TIME_MASK, param_s1);
	u16 ticks_off_s1 = FIELD_GET(PWM_PARAM_OFF_TIME_MASK, param_s1);

	state->duty_cycle = ticks_on_s0 * freq_ns;
	state->period = (ticks_on_s0 + ticks_off_s0) * freq_ns;
	state->enabled = FIELD_GET(PWM_CTRL_S0_STAY_CYCLE_MASK, pwm_ctrl) &&
			 (pwm_ctrl & PWM_CTRL_CLOCK_EN);
	state->polarity = pwm_ctrl & PWM_CTRL_POLARITY ? PWM_POLARITY_INVERSED :
							 PWM_POLARITY_NORMAL;

	ext = kzalloc(sizeof(*ext), GFP_KERNEL);
	if (ext) {
		ext->magic = MT3620_PWM_EXTENDED_STATE_MAGIC;
		ext->s0_stay_cycles =
			FIELD_GET(PWM_CTRL_S0_STAY_CYCLE_MASK, pwm_ctrl);
		ext->s1_duty_cycle = ticks_on_s1 * freq_ns;
		ext->s1_period = (ticks_on_s0 + ticks_off_s1) * freq_ns;
		ext->s1_stay_cycles =
			FIELD_GET(PWM_CTRL_S1_STAY_CYCLE_MASK, pwm_ctrl);

		switch (pwm->hwpwm) {
		case 0:
			ext->differential_output =
				FIELD_GET(PWM_GLO_CTRL_DP_SEL_PWM0, glo_ctrl);
			break;
		case 1:
			ext->differential_output =
				FIELD_GET(PWM_GLO_CTRL_DP_SEL_PWM1, glo_ctrl);
			break;
		case 2:
			ext->differential_output =
				FIELD_GET(PWM_GLO_CTRL_DP_SEL_PWM2, glo_ctrl);
			break;
		case 3:
			ext->differential_output =
				FIELD_GET(PWM_GLO_CTRL_DP_SEL_PWM3, glo_ctrl);
			break;
		default:
			dev_err(chip->dev, "bad pwm index (%d)\n", pwm->hwpwm);
		}

		ext->output_mode = pwm_ctrl & PWM_CTRL_IO_CTRL ?
					   MT3620_PWM_OUTPUT_OPEN_DRAIN :
					   MT3620_PWM_OUTPUT_PUSH_PULL;
		ext->global_kick_enable = pwm_ctrl & PWM_CTRL_GLOBAL_KICK_EN;
		state->extended_state = ext;
		state->extended_state_size = sizeof(*ext);
	} else {
		state->extended_state = NULL;
		state->extended_state_size = 0;
	}
}

static int mt3620_pwm_ioctl_global_kick(struct pwm_chip *chip)
{
	struct mt3620_pwm *pwm_mt3620 = to_mt3620_pwm(chip);

	u32 glo_ctrl = readl(pwm_mt3620->base + PWM_GLO_CTRL);
	int i = 0;

	glo_ctrl |= PWM_GLO_CTRL_GLOBAL_KICK;
	writel(glo_ctrl, pwm_mt3620->base + PWM_GLO_CTRL);

	// Update stuck_state for the PWMs that this global kick affects
	for (i = 0; i < chip->npwm; ++i) {
		struct mt3620_per_pwm_private *pwm_priv = pwm_get_chip_data(&chip->pwms[i]);
		u32 pwm_ctrl = readl(pwm_mt3620->base + PWM_CTRL_N +
					(PWM_N_OFFSET * i));

		if (pwm_ctrl & PWM_CTRL_GLOBAL_KICK_EN) {
			pwm_priv->stuck_state = check_stuck_config(pwm_mt3620, i);
		}
	}

	return 0;
}

static long mt3620_pwm_ioctl(struct pwm_chip *chip, unsigned int cmd,
		      unsigned long arg_)
{
	switch (cmd) {
	case MT3620_PWM_GLOBAL_KICK:
		return mt3620_pwm_ioctl_global_kick(chip);
	default:
		return -ENOTTY;
	}
}

static const struct pwm_ops mt3620_pwm_ops = {
	.get_state = mt3620_pwm_get_state,
	.apply = mt3620_pwm_apply,
	.ioctl = mt3620_pwm_ioctl,
	.owner = THIS_MODULE,
};

// The documented reset register in the PWM register block does not actually exist/function.
// The actual reset register is in the GPIO register block
static void mt3620_gpio_reg_reset(struct mt3620_pwm *pwm_mt3620)
{
	// Assert reset
	regmap_update_bits(pwm_mt3620->gpio_regmap, GPIO_PWM_GRP_GLOBAL_CTRL,
	                   GPIO_PWM_GRP_GLOBAL_SW_RST, 0);
	// Must then deassert reset
	regmap_update_bits(pwm_mt3620->gpio_regmap, GPIO_PWM_GRP_GLOBAL_CTRL,
	                   GPIO_PWM_GRP_GLOBAL_SW_RST, GPIO_PWM_GRP_GLOBAL_SW_RST);
}

static int mt3620_pwm_remove(struct platform_device *pdev)
{
	struct mt3620_pwm *pwm_mt3620 = platform_get_drvdata(pdev);

	if (!pwm_mt3620)
		return -EINVAL;

	// Reset the group
	mt3620_gpio_reg_reset(pwm_mt3620);

	return pwmchip_remove(&pwm_mt3620->chip);
}

static int mt3620_pwm_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	struct resource *res;
	struct mt3620_pwm *pwm_mt3620;
	struct device_node *gpio_node;

	u8 tick_clock_sel;
	u32 glo_ctrl;

	pwm_mt3620 = devm_kzalloc(&pdev->dev, sizeof(*pwm_mt3620), GFP_KERNEL);
	if (!pwm_mt3620) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "memory allocation failed\n");
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "failed to determine base address\n");
		goto out_no_remap;
	}

	pwm_mt3620->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pwm_mt3620->base)) {
		ret = PTR_ERR(pwm_mt3620->base);
		dev_err(&pdev->dev, "failed to remap base address err: %d\n",
			ret);
		goto out_no_remap;
	}

	pwm_mt3620->dev = &pdev->dev;
	platform_set_drvdata(pdev, pwm_mt3620);

	pwm_mt3620->chip.dev = &pdev->dev;
	pwm_mt3620->chip.ops = &mt3620_pwm_ops;
	pwm_mt3620->chip.base = -1;
	pwm_mt3620->chip.npwm = 4;
	pwm_mt3620->chip.of_xlate = of_pwm_xlate_with_flags;
	pwm_mt3620->chip.of_pwm_n_cells = 3; // matches of_pwm_xlate_with_flags
	pwm_mt3620->chip.can_sleep = true;

	pwm_mt3620->pwm_clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(pwm_mt3620->pwm_clk)) {
		dev_err(&pdev->dev, "failed to get clock (%d)\n", ret);
		ret = PTR_ERR(pwm_mt3620->pwm_clk);
		goto out_no_pwm;
	}

	switch (clk_get_rate(pwm_mt3620->pwm_clk)) {
	case FREQ_32KHZ:
		tick_clock_sel = PWM_GLO_CTRL_TICK_CLOCK_SEL_32KHZ;
		break;
	case FREQ_2MHZ:
		tick_clock_sel = PWM_GLO_CTRL_TICK_CLOCK_SEL_XTALDIV;
		break;
	case FREQ_26MHZ:
		tick_clock_sel = PWM_GLO_CTRL_TICK_CLOCK_SEL_XTAL;
		break;
	default:
		dev_err(&pdev->dev, "bad clock setting (%lu)\n",
			clk_get_rate(pwm_mt3620->pwm_clk));
		goto out_no_pwm;
	}


	// Need a reference to the related GPIO regmap in order to correctly
	// reset the PWM
	gpio_node = of_parse_phandle(pdev->dev.of_node, "microsoft,gpio-reg", 0);
	if (gpio_node) {
		pwm_mt3620->gpio_regmap = syscon_node_to_regmap(gpio_node);
		if (IS_ERR(pwm_mt3620->gpio_regmap)) {
			ret = PTR_ERR(pwm_mt3620->gpio_regmap);
			goto out_no_pwm;
		}
	} else {
		dev_err(&pdev->dev, "node has no gpio regmap.\n");
		ret = -EINVAL;
		goto out_no_pwm;
	}

	// Start with a reset of the group
	mt3620_gpio_reg_reset(pwm_mt3620);

	glo_ctrl = readl(pwm_mt3620->base + PWM_GLO_CTRL);

	// Select clock
	glo_ctrl &= ~PWM_GLO_CTRL_TICK_CLOCK_SEL_MASK;
	glo_ctrl |=
		FIELD_PREP(PWM_GLO_CTRL_TICK_CLOCK_SEL_MASK, tick_clock_sel);
	writel(glo_ctrl, pwm_mt3620->base + PWM_GLO_CTRL);

	ret = pwmchip_add(&pwm_mt3620->chip);

	if (ret) {
		dev_err(&pdev->dev, "failed to register device err: %d\n", ret);
		goto out_no_pwm;
	}

	for (i = 0; i < pwm_mt3620->chip.npwm; ++i) {
		struct mt3620_per_pwm_private *pwm_priv =
				devm_kzalloc(pwm_mt3620->chip.dev, sizeof(*pwm_priv), GFP_KERNEL);
		if (!pwm_priv) {
			ret = -ENOMEM;
			goto out_no_pwm;
		}

		pwm_set_chip_data(&pwm_mt3620->chip.pwms[i], pwm_priv);
	}

	return 0;

out_no_pwm:
	platform_set_drvdata(pdev, NULL);
	devm_iounmap(&pdev->dev, pwm_mt3620->base);
out_no_remap:
	devm_kfree(&pdev->dev, pwm_mt3620);
out:

	return ret;
}

static const struct of_device_id mt3620_pwm_match[] = {
	{ .compatible = "mediatek,mt3620-pwm" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt3620_pwm_match);

static struct platform_driver mt3620_pwm_driver = {
	.probe = mt3620_pwm_probe,
	.remove = mt3620_pwm_remove,
	.driver =
		{
			.name = "mt3620-pwm",
			.of_match_table = mt3620_pwm_match,
		},
};

module_platform_driver(mt3620_pwm_driver);

MODULE_DESCRIPTION("MT3620 PWM Driver");
MODULE_AUTHOR("Microsoft");
MODULE_LICENSE("GPL v2");
