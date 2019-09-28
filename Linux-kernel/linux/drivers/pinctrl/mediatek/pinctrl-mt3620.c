// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/pinctrl/mediatek/pinctrl-mt3620.c
 *
 * Copyright (c) 2018 MediaTek. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>

#include "pinctrl-mtk-common.h"
#include "pinctrl-mtk-mt3620.h"
//to do
#define MT3620_PIN_REG_DIR_OFFSET          0x020
#define MT3620_PIN_REG_IE_OFFSET           0x060
#define MT3620_PIN_REG_PULLUP_OFFSET       0x030
#define MT3620_PIN_REG_PULLDOWN_OFFSET     0x040
#define MT3620_PIN_REG_DOUT_OFFSET         0x010
#define MT3620_PIN_REG_DIN_OFFSET          0x004 /* not used, overridden by pinctrl_mt3620_din_offset */
//to do
#define MT3620_PIN_REG_PINMUX_OFFSET       0x020

//  GPIO#  regmap# base     desc                offset   DIN_I
//  0.. 3     0    38010000 GPIO_PWM_0          0         4
//  4.. 7     1    38020000 GPIO_PWM_1          0         4
//  8..11     2    38030000 GPIO_PWM_2          0         4
// 12..15     3    38040000 GPIO_3              0         4
// 16..19     4    38050000 GPIO_4              0         4
// 20..23     5    38060000 GPIO_5              0         4
// 24..25     6    30020000 ANTSEL0,ANTSEL1     0         0
// 26..30     7    38070500 ISU0                0         C
// 31..35     8    38080500 ISU1                0         C
// 36..40     9    38090500 ISU2                0         C
// 41..48    10    38000000 ADC_GPIO            0         4
// 49..55     6    30020000 NONMAP              2         0
// 56..60    11    380D0100 I2S0_GPIO           0         0
// 61..65    12    380E0100 I2S1_GPIO           0         0
// 66..70    13    380A0500 ISU3                0         C
// 71..75    14    380B0500 ISU4                0         C
// 76..80    15    380C0500 ISU5                0         C
// 81..93     6    30020000 NONMAP              9..21     0
//

static int pinctrl_mt3620_pin_start[] =   {0, 4, 8, 12, 16, 20, 24, 26, 31, 36, 41, 49, 56, 61, 66, 71, 76, 81, 94};
static int pinctrl_mt3620_pin_offset[] =  {0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  0,  0,  0,  0,  0,  9,  0};
static int pinctrl_mt3620_pin_regmap[] =  {0, 1, 2,  3,  4,  5,  6,  7,  8,  9, 10,  6, 11, 12, 13, 14, 15,  6,  0};
static int pinctrl_mt3620_pin_din_off[] = {4, 4, 4,  4,  4,  4,  0, 12, 12, 12,  4,  0,  0,  0, 12, 12, 12,  0,  0};

unsigned int pinctrl_mt3620_get_regmap(unsigned int *ppin)
{
	unsigned int i;

	BUG_ON(ppin == NULL);

	for (i = 0; i < ARRAY_SIZE(pinctrl_mt3620_pin_start) - 1; ++i) {
		if (*ppin < pinctrl_mt3620_pin_start[i + 1]) {
			if (!pinctrl_mt3620_pin_regmap[i]) {
				// ensure device tree has configured enough register resources
				break;
			}
			*ppin -= pinctrl_mt3620_pin_start[i];
			*ppin += pinctrl_mt3620_pin_offset[i];
			return pinctrl_mt3620_pin_regmap[i];
		}
	}

	// default to safe regmap
	return pinctrl_mt3620_pin_regmap[0];
}

unsigned int pinctrl_mt3620_din_offset(unsigned int pin)
{
	unsigned int i;
	for (i = 0; i < ARRAY_SIZE(pinctrl_mt3620_pin_start) - 1; ++i) {
		if (pin < pinctrl_mt3620_pin_start[i + 1]) {
			break;
		}
	}
	return pinctrl_mt3620_pin_din_off[i];
}

static const struct mtk_pinctrl_devdata mt3620_pinctrl_data = {
	.pins = mtk_pins_mt3620,
	.npins = ARRAY_SIZE(mtk_pins_mt3620),
	.dir_offset = MT3620_PIN_REG_DIR_OFFSET,
	.ies_offset = MTK_PINCTRL_NOT_SUPPORT,
	.smt_offset = MTK_PINCTRL_NOT_SUPPORT,
	.pullen_offset = MTK_PINCTRL_NOT_SUPPORT,
	.pullsel_offset = MT3620_PIN_REG_PULLUP_OFFSET,
	.pulldn_offset = MT3620_PIN_REG_PULLDOWN_OFFSET,
	.dout_offset = MT3620_PIN_REG_DOUT_OFFSET,
	// (.din_offset is overriden via spec_din_offset)
	.pinmux_offset = MTK_PINCTRL_NOT_SUPPORT,
	.spec_get_regmap = pinctrl_mt3620_get_regmap,
	.spec_din_offset = pinctrl_mt3620_din_offset,
	.mode_bits = 4,
	.modes_per_reg = 8,
	.offset_bits = 8,
	.port_shf = 4,
	.mux_port_shf = 2,
	.port_mask = 0,           // 0 since there's 1 register per block
	.port_align = 4,
};


static int mt3620_pinctrl_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Starting mt3620 pinctrl");
	return mtk_pctrl_init(pdev, &mt3620_pinctrl_data, NULL);
}

static const struct of_device_id mt3620_pctrl_match[] = {
	{
		.compatible = "mediatek,mt3620-pinctrl",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, mt3620_pctrl_match);

static struct platform_driver mtk_pinctrl_driver = {
	.probe = mt3620_pinctrl_probe,
	.driver = {
		.name = "mediatek-mt3620-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = mt3620_pctrl_match,
	},
};

static int __init mtk_pinctrl_init(void)
{
	return platform_driver_register(&mtk_pinctrl_driver);
}

module_init(mtk_pinctrl_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek Pinctrl Driver");
