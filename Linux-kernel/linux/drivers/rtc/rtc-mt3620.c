// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 * MTK DMAengine support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/printk.h>
#include <azure-sphere/security_monitor.h>

/*
 * Register definitions
 */
#define REG_RTC_TC_YEAR			0x0040
#define REG_RTC_TC_MON			0x0044
#define REG_RTC_TC_DOM			0x0048
#define REG_RTC_TC_DOW			0x004C
#define REG_RTC_TC_HOUR			0x0050
#define REG_RTC_TC_MIN			0x0054
#define REG_RTC_TC_SEC			0x0058
#define REG_RTC_AL_YEAR			0x0060
#define REG_RTC_AL_MON			0x0064
#define REG_RTC_AL_DOM			0x0068
#define REG_RTC_AL_DOW			0x006C
#define REG_RTC_AL_HOUR			0x0070
#define REG_RTC_AL_MIN			0x0074
#define REG_RTC_AL_SEC			0x0078
#define REG_RTC_AL_CTL			0x007C
#define RTC_BIT_ALMEN			0x0001
#define REG_RTC_PMU_EN			0x0030
#define RTC_BIT_EVENT_EXT		0x0040
#define RTC_BIT_EVENT_TIMER		0x0020
#define RTC_BIT_EVENT_ALARM		0x0010

/**
 * struct mt3620_vendor_data - per-vendor variations
 * @ops: the vendor-specific operations used on this silicon version
 * @irqflags: special IRQ flags per variant
 */
struct mt3620_vendor_data {
	struct rtc_class_ops ops;
	unsigned long irqflags;
};

struct mt3620_local {
	struct mt3620_vendor_data *vendor;
	struct rtc_device *rtc;
	struct device *dev;
	void __iomem *base;
	int irq;
};

static int mt3620_alarm_irq_enable(struct device *dev,
	unsigned int enabled)
{
	return -1;
}

static int mt3620_rtc_set_time(struct device *dev, struct rtc_time *time)
{
	struct azure_sphere_rtc_time as_time;

	memset(&as_time, 0, sizeof(struct azure_sphere_rtc_time));

	/* Range 1900 to 1999 is invalid */
	if (time->tm_year < 100) {
		return -EINVAL;
	}
	/**
	 * RTC's epoch is initialized to year reference 2000 and Unix to 1900,
	 * subtract 100 when writing to rtc.
	 */
	time->tm_year -= 100;
	/**
	 * RTC's month reference ranges from 1~12 and Unix from 0~11,
	 * add 1 when writing to rtc.
	 */
	time->tm_mon += 1;

	as_time.time_sec   = time->tm_sec;
	as_time.time_min   = time->tm_min;
	as_time.time_hour  = time->tm_hour;
	as_time.time_mday  = time->tm_mday;
	as_time.time_mon   = time->tm_mon;
	as_time.time_year  = time->tm_year;
	as_time.time_wday  = time->tm_wday;
	as_time.time_yday  = time->tm_yday;
	as_time.time_isdst = time->tm_isdst;

	return azure_sphere_set_rtc_current_time(&as_time);
}

static int mt3620_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct mt3620_local *ldata = dev_get_drvdata(dev);
	int ret = 0;

	tm->tm_year = readl(ldata->base + REG_RTC_TC_YEAR);
	tm->tm_mon  = readl(ldata->base + REG_RTC_TC_MON);
	tm->tm_mday = readl(ldata->base + REG_RTC_TC_DOM);
	tm->tm_wday = readl(ldata->base + REG_RTC_TC_DOW);
	tm->tm_hour = readl(ldata->base + REG_RTC_TC_HOUR);
	tm->tm_min  = readl(ldata->base + REG_RTC_TC_MIN);
	tm->tm_sec  = readl(ldata->base + REG_RTC_TC_SEC);

	/* This fix is needed one-time-only as we transition from zero-based month to
	 * one-based month and re-base year reference from 1900 to 2000. This ensures we read correct
	 * RTC year/month offsets before and after transition.
	 */
	if (tm->tm_year > 100) {
		/* Add 1 to month, subtract 100 from year and push to RTC */
		ret = mt3620_rtc_set_time(dev, tm);
	}
	/**
	 * RTC's epoch is initialized to year reference 2000 and Unix to 1900,
	 * add 100 when reading from rtc.
	 */
	tm->tm_year += 100;
	/**
	 * RTC's month reference ranges from 1~12 and Unix from 0~11,
	 * subtract 1 when reading from rtc. It is safe to subtract
	 * unconditionally as the reset value of RTC_TC_MON is 1.
	 */
	tm->tm_mon -= 1;

	return ret;
}


static int mt3620_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct mt3620_local *ldata = dev_get_drvdata(dev);

	alarm->time.tm_year = readl(ldata->base + REG_RTC_AL_YEAR);
	alarm->time.tm_mon  = readl(ldata->base + REG_RTC_AL_MON);
	alarm->time.tm_mday = readl(ldata->base + REG_RTC_AL_DOM);
	alarm->time.tm_wday = readl(ldata->base + REG_RTC_AL_DOW);
	alarm->time.tm_hour = readl(ldata->base + REG_RTC_AL_HOUR);
	alarm->time.tm_min  = readl(ldata->base + REG_RTC_AL_MIN);
	alarm->time.tm_sec  = readl(ldata->base + REG_RTC_AL_SEC);
	alarm->pending = 0;
	alarm->enabled = readl(ldata->base + REG_RTC_AL_CTL) & RTC_BIT_ALMEN;

	/**
	 * RTC's epoch is initialized to year reference 2000 and Unix to 1900,
	 * add 100 when reading from rtc.
	 */
	alarm->time.tm_year += 100;
	/**
	 * RTC's month reference ranges from 1~12 and Unix from 0~11,
	 * subtract 1 when reading from rtc. It is safe to subtract
	 * unconditionally as the reset value of RTC_AL_MON is 1.
	 */
	alarm->time.tm_mon -= 1;

	return 0;
}

static int mt3620_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct azure_sphere_rtc_wake_alarm as_alarm;

	memset(&as_alarm, 0, sizeof(struct azure_sphere_rtc_wake_alarm));

	/* Range 1900 to 1999 is invalid */
	if (alarm->time.tm_year < 100) {
		return -EINVAL;
	}
	/**
	 * RTC's epoch is initialized to year reference 2000 and Unix to 1900,
	 * subtract 100 when writing to rtc.
	 */
	alarm->time.tm_year -= 100;
	/**
	 * RTC's month reference ranges from 1~12 and Unix from 0~11,
	 * add 1 when writing to rtc.
	 */
	alarm->time.tm_mon += 1;

	as_alarm.time.time_sec   = alarm->time.tm_sec;
	as_alarm.time.time_min   = alarm->time.tm_min;
	as_alarm.time.time_hour  = alarm->time.tm_hour;
	as_alarm.time.time_mday  = alarm->time.tm_mday;
	as_alarm.time.time_mon   = alarm->time.tm_mon;
	as_alarm.time.time_year  = alarm->time.tm_year;
	as_alarm.time.time_wday  = alarm->time.tm_wday;
	as_alarm.time.time_yday  = alarm->time.tm_yday;
	as_alarm.time.time_isdst = alarm->time.tm_isdst;
	as_alarm.enabled = alarm->enabled;
	as_alarm.pending = alarm->pending;

	return azure_sphere_set_rtc_alarm(&as_alarm);
}

static const struct rtc_class_ops mt3620_rtc_ops = {
		.read_time = mt3620_rtc_read_time,
		.set_time = mt3620_rtc_set_time,
		.read_alarm = mt3620_rtc_read_alarm,
		.set_alarm = mt3620_rtc_set_alarm,
		.alarm_irq_enable = mt3620_alarm_irq_enable,
};


static int mt3620_rtc_remove(struct platform_device *pdev)
{
	struct mt3620_local *ldata = platform_get_drvdata(pdev);

	devm_iounmap(&pdev->dev, ldata->base);
	devm_kfree(&pdev->dev, ldata);

	return 0;
}


static int mt3620_rtc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct mt3620_local *ldata;


	ldata = devm_kzalloc(&pdev->dev, sizeof(*ldata), GFP_KERNEL);
	if (!ldata) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s - memory allocation failed\n", __FUNCTION__);
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ldata->base = devm_ioremap_resource(&pdev->dev, res);

	if (!ldata->base) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s - IO resource reservation failed\n", __FUNCTION__);
		goto out_no_remap;
	}

	ldata->dev = &pdev->dev;
	platform_set_drvdata(pdev, ldata);

	ldata->rtc = devm_rtc_device_register(&pdev->dev, "mt3620-rtc",
					 &mt3620_rtc_ops, THIS_MODULE);

	if (IS_ERR(ldata->rtc)) {
		ret = PTR_ERR(ldata->rtc);
		printk(KERN_ERR "%s - RTC device registration failed\n", __FUNCTION__);
		goto out_no_rtc;
	}

	return 0;

out_no_rtc:
	platform_set_drvdata(pdev, NULL);
	iounmap(ldata->base);
out_no_remap:
	devm_kfree(&pdev->dev, ldata);
out:

	return ret;
}

static SIMPLE_DEV_PM_OPS(mt3620_pm_ops, null, null);

static const struct of_device_id mt3620_rtc_match[] = {
	{.compatible = "mediatek,mt3620-rtc"},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt3620_rtc_match);

static struct platform_driver mt3620_rtc_driver = {
	.probe	= mt3620_rtc_probe,
	.remove	= mt3620_rtc_remove,
	.driver	= {
		.name	= "mt3620-rtc",
		.pm = &mt3620_pm_ops,
		.of_match_table	= mt3620_rtc_match,
	},
};

module_platform_driver(mt3620_rtc_driver);

MODULE_DESCRIPTION("MT3620 RTC Driver");
MODULE_LICENSE("GPL v2");
