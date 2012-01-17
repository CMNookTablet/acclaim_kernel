/*
 * /drivers/mfd/twl6030-power.c
 *
 * Power control
 *
 * Copyright (C) 2010 Texas Instruments Corporation
 * Copyright (c) 2011 Barnes & Noble Inc.
 *
 * Written by Rajeev Kulkarni <rajeevk@ti.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl.h>

#define TWL6030_PHOENIX_DEV_ON			0x25
#define TWL6030_PHOENIX_START_CONDITION	0x1f

#define APP_DEVOFF	(1<<0)
#define CON_DEVOFF	(1<<1)
#define MOD_DEVOFF	(1<<2)

#define STRT_ON_PWRON		(1<<0)
#define STRT_ON_RPWRON		(1<<1)
#define STRT_ON_USB_ID		(1<<2)
#define STRT_ON_PLUG_DET	(1<<3)
#define STRT_ON_RTC			(1<<4)
#define FIRST_BAT_INS		(1<<5)
#define RESTART_BB			(1<<6)

static void twl6030_poweroff(void);
static struct kobject *twl6030_power_kobj;

static ssize_t twl6030_poweroff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_notice("\n For hard poweroff do --> echo 1 > poweroff \n"); 
	return 0;
}

static ssize_t twl6030_poweroff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pr_emerg("\n **** TWL6030 POWER OFF CALLED ****\n");
	twl6030_poweroff();
	return 10;
}

static ssize_t twl6030_poweron_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 uninitialized_var(val);
	int err;

	err = twl_i2c_read_u8(TWL6030_MODULE_ID0, &val, 
						TWL6030_PHOENIX_START_CONDITION);

	if (err) {
		pr_err("failed to read PHOENIX_START_CONDITION: %d\n", err);
		return err;
	}

	// this may seem a little bit crazy but the kernel
	// guarantees a 4k zeroed page in buf, this is more than
	// enough for all the strcats below

	if (val & STRT_ON_PWRON) strcat(buf, "PWRON ");
	if (val & STRT_ON_RPWRON) strcat(buf, "RPWRON ");
	if (val & STRT_ON_USB_ID) strcat(buf, "USB_ID ");
	if (val & STRT_ON_PLUG_DET) strcat(buf, "PLUG_DET ");
	if (val & STRT_ON_RTC) strcat(buf, "RTC ");
	if (val & FIRST_BAT_INS) strcat(buf, "FIRST_BAT_INS ");
	if (val & RESTART_BB) strcat(buf, "RESTART_BB ");
	if (val == 0) strcat(buf, "ZERO");

	strcat(buf, "\n");
	return strlen(buf);
}

static DEVICE_ATTR(poweroff, S_IRUGO | S_IWUSR, twl6030_poweroff_show,twl6030_poweroff_store);
static DEVICE_ATTR(poweron, S_IRUGO, twl6030_poweron_show, NULL);

static struct attribute *twl6030_power_attributes[] = {
	    &dev_attr_poweroff.attr,
		&dev_attr_poweron.attr,
	    NULL
};

static struct attribute_group twl6030_power_attribute_group = {
	    .attrs = twl6030_power_attributes
};

static void twl6030_poweroff(void)
{
	u8 uninitialized_var(val);
	int err;

	err = twl_i2c_read_u8(TWL6030_MODULE_ID0, &val,
				  TWL6030_PHOENIX_DEV_ON);
	if (err) {
		pr_warning("I2C error %d reading PHONIX_DEV_ON\n", err);
		return;
	}

	val |= APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;

	err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,
				   TWL6030_PHOENIX_DEV_ON);

	if (err) {
		pr_warning("I2C error %d writing PHONIX_DEV_ON\n", err);
		return;
	}

	return;
}

static int __init twl6030_power_init(void)
{
	pm_power_off = twl6030_poweroff;
	twl6030_power_kobj = kobject_create_and_add("twl6030_power", kernel_kobj);

	if ( twl6030_power_kobj ){
		sysfs_create_group(twl6030_power_kobj , &twl6030_power_attribute_group);
	} else {
		pr_err("Failed to allocate memory for kobject\n");
		return -ENOMEM;
	}

	return 0;
}

static void __exit twl6030_power_exit(void)
{
	pm_power_off = NULL;
	if ( twl6030_power_kobj ){
		sysfs_remove_group(twl6030_power_kobj, &twl6030_power_attribute_group);
		kobject_put(twl6030_power_kobj);
	}
}

module_init(twl6030_power_init);
module_exit(twl6030_power_exit);

MODULE_DESCRIPTION("Triton2 device power control");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rajeev Kulkarni");
