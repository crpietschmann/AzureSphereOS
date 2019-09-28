// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere null console driver
 *
 * Copyright (c) 2018 Microsoft Corporation. All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>


struct null_console_private {
	struct tty_port port;
} null_console_instance;

static int null_console_write_tty(struct tty_struct *tty,
	      const unsigned char *buf, int count)
{
	// Drop data
	return count;
}

static int null_console_write_room(struct tty_struct *tty)
{
	// Fake a buffer with free space
	return 512;
}

static int null_console_chars_in_buffer(struct tty_struct *tty)
{
	// Never any incoming data
	return 0;
}

static int null_console_open(struct tty_struct *tty, struct file *filp)
{
	struct null_console_private *null_console = &null_console_instance;
	struct tty_port *port = &null_console->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (!port->tty) {
		tty->driver_data = null_console;
		tty->port = port;
		port->tty = tty;
	}

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void null_console_close(struct tty_struct *tty, struct file *filp)
{
	struct tty_port *port = &null_console_instance.port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (tty->count == 1)
		port->tty = NULL;

	spin_unlock_irqrestore(&port->lock, flags);
}


static struct tty_driver *null_console_driver;

static const struct tty_operations null_console_ops = {
	.open		= null_console_open,
	.close		= null_console_close,
	.write		= null_console_write_tty,
	.write_room	= null_console_write_room,
	.chars_in_buffer = null_console_chars_in_buffer,
};

/*
 * The console driver
 */
static void null_console_write(struct console *co, const char *s, unsigned count)
{
}

static struct tty_driver *null_console_device(struct console *co, int *index)
{
	*index = co->index;
	return null_console_driver;
}

static int null_console_setup(struct console *co, char *options)
{
	return 0;
}

static struct console null_console_cons = {
	.name		= "nullconsole",
	.write		= null_console_write,
	.device		= null_console_device,
	.setup		= null_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
};

static int __init null_console_init(void)
{
	struct tty_driver *driver;
	int err;

	driver = alloc_tty_driver(1);
	if (!driver)
		return -ENOMEM;

	tty_port_init(&null_console_instance.port);

	driver->driver_name = "nullconsole";
	driver->name = "nullconsole";
	driver->major = 0; 	/* dynamic */
	driver->minor_start = 0;
	driver->type = TTY_DRIVER_TYPE_SYSTEM;
	driver->subtype = SYSTEM_TYPE_SYSCONS;
	driver->init_termios = tty_std_termios;
	tty_set_operations(driver, &null_console_ops);
	tty_port_link_device(&null_console_instance.port, driver, 0);
	err = tty_register_driver(driver);
	if (err) {
		put_tty_driver(driver);
		tty_port_destroy(&null_console_instance.port);
		return err;
	}
	null_console_driver = driver;

	register_console(&null_console_cons);

	return -ENODEV;
}
device_initcall(null_console_init);
