/***************************************************************************
 *   Copyright (C) 2013 by Paul Sokolovsky                                 *
 *   pfalcon@users.sourceforge.net                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"
#include "hello.h"
#include "simplejtag_proto.h"
#include "serial.h"

static int fd;

static char *port;

static int simplejtag_l0_read(void)
{
	static char cmd = CMD_READ_TDO;
	char buf;
	serial_write(fd, &cmd, 1);
	serial_read(fd, &buf, 1);
	if ((buf & ~1) != RESP_VAL)
		LOG_ERROR("Unexpected return code on read: %02x", buf);
	return buf & 1;
}

static void simplejtag_l0_write(int tck, int tms, int tdi)
{
	char cmd = CMD_WRITE_TCK_TMS_TDI;
	char buf;

	if (tck)
		cmd |= BIT_TCK;

	if (tms)
		cmd |= BIT_TMS;

	if (tdi)
		cmd |= BIT_TDI;

	serial_write(fd, &cmd, 1);
	serial_read(fd, &buf, 1);
	if (buf != RESP_ACK)
		LOG_ERROR("Unexpected return code on write: %02x", buf);
}

static void simplejtag_l0_reset(int trst, int srst)
{
	/* Even if reset_config set to none, this method gets called */
	if (trst || srst)
		LOG_ERROR("simplejtag_l0 currently doesn't support hardware reset signaling");
}

static void simplejtag_l0_led(int on)
{
	char cmd = on ? CMD_BLINK_ON : CMD_BLINK_OFF;
	serial_write(fd, &cmd, 1);
	/* Ignore status */
	serial_read(fd, &cmd, 1);
}

static struct bitbang_interface simplejtag_l0_bitbang = {
		.read = &simplejtag_l0_read,
		.write = &simplejtag_l0_write,
		.reset = &simplejtag_l0_reset,
		.blink = &simplejtag_l0_led,
	};

static int simplejtag_l0_init(void)
{
	if (port == NULL) {
		LOG_ERROR("You need to specify the serial port using 'simplejtag_l0 port'");
		return ERROR_JTAG_INIT_FAILED;
	}

	fd = serial_open(port);
	if (fd == -1) {
		LOG_ERROR("Could not open serial port");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (serial_setspeed(fd, SERIAL_NORMAL, SHORT_TIMEOUT) == -1) {
		LOG_ERROR("Error configuring the serial port.");
		return ERROR_JTAG_INIT_FAILED;
	}

	bitbang_interface = &simplejtag_l0_bitbang;

	return ERROR_OK;
}

static int simplejtag_l0_quit(void)
{
	return ERROR_OK;
}

COMMAND_HANDLER(simplejtag_l0_handle_port_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	port = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

static const struct command_registration simplejtag_l0_subcommand_handlers[] = {
	{
		.name = "port",
		.usage = "/dev/ttyUSB0",
		.handler = simplejtag_l0_handle_port_command,
		.mode = COMMAND_CONFIG,
		.help = "name of the serial port to use",
	},
	COMMAND_REGISTRATION_DONE,
};

static const struct command_registration simplejtag_l0_command_handlers[] = {
	{
		.name = "simplejtag_l0",
		.mode = COMMAND_ANY,
		.help = "perform simplejtag level 0 adapter management",
		.chain = simplejtag_l0_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


struct jtag_interface simplejtag_l0_interface = {
	.name = "simplejtag_l0",

	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = simplejtag_l0_command_handlers,
	.transports = jtag_only,

	.execute_queue = bitbang_execute_queue,
	.init = simplejtag_l0_init,
	.quit = simplejtag_l0_quit,
};
