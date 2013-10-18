/***************************************************************************
 *   Copyright (C) 2010 by Michal Demin                                    *
 *   Several fixes by R. Diez in 2013.                                     *
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
#include <log.h>
#include "serial.h"

/* low level serial port */
/* TODO add support for WIN32 and others ! */
int serial_open(char *port)
{
	int fd;
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	return fd;
}

int serial_setspeed(int fd, char speed, cc_t timeout)
{
	struct termios t_opt;
	speed_t baud = (speed == SERIAL_FAST) ? B1000000 : B115200;

	/* set the serial port parameters */
	fcntl(fd, F_SETFL, 0);
	if (0 != tcgetattr(fd, &t_opt))
		return -1;

	if (0 != cfsetispeed(&t_opt, baud))
		return -1;

	if (0 != cfsetospeed(&t_opt, baud))
		return -1;

	t_opt.c_cflag |= (CLOCAL | CREAD);
	t_opt.c_cflag &= ~PARENB;
	t_opt.c_cflag &= ~CSTOPB;
	t_opt.c_cflag &= ~CSIZE;
	t_opt.c_cflag |= CS8;
	t_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* The serial port may have been configured for human interaction with
	   the Bus Pirate console, but OpenOCD is going to use a binary protocol,
	   so make sure to turn off any CR/LF translation and the like. */
	t_opt.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

	t_opt.c_oflag &= ~OPOST;
	t_opt.c_cc[VMIN] = 0;
	t_opt.c_cc[VTIME] = timeout;

	/* Note that, in the past, TCSANOW was used below instead of TCSADRAIN,
	   and CMD_UART_SPEED did not work properly then, at least with
	   the Bus Pirate v3.5 (USB). */
	if (0 != tcsetattr(fd, TCSADRAIN, &t_opt)) {
		/* According to the Linux documentation, this is actually not enough
		   to detect errors, you need to call tcgetattr() and check that
		   all changes have been performed successfully. */
		return -1;
	}

	return 0;
}

#define LINE_SIZE      81
#define BYTES_PER_LINE 16
static void serial_dump_buffer(char *buf, int size)
{
	char line[LINE_SIZE];
	char tmp[10];
	int offset = 0;

	line[0] = 0;
	while (offset < size) {
		snprintf(tmp, 5, "%02x ", (uint8_t)buf[offset]);
		offset++;

		strcat(line, tmp);

		if (offset % BYTES_PER_LINE == 0) {
			LOG_DEBUG("%s", line);
			line[0] = 0;
		}
	}

	if (line[0] != 0)
		LOG_DEBUG("%s", line);
}

int serial_write(int fd, char *buf, int size)
{
	int ret = 0;

	ret = write(fd, buf, size);

	LOG_DEBUG("size = %d ret = %d", size, ret);
	serial_dump_buffer(buf, size);

	if (ret != size)
		LOG_ERROR("Error sending data");

	return ret;
}

int serial_read(int fd, char *buf, int size)
{
	int len = 0;
	int ret = 0;
	int timeout = 0;

	while (len < size) {
		ret = read(fd, buf+len, size-len);
		if (ret == -1)
			return -1;

		if (ret == 0) {
			timeout++;

			if (timeout >= 10)
				break;

			continue;
		}

		len += ret;
	}

	LOG_DEBUG("should have read = %d actual size = %d", size, len);
	serial_dump_buffer(buf, len);

	if (len != size)
		LOG_ERROR("Error reading data");

	return len;
}
