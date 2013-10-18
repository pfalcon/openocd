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

#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

enum {
        SERIAL_NORMAL = 0,
        SERIAL_FAST = 1
};

static const cc_t SHORT_TIMEOUT  = 1; /* Must be at least 1. */
static const cc_t NORMAL_TIMEOUT = 10;

int serial_open(char *port);
int serial_setspeed(int fd, char speed, cc_t timeout);
int serial_write(int fd, char *buf, int size);
int serial_read(int fd, char *buf, int size);
