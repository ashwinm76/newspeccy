/*
 * Z80SIM  -  a Z80-CPU simulator
 *
 * Copyright (C) 1987-2017 by Udo Munk
 *
 * This module contains a complex I/O-simulation for running
 * CP/M, MP/M, UCSD p-System...
 *
 * Please note this doesn't emulate any hardware which
 * ever existed, we've got all virtual circuits in here!
 *
 * History:
 * 28-SEP-1987 Development on TARGON/35 with AT&T Unix System V.3
 * 19-MAY-1989 Additions for CP/M 3.0 and MP/M
 * 23-DEC-1990 Ported to COHERENT 3.0
 * 10-JUN-1992 Some optimisation done
 * 25-JUN-1992 Flush output of stdout only at every OUT to port 0
 * 25-JUN-1992 Comments in english and ported to COHERENT 4.0
 * 05-OCT-2006 modified to compile on modern POSIX OS's
 * 18-NOV-2006 added a second harddisk
 * 08-DEC-2006 modified MMU so that segment size can be configured
 * 10-DEC-2006 started adding serial port for a passive TCP/IP socket
 * 14-DEC-2006 started adding serial port for a client TCP/IP socket
 * 25-DEC-2006 CPU speed option and 100 ticks interrupt
 * 19-FEB-2007 improved networking
 * 22-JUL-2007 added second FDC sector port for implementing large harddisks
 * 30-OCT-2007 printer port returns EOF on input
 * 03-SEP-2007 improved the clock chip for support of other OS's
 * 19-SEP-2007 delay circuit modified to delay x * 10ms
 * 05-DEC-2007 fixed socket shutdown issues for NetBSD
 * 06-DEC-2007 added hardware control port to reset I/O, CPU and reboot and such
 * 07-DEC-2007 conditional compile pipes for aux device, problems with Cygwin
 * 17-DEC-2007 conditional compile async TCP/IP server, problems with Cygwin
 * 03-FEB-2008 added hardware control port to reset CPU, MMU and abort sim
 * 07-APR-2008 added port to set/get CPU speed
 * 13-AUG-2008 work on console I/O busy waiting detection
 * 24-AUG-2008 changed terminal line discipline to not add CR if LF send
 * xx-OCT-2008 some improvements here and there
 * xx-JAN-2014 some improvements here and there
 * 02-MAR-2014 source cleanup and improvements
 * 03-MAI-2014 improved network code, telnet negotiation rewritten
 * 16-JUL-2014 unused I/O ports need to return FF, see survey.mac
 * 17-SEP-2014 FDC error 8 for DMA overrun, as reported by Alan Cox
 * 17-SEP-2014 fixed bug in MMU bank select, as reported by Alan Cox
 * 31-JAN-2015 took over some improvements made for the Z-1 emulation
 * 28-FEB-2015 cleanup for 1.25 release
 * 09-MAR-2016 moved pipes to /tmp/.z80pack
 * 14-MAR-2016 renamed the used disk images to drivex.dsk
 * 12-MAY-2016 find disk images at -d <path>, ./disks and DISKSDIR
 * 22-JUL-2016 added support for read only disks
 * 21-DEC-2016 moved MMU out to the new memory interface module
 * 20-MAR-2017 renamed pipes for auxin/auxout
 * 29-JUN-2017 system reset overworked
 */

/*
 *	This module contains the I/O handlers for a simulation
 *	of the hardware required for a CP/M / MP/M system.
 *
 *	Used I/O ports:
 *
 *	 0 - KBD(input)/SPKR(output)
 *	 1 - display command (output)
 *	 5 - display data (input/output)
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <time.h>
#include <netdb.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include "sim.h"
#include "simglb.h"
#include "memory.h"
#include "il9341.h"
#include "lcd_emu.h"

#define BUFSIZE 256		/* max line length of command buffer */
#define MAX_BUSY_COUNT 10	/* max counter to detect I/O busy waiting
				   on the console status port */

extern int boot(void);
extern void reset_cpu(void);

/*
 *	Forward declaration of the I/O handlers for all used ports
 */
static BYTE io_trap_in(void);
static void io_trap_out(BYTE);
static BYTE port_fe_in(void), il9341_data_in(void);
static void port_fe_out(BYTE), il9341_cmd_out(BYTE), il9341_data_out(BYTE);

/*
 *	Forward declaration of support functions
 */
static void int_timer(int);

/*
 *	This array contains function pointers for every
 *	input port.
 */
static BYTE (*port_in[256]) (void) = {
	io_trap_in, 	/* port 0 */
	io_trap_in,		/* port 1 */
	io_trap_in,		/* port 2 */
	io_trap_in,		/* port 3 */
	io_trap_in,		/* port 4 */
	il9341_data_in,	/* port 5 */
	io_trap_in,		/* port 6 */
	io_trap_in,		/* port 7 */
	io_trap_in,		/* port 8 */
	io_trap_in,		/* port 9 */
	io_trap_in,		/* port 10 */
	io_trap_in,		/* port 11 */
	io_trap_in,		/* port 12 */
	io_trap_in,		/* port 13 */
	io_trap_in,		/* port 14 */
	io_trap_in,		/* port 15 */
	io_trap_in,		/* port 16 */
	io_trap_in,		/* port 17 */
	io_trap_in,		/* port 18 */
	io_trap_in,		/* port 19 */
	io_trap_in,		/* port 20 */
	io_trap_in,		/* port 21 */
	io_trap_in,		/* port 22 */
	io_trap_in,		/* port 23 */
	io_trap_in,		/* port 24 */
	io_trap_in,		/* port 25 */
	io_trap_in,		/* port 26 */
	io_trap_in,		/* port 27 */
	io_trap_in,		/* port 28 */
	io_trap_in,		/* port 29 */
	io_trap_in,		/* port 30 */
	io_trap_in,		/* port 31 */
	io_trap_in,		/* port 32 */
	io_trap_in,		/* port 33 */
	io_trap_in,		/* port 34 */
	io_trap_in,		/* port 35 */
	io_trap_in,		/* port 36 */
	io_trap_in,		/* port 37 */
	io_trap_in,		/* port 38 */
	io_trap_in,		/* port 39 */
	io_trap_in,		/* port 40 */
	io_trap_in,		/* port 41 */
	io_trap_in,		/* port 42 */
	io_trap_in,		/* port 43 */
	io_trap_in,		/* port 44 */
	io_trap_in,		/* port 45 */
	io_trap_in,		/* port 46 */
	io_trap_in,		/* port 47 */
	io_trap_in,		/* port 48 */
	io_trap_in,		/* port 49 */
	io_trap_in,		/* port 50 */
	io_trap_in,		/* port 51 */
	io_trap_in,		/* port 52 */
	io_trap_in,		/* port 53 */
	io_trap_in,		/* port 54 */
	io_trap_in,		/* port	55 */
	io_trap_in,		/* port	56 */
	io_trap_in,		/* port	57 */
	io_trap_in,		/* port	58 */
	io_trap_in,		/* port	59 */
	io_trap_in,		/* port	60 */
	io_trap_in,		/* port	61 */
	io_trap_in,		/* port	62 */
	io_trap_in,		/* port	63 */
	io_trap_in,		/* port	64 */
	io_trap_in,		/* port	65 */
	io_trap_in,		/* port	66 */
	io_trap_in,		/* port	67 */
	io_trap_in,		/* port	68 */
	io_trap_in,		/* port	69 */
	io_trap_in,		/* port	70 */
	io_trap_in,		/* port	71 */
	io_trap_in,		/* port	72 */
	io_trap_in,		/* port	73 */
	io_trap_in,		/* port	74 */
	io_trap_in,		/* port	75 */
	io_trap_in,		/* port	76 */
	io_trap_in,		/* port	77 */
	io_trap_in,		/* port	78 */
	io_trap_in,		/* port	79 */
	io_trap_in,		/* port	80 */
	io_trap_in,		/* port	81 */
	io_trap_in,		/* port	82 */
	io_trap_in,		/* port	83 */
	io_trap_in,		/* port	84 */
	io_trap_in,		/* port	85 */
	io_trap_in,		/* port	86 */
	io_trap_in,		/* port	87 */
	io_trap_in,		/* port	88 */
	io_trap_in,		/* port	89 */
	io_trap_in,		/* port	90 */
	io_trap_in,		/* port	91 */
	io_trap_in,		/* port	92 */
	io_trap_in,		/* port	93 */
	io_trap_in,		/* port	94 */
	io_trap_in,		/* port	95 */
	io_trap_in,		/* port	96 */
	io_trap_in,		/* port	97 */
	io_trap_in,		/* port	98 */
	io_trap_in,		/* port	99 */
	io_trap_in,		/* port	100 */
	io_trap_in,		/* port 101 */
	io_trap_in,		/* port	102 */
	io_trap_in,		/* port	103 */
	io_trap_in,		/* port	104 */
	io_trap_in,		/* port	105 */
	io_trap_in,		/* port	106 */
	io_trap_in,		/* port	107 */
	io_trap_in,		/* port	108 */
	io_trap_in,		/* port	109 */
	io_trap_in,		/* port	110 */
	io_trap_in,		/* port	111 */
	io_trap_in,		/* port	112 */
	io_trap_in,		/* port	113 */
	io_trap_in,		/* port	114 */
	io_trap_in,		/* port	115 */
	io_trap_in,		/* port	116 */
	io_trap_in,		/* port	117 */
	io_trap_in,		/* port	118 */
	io_trap_in,		/* port	119 */
	io_trap_in,		/* port	120 */
	io_trap_in,		/* port	121 */
	io_trap_in,		/* port	122 */
	io_trap_in,		/* port	123 */
	io_trap_in,		/* port	124 */
	io_trap_in,		/* port	125 */
	io_trap_in,		/* port	126 */
	io_trap_in,		/* port	127 */
	io_trap_in,		/* port	128 */
	io_trap_in,		/* port	129 */
	io_trap_in,		/* port	130 */
	io_trap_in,		/* port	131 */
	io_trap_in,		/* port	132 */
	io_trap_in,		/* port	133 */
	io_trap_in,		/* port	134 */
	io_trap_in,		/* port	135 */
	io_trap_in,		/* port	136 */
	io_trap_in,		/* port	137 */
	io_trap_in,		/* port	138 */
	io_trap_in,		/* port	139 */
	io_trap_in,		/* port	140 */
	io_trap_in,		/* port	141 */
	io_trap_in,		/* port	142 */
	io_trap_in,		/* port	143 */
	io_trap_in,		/* port	144 */
	io_trap_in,		/* port	145 */
	io_trap_in,		/* port	146 */
	io_trap_in,		/* port	147 */
	io_trap_in,		/* port	148 */
	io_trap_in,		/* port	149 */
	io_trap_in,		/* port	150 */
	io_trap_in,		/* port	151 */
	io_trap_in,		/* port	152 */
	io_trap_in,		/* port	153 */
	io_trap_in,		/* port	154 */
	io_trap_in,		/* port	155 */
	io_trap_in,		/* port	156 */
	io_trap_in,		/* port	157 */
	io_trap_in,		/* port	158 */
	io_trap_in,		/* port	159 */
	io_trap_in,		/* port	160 */
	io_trap_in,		/* port	161 */
	io_trap_in,		/* port	162 */
	io_trap_in,		/* port	163 */
	io_trap_in,		/* port	164 */
	io_trap_in,		/* port	165 */
	io_trap_in,		/* port	166 */
	io_trap_in,		/* port	167 */
	io_trap_in,		/* port	168 */
	io_trap_in,		/* port	169 */
	io_trap_in,		/* port	170 */
	io_trap_in,		/* port	171 */
	io_trap_in,		/* port	172 */
	io_trap_in,		/* port	173 */
	io_trap_in,		/* port	174 */
	io_trap_in,		/* port	175 */
	io_trap_in,		/* port	176 */
	io_trap_in,		/* port	177 */
	io_trap_in,		/* port	178 */
	io_trap_in,		/* port	179 */
	io_trap_in,		/* port	180 */
	io_trap_in,		/* port	181 */
	io_trap_in,		/* port	182 */
	io_trap_in,		/* port	183 */
	io_trap_in,		/* port	184 */
	io_trap_in,		/* port	185 */
	io_trap_in,		/* port	186 */
	io_trap_in,		/* port	187 */
	io_trap_in,		/* port	188 */
	io_trap_in,		/* port	189 */
	io_trap_in,		/* port	190 */
	io_trap_in,		/* port	191 */
	io_trap_in,		/* port	192 */
	io_trap_in,		/* port	193 */
	io_trap_in,		/* port	194 */
	io_trap_in,		/* port	195 */
	io_trap_in,		/* port	196 */
	io_trap_in,		/* port	197 */
	io_trap_in,		/* port	198 */
	io_trap_in,		/* port	199 */
	io_trap_in,		/* port	200 */
	io_trap_in,		/* port 201 */
	io_trap_in,		/* port	202 */
	io_trap_in,		/* port	203 */
	io_trap_in,		/* port	204 */
	io_trap_in,		/* port	205 */
	io_trap_in,		/* port	206 */
	io_trap_in,		/* port	207 */
	io_trap_in,		/* port	208 */
	io_trap_in,		/* port	209 */
	io_trap_in,		/* port	210 */
	io_trap_in,		/* port	211 */
	io_trap_in,		/* port	212 */
	io_trap_in,		/* port	213 */
	io_trap_in,		/* port	214 */
	io_trap_in,		/* port	215 */
	io_trap_in,		/* port	216 */
	io_trap_in,		/* port	217 */
	io_trap_in,		/* port	218 */
	io_trap_in,		/* port	219 */
	io_trap_in,		/* port	220 */
	io_trap_in,		/* port	221 */
	io_trap_in,		/* port	222 */
	io_trap_in,		/* port	223 */
	io_trap_in,		/* port	224 */
	io_trap_in,		/* port	225 */
	io_trap_in,		/* port	226 */
	io_trap_in,		/* port	227 */
	io_trap_in,		/* port	228 */
	io_trap_in,		/* port	229 */
	io_trap_in,		/* port	230 */
	io_trap_in,		/* port	231 */
	io_trap_in,		/* port	232 */
	io_trap_in,		/* port	233 */
	io_trap_in,		/* port	234 */
	io_trap_in,		/* port	235 */
	io_trap_in,		/* port	236 */
	io_trap_in,		/* port	237 */
	io_trap_in,		/* port	238 */
	io_trap_in,		/* port	239 */
	io_trap_in,		/* port	240 */
	io_trap_in,		/* port	241 */
	io_trap_in,		/* port	242 */
	io_trap_in,		/* port	243 */
	io_trap_in,		/* port	244 */
	io_trap_in,		/* port	245 */
	io_trap_in,		/* port	246 */
	io_trap_in,		/* port	247 */
	io_trap_in,		/* port	248 */
	io_trap_in,		/* port	249 */
	io_trap_in,		/* port	250 */
	io_trap_in,		/* port	251 */
	io_trap_in,		/* port	252 */
	io_trap_in,		/* port	253 */
	port_fe_in,		/* port	254 */
	io_trap_in		/* port	255 */
};

/*
 *	This array contains function pointers for every
 *	output port.
 */
static void (*port_out[256]) (BYTE) = {
	io_trap_out,		/* port 0 */
	il9341_cmd_out, 	/* port 1 */
	io_trap_out,		/* port 2 */
	io_trap_out,		/* port 3 */
	io_trap_out,		/* port 4 */
	il9341_data_out,	/* port 5 */
	io_trap_out,		/* port 6 */
	io_trap_out,		/* port 7 */
	io_trap_out,		/* port 8 */
	io_trap_out,		/* port 9 */
	io_trap_out,		/* port 10 */
	io_trap_out,		/* port 11 */
	io_trap_out,		/* port 12 */
	io_trap_out,		/* port 13 */
	io_trap_out,		/* port 14 */
	io_trap_out,		/* port 15 */
	io_trap_out,		/* port 16 */
	io_trap_out,		/* port 17 */
	io_trap_out,		/* port 18 */
	io_trap_out,		/* port 19 */
	io_trap_out,		/* port 20 */
	io_trap_out,		/* port 21 */
	io_trap_out,		/* port 22 */
	io_trap_out,		/* port 23 */
	io_trap_out,		/* port 24 */
	io_trap_out,		/* port 25 */
	io_trap_out,		/* port 26 */
	io_trap_out,		/* port 27 */
	io_trap_out,		/* port 28 */
	io_trap_out,		/* port 29 */
	io_trap_out,		/* port 30 */
	io_trap_out,		/* port 31 */
	io_trap_out,		/* port 32 */
	io_trap_out,		/* port 33 */
	io_trap_out,		/* port 34 */
	io_trap_out,		/* port 35 */
	io_trap_out,		/* port 36 */
	io_trap_out,		/* port 37 */
	io_trap_out,		/* port 38 */
	io_trap_out,		/* port 39 */
	io_trap_out,		/* port 40 */
	io_trap_out,		/* port 41 */
	io_trap_out,		/* port 42 */
	io_trap_out,		/* port 43 */
	io_trap_out,		/* port 44 */
	io_trap_out,		/* port 45 */
	io_trap_out,		/* port 46 */
	io_trap_out,		/* port 47 */
	io_trap_out,		/* port 48 */
	io_trap_out,		/* port 49 */
	io_trap_out,		/* port 50 */
	io_trap_out,		/* port 51 */
	io_trap_out,		/* port 52 */
	io_trap_out,		/* port 53 */
	io_trap_out,		/* port 54 */
	io_trap_out,		/* port	55 */
	io_trap_out,		/* port	56 */
	io_trap_out,		/* port	57 */
	io_trap_out,		/* port	58 */
	io_trap_out,		/* port	59 */
	io_trap_out,		/* port	60 */
	io_trap_out,		/* port	61 */
	io_trap_out,		/* port	62 */
	io_trap_out,		/* port	63 */
	io_trap_out,		/* port	64 */
	io_trap_out,		/* port	65 */
	io_trap_out,		/* port	66 */
	io_trap_out,		/* port	67 */
	io_trap_out,		/* port	68 */
	io_trap_out,		/* port	69 */
	io_trap_out,		/* port	70 */
	io_trap_out,		/* port	71 */
	io_trap_out,		/* port	72 */
	io_trap_out,		/* port	73 */
	io_trap_out,		/* port	74 */
	io_trap_out,		/* port	75 */
	io_trap_out,		/* port	76 */
	io_trap_out,		/* port	77 */
	io_trap_out,		/* port	78 */
	io_trap_out,		/* port	79 */
	io_trap_out,		/* port	80 */
	io_trap_out,		/* port	81 */
	io_trap_out,		/* port	82 */
	io_trap_out,		/* port	83 */
	io_trap_out,		/* port	84 */
	io_trap_out,		/* port	85 */
	io_trap_out,		/* port	86 */
	io_trap_out,		/* port	87 */
	io_trap_out,		/* port	88 */
	io_trap_out,		/* port	89 */
	io_trap_out,		/* port	90 */
	io_trap_out,		/* port	91 */
	io_trap_out,		/* port	92 */
	io_trap_out,		/* port	93 */
	io_trap_out,		/* port	94 */
	io_trap_out,		/* port	95 */
	io_trap_out,		/* port	96 */
	io_trap_out,		/* port	97 */
	io_trap_out,		/* port	98 */
	io_trap_out,		/* port	99 */
	io_trap_out,		/* port	100 */
	io_trap_out,		/* port 101 */
	io_trap_out,		/* port	102 */
	io_trap_out,		/* port	103 */
	io_trap_out,		/* port	104 */
	io_trap_out,		/* port	105 */
	io_trap_out,		/* port	106 */
	io_trap_out,		/* port	107 */
	io_trap_out,		/* port	108 */
	io_trap_out,		/* port	109 */
	io_trap_out,		/* port	110 */
	io_trap_out,		/* port	111 */
	io_trap_out,		/* port	112 */
	io_trap_out,		/* port	113 */
	io_trap_out,		/* port	114 */
	io_trap_out,		/* port	115 */
	io_trap_out,		/* port	116 */
	io_trap_out,		/* port	117 */
	io_trap_out,		/* port	118 */
	io_trap_out,		/* port	119 */
	io_trap_out,		/* port	120 */
	io_trap_out,		/* port	121 */
	io_trap_out,		/* port	122 */
	io_trap_out,		/* port	123 */
	io_trap_out,		/* port	124 */
	io_trap_out,		/* port	125 */
	io_trap_out,		/* port	126 */
	io_trap_out,		/* port	127 */
	io_trap_out,		/* port	128 */
	io_trap_out,		/* port	129 */
	io_trap_out,		/* port	130 */
	io_trap_out,		/* port	131 */
	io_trap_out,		/* port	132 */
	io_trap_out,		/* port	133 */
	io_trap_out,		/* port	134 */
	io_trap_out,		/* port	135 */
	io_trap_out,		/* port	136 */
	io_trap_out,		/* port	137 */
	io_trap_out,		/* port	138 */
	io_trap_out,		/* port	139 */
	io_trap_out,		/* port	140 */
	io_trap_out,		/* port	141 */
	io_trap_out,		/* port	142 */
	io_trap_out,		/* port	143 */
	io_trap_out,		/* port	144 */
	io_trap_out,		/* port	145 */
	io_trap_out,		/* port	146 */
	io_trap_out,		/* port	147 */
	io_trap_out,		/* port	148 */
	io_trap_out,		/* port	149 */
	io_trap_out,		/* port	150 */
	io_trap_out,		/* port	151 */
	io_trap_out,		/* port	152 */
	io_trap_out,		/* port	153 */
	io_trap_out,		/* port	154 */
	io_trap_out,		/* port	155 */
	io_trap_out,		/* port	156 */
	io_trap_out,		/* port	157 */
	io_trap_out,		/* port	158 */
	io_trap_out,		/* port	159 */
	io_trap_out,		/* port	160 */
	io_trap_out,		/* port	161 */
	io_trap_out,		/* port	162 */
	io_trap_out,		/* port	163 */
	io_trap_out,		/* port	164 */
	io_trap_out,		/* port	165 */
	io_trap_out,		/* port	166 */
	io_trap_out,		/* port	167 */
	io_trap_out,		/* port	168 */
	io_trap_out,		/* port	169 */
	io_trap_out,		/* port	170 */
	io_trap_out,		/* port	171 */
	io_trap_out,		/* port	172 */
	io_trap_out,		/* port	173 */
	io_trap_out,		/* port	174 */
	io_trap_out,		/* port	175 */
	io_trap_out,		/* port	176 */
	io_trap_out,		/* port	177 */
	io_trap_out,		/* port	178 */
	io_trap_out,		/* port	179 */
	io_trap_out,		/* port	180 */
	io_trap_out,		/* port	181 */
	io_trap_out,		/* port	182 */
	io_trap_out,		/* port	183 */
	io_trap_out,		/* port	184 */
	io_trap_out,		/* port	185 */
	io_trap_out,		/* port	186 */
	io_trap_out,		/* port	187 */
	io_trap_out,		/* port	188 */
	io_trap_out,		/* port	189 */
	io_trap_out,		/* port	190 */
	io_trap_out,		/* port	191 */
	io_trap_out,		/* port	192 */
	io_trap_out,		/* port	193 */
	io_trap_out,		/* port	194 */
	io_trap_out,		/* port	195 */
	io_trap_out,		/* port	196 */
	io_trap_out,		/* port	197 */
	io_trap_out,		/* port	198 */
	io_trap_out,		/* port	199 */
	io_trap_out,		/* port	200 */
	io_trap_out,		/* port 201 */
	io_trap_out,		/* port	202 */
	io_trap_out,		/* port	203 */
	io_trap_out,		/* port	204 */
	io_trap_out,		/* port	205 */
	io_trap_out,		/* port	206 */
	io_trap_out,		/* port	207 */
	io_trap_out,		/* port	208 */
	io_trap_out,		/* port	209 */
	io_trap_out,		/* port	210 */
	io_trap_out,		/* port	211 */
	io_trap_out,		/* port	212 */
	io_trap_out,		/* port	213 */
	io_trap_out,		/* port	214 */
	io_trap_out,		/* port	215 */
	io_trap_out,		/* port	216 */
	io_trap_out,		/* port	217 */
	io_trap_out,		/* port	218 */
	io_trap_out,		/* port	219 */
	io_trap_out,		/* port	220 */
	io_trap_out,		/* port	221 */
	io_trap_out,		/* port	222 */
	io_trap_out,		/* port	223 */
	io_trap_out,		/* port	224 */
	io_trap_out,		/* port	225 */
	io_trap_out,		/* port	226 */
	io_trap_out,		/* port	227 */
	io_trap_out,		/* port	228 */
	io_trap_out,		/* port	229 */
	io_trap_out,		/* port	230 */
	io_trap_out,		/* port	231 */
	io_trap_out,		/* port	232 */
	io_trap_out,		/* port	233 */
	io_trap_out,		/* port	234 */
	io_trap_out,		/* port	235 */
	io_trap_out,		/* port	236 */
	io_trap_out,		/* port	237 */
	io_trap_out,		/* port	238 */
	io_trap_out,		/* port	239 */
	io_trap_out,		/* port	240 */
	io_trap_out,		/* port	241 */
	io_trap_out,		/* port	242 */
	io_trap_out,		/* port	243 */
	io_trap_out,		/* port	244 */
	io_trap_out,		/* port	245 */
	io_trap_out,		/* port	246 */
	io_trap_out,		/* port	247 */
	io_trap_out,		/* port	248 */
	io_trap_out,		/* port	249 */
	io_trap_out,		/* port	250 */
	io_trap_out,		/* port	251 */
	io_trap_out,		/* port	252 */
	io_trap_out,		/* port	253 */
	port_fe_out,		/* port	254 */
	io_trap_out		/* port	255 */
};

/*
 *	This function initialises the I/O handlers:
 */
void init_io(void)
{
	static struct itimerval tim;
	static struct sigaction newact;

    il9341_init();

    // Set up the 50ms timer
	newact.sa_handler = int_timer;
	memset((void *) &newact.sa_mask, 0, sizeof(newact.sa_mask));
	newact.sa_flags = 0;
	sigaction(SIGALRM, &newact, NULL);
	tim.it_value.tv_sec = 0;
	tim.it_value.tv_usec = 50000;
	tim.it_interval.tv_sec = 0;
	tim.it_interval.tv_usec = 50000;
	setitimer(ITIMER_REAL, &tim, NULL);
}

/*
 *	This function stops the I/O handlers:
 *
 */
void exit_io(void)
{
}

/*
 *	reset the CPU and I/O system
 */
void reset_system(void)
{
	extern BYTE *wrk_ram;

	/* reset CPU */
	reset_cpu();
	wrk_ram	= mem_base();

	/* reboot */
	boot();
}

/*
 *	This function is called for every IN opcode from the
 *	CPU emulation. It calls the handler for the port,
 *	from which input is wanted.
 */
BYTE io_in(BYTE addrl, BYTE addrh)
{
	io_port_h = addrh;

	io_port = addrl;
	io_data = (*port_in[addrl]) ();
	//printf("input %02x from port %02x\r\n", io_data, io_port);
	return(io_data);
}

/*
 *	This function is called for every OUT opcode from the
 *	CPU emulation. It calls the handler for the port,
 *	to which output is wanted.
 */
void io_out(BYTE addrl, BYTE addrh, BYTE data)
{
	io_port_h = addrh;

	io_port = addrl;
	io_data = data;

	busy_loop_cnt[0] = 0;

	(*port_out[addrl]) (data);
	//printf("output %02x to port %02x\r\n", io_data, io_port)";
}

/*
 *	I/O input trap handler
 */
static BYTE io_trap_in(void)
{
	if (i_flag) {
		cpu_error = IOTRAPIN;
		cpu_state = STOPPED;
	}
	return((BYTE) 0xff);
}

/*
 *	I/O output trap handler
 */
static void io_trap_out(BYTE data)
{
	data = data;	/* to avoid compiler warning */

	if (i_flag) {
		cpu_error = IOTRAPOUT;
		cpu_state = STOPPED;
	}
}

/*
 *	I/O handler for keyboard
 */
static BYTE port_fe_in(void)
{
	static BYTE keys[2];
	static int keycount;
	static BYTE ports[2];
	static int symbol_shift = 0;

	// Key tables
	static BYTE keytable[] = {
		0xfe, // 0
		0xfe, // 1
		0xfd, // 2
		0xfb, // 3
		0xf7, // 4
		0xef, // 5
		0xef, // 6
		0xf7, // 7
		0xfb, // 8
		0xfd, // 9
		0xfe, // a
		0xef, // b
		0xf7, // c
		0xfb, // d
		0xfb, // e
		0xf7, // f
		0xef, // g
		0xef, // h
		0xfb, // i
		0xf7, // j
		0xfb, // k 
		0xfd, // l
		0xfb, // m
		0xf7, // n
		0xfd, // o
		0xfe, // p
		0xfe, // q
		0xf7, // r
		0xfd, // s
		0xef, // t
		0xf7, // u
		0xef, // v
		0xfd, // w
		0xfb, // x
		0xef, // y
		0xfd, // z 
	};

	if ((keycount == 0) || symbol_shift)
	{
		struct timeval tv;
		fd_set fds;
		tv.tv_sec = 0;
		tv.tv_usec = 0;
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
		if (FD_ISSET(STDIN_FILENO, &fds))
		{
			char c = fgetc(stdin);

			// numbers
			if (c >= '0' && c <= '9')
			{
				if (c >= '1' && c <= '5')
				{
					ports[keycount] = 0xf7;
				}
				else
				{
					ports[keycount] = 0xef;
				}
				keys[keycount] = keytable[c-'0'];
				keycount++;
			}
			// letters
			else if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))
			{
				char cc = c;

				if (c < 'a')
				{
					cc = 'a' + (c-'A');
				}
				switch(cc)
				{
					case 'q': case 'w': case 'e': case 'r': case 't': ports[keycount] = 0xfb; break;
					case 'y': case 'u': case 'i': case 'o': case 'p': ports[keycount] = 0xdf; break;
					case 'a': case 's': case 'd': case 'f': case 'g': ports[keycount] = 0xfd; break;
					case 'h': case 'j': case 'k': case 'l':           ports[keycount] = 0xbf; break;
					case 'z': case 'x': case 'c': case 'v':           ports[keycount] = 0xfe; break;
					case 'b': case 'n': case 'm':                     ports[keycount] = 0x7f; break;
					default: break;
				}
				keys[keycount] = keytable[10 + cc - 'a'];
				keycount++;
				if (c < 'a')
				{
					if (!symbol_shift)
					{
						// caps shift
						keys[keycount] = 0xfe;
						ports[keycount] = 0xfe;
						keycount++;
					}
				}
			}
			// tab (used as BREAK)
			else if (c == '\t')
			{
				keys[1] = 0xfe;
				ports[1] = 0x7f;
				keys[0] = 0xfe;
				ports[0] = 0xfe;
				keycount = 2;
			}
			// enter
			else if (c == '\r')
			{
				keys[0] = 0xfe;
				ports[0] = 0xbf;
				keycount++;
			}
			// space
			else if (c == ' ')
			{
				keys[0] = 0xfe;
				ports[0] = 0x7f;
				keycount++;
			}
			// sym shift
			else if (c == '`')
			{
				keys[0] = 0xfd;
				ports[0] = 0x7f;
				keycount = 1;
				symbol_shift = 1;
			}
			// caps+sym shift
			else if (c == '~')
			{
				keys[0] = 0xfd;
				ports[0] = 0x7f;
				keys[1] = 0xfe;
				ports[1] = 0xfe;
				keycount = 2;
			}
			// shifted numbers
			else
			{
				keys[1] = 0xfe;
				ports[1] = 0xfe;
				keycount = 2;
				switch(c)
				{
					case '!' : keys[0] = 0xfe; ports[0] = 0xf7; break;
					case '@' : keys[0] = 0xfd; ports[0] = 0xf7; break;
					case '#' : keys[0] = 0xfb; ports[0] = 0xf7; break;
					case '$' : keys[0] = 0xf7; ports[0] = 0xf7; break;
					case '%' : keys[0] = 0xef; ports[0] = 0xf7; break;
					case '^' : keys[0] = 0xef; ports[0] = 0xef; break;
					case '&' : keys[0] = 0xf7; ports[0] = 0xef; break;
					case '*' : keys[0] = 0xfb; ports[0] = 0xef; break;
					case '(' : keys[0] = 0xfd; ports[0] = 0xef; break;
					case ')' : keys[0] = 0xfe; ports[0] = 0xef; break;
					default: keycount = 0; break;
				}
			}
		}
	}

	if (symbol_shift && (keycount < 2))
	{
		return 0xff;
	}
	else if (keycount && (io_port_h == ports[keycount-1]))
	{
		if (symbol_shift && keycount == 2 && ports[1] == 0x7f)
		{
			symbol_shift = 0;
			keycount = 0;
			return keys[1] & keys[0];
		}
		symbol_shift = 0;
		return keys[--keycount];
	}
	else
	{
		return 0xff;
	}
}

/*
 *	I/O handler for speaker
 */
static void port_fe_out(BYTE data)
{
	// silence the warning
	data = data;
	//fb_set_border(data & 0x7);
}
/*
 *	I/O handler for read display RAM
 */
static BYTE il9341_data_in(void)
{
	return((BYTE) 0);
}

/*
 *	I/O handler for write display command
 */
static void il9341_cmd_out(BYTE data)
{
	il9341_wr_cmd(data);
}

/*
 *	I/O handler for write display RAM
 */
static void il9341_data_out(BYTE data)
{
	il9341_wr_data(data);
}

/*
 *	timer interrupt causes maskable CPU interrupt and display update
 */
static void int_timer(int sig)
{
	sig = sig;	/* to avoid compiler warning */

	int_int = 1;
	int_data = 0xff;	/* RST 38H for IM 0, 0FFH for IM 2 */
 	il9341_update();
}

