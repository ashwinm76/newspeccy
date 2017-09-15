/*
 * Z80SIM  -  a Z80-CPU simulator
 *
 * Copyright (C) 2016-2017 by Udo Munk
 *
 * This module implements the memory for z80sim
 *
 * History:
 * 22-DEC-2016 stuff moved to here for better memory abstraction
 * 03-FEB-17 added ROM initialisation
 */

#include "sim.h"
#include "lcd_emu.h"
#include "memory.h"

/* non banked memory */
BYTE memory[MEMORY_SIZE];

void init_memory(void)
{
}

void init_rom(void)
{
}

void memwrt(WORD addr, BYTE data)
{
	if (addr > MEMORY_SIZE-1)
	{
		return;
	}
	memory[addr] = data; 
	fbwr(addr, data);
}

BYTE memrdr(WORD addr)
{
	if (addr > MEMORY_SIZE-1)
	{
		return 0;
	}
	return memory[addr];
}
