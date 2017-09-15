#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include "simglb.h"
#include "memory.h"

#ifdef LCD_EMU
#include "il9341.h"
#ifdef LOG_LCD_MEM
FILE* mem_log_file;
#endif
#endif

// Bright Red, Green and Blue colour masks for RGB565 colour
#define RM      0xf800
#define GM      0x07e0
#define BM      0x001f
// Normal Red, Green and Blue colour masks for RGB565 colour
#define RM0     0xb800
#define GM0     0x05e0
#define BM0     0x0017

WORD colour_table[] = 
{
        // Spectrum colours are:         n GRB
	
	// Normal colours
	0               ,// COLOUR_BLACK  = 0 000
	BM0             ,// COLOUR_BLUE   = 1 001
	RM0             ,// COLOUR_RED    = 2 010
	RM0 | BM0       ,// COLOUR_PURPLE = 3 011
	GM0             ,// COLOUR_GREEN  = 4 100
	GM0 | BM0       ,// COLOUR_CYAN   = 5 101
	GM0 | RM0       ,// COLOUR_YELLOW = 6 110
	GM0 | RM0 | BM0 ,// COLOUR_WHITE  = 7 111

	// Bright colours
	0            ,// COLOUR_BLACK  = 0 000
	BM           ,// COLOUR_BLUE   = 1 001
	RM           ,// COLOUR_RED    = 2 010
	RM | BM      ,// COLOUR_PURPLE = 3 011
	GM           ,// COLOUR_GREEN  = 4 100
	GM | BM      ,// COLOUR_CYAN   = 5 101
	GM | RM      ,// COLOUR_YELLOW = 6 110
	GM | RM | BM ,// COLOUR_WHITE  = 7 111
};

void fbinit()
{
#ifdef LOG_LCD_MEM
	mem_log_file = fopen("memory.log", "w");
	if (mem_log_file == NULL)
	{
		perror("memory log");
		exit(1);
	}
#endif
}

static void fill(int startx, int endx, int starty, int endy, int colour)
{
	il9341_set_window(startx, endx, starty, endy);
	il9341_wr_cmd(0x2c);
	for(int y=starty; y<=endy; y++)
	{
		for(int x=startx; x<=endx; x++)
		{
			il9341_wr_data(colour >> 8);
			il9341_wr_data(colour & 0xff);
		}
	}
}
void fb_set_border(BYTE colour)
{
#ifdef LOG_LCD_MEM
	fprintf(mem_log_file, "BORDER WR, PC = 0x%04x\r\n", PC);
#endif
	colour &= 0x7;
	
	fill(0, 319, 0, LCD_WIN_Y_START-1, colour_table[colour]);
	fill(0, LCD_WIN_X_START-1, 0, 239, colour_table[colour]);
	fill(LCD_WIN_X_END+1, 319, 0, 239, colour_table[colour]);
	fill(0, 319, LCD_WIN_Y_END+1, 239, colour_table[colour]);
}

void fbwr(WORD addr, BYTE data)
{
#ifdef LOG_LCD_MEM
	if (addr >= 16384 && addr <= (16384 + 6144 - 1))
	{
		fprintf(mem_log_file, "FB WR, PC = 0x%04x, ADDR = 0x%04x\n", PC, addr);
	}
	else if (addr >= (16384 + 6144) && addr <= (16384 + 6144 + 768 - 1))
	{
		fprintf(mem_log_file, "ATTR WR, PC = 0x%04x, ADDR = 0x%04x\n", PC, addr);
	}
#endif

	if (addr < 16384 || addr >= (16384 + 6144 + 768))
	{
	  return;
	}

	if (addr >= (16384 + 6144))
	{
		// 768 attrs = 0x300
		// attr addresses: 0x5800 - 0x5aff
		// charx = addr & 0x1f
		// charline = ((addr - 16384 - 6144) >> 5) & 7 [0..7]
		//			= (addr & 0x3ff) >> 5 & 7
		//			= (addr >> 5) & 0x7
		// third = charline >> 3
		//		 = (addr>>5 & 0x1f) >> 3
		//		 = (addr >> 8) & 3
		//		 = addr[MSB] & 3
		// fb-char-start-offset = (third * 32*8*8) + charline*32 + charx
		//						= (third << 11) + charline<<5 + charx
		// x = LCD_WIN_X_START + charx*8
		//   = LCD_WIN_X_START + (charx << 3)
		// y = LCD_WIN_Y_START + (8*third + charline)*8
		//   = LCD_WIN_Y_START + ((third << 3) + charline) << 3
		int charx = addr & 0x1f;
		int charline = (addr >> 5) & 0x7;
		int third = (addr >> 8) & 3;
		int fb_addr = 16384 + (third << 11) + (charline << 5) + charx;
		int y = LCD_WIN_Y_START + (8*third + charline)*8;
		int ink_colour = data & 7;
		int paper_colour = (data >> 3) & 7;
		if (data & 0x40)
		{
			ink_colour += 8;
			paper_colour += 8;
		}
		//fprintf(mem_log_file, "fbaddr = %04x, third = %d, charx=%d, charline=%d, ink = %d, paper = %d\r\n", fb_addr, third, charx, charline, ink_colour, paper_colour);
		for(int l=0; l<8; l++)
		{
			BYTE row = memory[fb_addr];
			int x = LCD_WIN_X_START + charx*8;
			il9341_set_window(x, LCD_WIN_X_END, y, LCD_WIN_Y_END);
			il9341_wr_cmd(0x2c);
			for(int i=0; i<8; i++)
			{
				WORD pixel_colour = colour_table[(row & 0x80) ? ink_colour : paper_colour];
				il9341_wr_data(pixel_colour >> 8);
				il9341_wr_data(pixel_colour & 0xff);
				//fputc((row & 0x80) ? '#' : '.', mem_log_file);
				row <<= 1;
			}
			//fputc('\r', mem_log_file);
			//fputc('\n', mem_log_file);
			fb_addr += 8*32;
			y++;
		}
		return;
	}

	// x = (addr%32) * 8
	// line = (addr >> 5) - 0x200 -> 0 to 191 [0->0xbf]
	// third = line >> 6
	// y = (((line&0x3f)*8)&0x3f) + ((line&0x3f)*8)/64 + third*64
	//   = (((line&0x3f)<<3)&0x3f) + ((line&0x3f)<<3)>>6 + line&0xc0
	//   = (((line&0x3f)<<3)&0x3f) + (line&0x3f)>>3 + line&0xc0
	// charx = x/8 
	//       = addr%32 = addr&0x1f
	// chary = y/8

	int x = LCD_WIN_X_START + ((addr & 0x1f) << 3);
	int line = (addr-16384)>>5;
	int y = LCD_WIN_Y_START + (((line&0x3f)<<3)&0x3f) + ((line&0x3f)>>3) + (line&0xc0);
	il9341_set_window(x, x+8, y, y+1);
	il9341_wr_cmd(0x2c);
	//fprintf(mem_log_file, "FB addr = %04x, charx=%d, charline=%d\r\n", addr, (x-LCD_WIN_X_START)/8, (y-LCD_WIN_Y_START)/8);
	for(int i=0; i<8; i++)
	{
		WORD attrib_index = 16384 + 6144 + (y >> 3) + (addr & 0x1f);
		int ink_colour = memory[attrib_index] & 7;
		int paper_colour = (memory[attrib_index] >> 3) & 7;
		if (memory[attrib_index] & 0x40)
		{
			ink_colour += 8;
			paper_colour += 8;
		}
		WORD pixel_colour = colour_table[(data & 0x80) ? ink_colour : paper_colour];
		il9341_wr_data(pixel_colour >> 8);
		il9341_wr_data(pixel_colour & 0xff);
		//fputc((data & 0x80) ? '#' : '.', mem_log_file);
		data <<= 1;
	}
	//fputc('\r', mem_log_file);
	//fputc('\n', mem_log_file);
}
