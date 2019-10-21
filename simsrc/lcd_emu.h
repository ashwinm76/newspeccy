#ifndef _LCD_EMU_H_
#define _LCD_EMU_H_

void fbinit();
void fbwr(WORD addr, BYTE data);
void fb_set_border(BYTE colour);

#endif
