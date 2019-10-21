#ifndef __IL9341_H__
#define __IL9341_H__

void il9341_init();
void il9341_wr_cmd(BYTE cmd);
void il9341_wr_data(BYTE data);
BYTE il9341_rd_data();
void il9341_update();
void il9341_set_window(int startx, int endx, int starty, int endy);

#endif
