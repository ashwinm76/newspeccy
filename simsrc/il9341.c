
#include "SDL.h"
#include "sim.h"

int data_count;
BYTE current_cmd;
SDL_Surface *framebuffer;
SDL_Surface *display_surface;
SDL_Window *display_window;
SDL_Renderer *display_renderer;
unsigned long fb_x, fb_y;
unsigned long fb_window_start_x;
unsigned long fb_window_end_x;
unsigned long fb_window_start_y;
unsigned long fb_window_end_y;
int initialised = 0;

void il9341_init()
{
  if (initialised)
  {
    return;
  }

  SDL_LogSetPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO);
	
  if(SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_Init fail : %s\n", SDL_GetError());
    exit(1);
	}

  framebuffer = SDL_CreateRGBSurfaceWithFormat(0, 320, 240, 16, SDL_PIXELFORMAT_RGB565);
  if (framebuffer == NULL)
  {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_CreateRGBSurfaceWithFormat fail : %s\n", SDL_GetError());
    exit(1);
  }

	if(SDL_CreateWindowAndRenderer(320, 240, SDL_WINDOW_SHOWN, &display_window, &display_renderer))
	{
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Window creation fail : %s\n",SDL_GetError());
		exit(1);
	}

	display_surface = SDL_GetWindowSurface(display_window);

	SDL_SetRenderDrawColor(display_renderer, 0xFF, 0xFF, 0xFF, 0xFF);
	SDL_RenderClear(display_renderer);

	SDL_UpdateWindowSurface(display_window);

  initialised = 1;
}

void il9341_wr_cmd(BYTE cmd)
{
  current_cmd = cmd;

  switch(cmd)
  {
    case 0x01: // soft reset
    case 0x11: // sleep out
    case 0x28: // display off
    case 0x29: // display on
      data_count = 0; break;

    case 0xc0: // power control 1
    case 0xc1: // power control 2
    case 0xc7: // vcom control 1
    case 0x36: // memory access control
    case 0x3a: // COLMODE pixel format set
    case 0xb7: // entry mode set
      data_count = 1; break;

    case 0xc5: // vcom control 1
    case 0xb1: // frame rate control
      data_count = 2; break;

    case 0x2a: // column address set
    case 0x2b: // page address set
      data_count = 4; break;

    case 0x2c: // memory write
      fb_x = fb_window_start_x;
      fb_y = fb_window_start_y;
      data_count = -1; break;

    default: break;
  }
}

void il9341_wr_data(BYTE data)
{
  static int wr_count = 0;

  switch(current_cmd)
  {
    case 0x2a: // column address set
      if (data_count > 2)
      {
        fb_window_start_x <<= 8;
        fb_window_start_x |= data;
        fb_window_start_x &= 0xffff;
      }
      else
      {
        fb_window_end_x <<= 8;
        fb_window_end_x |= data;
        fb_window_end_x &= 0xffff;
      }
      data_count--;
      break;

    case 0x2b: // page address set
      if (data_count > 2)
      {
        fb_window_start_y <<= 8;
        fb_window_start_y |= data;
        fb_window_start_y &= 0xffff;
      }
      else
      {
        fb_window_end_y <<= 8;
        fb_window_end_y |= data;
        fb_window_end_y &= 0xffff;
      }
      data_count--;
      break;

    case 0x2c: // memory write
      {
        if (fb_x > fb_window_end_x)
        {
          fb_x = fb_window_start_x;
          fb_y++;
        }
        if (fb_y > fb_window_end_y)
        {
          return;
        }
        int index = 2*(fb_y * 320 + fb_x) + wr_count;
        // Convert little to big endian if required.
        if (SDL_BYTEORDER == SDL_LIL_ENDIAN)
        {
          index ^= 1;
        }
        ((unsigned char*)framebuffer->pixels)[index] = data;
        wr_count++;
        if (wr_count == 2)
        {
          fb_x++;
          wr_count = 0;
        }
      }
      break;

    default: break;
  }
}

BYTE il9341_rd_data()
{
  return 0;
}

void il9341_update()
{
  SDL_BlitSurface(framebuffer, NULL, display_surface, NULL);
  SDL_UpdateWindowSurface(display_window);
}

void il9341_set_window(int startx, int endx, int starty, int endy)
{
  startx &= 0xffff;
  endx &= 0xffff;
  starty &= 0xffff;
  endy &= 0xffff;

  il9341_wr_cmd(0x2a);
  il9341_wr_data(startx >> 8);
  il9341_wr_data(startx & 0xff);
  il9341_wr_data(endx >> 8);
  il9341_wr_data(endx & 0xff);

  il9341_wr_cmd(0x2b);
  il9341_wr_data(starty >> 8);
  il9341_wr_data(starty & 0xff);
  il9341_wr_data(endy >> 8);
  il9341_wr_data(endy & 0xff);
}
