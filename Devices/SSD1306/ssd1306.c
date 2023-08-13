/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>
 * modification for STM32f10x: Alexandr Pochtovy<alex.mail.prime@gmail.com>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015
	Copyright (C) Alexandr Pochtovy, 2022

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#include "ssd1306.h"


//SSD1306 command buffer
static uint8_t SSD1306_CommandBuffer[64] =
{				0xAE,			//1		display off
	0x80, 	0xD5, 	//2		set display clock divide ratio/
	0x80, 	0x80,	//3		oscillator frequency
	0x80, 	0xA8,	//4		set multiplex ratio(1 to 64)
	0x80, 	0x1F, 	//5		1F = 128x31 (3F = 128x64)
	0x80, 	0xD3, 	//6		set display offset
	0x80, 	0x00, 	//7		not offset
	0x80, 	0x40,	//8		set display start line address
	0x80, 	0x8D, 	//9		set DC-DC enable
	0x80, 	0x14, 	//10	DC-DC internal
	0x80, 	0x20, 	//11	Set Memory Addressing Mode
	0x80, 	0x00, 	//12	00,Horizontal Addressing Mode,01,Vertical Addressing Mode,10,Page Addressing Mode (RESET),11,Invalid
	0x80, 	0xA1, 	//13	set segment re-map 0 to 127
	0x80, 	0xC8, 	//14	Set COM Output Scan Direction
	0x80, 	0xDA, 	//15	set com pins hardware configuration
	0x80, 	0x02, 	//16	02 - 128x32 12 - 128x64
	0x80, 	0x81, 	//17	set contrast control register
	0x80, 	0x7F, 	//18	contrast 0x8F..0xCF
	0x80, 	0xD9, 	//19	set pre-charge period
	0x80, 	0xF1, 	//20	internal DC-DC
	0x80, 	0xDB, 	//21	set vcomh
	0x80, 	0x40, 	//22	0x00 - 0x70
	0x80, 	0xA4, 	//23	0xa4 Output follows RAM | 0xa5 Output ignores RAM
	0x80, 	0xA6, 	//24		no inverse | 0xA8 inverse
	0x80, 	0xAF, 	//25		turn on SSD1306 panel
	0x80, 	0x22,	//26
	0x80, 	0x00,	//27
	0x80, 	0xFF,	//28
	0x80, 	0x21,	//29
	0x80, 	0x00,	//30
	0x80, 	0x7F		//31
};

static uint8_t SSD1306_CommandB1[32] =
{	0xAE,	//1		display off
	0xD5, 	//2		set display clock divide ratio/
	0x80,	//3		oscillator frequency
	0xA8,	//4		set multiplex ratio(1 to 64)
	0x3F, 	//5		1F = 128x31 (3F = 128x64)
	0xD3, 	//6		set display offset
	0x00, 	//7		not offset
	0x40,	//8		set display start line address
	0x8D, 	//9		set DC-DC enable
	0x14, 	//10	DC-DC internal
	0x20, 	//11	Set Memory Addressing Mode
	0x00, 	//12	00,Horizontal Addressing Mode,01,Vertical Addressing Mode,10,Page Addressing Mode (RESET),11,Invalid
	0xA1, 	//13	set segment re-map 0 to 127
	0xC8, 	//14	Set COM Output Scan Direction
	0xDA, 	//15	set com pins hardware configuration
	0x12, 	//16	02 - 128x32 12 - 128x64
	0x81, 	//17	set contrast control register
	0x8F, 	//18	contrast 0x8F..0xCF
	0xD9, 	//19	set pre-charge period
	0xF1, 	//20	internal DC-DC
	0xDB, 	//21	set vcomh
	0x40, 	//22	0x00 - 0x70
	0xA4, 	//23	0xa4 Output follows RAM | 0xa5 Output ignores RAM
	0xA6, 	//24		no inverse | 0xA8 inverse
	0x2E,	//25
	0xAF, 	//26		turn on SSD1306 panel
	0x22,	//27
	0x00,	//28
	0xFF,	//29
	0x21,	//30
	0x00,	//31
	0x7F		//32
};
//  SSD1306 single draw command buffer, 0xB0-1
static uint8_t SSD1306_CommandDraw[5] = {0xB0, 0x80, 0x00, 0x80, 0x10};

// SSD1306 data buffer
static uint8_t SSD1306_FrameBuffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8] = {0};

OLED_type OLED;
/* Private variable */

//================================================================================================
OLED_Connect_Status SSD1306_Init(I2C_Connection *_i2c) {
	if (_i2c->i2cStatus == I2C_Bus_Free) {//send SSD1306_oled initial string commands
		_i2c->addr = SSD1306_I2C_ADDR;
		_i2c->mode = I2C_MODE_WRITE;
		switch (OLED.step) {
		case 0://send 4
			SSD1306_Fill(SSD1306_COLOR_BLACK);// Clear screen buffer
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 4;
			_i2c->rxtxp = &SSD1306_CommandB1[0];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 1://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[4];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 2://send 4
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 4;
			_i2c->rxtxp = &SSD1306_CommandB1[5];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 3://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[9];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 4://send 4
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 4;
			_i2c->rxtxp = &SSD1306_CommandB1[10];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 5://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[14];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 6://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[15];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 7://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[16];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 8://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[17];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 9://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[18];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 10://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[19];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 11://send 6
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 6;
			_i2c->rxtxp = &SSD1306_CommandB1[20];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 12://send 5
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 5;
			_i2c->rxtxp = &SSD1306_CommandB1[26];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;
		case 13://send 1
			_i2c->reg = 0x00;//_i2c->reg = 0x80;
			_i2c->len = 1;
			_i2c->rxtxp = &SSD1306_CommandB1[31];//SSD1306_CommandBuffer 1;
			OLED.step++;
			break;

		case 14:
			if (SSD1306_CommandDraw[0] < (0xB0 + SSD1306_HEIGHT / 8) ) {
				_i2c->reg = 0x80;
				_i2c->len = 5;
				_i2c->rxtxp = &SSD1306_CommandDraw[0];
				OLED.step = 15;
			} else {
				SSD1306_CommandDraw[0] = 0xB0;
				OLED.step = 16;
			}
			break;

		case 15:
			_i2c->reg = 0x40;
			_i2c->len = SSD1306_WIDTH;
			_i2c->rxtxp = &SSD1306_FrameBuffer[SSD1306_WIDTH * (SSD1306_CommandDraw[0] - 0xB0)];
			SSD1306_CommandDraw[0]++;
			OLED.step = 14;
			break;

		case 16:
			OLED.SSD1306.CurrentX = 0;// Set default values
			OLED.SSD1306.CurrentY = 0;
			OLED.SSD1306.Initialized = 1;// Initialized OK
			OLED.SSD1306.Inverted = 0;
			OLED.SSD1306.DisplayOn = 1;
			OLED.status = OLED_OK;
			OLED.step = 0;
			return OLED_Complite;
			break;

		default:
			OLED.step = 0;
			break;
		}
		//I2C_Start_IRQ(_i2c);
		I2C_Start_DMA(_i2c);
	}
	return OLED_Processing;
}

OLED_Connect_Status SSD1306_UpdateScreen(I2C_Connection *_i2c) {
	if (_i2c->i2cStatus == I2C_Bus_Free) {
		_i2c->addr = SSD1306_I2C_ADDR;
		_i2c->mode = I2C_MODE_WRITE;
		switch (OLED.step) {
		case 0:
			if (SSD1306_CommandDraw[0] < (0xB0 + SSD1306_HEIGHT / 8) ) {
				_i2c->reg = 0x80;
				_i2c->len = 5;
				_i2c->rxtxp = &SSD1306_CommandDraw[0];
				OLED.step = 1;
			} else {
				SSD1306_CommandDraw[0] = 0xB0;
				OLED.step = 0;
				return OLED_Complite;
			}
			break;

		case 1:
			_i2c->reg = 0x40;
			_i2c->len = SSD1306_WIDTH;
			_i2c->rxtxp = &SSD1306_FrameBuffer[SSD1306_WIDTH * (SSD1306_CommandDraw[0] - 0xB0)];
			SSD1306_CommandDraw[0]++;
			OLED.step = 0;
			break;

		default:
			OLED.step = 0;
			break;
		}
		//I2C_Start_IRQ(_i2c);
		I2C_Start_DMA(_i2c);
	}
	return OLED_Processing;
}

void SSD1306_GotoXY(uint8_t x, uint8_t y) {
	/* Set write pointers */
	OLED.SSD1306.CurrentX = x;
	OLED.SSD1306.CurrentY = y;
}

void SSD1306_ToggleInvert(void) {
	/* Toggle invert */
	OLED.SSD1306.Inverted = !OLED.SSD1306.Inverted;
		/* Do memory toggle */
	for (uint16_t i = 0; i < sizeof(SSD1306_FrameBuffer); i++) {
		SSD1306_FrameBuffer[i] = ~SSD1306_FrameBuffer[i];
	}
}

void SSD1306_Fill(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_FrameBuffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_FrameBuffer));
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR_t color) {
	if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT)) {
		return;// do nothing
	}
	/* Check if pixels are inverted */
	if (OLED.SSD1306.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}
	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_FrameBuffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_FrameBuffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

char SSD1306_WriteChar(char ch, FontDef_t Font, SSD1306_COLOR_t color) {
	uint32_t i;
	uint32_t b;
	uint32_t j;
	
	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (OLED.SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (OLED.SSD1306.CurrentY + Font.FontHeight)
	) {
		/* Error */
		return 0;
	}
	
	/* Go through font */
	for (i = 0; i < Font.FontHeight; i++) {
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel(OLED.SSD1306.CurrentX + j, (OLED.SSD1306.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				SSD1306_DrawPixel(OLED.SSD1306.CurrentX + j, (OLED.SSD1306.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}
	
	/* Increase pointer */
	OLED.SSD1306.CurrentX += Font.FontWidth;
	
	/* Return character written */
	return ch;
}

char SSD1306_WriteString(char* str, FontDef_t Font, SSD1306_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_WriteChar(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}
		
		/* Increase string pointer */
		str++;
	}
	
	/* Everything OK, zero should be returned */
	return *str;
}

// Draw line by Bresenhem's algorithm
void SSD1306_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR_t color) {
	int32_t dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	int32_t dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	int32_t sx = (x0 < x1) ? 1 : -1;
	int32_t sy = (y0 < y1) ? 1 : -1;
	int32_t err = ((dx > dy) ? dx : -dy) / 2;
	int32_t e2;
	int32_t i;
	int32_t tmp;
	
	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}
		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}
		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			SSD1306_DrawPixel(x0, i, color);
		}
		/* Return from function */
		return;
	}
	else if (dy == 0) {
		if (y1 < y0) {
				tmp = y1;
				y1 = y0;
				y0 = tmp;
			}
		if (x1 < x0) {
				tmp = x1;
				x1 = x0;
				x0 = tmp;
			}
			/* Horizontal line */
			for (i = x0; i <= x1; i++) {
				SSD1306_DrawPixel(i, y0, color);
			}
			/* Return from function */
			return;
	}
	else {
		do {
			SSD1306_DrawPixel(x0, y0, color);
			e2 = err;
			if (e2 > -dx) {
				err -= dy;
				x0 += sx;
			}
			if (e2 < dy) {
				err += dx;
				y0 += sy;
			}
		} while ((x0 != x1) || (y0 != y1));
	}
}

/*Convert Degrees to Radians*/
float SSD1306_DegToRad(float par_deg) {
    return par_deg * 3.14 / 180.0;
}

/*Normalize degree to [0;360]*/
uint16_t ssd1306_NormalizeTo0_360(uint16_t par_deg) {
  uint16_t loc_angle;
  if(par_deg <= 360)
  {
    loc_angle = par_deg;
  }
  else
  {
    loc_angle = par_deg % 360;
    loc_angle = ((par_deg != 0)?par_deg:360);
  }
  return loc_angle;
}

void SSD1306_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR_t color) {
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}
	
	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}
	
	/* Draw 4 lines */
	SSD1306_DrawLine(x, y, x + w, y, color);         /* Top line */
	SSD1306_DrawLine(x, y + h, x + w, y + h, color); /* Bottom line */
	SSD1306_DrawLine(x, y, x, y + h, color);         /* Left line */
	SSD1306_DrawLine(x + w, y, x + w, y + h, color); /* Right line */
}

void SSD1306_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR_t color) {
	uint8_t i;
	
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}
	
	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}
	
	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		SSD1306_DrawLine(x, y + i, x + w, y + i, color);
	}
}

void SSD1306_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, SSD1306_COLOR_t color) {
	/* Draw lines */
	SSD1306_DrawLine(x1, y1, x2, y2, color);
	SSD1306_DrawLine(x2, y2, x3, y3, color);
	SSD1306_DrawLine(x3, y3, x1, y1, color);
}

void SSD1306_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, SSD1306_COLOR_t color) {
	int16_t deltax = 0;
	int16_t deltay = 0;
	int16_t x = 0;
	int16_t y = 0;
	int16_t xinc1 = 0;
	int16_t xinc2 = 0;
	int16_t	yinc1 = 0;
	int16_t yinc2 = 0;
	int16_t den = 0;
	int16_t num = 0;
	int16_t numadd = 0;
	int16_t numpixels = 0;
	int16_t	curpixel = 0;
	
	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		SSD1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawPixel(x0 + x, y0 + y, c);
        SSD1306_DrawPixel(x0 - x, y0 + y, c);
        SSD1306_DrawPixel(x0 + x, y0 - y, c);
        SSD1306_DrawPixel(x0 - x, y0 - y, c);

        SSD1306_DrawPixel(x0 + y, y0 + x, c);
        SSD1306_DrawPixel(x0 - y, y0 + x, c);
        SSD1306_DrawPixel(x0 + y, y0 - x, c);
        SSD1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

void SSD1306_DrawFilledCircle(uint8_t x0, uint8_t y0, uint8_t r, SSD1306_COLOR_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, color);
    SSD1306_DrawPixel(x0, y0 - r, color);
    SSD1306_DrawPixel(x0 + r, y0, color);
    SSD1306_DrawPixel(x0 - r, y0, color);
    SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

        SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
        SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
    }
}

void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR_t color) {
    const uint8_t CIRCLE_APPROXIMATION_SEGMENTS 36
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1;
    uint8_t xp2;
    uint8_t yp1;
    uint8_t yp2;
    uint32_t count = 0;
    uint32_t loc_sweep = 0;
    float rad;

    loc_sweep = ssd1306_NormalizeTo0_360(sweep);

    count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;
    while(count < approx_segments)
    {
        rad = SSD1306_DegToRad(count*approx_degree);
        xp1 = x + (int8_t)(sin(rad)*radius);
        yp1 = y + (int8_t)(cos(rad)*radius);
        count++;
        if(count != approx_segments)
        {
            rad = SSD1306_DegToRad(count*approx_degree);
        }
        else
        {
            rad = SSD1306_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t)(sin(rad)*radius);
        yp2 = y + (int8_t)(cos(rad)*radius);
        SSD1306_DrawLine(xp1,yp1,xp2,yp2,color);
    }
    return;
}

void SSD1306_ON(I2C_Connection *_i2c) {
	SSD1306_CommandBuffer[0] = 0x8D;
	SSD1306_CommandBuffer[1] = 0x80;
	SSD1306_CommandBuffer[2] = 0x14;
	SSD1306_CommandBuffer[3] = 0x80;
	SSD1306_CommandBuffer[4] = 0xAF;
	_i2c->addr = SSD1306_I2C_ADDR;
	_i2c->len = 5;
	_i2c->reg = 0x80;
	_i2c->mode = I2C_MODE_WRITE;
	_i2c->rxtxp = &SSD1306_CommandBuffer[0];
	I2C_Start_DMA(_i2c);
}

void SSD1306_OFF(I2C_Connection *_i2c) {
	SSD1306_CommandBuffer[0] = 0x8D;
	SSD1306_CommandBuffer[1] = 0x80;
	SSD1306_CommandBuffer[2] = 0x10;
	SSD1306_CommandBuffer[3] = 0x80;
	SSD1306_CommandBuffer[4] = 0xAE;
	_i2c->addr = SSD1306_I2C_ADDR;
	_i2c->len = 5;
	_i2c->reg = 0x80;
	_i2c->mode = I2C_MODE_WRITE;
	_i2c->rxtxp = &SSD1306_CommandBuffer[0];
	I2C_Start_DMA(_i2c);
}
