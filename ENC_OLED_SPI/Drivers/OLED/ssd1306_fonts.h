#include <stdint.h>

#ifndef __SSD1306_FONTS_H__
#define __SSD1306_FONTS_H__

#include "ssd1306_conf.h"
#include "stm32h7xx_hal.h"
#include "string.h"

//typedef struct {
//	const uint8_t FontWidth;    /*!< Font width in pixels */
//	uint8_t FontHeight;   /*!< Font height in pixels */
//	const uint16_t *data; /*!< Pointer to data font data array */
//} FontDef;

typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	uint8_t CharBytes;    /*!< Count of bytes for one character */
	const uint8_t *data; /*!< Pointer to data font data array */
} FontDef_t;

typedef struct {
	uint16_t Length;      /*!< String length in units of pixels */
	uint16_t Height;      /*!< String height in units of pixels */
} FONTS_SIZE_t;



//extern FontDef Font_6x8;
#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef_t Font_7x10;
#endif
//extern FontDef Font_11x18;

//extern FontDef Font_16x26;
#ifdef SSD1306_INCLUDE_FONT_24x32
extern FontDef_t Font_24x32;
#endif

char* FONTS_GetStringSize(char* str, FONTS_SIZE_t* SizeStruct, FontDef_t* Font);

#endif // __SSD1306_FONTS_H__
