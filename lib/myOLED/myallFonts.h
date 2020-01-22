/*
 * allFonts.h font header for GLCD library 
 * The fonts listed below will be available in a sketch if this file is included
 *
 * If you create your own fonts you can add the header to this file
 *
 * Note that the build environment only holds a font in Flash if its selected 
 * so there is no penalty to including a font file here if its not used 
 */

#ifndef _myallFonts_h_
#define _myallFonts_h_

#ifdef __AVR__
#include <avr/pgmspace.h>
/** declare a font for AVR. */
#define GLCDFONTDECL(_n) static const uint8_t __attribute__ ((progmem))_n[]
#define readFontByte(addr) pgm_read_byte(addr)
#else  // __AVR__
/** declare a font. */
#define GLCDFONTDECL(_n) static const uint8_t _n[]
/** Fake read from flash. */
#define readFontByte(addr) (*(const unsigned char *)(addr))
#endif  // __AVR__
//------------------------------------------------------------------------------
// Font Indices
#define FONT_LENGTH			  0
#define FONT_FIXED_WIDTH	2
#define FONT_HEIGHT			  3
#define FONT_FIRST_CHAR		4
#define FONT_CHAR_COUNT		5
#define FONT_WIDTH_TABLE	6
//
// FONT_LENGTH is a 16 bit Big Endian length field.
// Unfortunately, FontCreator2 screwed up the value it put in the field
// so it is pretty much meaningless. However it still is used to indicate
// some special things.
// 00 00 (fixed width font with 1 padding pixel on right and below)
// 00 01 (fixed width font with no padding pixels)

// any other value means variable width font in FontCreator2 (thiele)
// format with pixel padding

#include "Adafruit5x7.h"        // Font from Adafruit GFX library
#endif

