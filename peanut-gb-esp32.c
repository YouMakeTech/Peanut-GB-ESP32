/* Peanut-GB-ESP32: A Game Boy (DMG) emulator for the Espressif ESP32 
 * microcontroller based off the Peanut-GB library
 *
 * Copyright (c) 2022 Vincent Mistler (YouMakeTech)
 *
 * Peanut-GB-ESP32 is an implementation of Peanut-GB for the ESP32.
 * Peanut-GB is a Game Boy (DMG) emulator single header library written in C99. 
 * https://github.com/deltabeard/Peanut-GB
 * Peanut-GB is Copyright (c) 2018-2022 Mahyar Koshkouei
 * 
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Please note that at least three parts of source code within this project was
 * taken from the SameBoy project at https://github.com/LIJI32/SameBoy/ which at
 * the time of this writing is released under the MIT License. Occurrences of
 * this code is marked as being taken from SameBoy with a comment.
 * SameBoy, and code marked as being taken from SameBoy,
 * is Copyright (c) 2015-2019 Lior Halphon.
 */

#include <Arduino.h>
#include <SPI.h>

// Peanut-GB emulator settings
#define ENABLE_SOUND 0
#define ENABLE_LCD 1
#define PEANUT_GB_HIGH_LCD_ACCURACY 1
#define PEANUT_GB_USE_BIOS 0

// Project headers
#include "peanut_gb.h"
#include "rom.h"
#include "mk_ili9225.h"

// GPIO connections for buttons
#define GPIO_UP		0
#define GPIO_DOWN	2
#define GPIO_LEFT	19
#define GPIO_RIGHT	21
#define GPIO_A		22
#define GPIO_B		27
#define GPIO_SELECT	32
#define GPIO_START	33

// GPIO connections for ILI9225 LCD
#define GPIO_CS		15
#define GPIO_CLK	14
#define GPIO_SDI	13
#define GPIO_RS		25
#define GPIO_RST	26
#define GPIO_LED	-1  // -1 = directly connected to +3.3V

// GPIO connections for MAX98357A I2S Amplifier
#define GPIO_BCLK   18
#define GPIO_LRC    5
#define GPIO_DIN    23

// Global variables
static struct gb_s gb;      // Emulator context. Only values within the `direct` struct may be modified directly by the front-end implementation.
static uint8_t ram[32768];  // Cartridge RAM
static SPIClass *spi = NULL;       // Object for the SPI communication bus
static unsigned long start_time;   // start time used to compute Frames Per Second (FPS)
static unsigned long end_time;     // end time used to compute Frames Per Second (FPS)
static unsigned long frames;       // number of frames rendered since start_time (to compute FPS)
static const float target_period_us = 1000000 / VERTICAL_SYNC; // Target period for screen refresh of 59.7275 Hz
static float delta;                // delay that will draw the screen at a rate of 59.7275 Hz
static uint16_t fb[SCREEN_SIZE_Y]; // buffer to store pixels for the current line

struct gb_priv
{
    uint32_t lcd_line_hashes[LCD_HEIGHT];
    uint dma_pixel_buffer_chan;
};
static struct gb_priv gb_priv = { 0 };

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
	return rom[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
	return ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	ram[addr] = val;
}

/**
 * Notify front-end of error.
 */
void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t val)
{
    const char* gb_err_str[4] = {
			"UNKNOWN",
			"INVALID OPCODE",
			"INVALID READ",
			"INVALID WRITE"
		};
	Serial.printf("Error %d occurred: %s\n. Abort.\n",
		gb_err,
		gb_err >= GB_INVALID_MAX ?
		gb_err_str[0] : gb_err_str[gb_err]);
}


#if ENABLE_LCD

/* Functions required for communication with the ILI9225. */
void mk_ili9225_set_rst(bool state)
{
	digitalWrite(GPIO_RST, state);
}

void mk_ili9225_set_rs(bool state)
{
	digitalWrite(GPIO_RS, state);
}

void mk_ili9225_set_cs(bool state)
{
    digitalWrite(GPIO_CS, state);     
}

void mk_ili9225_set_led(bool state)
{
	if (GPIO_LED>0) digitalWrite(GPIO_LED, state);
}

void mk_ili9225_spi_write16(const uint16_t *halfwords, size_t len)
{
    spi->writePixels(halfwords,len*2);
}

void mk_ili9225_delay_ms(unsigned ms)
{
	delay(ms);
}

/**
 * @brief function called by Peanut-GB to draw a single line on the LCD
 * 
 * @param gb 
 * @param pixels 
 * @param line 
 */
void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
		   const uint_fast8_t line)
{
    const uint16_t palette[3][4] = {
		{ 0xFFFF, 0x651F, 0x001F, 0x0000 },
		{ 0xFFFF, 0xFC10, 0x89C7, 0x0000 },
		{ 0xFFFF, 0x651F, 0x001F, 0x0000 }
	};

	for(unsigned int x = 0; x < LCD_WIDTH; x++)
	{
		fb[x] = palette[(pixels[x] & LCD_PALETTE_ALL) >> 4]
				[pixels[x] & 3];
	}
    mk_ili9225_set_x((SCREEN_SIZE_X - 16) - line);
    mk_ili9225_write_pixels(fb, LCD_WIDTH);
}
#endif

/**
 * @brief Update button states, called by an interrupt when the state of any button has changed
 * 
 */
void onButtonUpChange()
{
    gb.direct.joypad_bits.up=digitalRead(GPIO_UP);
}

void onButtonDownChange()
{
    gb.direct.joypad_bits.down=digitalRead(GPIO_DOWN);
}

void onButtonLeftChange()
{
    gb.direct.joypad_bits.left=digitalRead(GPIO_LEFT);
}

void onButtonRightChange()
{
    gb.direct.joypad_bits.right=digitalRead(GPIO_RIGHT);
}

void onButtonAChange()
{
    gb.direct.joypad_bits.a=digitalRead(GPIO_A);
}

void onButtonBChange()
{
    gb.direct.joypad_bits.b=digitalRead(GPIO_B);
}

void onButtonSelectChange()
{
    gb.direct.joypad_bits.select=digitalRead(GPIO_SELECT);
}

void onButtonStartChange()
{
    gb.direct.joypad_bits.start=digitalRead(GPIO_START);
}

void setup()
{
    enum gb_init_error_e ret;

    // Initialise USB serial connection for debugging
    Serial.begin(115200);
    Serial.println("INIT: ");

    ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, &gb_priv);
    Serial.println("GB: ");
    if (ret != GB_INIT_NO_ERROR)
	{
        Serial.printf("gb_init returned an error: %d\n", ret);
    }
    
    // Initialise GPIO pins
    pinMode(GPIO_UP, INPUT_PULLUP);
    pinMode(GPIO_DOWN, INPUT_PULLUP);
    pinMode(GPIO_LEFT, INPUT_PULLUP);
    pinMode(GPIO_RIGHT, INPUT_PULLUP);
    pinMode(GPIO_A, INPUT_PULLUP);
    pinMode(GPIO_B, INPUT_PULLUP);
    pinMode(GPIO_SELECT, INPUT_PULLUP);
    pinMode(GPIO_START, INPUT_PULLUP);
    pinMode(GPIO_CS, OUTPUT);
	pinMode(GPIO_RS, OUTPUT);
	pinMode(GPIO_RST, OUTPUT);
	if (GPIO_LED>0) pinMode(GPIO_LED, OUTPUT);
    pinMode(GPIO_BCLK, OUTPUT);
    pinMode(GPIO_LRC, OUTPUT);
    pinMode(GPIO_DIN, OUTPUT);

    // Attach an interrupt to update the button status when the state of any button has changed
    attachInterrupt(GPIO_UP, onButtonUpChange, CHANGE);
    attachInterrupt(GPIO_DOWN, onButtonDownChange, CHANGE);
    attachInterrupt(GPIO_LEFT, onButtonLeftChange, CHANGE);
    attachInterrupt(GPIO_RIGHT, onButtonRightChange, CHANGE);
    attachInterrupt(GPIO_A, onButtonAChange, CHANGE);
    attachInterrupt(GPIO_B, onButtonBChange, CHANGE);
    attachInterrupt(GPIO_SELECT, onButtonSelectChange, CHANGE);
    attachInterrupt(GPIO_START, onButtonStartChange, CHANGE);

#if ENABLE_LCD
    // Initialize SPI bus for communication with LCD
    spi = new SPIClass(HSPI);
    spi->begin(GPIO_CLK, -1, GPIO_SDI, GPIO_CS); //SCLK, MISO, MOSI, CS
    spi->beginTransaction(SPISettings(40*1000*1000, MSBFIRST, SPI_MODE0));  // 40MHz, Clock Polarity=0, Clock Phase=0
    
    gb_init_lcd(&gb, &lcd_draw_line);
    mk_ili9225_init();

    // Clear LCD screen
    for(uint8_t y=0;y<SCREEN_SIZE_Y;y++) {
        fb[y]=0x0000; // background color RGB565 (e.g. black = 0x0000, red=0xf800)
    }
    mk_ili9225_set_window(0,SCREEN_SIZE_X-1,0,SCREEN_SIZE_Y-1);
    for(uint8_t x=0;x<SCREEN_SIZE_X;x++) {
        mk_ili9225_set_address(x,0);
        mk_ili9225_write_pixels(fb, SCREEN_SIZE_Y);
    }
    
    // Set LCD window to DMG size.
    mk_ili9225_set_window(16, LCD_HEIGHT + 15, 31, LCD_WIDTH + 30);
    Serial.println("LCD: ");

    // Speed improvements options
//    gb.direct.interlace=true;
    gb.direct.frame_skip=true;
#endif

#if ENABLE_SOUND
    audio_init();
    Serial.println("AUDIO: ");
#endif

    // Initialize variables for Frames Per Second computation
    start_time=micros();
    delta=0;
    frames=0;
}

void loop()
{
    // Execute CPU cycles for one frame
    // Ideally, gb_run_frame is called at 59.7275 Hz
    gb_run_frame(&gb);
    frames++;

    // Use a delay that will draw the screen at a rate of 59.7275 Hz */
    delayMicroseconds((uint32_t)delta);

    if(frames>60)
    {
        // Compute the delay delta that will draw the screen at a rate of 59.7275 Hz */
        end_time=micros();
        delta += (target_period_us - (float)(end_time - start_time)/frames);
        if(delta<1) {
            delta=1;
        }
        
        // Display Frames Per Second
        Serial.printf("FPS=%d delta=%.0f us\n",(1000*1000*frames)/(end_time-start_time),delta);
        
        frames=0;
        start_time=end_time;
    }
}