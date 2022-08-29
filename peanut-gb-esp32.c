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
 */

#include <stdio.h>
#include "driver/i2s.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Peanut-GB emulator settings
#define ENABLE_SOUND 1
#define ENABLE_LCD 1
#define PEANUT_GB_HIGH_LCD_ACCURACY 1
#define PEANUT_GB_USE_BIOS 0

// Project headers
#if ENABLE_SOUND
    #include "minigb_apu.h"
#endif
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
// static unsigned long start_time;   // start time used to compute Frames Per Second (FPS)
// static unsigned long end_time;     // end time used to compute Frames Per Second (FPS)
static unsigned long frames;       // number of frames rendered since start_time (to compute FPS)
// static const float target_period_us = 1000000 / VERTICAL_SYNC; // Target period for screen refresh of 59.7275 Hz
// static float delta;                // delay that will draw the screen at a rate of 59.7275 Hz
static uint16_t fb[SCREEN_SIZE_Y]; // buffer to store pixels for the current line

#if ENABLE_SOUND
    // stream contains 549 audio samples
    // each sample consits of a 2x16 bits words 
    // (16 bits for the left channel + 16 bits for the right channel in stereo interleaved format)
    // This is intended to be played at 32768 Hz
    uint8_t stream[AUDIO_SAMPLES*4];
#endif

#if ENABLE_LCD
    static spi_device_handle_t spi_lcd;
#endif

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
	printf("Error %d occurred: %s\n. Abort.\n",
		gb_err,
		gb_err >= GB_INVALID_MAX ?
		gb_err_str[0] : gb_err_str[gb_err]);
}


#if ENABLE_LCD

/* Functions required for communication with the ILI9225. */
void mk_ili9225_set_rst(bool state)
{
    gpio_set_level(GPIO_RST, state);
}

void mk_ili9225_set_rs(bool state)
{
    gpio_set_level(GPIO_RS, state);
}

void mk_ili9225_set_cs(bool state)
{
    gpio_set_level(GPIO_CS, state);
}

void mk_ili9225_set_led(bool state)
{
    if (GPIO_LED>0) gpio_set_level(GPIO_LED, state);
}

void mk_ili9225_spi_write16(uint16_t *halfwords, size_t len)
{
    esp_err_t ret;
    spi_transaction_t t;

    // swap lsb & msb in halfwords
    uint16_t w;
    uint16_t msb;
    uint16_t lsb;
    for(size_t i=0;i<len;i++) {
        w=halfwords[i];
//        printf("%X\n",w);
        msb=(w & 0xFF00) >> 8;
        lsb=(w & 0x00FF) << 8;
        halfwords[i]=msb + lsb;
    }

    memset(&t, 0, sizeof(t));       // Zero out the transaction
    t.length=len*16;                // transaction length in bits.
    t.tx_buffer=halfwords;
    ret=spi_device_polling_transmit(spi_lcd, &t);  // Transmit!
    assert(ret==ESP_OK);            // Should have had no issues.
}

void mk_ili9225_delay_ms(unsigned ms)
{
	vTaskDelay(ms / portTICK_PERIOD_MS);
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
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    switch(gpio_num) {
        case GPIO_UP:
            gb.direct.joypad_bits.up=gpio_get_level(GPIO_UP);
            break;
        case GPIO_DOWN:
            gb.direct.joypad_bits.down=gpio_get_level(GPIO_DOWN);
            break;
        case GPIO_LEFT:
            gb.direct.joypad_bits.left=gpio_get_level(GPIO_LEFT);
            break;
        case GPIO_RIGHT:
            gb.direct.joypad_bits.right=gpio_get_level(GPIO_RIGHT);
            break;
        case GPIO_A:
            gb.direct.joypad_bits.a=gpio_get_level(GPIO_A);
            break;
        case GPIO_B:
            gb.direct.joypad_bits.b=gpio_get_level(GPIO_B);
            break;
        case GPIO_SELECT:
            gb.direct.joypad_bits.select=gpio_get_level(GPIO_SELECT);
            break;
        case GPIO_START:
            gb.direct.joypad_bits.start=gpio_get_level(GPIO_START);
            break;
    }
}


void app_main()
{
    enum gb_init_error_e gb_ret;
    printf("INIT: \n");

    // Initialise GameBoy emulator
    gb_ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, NULL);
    printf("GB: \n");
    switch(gb_ret)
    {
        case GB_INIT_NO_ERROR:
            break;
        case GB_INIT_CARTRIDGE_UNSUPPORTED:
            printf("gb_init returned an error: Unsupported cartridge.\n");
            break;
        case GB_INIT_INVALID_CHECKSUM:
            printf("gb_init returned an error: Checksum failure.\n");
            break;
        default:
             printf("gb_init returned an unknown error: %d\n", gb_ret);
    }

    // Initialise GPIO pins for the buttons
    gpio_reset_pin(GPIO_UP);
    gpio_reset_pin(GPIO_DOWN);
    gpio_reset_pin(GPIO_LEFT);
    gpio_reset_pin(GPIO_RIGHT);
    gpio_reset_pin(GPIO_A);
    gpio_reset_pin(GPIO_B);
    gpio_reset_pin(GPIO_SELECT);
    gpio_reset_pin(GPIO_START);

    gpio_set_direction(GPIO_UP, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_DOWN, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_LEFT, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_RIGHT, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_A, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_B, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_SELECT, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_START, GPIO_MODE_INPUT);
    
    gpio_pullup_en(GPIO_UP);
    gpio_pullup_en(GPIO_DOWN);
    gpio_pullup_en(GPIO_LEFT);
    gpio_pullup_en(GPIO_RIGHT);
    gpio_pullup_en(GPIO_A);
    gpio_pullup_en(GPIO_B);
    gpio_pullup_en(GPIO_SELECT);
    gpio_pullup_en(GPIO_START);

    // Attach an interrupt to update the button status when the state of any button has changed
    gpio_set_intr_type(GPIO_UP, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_DOWN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_LEFT, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_RIGHT, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_A, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_B, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_SELECT, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_START, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_UP, gpio_isr_handler, (void*) GPIO_UP);
    gpio_isr_handler_add(GPIO_DOWN, gpio_isr_handler, (void*) GPIO_DOWN);
    gpio_isr_handler_add(GPIO_LEFT, gpio_isr_handler, (void*) GPIO_LEFT);
    gpio_isr_handler_add(GPIO_RIGHT, gpio_isr_handler, (void*) GPIO_RIGHT);
    gpio_isr_handler_add(GPIO_A, gpio_isr_handler, (void*) GPIO_A);
    gpio_isr_handler_add(GPIO_B, gpio_isr_handler, (void*) GPIO_B);
    gpio_isr_handler_add(GPIO_SELECT, gpio_isr_handler, (void*) GPIO_SELECT);
    gpio_isr_handler_add(GPIO_START, gpio_isr_handler, (void*) GPIO_START);

#if ENABLE_LCD
    //Initialize non-SPI GPIOs for the ILI9225 display
    gpio_set_direction(GPIO_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_RS, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_RST, GPIO_MODE_OUTPUT);
    if (GPIO_LED>0) gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    // Initialize the SPI bus for communication with the LCD / SD Card reader
    spi_bus_config_t spi_bus_config = {
        .miso_io_num=-1,                    // Not used
        .mosi_io_num=GPIO_SDI,
        .sclk_io_num=GPIO_CLK,
        .quadwp_io_num=-1,                  // Not used
        .quadhd_io_num=-1,                  // Not used
        .max_transfer_sz=SCREEN_SIZE_Y*2    // Maximum transfer size, in bytes
    };
    spi_bus_initialize(HSPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);

    // Attach the LCD to the SPI bus
    spi_device_interface_config_t spi_lcd_interface_config = {
        .clock_speed_hz=40*1000*1000,       // 40 Mhz
        .mode=0,                            // Clock Polarity=0, Clock Phase=0
        .spics_io_num=GPIO_CS,              // CS pin
        .queue_size=7,                      // We want to be able to queue 7 transactions at a time
    };
    spi_bus_add_device(HSPI_HOST, &spi_lcd_interface_config, &spi_lcd);

    // Initialize the LCD    
    mk_ili9225_init();
    printf("LCD: mk_ili9225_init() complete\n");

    // Clear LCD screen
    for(uint8_t y=0;y<SCREEN_SIZE_Y;y++) {
        fb[y]=0xf800; // background color RGB565 (e.g. black = 0x0000, red=0xf800)
    }
    mk_ili9225_set_window(0,SCREEN_SIZE_X-1,0,SCREEN_SIZE_Y-1);
    for(uint8_t x=0;x<SCREEN_SIZE_X;x++) {
        mk_ili9225_set_address(x,0);
        mk_ili9225_write_pixels(fb, SCREEN_SIZE_Y);
    }
    
    // Set LCD window to DMG size.
    mk_ili9225_set_window(16, LCD_HEIGHT + 15, 31, LCD_WIDTH + 30);
    gb_init_lcd(&gb, &lcd_draw_line);
    printf("LCD: \n");

    // Speed improvements options
//    gb.direct.interlace=true;
    gb.direct.frame_skip=true;
#endif

#if ENABLE_SOUND
    // Initialize I2S sound driver
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 9,     // buffer size = 9x61 samples = 549 samples. Each samples is 32 bits (16 bits right channel + 16 bits left channel)
        .dma_buf_len = 61,
        .use_apll = true,       // I2S using APLL as main I2S clock, enable it to get accurate clock */
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = GPIO_BCLK,
        .ws_io_num = GPIO_LRC,
        .data_out_num = GPIO_DIN,
        .data_in_num = -1 // Not used
    };
    i2s_driver_install(0, &i2s_config, 0, NULL);
    i2s_set_pin(0, &pin_config);

    // Initialize audio emulation
    audio_init();
    printf("AUDIO: \n");
#endif

    // Initialize variables for Frames Per Second computation
    // start_time=micros();
    // delta=0;
    // frames=0;

    while(true)
    {
        // Execute CPU cycles for one frame
        // Ideally, gb_run_frame is called at 59.7275 Hz
        gb_run_frame(&gb);
        frames++;

#if ENABLE_SOUND
        // stream contains 549 audio samples
        // each sample consits of a 2x16 bits words 
        // (16 bits for the left channel + 16 bits for the right channel in stereo interleaved format)
        // This is intended to be played at 32768 Hz
	    audio_callback(NULL, stream, AUDIO_SAMPLES*4);
        size_t i2s_bytes_write = 0;
        i2s_write(0, stream, AUDIO_SAMPLES*4, &i2s_bytes_write, 10000);
        //printf("i2s_bytes_write = %d\n", i2s_bytes_write);
#endif    

        // Use a delay that will draw the screen at a rate of 59.7275 Hz */
        //delayMicroseconds((uint32_t)delta);

        // if(frames>60)
        // {
        //     // Compute the delay delta that will draw the screen at a rate of 59.7275 Hz */
        //     end_time=micros();
        //     // delta += (target_period_us - (float)(end_time - start_time)/frames);
        //     // if(delta<1) {
        //     //     delta=1;
        //     // }
        
        //     // Display Frames Per Second
        //     printf("FPS=%d delta=%.0f us\n",(1000*1000*frames)/(end_time-start_time),delta);
        
        //     frames=0;
        //     start_time=end_time;
        // }
    }
}