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
#define ENABLE_SOUND 0
#define ENABLE_LCD 1
#define PEANUT_GB_HIGH_LCD_ACCURACY 1
#define PEANUT_GB_USE_BIOS 0

// Project headers
#include "minigb_apu.h"
#define AUDIO_BUFFER_SIZE_BYTES (AUDIO_SAMPLES*4) 
#include "peanut_gb.h"
#include "rom.h"

// GPIO connections for buttons
#define GPIO_UP		0
#define GPIO_DOWN	2
#define GPIO_LEFT	4
#define GPIO_RIGHT	5
#define GPIO_A		18
#define GPIO_B		19
#define GPIO_SELECT	21
#define GPIO_START	22

// GPIO connections for ILI9225 LCD
#define GPIO_CS		15
#define GPIO_CLK	14
#define GPIO_SDI	13
#define GPIO_RS		26
#define GPIO_RST	27
#define GPIO_LED	-1  // -1 = directly connected to +3.3V

// GPIO connections for MAX98357A I2S Amplifier
#define GPIO_BCLK   32
#define GPIO_LRC    25
#define GPIO_DIN    33

// Global variables
static struct gb_s gb;      // Emulator context. Only values within the `direct` struct may be modified directly by the front-end implementation.
static uint8_t ram[32768];  // Cartridge RAM
static int64_t start_time;   // start time used to compute Frames Per Second (FPS)
static int64_t end_time;     // end time used to compute Frames Per Second (FPS)
static int64_t frames;       // number of frames rendered since start_time (to compute FPS)
static const float target_period_us = 1000000 / VERTICAL_SYNC; // Target period for screen refresh of 64 Hz
static float delta;                // delay that will draw the screen at a rate of 64 Hz

// stream contains 512 (=AUDIO_SAMPLES) audio samples
// each sample is 32 bits
// (16 bits for the left channel + 16 bits for the right channel in stereo interleaved format)
// This is intended to be played at 32768Hz (=AUDIO_SAMPLE_RATE)
int16_t *stream;
size_t i2s_bytes_write = 0;

#if ENABLE_LCD
    // To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
    // but less overhead for setting up / finishing transfers. Make sure 144 is dividable by this.
    #define PARALLEL_LINES (16)

    // Maximum transfer size, in bytes
    #define ILI9225_MAX_TRANSFER_SIZE_BYTES (PARALLEL_LINES*LCD_WIDTH*sizeof(uint16_t))

    // Transaction queue size. This sets how many transactions can be 'in the air' 
    // (queued using spi_device_queue_trans but not yet finished using spi_device_get_trans_result) at the same time
    #define ILI9225_TRANSACTION_QUEUE_SIZE (LCD_HEIGHT/PARALLEL_LINES)

    static spi_device_handle_t spi_lcd;
    static spi_transaction_t spi_transaction[ILI9225_TRANSACTION_QUEUE_SIZE];
    static uint16_t current_transaction;
    static uint16_t *line_data[ILI9225_TRANSACTION_QUEUE_SIZE];  // buffers to store pixels
    static bool transaction_in_progress=false;

    void ILI9225_write16(uint16_t value) {
        uint16_t value_swapped;
        esp_err_t ret;

        // swap lsb & msb in value
        uint16_t msb;
        uint16_t lsb;
        msb=(value & 0xFF00) >> 8;
        lsb=(value & 0x00FF) << 8;
        value_swapped=(msb | lsb);

        spi_transaction_t spi_transaction;
        memset(&spi_transaction,0,sizeof(spi_transaction)); // Zero out the SPI transaction
        spi_transaction.length=16;                // transaction length in bits.
        spi_transaction.tx_buffer=&value_swapped;
        ret=spi_device_transmit(spi_lcd,&spi_transaction);
        assert(ret==ESP_OK);            // Should have had no issues.
    }

    void ILI9225_set_register(uint16_t cmd, uint16_t data) {
        //printf("set_register(0x%04X,0x%04X)\n",cmd,data);
        gpio_set_level(GPIO_CS,0);
        gpio_set_level(GPIO_RS,0);   // command mode
        ILI9225_write16(cmd);
        gpio_set_level(GPIO_RS,1);   // data mode
        ILI9225_write16(data);
        gpio_set_level(GPIO_CS,1);
    }

    void ILI9225_init_display(void) {
        esp_err_t ret;
        //Initialize non-SPI GPIOs for the ILI9225 display
        gpio_set_direction(GPIO_CS, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_RS, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_RST, GPIO_MODE_OUTPUT);

        // Initialize the SPI bus for communication with the LCD / SD Card reader
        spi_bus_config_t spi_bus_config = {
            .miso_io_num=-1,                    // Not used
            .mosi_io_num=GPIO_SDI,
            .sclk_io_num=GPIO_CLK,
            .quadwp_io_num=-1,                  // Not used
            .quadhd_io_num=-1,                  // Not used
            .max_transfer_sz=ILI9225_MAX_TRANSFER_SIZE_BYTES    // Maximum transfer size, in bytes
        };
        ret=spi_bus_initialize(HSPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
        ESP_ERROR_CHECK(ret);
        // Attach the LCD to the SPI bus
        spi_device_interface_config_t spi_lcd_interface_config = {
            .clock_speed_hz=32*1000*1000,       // 32 Mhz
            .mode=0,                            // Clock Polarity=0, Clock Phase=0
            .spics_io_num=GPIO_CS,          // CS pin
            .queue_size=ILI9225_TRANSACTION_QUEUE_SIZE, // We want to be able to queue N transactions at a time
        };
        ret=spi_bus_add_device(HSPI_HOST, &spi_lcd_interface_config, &spi_lcd);
        ESP_ERROR_CHECK(ret);

        // Zero out the SPI transactions
        for(uint16_t i=0;i<ILI9225_TRANSACTION_QUEUE_SIZE;i++)
            memset(&spi_transaction[i],0,sizeof(spi_transaction_t));

        // Initial pin values
        gpio_set_level(GPIO_CLK,0);
        gpio_set_level(GPIO_RS,0);
        gpio_set_level(GPIO_CS,1);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        // Hardware reset
        gpio_set_level(GPIO_RST,1);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_RST,0);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_RST,1);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        // START Initial sequence
        ILI9225_set_register(0x10,0x0000);
        ILI9225_set_register(0x11,0x0000);
        ILI9225_set_register(0x12,0x0000);
        ILI9225_set_register(0x13,0x0000);
        ILI9225_set_register(0x14,0x0000);
        vTaskDelay(40 / portTICK_PERIOD_MS);
        ILI9225_set_register(0x11,0x0018);
        ILI9225_set_register(0x12,0x6121);
        ILI9225_set_register(0x13,0x006F);
        ILI9225_set_register(0x14,0x495F);
        ILI9225_set_register(0x10,0x0800);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ILI9225_set_register(0x11,0x103B);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ILI9225_set_register(0x01,0x011C);
        ILI9225_set_register(0x02,0x0100);
        ILI9225_set_register(0x03,0x1038);
        ILI9225_set_register(0x07,0x0000);
        ILI9225_set_register(0x08,0x0808);
        ILI9225_set_register(0x0B,0x1100);
        ILI9225_set_register(0x0C,0x0000);
        ILI9225_set_register(0x0F,0x0701);
        ILI9225_set_register(0x15,0x0020);
        ILI9225_set_register(0x20,0x0000);
        ILI9225_set_register(0x21,0x0000);
        
        ILI9225_set_register(0x30,0x0000);
        ILI9225_set_register(0x31,0x00DB);
        ILI9225_set_register(0x32,0x0000);
        ILI9225_set_register(0x33,0x0000);
        ILI9225_set_register(0x34,0x00DB);
        ILI9225_set_register(0x35,0x0000);
        ILI9225_set_register(0x36,0x00AF);
        ILI9225_set_register(0x37,0x0000);
        ILI9225_set_register(0x38,0x00DB);
        ILI9225_set_register(0x39,0x0000);
        
        ILI9225_set_register(0x50,0x0000);
        ILI9225_set_register(0x51,0x0808);
        ILI9225_set_register(0x52,0x080A);
        ILI9225_set_register(0x53,0x000A);
        ILI9225_set_register(0x54,0x0A08);
        ILI9225_set_register(0x55,0x0808);
        ILI9225_set_register(0x56,0x0000);
        ILI9225_set_register(0x57,0x0A00);
        ILI9225_set_register(0x58,0x0710);
        ILI9225_set_register(0x59,0x0710);
        
        ILI9225_set_register(0x07,0x0012);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ILI9225_set_register(0x07,0x1017);
        // END Initial Sequence
    }

    void ILI9225_set_entry_mode(uint16_t entry_mode) {
        ILI9225_set_register(0x03,entry_mode);
    }

    void ILI9225_set_window(uint16_t x_min,uint16_t x_max,uint16_t y_min,uint16_t y_max) {
        ILI9225_set_register(0x36,x_max);
        ILI9225_set_register(0x37,x_min);
        ILI9225_set_register(0x38,y_max);
        ILI9225_set_register(0x39,y_min);
    }

    void ILI9225_set_adress(uint16_t x,uint16_t y) {
        ILI9225_set_register(0x20,x);
        ILI9225_set_register(0x21,y);
    }

    void ILI9225_wait_for_transaction_completion() {
        // Wait for the completion of sending the previous line if necessary        
        esp_err_t ret;
        spi_transaction_t *rtrans;
        if(transaction_in_progress) {
            for(uint16_t i=0;i<ILI9225_TRANSACTION_QUEUE_SIZE;i++) {
                ret=spi_device_get_trans_result(spi_lcd,&rtrans,portMAX_DELAY);
                assert(ret==ESP_OK);
            }
            transaction_in_progress=false;
        }
    }

    void ILI9225_write_pixels_start() {
        gpio_set_level(GPIO_CS,0);
        gpio_set_level(GPIO_RS,0);   // command mode
        ILI9225_write16(0X22);     // Write Data to GRAM
        gpio_set_level(GPIO_RS,1);   // data mode
    }

    void ILI9225_write_lines(uint16_t *line_data) {
        esp_err_t ret;

        spi_transaction[current_transaction].length=PARALLEL_LINES*LCD_WIDTH*16; // transaction length in bits.
        spi_transaction[current_transaction].tx_buffer=line_data;
        ret=spi_device_queue_trans(spi_lcd,&spi_transaction[current_transaction],portMAX_DELAY);  // Queue the transaction
        assert(ret==ESP_OK);            // Should have had no issues.
        transaction_in_progress=true;

        // When we are here, the SPI driver is busy (in the background) getting the transaction sent. That happens
        // mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
        // finish because we may as well spend the time calculating the next line.
    }

    void ILI9225_write_pixels_end() {
        gpio_set_level(GPIO_CS,1);
    }

    void ILI9225_fill(uint16_t color) {
            ILI9225_set_entry_mode(0x1028);
            ILI9225_set_window(0,175,0,219); // x_min, x_max, y_min, y_max
            
            // in the entry mode 0x1028,
            // the top left corner has coordinates (x=175,y0)
            ILI9225_set_adress(175,0);
            
            // fill the screen with the specified color
            ILI9225_write_pixels_start();
            for(size_t i=0;i<220*176;i++) {
                ILI9225_write16(color);
            }
            ILI9225_write_pixels_end();
    }

    /**
     * @brief function called by Peanut-GB to draw a single line on the LCD
     * 
     * @param gb      emulator context
     * @param pixels  The 160 pixels to draw.
     * 			      Bits 1-0 are the colour to draw.
     * 		          Bits 5-4 are the palette, where:
     * 				  OBJ0 = 0b00,
     * 				  OBJ1 = 0b01,
     * 				  BG = 0b10
     * 			      Other bits are undefined.
     * 			      Bits 5-4 are only required by front-ends
     * 			      which want to use a different colour for
     * 			      different object palettes. This is what
     * 			      the Game Boy Color (CGB) does to DMG
     * 			      games.
     * @param line    Line to draw pixels on. This is
     *                guaranteed to be between 0-143 inclusive.
     */
    void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
            const uint_fast8_t line)
    {
        // Game Boy original color palette (monochrome 4-shades of green!)
        // Note that the MSB & LSB are swapped in the palette
        // e.g. 0xE19D corresponds to RGB565 color 0x9DE1
        // This is required because the SPI driver transmits byte by byte instead of 16 bits words
        // 0x9DE1 = Lightest Green
        // 0x8D61 = Light Green
        // 0x3306 = Dark Green
        // 0x09C1 = Darkest Green
        // const uint16_t palette[3][4] = {
        //     { 0xE19D, 0x618D, 0x0633, 0xC109 },     // OBJ0
        //     { 0xE19D, 0x618D, 0x0633, 0xC109 },     // OBJ1
        //     { 0xE19D, 0x618D, 0x0633, 0xC109 }      // BG
        // };
        
        // Game Boy Pocket palette (monochrome 4-shades)
        // const uint16_t palette[3][4] = {
        //     { 0xFFFF, 0x55AD, 0xAA52, 0x0000 },     // OBJ0
        //     { 0xFFFF, 0x55AD, 0xAA52, 0x0000 },     // OBJ1
        //     { 0xFFFF, 0x55AD, 0xAA52, 0x0000 }      // BG
        // };

        // Example of Game Boy Color palette
        const uint16_t palette[3][4] = {
            { 0xFFFF, 0x1F65, 0x1F00, 0x0000 },     // OBJ0
            { 0xFFFF, 0x10FC, 0xC789, 0x0000 },     // OBJ1
            { 0xFFFF, 0x1F65, 0x1F00, 0x0000 }      // BG
        };

        uint16_t y=line%PARALLEL_LINES;
        bool ready_to_send=((line+1)%PARALLEL_LINES)==0;
        uint16_t *current_line_data=line_data[current_transaction];

        for(uint16_t x=0;x<LCD_WIDTH;x++)
        {
            current_line_data[y*LCD_WIDTH+x] = palette[(pixels[x] & LCD_PALETTE_ALL) >> 4]
                    [pixels[x] & 3];
        }
        
        if(ready_to_send) {
            // The line_data buffer is full => copy it to line_data_ 
            // and send line_data_ to the LCD screen in the backgroung
            // while we continue to compute the next lines
            ILI9225_write_lines(current_line_data);
            current_transaction++;
            if(current_transaction>=ILI9225_TRANSACTION_QUEUE_SIZE) {
                current_transaction=0;
            }
        }
    }
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
    // Allocate memory for the pixel buffers
    for(uint16_t i=0;i<ILI9225_TRANSACTION_QUEUE_SIZE;i++)    
        line_data[i]=heap_caps_malloc(PARALLEL_LINES*LCD_WIDTH*sizeof(uint16_t), MALLOC_CAP_DMA);
        assert(line_data!=NULL);

    // Initialize the LCD    
    ILI9225_init_display();
    printf("LCD: ILI9225_init_display() complete\n");

    // Clear LCD screen
    ILI9225_fill(0x0000);
    
    // Set LCD window to DMG size.
    ILI9225_set_entry_mode(0x1028);
    ILI9225_set_window(16,159,31,190);

    gb_init_lcd(&gb, &lcd_draw_line);
    printf("LCD: \n");

    // Speed improvements options
    gb.direct.interlace=false;
    gb.direct.frame_skip=2;
#endif

    // Allocate memory for the stream buffer
    stream=heap_caps_malloc(AUDIO_BUFFER_SIZE_BYTES, MALLOC_CAP_DMA);
    assert(stream!=NULL);
    memset(stream,0,AUDIO_BUFFER_SIZE_BYTES);  // Zero out the stream buffer

    // Initialize I2S sound driver
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 2,             // The total number of DMA buffers to receive/transmit data
        .dma_buf_len = AUDIO_SAMPLES/2,   // Number of frames in a DMA buffer
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
    i2s_write(0, stream, AUDIO_BUFFER_SIZE_BYTES, &i2s_bytes_write, portMAX_DELAY);
    audio_init();

    printf("AUDIO: \n");

    // Initialize variables for Frames Per Second computation
    start_time=esp_timer_get_time();
    delta=0;
    frames=0;

    while(true)
    {
#if ENABLE_LCD        
        // Go back to the top left corner of the Game Boy screen
        ILI9225_set_adress(159,31); 

        // Prepare to write lines
        current_transaction=0;
        ILI9225_write_pixels_start();
#endif

        // Execute CPU cycles for one frame
        // Ideally, gb_run_frame is called at 59.7275 Hz
        gb_run_frame(&gb);
        frames++;

#if ENABLE_SOUND
        audio_callback(NULL, stream, AUDIO_BUFFER_SIZE_BYTES);
        i2s_write(0, stream, AUDIO_BUFFER_SIZE_BYTES, &i2s_bytes_write, portMAX_DELAY);
        // printf("i2s_bytes_write = %d\n", i2s_bytes_write);
#endif

#if ENABLE_LCD   
        // Before we continue, make sure all SPI transfers 
        // which were running in the background are now complete
        ILI9225_wait_for_transaction_completion();
        ILI9225_write_pixels_end();
#endif

        // Use a delay that will draw the screen at a rate of 59.7275 Hz */
        vTaskDelay(delta / portTICK_PERIOD_MS);

        if(frames>60)
        {
            // Compute the delay delta that will draw the screen at a rate of 59.7275 Hz */
            end_time=esp_timer_get_time();
            delta += ((target_period_us - (float)(end_time - start_time)/frames)/1000);
            if(delta<1) {
                 delta=1;
            }
        
            // Display Frames Per Second
            printf("FPS=%lld delta=%.0f ms\n",(1000000*frames)/(end_time-start_time),delta);
        
            frames=0;
            start_time=end_time;
        }
    }
}