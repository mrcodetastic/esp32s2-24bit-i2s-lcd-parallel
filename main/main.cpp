/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "rom/cache.h"

#include "esp32s2_i2s_lcd_24bit_parallel_dma.hpp"
#include "main.h"

// Blink LED so we know we're still alive.
const gpio_num_t ledPin = GPIO_NUM_15;  // 16 corresponds to GPIO15
static uint8_t s_led_state = 0;

// How many parallel clocks of 24 bits of data do we want to send out?
// Note: The memory/byte size that will be 3x this (3 bytes each time = 24 bites).
const size_t DMA_PAYLOAD_BITLENGTH = 16000;

static void blink_led(void)
{
    s_led_state = !s_led_state;
    gpio_set_level(ledPin, s_led_state);
}


void blank_task(void *arg)
{
    while (1) {
            blink_led();

            for (int i = 2; i >= 0; i--) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            fflush(stdout);
    }

} // end output_task

// Function to toggle a specific bit in a byte
uint8_t toggle_bit(uint8_t byte, int bit) {
    return byte ^ (1 << bit);
}


extern "C" void app_main(void)
{
    static config_t bus_cfg;

    gpio_reset_pin(ledPin);
    gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);
    blink_led();    

    for (int delaysec = 4; delaysec > 0; delaysec--)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ESP_LOGI("I2S-DMA", "Starting in %d...", delaysec);
    }
    
    xTaskCreate(blank_task, "LedBlinkTask", 4096, NULL, 10, &myTaskHandle);


    bus_cfg.bus_freq = 2*1000*1000;
    bus_cfg.parallel_width = 24;

    bus_cfg.pin_wr      = GPIO_NUM_39; 
    bus_cfg.invert_pclk = false;

    bus_cfg.pin_d0 = GPIO_NUM_40; // lsb of the right most byte of the three bytes
    bus_cfg.pin_d1 = GPIO_NUM_38;
    bus_cfg.pin_d2 = GPIO_NUM_37;
    bus_cfg.pin_d3 = GPIO_NUM_35;
    bus_cfg.pin_d4 = GPIO_NUM_36;
    bus_cfg.pin_d5 = GPIO_NUM_34; // blue
    bus_cfg.pin_d6 = GPIO_NUM_33;
    bus_cfg.pin_d7 = GPIO_NUM_18;
    bus_cfg.pin_d8 = GPIO_NUM_21; // start of second byte
    bus_cfg.pin_d9 = GPIO_NUM_17;
    bus_cfg.pin_d10 = GPIO_NUM_18;
    bus_cfg.pin_d11 = GPIO_NUM_16;
    bus_cfg.pin_d12 = -1;
    bus_cfg.pin_d13 = -1;
    bus_cfg.pin_d14 = -1;  
    bus_cfg.pin_d15 = -1; // end of byte 2
    bus_cfg.pin_d16 = GPIO_NUM_8; // start of third byte
    bus_cfg.pin_d17 = GPIO_NUM_7;
    bus_cfg.pin_d18 = -1;
    bus_cfg.pin_d19 = -1;
    bus_cfg.pin_d20 = -1;      
    bus_cfg.pin_d21 = -1;        
    bus_cfg.pin_d22 = -1;          
    bus_cfg.pin_d23 = -1;  // end of third byte

    printf("Configuring I2S LCD mode.\n");
    i2s_lcd_setup(bus_cfg);

    printf("Allocating DMA memory.\n");    
    size_t bytes_required = DMA_PAYLOAD_BITLENGTH * 3; // 3 bytes (24bits) per data output clock
    dma_allocate_memory(bytes_required);

    printf("Loading 24 bits at once payload.\n");   
    for (int j = 0; j < DMA_PAYLOAD_BITLENGTH; ++j) {
        uint8_t* bytes = &parallel_out_buffer[j * 3];
        uint8_t* byte1 = &bytes[0];
        uint8_t* byte2 = &bytes[1];
        uint8_t* byte3 = &bytes[2];

        for (int i = 1; i <= 23; ++i) {
            if (i <= 7) {
                *byte1 = toggle_bit(*byte1, i);  // Toggle bit in the first byte
            } else if (i <= 15) {
                *byte2 = toggle_bit(*byte2, i - 8);  // Toggle bit in the second byte
            } else {
                *byte3 = toggle_bit(*byte3, i - 16);  // Toggle bit in the third byte
            }

            if (i % 2 == 0) {
                *byte1 |= 1;  // Set bit 0 high for every second iteration
            } else {
                *byte1 &= ~1; // Ensure bit 0 is low for other iterations
            }
        }

             Cache_WriteBack_Addr((uint32_t) bytes, 3);    

    } // end bit waterfall

    /*
        The above code should generate and load this sequence of bits being flipped into the buffer. 
        This sequence should then be in the output.

        000000000000000000000010
        000000000000000000000111
        000000000000000000001110
        000000000000000000011111
        000000000000000000111110
        000000000000000001111111
        000000000000000011111110
        000000000000000111111111
        000000000000001111111110
        000000000000011111111111
        000000000000111111111110
        000000000001111111111111
        000000000011111111111110
        000000000111111111111111
        000000001111111111111110
        000000011111111111111111
        000000111111111111111110
        000001111111111111111111
        000011111111111111111110
        000111111111111111111111
        001111111111111111111110
        011111111111111111111111
        111111111111111111111110
        000000000000000000000010
        000000000000000000000111
        000000000000000000001110
        000000000000000000011111
        000000000000000000111110
        000000000000000001111111
        000000000000000011111110
        000000000000000111111111
        000000000000001111111110
        000000000000011111111111
        000000000000111111111110
        000000000001111111111111
        000000000011111111111110
        000000000111111111111111
        000000001111111111111110
        000000011111111111111111
        000000111111111111111110
        000001111111111111111111
        000011111111111111111110
        000111111111111111111111
        001111111111111111111110
        011111111111111111111111
        111111111111111111111110        

    */    


    printf("Commence circular loop of output.\n");   
    dma_start_output();

}