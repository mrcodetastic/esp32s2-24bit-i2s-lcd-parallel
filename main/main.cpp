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

    bus_cfg.parallel_width      = 24;

    bus_cfg.pin_wr = GPIO_NUM_39; 
    bus_cfg.pin_d0 = GPIO_NUM_40; // lsb of the right most byte of the three bytes
    bus_cfg.pin_d1 = GPIO_NUM_37;
    bus_cfg.pin_d2 = GPIO_NUM_38;
    bus_cfg.pin_d3 = GPIO_NUM_35;
    bus_cfg.pin_d4 = GPIO_NUM_36;
    bus_cfg.pin_d5 = GPIO_NUM_33; // blue
    bus_cfg.pin_d6 = GPIO_NUM_34;
    bus_cfg.pin_d7 = GPIO_NUM_18;
    bus_cfg.pin_d8 = -1; // start of second byte
    bus_cfg.pin_d9 = -1;
    bus_cfg.pin_d10 = -1;
    bus_cfg.pin_d11 = -1;
    bus_cfg.pin_d12 = -1;
    bus_cfg.pin_d13 = -1;
    bus_cfg.pin_d14 = -1;  
    bus_cfg.pin_d15 = GPIO_NUM_16; // end of byte 2
    bus_cfg.pin_d16 = -1; // start of third byte
    bus_cfg.pin_d17 = -1;
    bus_cfg.pin_d18 = -1;
    bus_cfg.pin_d19 = -1;
    bus_cfg.pin_d20 = -1;      
    bus_cfg.pin_d21 = -1;        
    bus_cfg.pin_d22 = -1;          
    bus_cfg.pin_d23 = GPIO_NUM_17;  // end of third byte

    printf("Configuring I2S LCD mode.\n");
    i2s_lcd_setup(bus_cfg);

    printf("Allocating DMA memory.\n");    
    size_t bytes_required = DMA_PAYLOAD_BITLENGTH * 3; // 3 bytes (24bits) per data output clock
    dma_allocate_memory(bytes_required);


    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("Loading 24 bits at once payload.\n");   
    int parallel_output_iteration = 0;
    for (int j = 0; j < DMA_PAYLOAD_BITLENGTH; ++j) {
            int offset = j * 3;
            
            // Byte 0 of 3 - least significant byte that has d7-d0
            if (parallel_output_iteration < 8)
            {
                    parallel_out_buffer[offset] = 1 << parallel_output_iteration;; 
            }
            // Byte 1 of 3 - d15-d8
            else if (parallel_output_iteration < 16)
            {
                        parallel_out_buffer[offset+1] = 1 << (parallel_output_iteration-8);
            }
            // Byte 3 of 3 - top 8 bits of the 24 bits output
            else if (parallel_output_iteration < 24)
            {
                        parallel_out_buffer[offset+2] = 1 << (parallel_output_iteration-16); 
            }
            
            // Start again
            if (parallel_output_iteration < 24)
            {
                    parallel_output_iteration++;
            } else { 
            parallel_output_iteration = 0;
            }
            
#ifdef USE_REAL_SLOW_PSRAM            
            // Must do this if using PSRAM
            Cache_WriteBack_Addr((uint32_t)&parallel_out_buffer[offset], 3);    
#endif            

    } // end bit waterfall

    printf("Commence circular loop of output.\n");   
    dma_start_output();

}