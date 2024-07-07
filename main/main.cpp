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
//#include "mbi_gclk_addr_data.h"
  
DMA_DATA_TYPE *global_buffer = NULL; // data of stuff
lldesc_t *dma_ll;

config_t bus_cfg;


static const char *TAG = "example";

static uint8_t s_led_state = 0;

// the number of the LED pin
const gpio_num_t ledPin = GPIO_NUM_15;  // 16 corresponds to GPIO15


TaskHandle_t myTaskHandle = NULL;


static void blink_led(void)
{
    s_led_state = !s_led_state;

    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(ledPin, s_led_state);

}


void output_task(void *arg)
{
    while (1)
    {
            blink_led();

            /* Print chip information */
            esp_chip_info_t chip_info;
            uint32_t flash_size;
            esp_chip_info(&chip_info);
            printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
                CONFIG_IDF_TARGET,
                chip_info.cores,
                (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
                (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
                (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
                (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

            unsigned major_rev = chip_info.revision / 100;
            unsigned minor_rev = chip_info.revision % 100;
            printf("silicon revision v%d.%d, ", major_rev, minor_rev);
            if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
                printf("Get flash size failed");
                return;
            }

            printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

            printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

            for (int i = 2; i >= 0; i--) {
                printf("Reprinting in %d seconds...\n", i);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            fflush(stdout);

    }

} // end output_task


static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(ledPin);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);
}


void mbi_prepare_data()
{
    /*
    // Firstly load across the pre-calculated gclk stuff   
    // Global buffer is 
    for (int i = 0; i < sizeof(dma_gclk_addr_data); i++) {

        // This will happen to be the least significant byte of each 3 bytes sent out at once.
        // So pin_d0-pin_d7 of the GPIO output
        int output_d0_d7_byte = i*3 +1;

        //global_buffer[output_d0_d7_byte] = dma_gclk_addr_data[i];         
       // global_buffer[output_d0_d7_byte] = (i%2 == 0) ?0b1:0b0;         
        //Cache_WriteBack_Addr((uint32_t) &global_buffer[output_d0_d7_byte], 16);               
    }
*/

}

extern "C" void app_main(void)
{
    configure_led();
    blink_led();    

    for (int delaysec = 4; delaysec > 0; delaysec--)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ESP_LOGI("I2S-DMA", "Starting in %d...", delaysec);
    }
    
    printf("Hello world!\n");

    xTaskCreate(output_task, "Output_Task", 4096, NULL, 10, &myTaskHandle);


    bus_cfg.bus_freq = 2*1000*1000;
    bus_cfg.parallel_width = 24;

    bus_cfg.pin_wr      = GPIO_NUM_39; 
    bus_cfg.invert_pclk = false;

    bus_cfg.pin_d0 = GPIO_NUM_40;
    bus_cfg.pin_d1 = GPIO_NUM_18;
    bus_cfg.pin_d2 = -1;
    bus_cfg.pin_d3 = -1;
    bus_cfg.pin_d4 = -1;
    bus_cfg.pin_d5 = GPIO_NUM_38; // blue
    bus_cfg.pin_d6 = -1;
    bus_cfg.pin_d7 = -1;
    bus_cfg.pin_d8 = -1; // start of second byte
    bus_cfg.pin_d9 = -1;
    bus_cfg.pin_d10 = -1;
    bus_cfg.pin_d11 = -1;
    bus_cfg.pin_d12 = -1;
    bus_cfg.pin_d13 = -1;
    bus_cfg.pin_d14 = -1;  
    bus_cfg.pin_d15 = -1; // end of byte 2
    bus_cfg.pin_d16 = -1; // start of third byte
    bus_cfg.pin_d17 = GPIO_NUM_36;
    bus_cfg.pin_d18 = -1;
    bus_cfg.pin_d19 = -1;
    bus_cfg.pin_d20 = -1;      
    bus_cfg.pin_d21 = -1;        
    bus_cfg.pin_d22 = -1;          
    bus_cfg.pin_d23 = GPIO_NUM_35;  // end of third byte

    i2s_lcd_setup_v2(bus_cfg);
    dma_allocate_v3(bus_cfg);
    mbi_prepare_data();

    dma_start_v2();

}