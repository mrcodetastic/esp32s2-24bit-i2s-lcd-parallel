#pragma once

#include <string.h> // memcpy
#include <stdbool.h>
#include <sys/types.h>
#include <freertos/FreeRTOS.h>
//#include <driver/i2s.h>
#include <rom/lldesc.h>
#include <rom/gpio.h>
#if (ESP_IDF_VERSION_MAJOR == 5)
#include <driver/i2s_types.h> //includes struct and reg
#else
#include <driver/i2s.h>
#include <soc/i2s_struct.h>
#endif

#include <soc/i2s_periph.h> //includes struct and reg

#define DMA_MAX (4096-4)

// The type used for this SoC
#define HUB75_DMA_DESCRIPTOR_T lldesc_t
#define DMA_DATA_TYPE uint8_t

//----------------------------------------------------------------------------

// Do we want to use PSRAM or not?
//#define USE_REAL_SLOW_PSRAM 1

//----------------------------------------------------------------------------

 struct config_t
  {
    // Default values
    uint32_t bus_freq;
    int8_t pin_wr; 
    bool   invert_pclk;
    int8_t parallel_width;

    union
    {
      int8_t pin_data[24];
      struct
      {
        int8_t pin_d0;
        int8_t pin_d1;
        int8_t pin_d2;
        int8_t pin_d3;
        int8_t pin_d4;
        int8_t pin_d5;
        int8_t pin_d6;
        int8_t pin_d7;
        int8_t pin_d8;
        int8_t pin_d9;
        int8_t pin_d10;
        int8_t pin_d11;
        int8_t pin_d12;
        int8_t pin_d13;
        int8_t pin_d14;
        int8_t pin_d15;
        int8_t pin_d16;          
        int8_t pin_d17;               
        int8_t pin_d18;                         
        int8_t pin_d19;                                   
        int8_t pin_d20;                                             
        int8_t pin_d21;                                                       
        int8_t pin_d22;                                                                 
        int8_t pin_d23;                                                                           
      };
    };
  };

//----------------------------------------------------------------------------

    esp_err_t i2s_lcd_setup         (config_t &_cfg);
    esp_err_t dma_allocate_memory   (const size_t payload_size); 
    esp_err_t dma_start_output      ();
