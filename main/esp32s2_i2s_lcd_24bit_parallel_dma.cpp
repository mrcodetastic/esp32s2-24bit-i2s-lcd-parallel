#include <sdkconfig.h>

#if !defined (CONFIG_IDF_TARGET_ESP32S2)
  #pragma error "Designed only for ESP32-S2"
#endif

#include "esp32s2_i2s_lcd_24bit_parallel_dma.hpp"

#include <driver/gpio.h>
#if (ESP_IDF_VERSION_MAJOR == 5)
#include <esp_private/periph_ctrl.h>
#else
// Break the compile 
DO NOT COMPILE
#endif

#include <soc/gpio_sig_map.h>
#include <soc/i2s_periph.h> //includes struct and reg

#if defined (ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

#include <esp_err.h>
#include <esp_log.h>

// Get CPU freq function.
#include <soc/rtc.h>
#include <rom/ets_sys.h> // ets delay

// New dma helper tools
#include "esp_dma_utils.h"
#include "rom/cache.h"

/**************************************************************/

#define ESP32_I2S_DEVICE I2S_NUM_0	

static const char *TAG = "edma_lcd_test";

/**************************************************************/
  
  extern DMA_DATA_TYPE *parallel_out_buffer;

/**************************************************************/

  static lldesc_t *dma_ll;

  // Static I2S0
  static i2s_dev_t* getDev() {
      return &I2S0;
  }

  // Static
  inline void _gpio_pin_init(int pin)
  {
    if (pin >= 0)
    {
      gpio_pad_select_gpio(pin);
      gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
      gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);      // esp32s3 as well?
    }
  }

  // esp32 s2 only
  inline int i2s_parallel_get_memory_width(int port, int width) {
    switch(width) {
    case 8:
      return 1;
    case 16:
      return 2;
    case 24:
      return 4;
    default:
      return -ESP_ERR_INVALID_ARG;
    }
  }

  static lldesc_t * allocate_dma_descriptors_gb(uint32_t count, size_t payload_size, DMA_DATA_TYPE *buffer)
  {
      lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
      if (dma == NULL) {
          ESP_LOGE("allocate_dma_descriptors()", "Could not allocate lldesc_t memory.");            
          return dma;
      }

      int n = 0;
      while (n < count) 
      {

          int dmachunklen = payload_size;
          if (dmachunklen > DMA_MAX) {
              dmachunklen = DMA_MAX;
          }

          dma[n].size = dmachunklen;
          dma[n].length = dmachunklen;

          dma[n].sosf = 0;
          dma[n].eof = 0;
          dma[n].owner = 1;     // 1 = dma   

          //dma[n].buf = (buffer);
          dma[n].buf = (uint8_t*)(buffer);

          ESP_LOGD("allocate_dma_descriptors()", "Linking to payload buff at memory location 0x%08X.", (uintptr_t)dma[n].buf);                     

          payload_size -= dmachunklen;        
          buffer += dmachunklen;  

          if ( n == (count-1) ) { // last element
            dma[n].empty = (uintptr_t)&dma[0];
            ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d back to pos 0", n);      

          } else {
            dma[n].empty = (uintptr_t)&dma[(n  + 1) % count];
            ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d to pos %d at memory location %08x.", n, (n+1), (uintptr_t)&dma[(n + 1) % count]);      
          }
            ESP_LOGD("allocate_dma_descriptors()", "Chunk len %d.", dmachunklen);          


        n++;


      }

      return dma;
  }

  static int ll_desc_get_required_num(uint32_t bytes_len)
  {
      int ll_desc_required = (bytes_len + DMA_MAX - 1) / DMA_MAX;
      ESP_LOGD("ll_desc_get_required_num()", "Gunna need %d dma lldesc's", ll_desc_required);      

      return ll_desc_required;
  }

  esp_err_t i2s_lcd_setup(config_t& _cfg) // The big one that gets everything setup.
  {
      auto dev = getDev();
      volatile int iomux_signal_base;
      volatile int iomux_clock;
    //  int irq_source;

      periph_module_reset(PERIPH_I2S0_MODULE);
      periph_module_enable(PERIPH_I2S0_MODULE);

      iomux_clock = I2S0O_WS_OUT_IDX;
    //  irq_source = ETS_I2S0_INTR_SOURCE;

      if ( _cfg.parallel_width == 24)
      {
        ESP_LOGI(TAG, "Configuring signal base for 24bit");
        iomux_signal_base = I2S0O_DATA_OUT0_IDX;

      } else {

        ESP_LOGI(TAG, "Configuring signal base for 16bit");
        iomux_signal_base = I2S0O_DATA_OUT8_IDX;
      }

      // Setup GPIOs
      int bus_width = _cfg.parallel_width;

      // Clock output GPIO setup
      _gpio_pin_init(_cfg.pin_wr); // clock

      // Data output GPIO setup
      int8_t* pins = _cfg.pin_data;  

      for(int i = 0; i < bus_width; i++)  {
      _gpio_pin_init(pins[i]);
      }

      // Route clock signal to clock pin (can route to two pins if we want)
      gpio_matrix_out(_cfg.pin_wr, iomux_clock, _cfg.invert_pclk, 0); // inverst clock if required

      // Route data pins
      for (size_t i = 0; i < bus_width; i++) {
        if (pins[i] >= 0) {
          gpio_matrix_out(pins[i], iomux_signal_base + i, false, false);
        }
      }

      ////////////////////////////// Clock configuration //////////////////////////////
      // Code borrowed from: https://github.com/espressif/esp-iot-solution/blob/master/components/bus/i2s_lcd_esp32s2_driver.c
      //unsigned int _div_num = (unsigned int) (160000000L / _cfg.bus_freq / i2s_parallel_get_memory_width(ESP32_I2S_DEVICE, bus_width));       


      // Configure the clock
      dev->clkm_conf.val = 0;
      dev->clkm_conf.clkm_div_num = 40; // 4mhz
      //dev->clkm_conf.clkm_div_num = 20; // 8mhz
      ESP_LOGI(TAG, "Clock divider is: %d", (dev->clkm_conf.clkm_div_num));


      dev->clkm_conf.clkm_div_b = 0;
      dev->clkm_conf.clkm_div_a = 63;
      dev->clkm_conf.clk_sel = 2;
      dev->clkm_conf.clk_en = 1;

      // Configure sampling rate
      //dev->sample_rate_conf.tx_bck_div_num = 40000000 / 2000000; // Fws = Fbck / 2
      dev->sample_rate_conf.tx_bck_div_num = 2;
      dev->sample_rate_conf.tx_bits_mod = _cfg.parallel_width;

      dev->timing.val = 0;

      dev->int_ena.val = 0;
      dev->int_clr.val = ~0;

      dev->conf2.val = 0;
      dev->conf2.lcd_en = 1;

      // Configuration data format
      dev->conf.val = 0;
     // dev->conf.tx_right_first = 1; // doesn't change anything if 0
     // dev->conf.tx_msb_right = 1;   // doesn't change anything if 0
      dev->conf.tx_dma_equal = 1;

      dev->conf1.tx_pcm_bypass = 1;
      dev->conf1.tx_stop_en = 1;

      dev->fifo_conf.val = 0;
      dev->fifo_conf.tx_data_num = 32;
      dev->fifo_conf.dscr_en = 1;    

      // Not requried for S2?
      dev->fifo_conf.tx_fifo_mod_force_en = 1;       
      //dev->fifo_conf.tx_fifo_mod = 1;   // doesn't seem to change anything if 0
      dev->conf_chan.tx_chan_mod = 0;//remove


      dev->lc_conf.ext_mem_bk_size = 0;
      dev->fifo_conf.tx_24msb_en = 0;


      //dev->int_ena.out_eof = 1;
    
      return ESP_OK;
    }

  esp_err_t dma_allocate_memory(size_t payload_size_bytes)
  {
      size_t alloc_size  = payload_size_bytes; // Must be multiples of 3bytes if 24bit output.
      size_t actual_size = 0;
      
     #ifdef USE_REAL_SLOW_PSRAM
                ESP_LOGI(TAG, "Allocating PSRAM DMA memory for global_buffer_gclk_cdata.");  
                esp_err_t err = esp_dma_malloc(alloc_size, ESP_DMA_MALLOC_FLAG_PSRAM, (void **) &parallel_out_buffer, &actual_size);
                assert(err == ESP_OK);
      #else
                ESP_LOGI(TAG, "Allocating internal SRAM DMA memory for global_buffer_gclk_cdata.");  
                parallel_out_buffer = static_cast<DMA_DATA_TYPE *>(heap_caps_malloc(alloc_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
                assert(parallel_out_buffer != nullptr);
      #endif




      size_t alignment_offset = actual_size - alloc_size;      

      ESP_LOGI(TAG, "Actual size is: %d bytes", actual_size);  
      ESP_LOGI(TAG, "Alignment offset is: %d ", alignment_offset);        

      if (parallel_out_buffer == NULL)  {
              ESP_LOGE(TAG, "DMA data buffer malloc failed.");
      } 

      // Zero out
      memset(parallel_out_buffer, 0b00000000, alloc_size); // zero it.      

#ifdef USE_REAL_SLOW_PSRAM      
      Cache_WriteBack_Addr((uint32_t) parallel_out_buffer, alloc_size);   
#endif      

      // dma ll desc
      int dma_node_cnt = ll_desc_get_required_num(alloc_size); // Number of DMA nodes  8000 / 4092
      dma_ll =  allocate_dma_descriptors_gb(dma_node_cnt, alloc_size, parallel_out_buffer);

      return ESP_OK;

  } // dma_allocate


    esp_err_t dma_start_output()
  {
    auto dev = getDev();

    // Configure burst most. Not sure what this actually does.
    dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;

    while (!dev->state.tx_idle);
    dev->conf.tx_start = 0;
    dev->conf.tx_reset = 1;
    dev->conf.tx_reset = 0;
    dev->conf.tx_fifo_reset = 1;
    dev->conf.tx_fifo_reset = 0;
    dev->out_link.addr = ((uint32_t)&dma_ll[0]) & 0xfffff; // ((uint32_t)&frames[0].dma[0]) & 0xfffff; // always frame 0
    dev->out_link.start = 1;
    ets_delay_us(1);
    dev->conf.tx_start = 1;

    return ESP_OK;

  } // end 
    
