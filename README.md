| Supported Targets |  ESP32-S2 |
| ----------------- |  -------- | 

# ESP32-S2 24bit LCD DMA using PSRAM
This is an example of outputting 24bits of data in parallel from PSRAM / SPIRAM via the LCD interface.

This esp-idf sketch has been written for the ESP32-S2 only.

## How does this work?
The biggest issue with using PSRAM is "cache coheriency" or something to do with the fact that there's a buffer between the CPU and the PSRAM when writing to it, but NOT when reading from it via DMA.

So there's the risk of stuff you write to PSRAM not actually being written as it's sitting in the cache, so the DMA read-out is incomplete.

Extensive use of `Cache_WriteBack_Addr` resolves for this when doing PSRAM write functions.

## Example output as seen in pulseview

This is just a screen capture of the lower 3  in the data that's loaded into PSRAM. Example data only. Look at the code to understand.

![image](https://github.com/mrcodetastic/esp32s2-24bit-i2s-lcd-parallel/assets/12006953/913ec09f-2ab9-459d-93c9-f073dd3fd5a4)
