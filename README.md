# SSD1306 STM32 SPI Non-Blocking Drivers

This repository has code for SSD1306 display drivers for STM32. The driver uses non-blocking data transmission by leveraging interrupts and DMA.

The drivers will work for any SSD1306 based LCD or OLED display.

# Tested Hardware

|     STM32      |        Display      |   Tested    |
| -------------- | ------------------- | ----------- |
|   STM32F446RE   | [Digilent PMOS OLED](https://store.digilentinc.com/pmod-oled-128-x-32-pixel-monochromatic-oled-display/) |  :heavy_check_mark:  |
|   STM32F446RE   | [Monochrome 128x32 SPI OLED graphic display](https://www.adafruit.com/product/661) |       :heavy_check_mark:      |

#

# Set-Up

To use these drivers you must set-up DMA enabled SPI, declare couple private variables and implement the SPI complete callback. 
You can see this set-up implemented in the example code. For more detail, read on...

### SPI with DMA

You must set up the SPI to work with DMA loading the Tx buffer. This can be done as follows:

1. In SPI init:
```
/* Associate DMA1 Stream 4 */
Spi_ssd1306Write.hdmatx = &hdma_spi2_tx;
```

2. Enable the HAL interrupt handlers
```
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

void SPI2_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&Spi_ssd1306Write);
}
```

### Using the driver

1. Configure display dimensions in ssd1306.h
```
/* SSD1306 width in pixels */
#define SSD1306_WIDTH 128

/* SSD1306 OLED height in pixels */
#define SSD1306_HEIGHT 32
```

2. The handle to the SPI peripheral and SSD1306 data buffer must be initialized in main.c
```
#include "ssd1306.h"

SSD1306_t SSD1306_Disp;

SPI_HandleTypeDef Spi_ssd1306Write;
```

The handle and buffer are external variables in driver src file ssd1306.c so names must match
```
/* Handle for SPI communication peripheral */
extern SPI_HandleTypeDef Spi_ssd1306Write;

/* This variable should be defined in main */
extern SSD1306_t SSD1306_Disp;
```

3. Set SPI complete callback to put SSD1306 strucuture in ready to transmit state
```
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *pSpi2_oledWrite)
{
  /* Set the SSD1306 state to ready */
  SSD1306_Disp.state = SSD1306_STATE_READY;
}
```

### Avoiding Race Conditions

If you write to the SSD1306 buffer during data transmission, you will invoke a race condition! To avoid this, you can wrap any sections modifying the buffer in a condition checking SSD1306 state.

Example:
```
if (SSD1306_Disp.state == SSD1306_STATE_READY) {
  /* Write to the buffer here */
}
```

Similarly, when using an RTOS, semaphore or mutex can protect the data buffer.
