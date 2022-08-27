#pragma once

// Registers memory map
#define DR_REG_DPORT_BASE 0x3ff00000
#define DR_REG_EFUSE_BASE 0x3ff00050
#define DR_REG_WDEV_BASE 0x3ff20000
#define DR_REG_RNG_BASE 0x3ff20e44
#define DR_REG_UART0_BASE 0x60000000
#define DR_REG_SPI1_BASE 0x60000100
#define DR_REG_SPI0_BASE 0x60000200
#define DR_REG_GPIO_BASE 0x60000300
#define DR_REG_HDRF_BASE 0x60000500 // Related to the power, gain and attenuation, no info availiable
#define DR_REG_TIMER_BASE 0x60000600
#define DR_REG_RTC_BASE 0x60000700
#define DR_REG_IO_MUX_BASE 0x60000800
#define DR_REG_WATCHDOG_BASE 0x60000900
#define DR_REG_SDIO_BASE 0x60000a00
#define DR_REG_SLC_BASE 0x60000b00
#define DR_REG_SAR_BASE 0x60000d00 // Related to the ADC, RF, clocks(cristals and PLL) and internal i2c, no info availiable
#define DR_REG_I2S_BASE 0x60000e00
#define DR_REG_UART1_BASE 0x60000f00

#define DR_REG_WIFI_BASE 0x600097FF


// Interrupts numbers
// From: https://github.com/espressif/ESP8266_RTOS_SDK/blob/d45071563cebe9ca520cbed2537dc840b4d6a1e6/components/esp8266/include/rom/ets_sys.h#L38
#define ETS_SLC_INUM        1
#define ETS_SPI_INUM        2
#define ETS_GPIO_INUM       4
#define ETS_UART_INUM       5
#define ETS_MAX_INUM        6
#define ETS_SOFT_INUM       7
#define ETS_WDT_INUM        8
#define ETS_FRC_TIMER1_INUM 9