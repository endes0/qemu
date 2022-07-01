#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "hw/misc/esp8266_reg.h"
#include "sysemu/block-backend.h"

typedef struct Esp8266CacheState Esp8266CacheState;
typedef struct Esp8266DportState Esp8266DportState;

typedef struct Esp8266CacheRegionState {
    Esp8266CacheState* cache;
    MemoryRegion mem;
    
    hwaddr base;
   
} Esp8266CacheRegionState;

typedef struct Esp8266CacheState {
    Esp8266DportState* dport;

    bool enabled;
    bool is_2MB_block; // If set, cache blocks are 2MB in size, otherwise 1MB
    bool is_1MB_offset; // If set, there is 1MB in the cache address space, otherwise 0MB

    uint32_t block;
    bool target; // Undocumented

    Esp8266CacheRegionState flash_cache;
} Esp8266CacheState;

typedef struct Esp8266DportState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    BlockBackend *flash_blk;
    Esp8266CacheState cache;

    // Edge triggered Interrupts
    // TODO: use irq
    bool wdt_irq_enabled;
    bool timer1_irq_enabled;

    // SPI Interrupts
    // TODO: Use irqs
    bool spi0_irq;
    bool hspi_irq;

    // IOSWAP
    //TODO: use irqs
    bool ioswap_uart;
    bool ioswap_uart0;
    bool ioswap_uart1;
    bool ioswap_spi;
    bool ioswap_hspi;
    bool ioswap_cspi;

    // Clock frequency
    //TODO: use irqs
    bool double_clk; // If set, clock is 160 MHz, otherwise 80 MHz
} Esp8266DportState;

#define TYPE_ESP8266_DPORT "misc.esp8266.dport"
#define ESP8266_DPORT(obj) OBJECT_CHECK(Esp8266DportState, (obj), TYPE_ESP8266_DPORT)
#define TYPE_ESP8266_CACHE_REGION "misc.esp8266.cache_region"
#define ESP8266_CACHE_REGION(obj) OBJECT_CHECK(Esp8266CacheRegionState, (obj), TYPE_ESP8266_CACHE_REGION)


REG32(DPORT_EDGE_INT_ENABLE, 0x4);
    FIELD(DPORT_EDGE_INT_ENABLE, WDT, 0, 1);
    FIELD(DPORT_EDGE_INT_ENABLE, TIMER1, 1, 1);

REG32(DPORT_SPI_CACHE, 0xC);
    FIELD(DPORT_SPI_CACHE, ENABLE, 8, 1);
    FIELD(DPORT_SPI_CACHE, FLUSH_START, 0, 1);
    FIELD(DPORT_SPI_CACHE, EMPTY, 1, 1);
    FIELD(DPORT_SPI_CACHE, BUSY, 9, 1);
    FIELD(DPORT_SPI_CACHE, BLOCK, 16, 3);
    FIELD(DPORT_SPI_CACHE, SIZE, 24, 1);
    FIELD(DPORT_SPI_CACHE, OFFSET, 25, 1);
    FIELD(DPORT_SPI_CACHE, TARGET, 26, 1);

REG32(DPORT_CTL, 0x14);
    FIELD(DPORT_CTL, DOUBLE_CLK, 0, 1);

REG32(DPORT_SPI_INTERRUPT_TYPE, 0x20);
    FIELD(DPORT_SPI_INTERRUPT_TYPE, SPI0, 4, 1);
    FIELD(DPORT_SPI_INTERRUPT_TYPE, SPI1, 7, 1);
    FIELD(DPORT_SPI_INTERRUPT_TYPE, I2S, 9, 1);

REG32(DPORT_SPI_CACHE_TARGET, 0x24);
    FIELD(DPORT_SPI_CACHE_TARGET, TARGET1, 3, 1);
    FIELD(DPORT_SPI_CACHE_TARGET, TARGET2, 4, 1);

REG32(DPORT_IOSWAP, 0x28);
    FIELD(DPORT_IOSWAP, UART, 0, 1);
    FIELD(DPORT_IOSWAP, SPI, 1, 1);
    FIELD(DPORT_IOSWAP, UART0, 2, 1);
    FIELD(DPORT_IOSWAP, UART1, 3, 1);
    FIELD(DPORT_IOSWAP, HSPI, 5, 1);
    FIELD(DPORT_IOSWAP, DOUBLE_HSPI, 6, 1);
    FIELD(DPORT_IOSWAP, DOUBLE_CSPI, 7, 1);
    








