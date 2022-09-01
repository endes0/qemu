#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"

#define TYPE_ESP8266_ADC "esp8266.adc"
#define ESP8266_ADC(obj) OBJECT_CHECK(Esp8266AdcState, (obj), TYPE_ESP8266_ADC)

typedef struct Esp8266AdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    bool vdd_mode;
    bool tout_mode;

} Esp8266AdcState;

REG32(SAR_CFG, 0x0)
    FIELD(SAR_CFG, UNK0, 0, 1) // enable adc? bg_init en_pwdet
    FIELD(SAR_CFG, START, 1, 1)
    FIELD(SAR_CFG, READS, 2, 3)
    FIELD(SAR_CFG, UNK1, 5, 1) // 
    FIELD(SAR_CFG, CLK_DIV, 8, 8)
    FIELD(SAR_CFG, READY_STATE, 24, 3)

REG32(SAR_TIM1, 0x4)

REG32(SAR_TIM2, 0x8)

REG32(SAR_CFG1, 0xc)
    FIELD(SAR_CFG1, CLK_DIV, 0, 8)
    FIELD(SAR_CFG1, READ_TOUT, 21, 1)
    FIELD(SAR_CFG1, READ_VDD, 23, 1)

REG32(SAR_UNK, 0x10)

REG32(SAR_W0, 0x30)
REG32(SAR_W1, 0x34)
REG32(SAR_W2, 0x38)
REG32(SAR_W3, 0x3c)
REG32(SAR_W4, 0x40)
REG32(SAR_W5, 0x44)
REG32(SAR_W6, 0x48)
REG32(SAR_W7, 0x4c)