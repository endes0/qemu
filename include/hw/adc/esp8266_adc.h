#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"

#define TYPE_ESP8266_ADC "esp8266.adc"
#define ESP8266_ADC(obj) OBJECT_CHECK(Esp8266AdcState, (obj), TYPE_ESP8266_ADC)

typedef struct Esp8266AdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint64_t calibration_reg;
} Esp8266AdcState;

REG32(TOS_CAL_DATA, 0x4c)
    FIELD(TOS_CAL_DATA, MODE, 0, 2)
