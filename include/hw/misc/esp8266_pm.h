#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"

#define TYPE_ESP8266_PM "esp8266.pm"
#define ESP8266_PM(obj) OBJECT_CHECK(Esp8266PmState, (obj), TYPE_ESP8266_PM)

typedef struct Esp8266PmState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint64_t calibration_reg;
} Esp8266PmState;

REG32(TOS_CAL_DATA, 0xc)
    FIELD(TOS_CAL_DATA, MODE, 0, 2)