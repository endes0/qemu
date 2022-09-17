#pragma once

#include "hw/hw.h"
#include "hw/misc/esp8266_reg.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"

// HDRF = Hardware Defined RF

typedef struct Esp8266HdrfState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

} Esp8266HdrfState;

#define TYPE_ESP8266_HDRF "misc.esp8266.hdrf"
#define ESP8266_HDRF(obj) \
    OBJECT_CHECK(Esp8266HdrfState, (obj), TYPE_ESP8266_HDRF)


REG32(IQ_EST, 0x7c)
FIELD(IQ_EST, FLAG_ENABLE, 31, 1)
