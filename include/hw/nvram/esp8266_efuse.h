#pragma once

#include "hw/hw.h"
#include "hw/misc/esp8266_reg.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "sysemu/block-backend.h"

#define TYPE_ESP8266_EFUSE "nvram.esp8266.efuse"
#define ESP8266_EFUSE(obj) \
    OBJECT_CHECK(Esp8266EfuseState, (obj), TYPE_ESP8266_EFUSE)

REG32(EFUSE_DATA0, 0x0)
FIELD(EFUSE_DATA0, IS_ESP8285, 4, 1)
FIELD(EFUSE_DATA0, CRC1, 16, 8)
FIELD(EFUSE_DATA0, MAC2, 24, 8)

REG32(EFUSE_DATA1, 0x4)
FIELD(EFUSE_DATA1, MAC1, 0, 16)
FIELD(EFUSE_DATA1, VERSION, 24, 4)

REG32(EFUSE_DATA2, 0x8)
FIELD(EFUSE_DATA2, IS_48_BITS_MAC, 12, 1)
FIELD(EFUSE_DATA2, GPIO_PAD, 13, 1)
FIELD(EFUSE_DATA2, IS_24_PINS, 14, 1)
FIELD(EFUSE_DATA2, USE_TYPE2, 15, 1)
FIELD(EFUSE_DATA2, CRC0, 24, 8)

REG32(EFUSE_DATA3, 0xc)
FIELD(EFUSE_DATA3, MAC0, 0, 24)
FIELD(EFUSE_DATA3, EMB_FLASH_SIZE, 26, 2)


typedef struct Esp8266EfuseState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    BlockBackend *blk;

    uint8_t *mac;
    uint32_t mac_size;
} Esp8266EfuseState;

/* returns NULL unless there is exactly one device */
static inline Esp8266EfuseState *esp8266_efuse_find(void)
{
    Object *o = object_resolve_path_type("", TYPE_ESP8266_EFUSE, NULL);

    return o ? ESP8266_EFUSE(o) : NULL;
}
