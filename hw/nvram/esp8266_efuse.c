/*
 * ESP8266 eFuse emulation
 *
 * Copyright (c) 2022
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "chardev/char-fe.h"
#include "hw/nvram/esp8266_efuse.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"

static const uint8_t crc8_table[] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
    0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
    0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
    0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
    0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
    0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
    0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
    0xd7, 0x89, 0x6b, 0x35
};

static uint8_t esp8266_crc8(uint8_t *data, int len)
{
    uint8_t crc = 0;
    int i;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        crc = crc8_table[crc];
    }
    return crc;
};

static uint64_t esp8266_efuse_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266EfuseState *s = ESP8266_EFUSE(opaque);
    uint8_t mac[6] = { 0, 0, 0, 0, 0, 0 };
    uint64_t r = 0;

    if (s->mac_size >= 6) {
        memcpy(mac, s->mac, s->mac_size);
    } else {
        mac[0] = 0x18;
        mac[1] = 0xFE;
        mac[2] = 0x34;
    }

    if (s->blk) {
        blk_pread(s->blk, addr, &r, sizeof(r));
        return r;
    }

    switch (addr) {
    case A_EFUSE_DATA0:
        uint8_t crc1[4] = { mac[5], mac[4], mac[3], 0 };
        r = FIELD_DP32(r, EFUSE_DATA0, IS_ESP8285, false);
        r = FIELD_DP32(r, EFUSE_DATA0, MAC2, mac[5]);
        r = FIELD_DP32(r, EFUSE_DATA0, CRC1, esp8266_crc8(crc1, 4));
        break;
    case A_EFUSE_DATA1:
        r = FIELD_DP32(r, EFUSE_DATA1, MAC1, (mac[3] << 8) | mac[4]);
        r = FIELD_DP32(r, EFUSE_DATA1, VERSION, 2);
        break;
    case A_EFUSE_DATA2:
        uint8_t crc[3] = { mac[2], mac[1], mac[0] };
        r = FIELD_DP32(r, EFUSE_DATA2, IS_48_BITS_MAC, true);
        r = FIELD_DP32(r, EFUSE_DATA2, GPIO_PAD, true); // TODO: GPIO
        r = FIELD_DP32(r, EFUSE_DATA2, IS_24_PINS, false); // TODO: GPIO
        r = FIELD_DP32(r, EFUSE_DATA2, USE_TYPE2, true); // TODO
        r = FIELD_DP32(r, EFUSE_DATA2, CRC0, esp8266_crc8(crc, 3));
        break;
    case A_EFUSE_DATA3:
        r = FIELD_DP32(r, EFUSE_DATA3, MAC0,
                       mac[0] << 16 | mac[1] << 8 | mac[2]);
        r = FIELD_DP32(r, EFUSE_DATA3, EMB_FLASH_SIZE, 1); // 4 Mb
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read from 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
    return r;
}

static void esp8266_efuse_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    // Esp8266EfuseState *s = ESP8266_EFUSE(opaque);
    switch (addr) {
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write to 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps esp8266_efuse_ops = {
    .read = esp8266_efuse_read,
    .write = esp8266_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp8266_efuse_init(Object *obj)
{
    Esp8266EfuseState *s = ESP8266_EFUSE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp8266_efuse_ops, s,
                          TYPE_ESP8266_EFUSE, A_EFUSE_DATA3 + 4);
    sysbus_init_mmio(sbd, &s->iomem);
}

static Property esp8266_efuse_properties[] = {
    DEFINE_PROP_DRIVE("drive", Esp8266EfuseState, blk),
    DEFINE_PROP_ARRAY("mac", Esp8266EfuseState, mac_size, mac, qdev_prop_uint8,
                      uint8_t),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp8266_efuse_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    // dc->realize = esp8266_efuse_realize;
    device_class_set_props(dc, esp8266_efuse_properties);
}

static const TypeInfo esp8266_efuse_info = {
    .name = TYPE_ESP8266_EFUSE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266EfuseState),
    .instance_init = esp8266_efuse_init,
    .class_init = esp8266_efuse_class_init
};

static void esp8266_efuse_register_types(void)
{
    type_register_static(&esp8266_efuse_info);
}

type_init(esp8266_efuse_register_types)
