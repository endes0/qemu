/*
 * ESP8266 hdrf: Hardware Defined RF(?)
 *
 * Copyright (c) 2022
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */


#include "qemu/osdep.h"
#include "hw/boards.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/misc/esp8266_hdrf.h"
#include "hw/misc/esp8266_reg.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"


static uint64_t esp8266_hdrf_read(void *opaque, hwaddr addr, unsigned int size)
{
    // Esp8266HdrfState *s = ESP8266_HDRF(opaque);
    uint32_t r = 0;

    switch (addr) {
    case A_IQ_EST:
        // TODO: implement
        r = FIELD_DP32(r, IQ_EST, FLAG_ENABLE, 1);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read from 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }


    return r;
}

static void esp8266_hdrf_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned int size)
{
    // Esp8266HdrfState *s = ESP8266_HDRF(opaque);

    switch (addr) {
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write to 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps esp8266_hdrf_ops = {
    .read = esp8266_hdrf_read,
    .write = esp8266_hdrf_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_hdrf_init(Object *obj)
{
    Esp8266HdrfState *s = ESP8266_HDRF(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp8266_hdrf_ops, s,
                          TYPE_ESP8266_HDRF, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const TypeInfo esp8266_hdrf_info = { .name = TYPE_ESP8266_HDRF,
                                            .parent = TYPE_SYS_BUS_DEVICE,
                                            .instance_size =
                                                sizeof(Esp8266HdrfState),
                                            .instance_init =
                                                esp8266_hdrf_init };

static void esp8266_hdrf_register_types(void)
{
    type_register_static(&esp8266_hdrf_info);
}

type_init(esp8266_hdrf_register_types)
