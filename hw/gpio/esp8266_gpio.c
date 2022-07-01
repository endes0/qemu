/*
 * ESP8266 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2022 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp8266_gpio.h"



static uint64_t esp8266_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266GpioState *s = ESP8266_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_GPIO_IN:
        r = FIELD_DP32(r, GPIO_IN, STRAP, s->strap_mode);
        r = FIELD_DP32(r, GPIO_IN, IN_DATA, 0); //TODO
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: Unimplemented GPIO register read: 0x%" HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    return r;
}

static void esp8266_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    qemu_log_mask(LOG_UNIMP, "%s: Unimplemented GPIO register Write: 0x%" HWADDR_PRIx ", val: %ld\n", __func__, addr, value);
}

static const MemoryRegionOps gpio_ops = {
    .read =  esp8266_gpio_read,
    .write = esp8266_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp8266_gpio_reset(DeviceState *dev)
{
}

static void esp8266_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp8266_gpio_init(Object *obj)
{
    Esp8266GpioState *s = ESP8266_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &gpio_ops, s,
                          TYPE_ESP8266_GPIO, 0x300);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp8266_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp8266GpioState, strap_mode, ESP8266_STRAP_MODE_FLASH_BOOT),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp8266_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp8266_gpio_reset;
    dc->realize = esp8266_gpio_realize;
    device_class_set_props(dc, esp8266_gpio_properties);
}

static const TypeInfo esp8266_gpio_info = {
    .name = TYPE_ESP8266_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266GpioState),
    .instance_init = esp8266_gpio_init,
    .class_init = esp8266_gpio_class_init
};

static void esp8266_gpio_register_types(void)
{
    type_register_static(&esp8266_gpio_info);
}

type_init(esp8266_gpio_register_types)
