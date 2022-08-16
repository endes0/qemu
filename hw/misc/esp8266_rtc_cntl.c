/*
 * ESP8266 RTC_CNTL (RTC block controller) device
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp8266_reg.h"
#include "hw/misc/esp8266_rtc_cntl.h"


static uint64_t esp8266_rtc_cntl_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266RtcCntlState *s = ESP8266_RTC_CNTL(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_RTC_INT_ENABLE:
        // TODO: implement this
        r = FIELD_DP32(r, RTC_INT_ENABLE, WAKEUP_STATE, 1);
        break;
    case A_RTC_SLEEP_TARGET:
        qemu_log_mask(LOG_UNIMP, "%s: read from RTC_SLEEP_TARGET\n", __func__);
        break;
    case A_RTC_STATE1:
        r = FIELD_DP32(r, RTC_STATE1, HW_RESET_CAUSE, s->reset_cause);
        break;
    case A_RTC_STATE2 ... A_RTC_INT_CLEAR:
    case A_RTC_STORE0 ... A_PAD_XPD_DCDC_CONF:
        qemu_log_mask(LOG_UNIMP, "%s: read from 0x%" HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    return r;
}

static void esp8266_rtc_cntl_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size)
{
    //Esp8266RtcCntlState *s = ESP8266_RTC_CNTL(opaque);
    switch (addr) {
        default:
            qemu_log_mask(LOG_UNIMP, "%s: write to 0x%" HWADDR_PRIx " value: %ld\n", __func__, addr, value);
            break;
    }
}

static const MemoryRegionOps esp8266_rtc_cntl_ops = {
    .read =  esp8266_rtc_cntl_read,
    .write = esp8266_rtc_cntl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp8266_rtc_cntl_reset(DeviceState *dev)
{
    //Esp8266RtcCntlState *s = ESP8266_RTC_CNTL(dev);
}

static void esp8266_rtc_cntl_realize(DeviceState *dev, Error **errp)
{
}

static void esp8266_rtc_cntl_init(Object *obj)
{
    Esp8266RtcCntlState *s = ESP8266_RTC_CNTL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp8266_rtc_cntl_ops, s,
                          TYPE_ESP8266_RTC_CNTL, ESP8266_RTC_CNTL_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    /*sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->dig_reset_req, ESP8266_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->cpu_reset_req[0], ESP8266_RTC_CPU_RESET_GPIO, ESP8266_CPU_COUNT);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->cpu_stall_req[0], ESP8266_RTC_CPU_STALL_GPIO, ESP8266_CPU_COUNT);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->clk_update, ESP8266_RTC_CLK_UPDATE_GPIO, 1);*/

    s->reset_cause = ESP8266_POWERON_RESET;
}

static Property esp8266_rtc_cntl_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp8266_rtc_cntl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp8266_rtc_cntl_reset;
    dc->realize = esp8266_rtc_cntl_realize;
    device_class_set_props(dc, esp8266_rtc_cntl_properties);
}

static const TypeInfo esp8266_rtc_cntl_info = {
    .name = TYPE_ESP8266_RTC_CNTL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266RtcCntlState),
    .instance_init = esp8266_rtc_cntl_init,
    .class_init = esp8266_rtc_cntl_class_init
};

static void esp8266_rtc_cntl_register_types(void)
{
    type_register_static(&esp8266_rtc_cntl_info);
}

type_init(esp8266_rtc_cntl_register_types)
