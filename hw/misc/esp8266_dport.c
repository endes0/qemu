/*
 * ESP8266 DPORT: Here lays some registers and the MMU for the SPI memory-mapping cache.
 *
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
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/misc/esp8266_reg.h"
#include "hw/misc/esp8266_dport.h"

/* 
 * This zone is very undocumented. We can asume that the ESP8266 has a very similar MMU to the ESP32, so somewhere in the dport space we can find the MMU pages status.
 * We also dont know how the illegal access are handled.
 * For now, we just read and write from the flash memory without implementing a more acurrate MMU.
 * More info:
 * https://arduino-esp8266.readthedocs.io/en/latest/mmu.html#
 * 
*/
//TODO: More accurated MMU: Cache data sync(MMU pages status, cache flush, cache invalidate, etc) and illegal access handling.

#define ESP8266_CACHE_BASE 0x40200000 

#define ESP8266_DPORT_SIZE 0x50 
#define ESP8266_CACHE_SIZE (1 * 1024 * 1024)

static uint64_t esp8266_dport_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266DportState *s = ESP8266_DPORT(opaque);
    uint64_t ret = 0;

    switch (addr) {
        case A_DPORT_EDGE_INT_ENABLE:
            ret = FIELD_DP32(ret, DPORT_EDGE_INT_ENABLE, WDT, s->wdt_irq_enabled);
            ret = FIELD_DP32(ret, DPORT_EDGE_INT_ENABLE, TIMER1, s->timer1_irq_enabled);
            return ret;
        case A_DPORT_SPI_CACHE:
            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, ENABLE, s->cache.enabled);

            //TODO: implement
            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, FLUSH_START, true); 
            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, EMPTY, true);
            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, BUSY, false);
            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, TARGET, false);

            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, BLOCK, s->cache.block);
            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, SIZE, s->cache.is_2MB_block);
            ret = FIELD_DP32(ret, DPORT_SPI_CACHE, OFFSET, s->cache.is_1MB_offset);
            return ret;
        case A_DPORT_CTL:
            ret = FIELD_DP32(ret, DPORT_CTL, DOUBLE_CLK, s->double_clk);
            return ret;
        case A_DPORT_SPI_INTERRUPT_TYPE:
            ret = FIELD_DP32(ret, DPORT_SPI_INTERRUPT_TYPE, SPI0, s->spi0_irq);
            ret = FIELD_DP32(ret, DPORT_SPI_INTERRUPT_TYPE, SPI1, s->hspi_irq);
            ret = FIELD_DP32(ret, DPORT_SPI_INTERRUPT_TYPE, I2S, 0); //Not implemented
            return ret;
        case A_DPORT_SPI_CACHE_TARGET:
            //TODO: Implement. No info found.
            return 0;
        case A_DPORT_IOSWAP:
            ret = FIELD_DP32(ret, DPORT_IOSWAP, UART, s->ioswap_uart);
            ret = FIELD_DP32(ret, DPORT_IOSWAP, SPI, s->ioswap_spi);
            ret = FIELD_DP32(ret, DPORT_IOSWAP, UART0, s->ioswap_uart0);
            ret = FIELD_DP32(ret, DPORT_IOSWAP, UART1, s->ioswap_uart1);
            ret = FIELD_DP32(ret, DPORT_IOSWAP, DOUBLE_HSPI, s->ioswap_hspi);
            ret = FIELD_DP32(ret, DPORT_IOSWAP, DOUBLE_CSPI, s->ioswap_cspi);
            return ret;
        default:
            qemu_log_mask(LOG_UNIMP, "%s: Unimplemented DPORT register read: 0x%" HWADDR_PRIx "\n", __func__, addr);
            return 0;
    }
}

static void esp8266_dport_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    Esp8266DportState *s = ESP8266_DPORT(opaque);

    switch (addr) {
        case A_DPORT_EDGE_INT_ENABLE:
            s->wdt_irq_enabled = FIELD_EX32(value, DPORT_EDGE_INT_ENABLE, WDT);
            s->timer1_irq_enabled = FIELD_EX32(value, DPORT_EDGE_INT_ENABLE, TIMER1);
            break;
        case A_DPORT_SPI_CACHE:
            s->cache.enabled = FIELD_EX32(value, DPORT_SPI_CACHE, ENABLE);
            s->cache.block = FIELD_EX32(value, DPORT_SPI_CACHE, BLOCK);
            s->cache.is_2MB_block = FIELD_EX32(value, DPORT_SPI_CACHE, SIZE);
            s->cache.is_1MB_offset = FIELD_EX32(value, DPORT_SPI_CACHE, OFFSET);

            if (s->cache.enabled) {
                qemu_log("%s: Cache enabled at \n", __func__);
                qemu_log("%s:    block: %d\n", __func__, s->cache.block);
                qemu_log("%s:    is 2MB size: %d\n", __func__, s->cache.is_2MB_block);
                qemu_log("%s:    is 1 MB offset: %d\n", __func__, s->cache.is_1MB_offset);
            }

            //memory_region_set_enabled(&s->cache->flash_cache.mem, s->cache->enabled);

            // Throw Unimplement
            if (FIELD_EX32(value, DPORT_SPI_CACHE, FLUSH_START)) {
                qemu_log_mask(LOG_UNIMP, "%s: Unimplemented Cache flush operation" HWADDR_PRIx "\n", __func__);
            }
            if (FIELD_EX32(value, DPORT_SPI_CACHE, EMPTY)) {
                qemu_log_mask(LOG_UNIMP, "%s: Unimplemented Cache empty operation" HWADDR_PRIx "\n", __func__);
            }
            if (FIELD_EX32(value, DPORT_SPI_CACHE, BUSY)) {
                qemu_log_mask(LOG_UNIMP, "%s: Unimplemented Cache busy operation" HWADDR_PRIx "\n", __func__);
            }
            if (FIELD_EX32(value, DPORT_SPI_CACHE, TARGET)) {
                qemu_log_mask(LOG_UNIMP, "%s: Unimplemented Cache target operation" HWADDR_PRIx "\n", __func__);
            }
            break;
        case A_DPORT_CTL:
            s->double_clk = FIELD_EX32(value, DPORT_CTL, DOUBLE_CLK);
            break;
        case A_DPORT_SPI_INTERRUPT_TYPE:
            // Read only, report error
            error_report("%s: Illegal DPORT register write: 0x%" HWADDR_PRIx "\n", __func__, addr);
            break;
        case A_DPORT_SPI_CACHE_TARGET:
            // TODO: Implement. No info found.
            break;
        case A_DPORT_IOSWAP:
            s->ioswap_uart = FIELD_EX32(value, DPORT_IOSWAP, UART);
            s->ioswap_spi = FIELD_EX32(value, DPORT_IOSWAP, SPI);   
            s->ioswap_uart0 = FIELD_EX32(value, DPORT_IOSWAP, UART0);
            s->ioswap_uart1 = FIELD_EX32(value, DPORT_IOSWAP, UART1);
            s->ioswap_hspi = FIELD_EX32(value, DPORT_IOSWAP, DOUBLE_HSPI);
            s->ioswap_cspi = FIELD_EX32(value, DPORT_IOSWAP, DOUBLE_CSPI);
            break;
    }
}

static const MemoryRegionOps esp8266_dport_ops = {
    .read = esp8266_dport_read,
    .write = esp8266_dport_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint32_t esp8266_cache_to_spi_addr(Esp8266CacheState *s, uint32_t addr)
{
    // Based on: https://github.com/espressif/ESP8266_RTOS_SDK/blob/d45071563cebe9ca520cbed2537dc840b4d6a1e6/components/spi_flash/src/spi_flash.c#L754
    uint32_t spi_addr = addr + 0x200000 * s->block;

    if (!s->is_2MB_block) {
        if (s->is_1MB_offset) {
            spi_addr += 0x100000;
        }
    }
    return spi_addr;
}

static uint64_t esp8266_cache_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266CacheRegionState *s = opaque; //ESP8266_CACHE_REGION(opaque);
    uint64_t ret = 0;
    if (s->cache->enabled) {
        uint32_t spi_addr = esp8266_cache_to_spi_addr(s->cache, addr);
        blk_pread(s->cache->dport->flash_blk, spi_addr, &ret, size);
        return ret;
    } else {
        error_report("%s: Cache not enabled. (Addr: 0x%" HWADDR_PRIx ")\n", __func__, addr);
        return 0;
    }
}

static void esp8266_cache_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    Esp8266CacheRegionState *s = opaque; //ESP8266_CACHE_REGION(opaque);
    if (s->cache->enabled) {
        uint32_t spi_addr = esp8266_cache_to_spi_addr(s->cache, addr);
        blk_pwrite(s->cache->dport->flash_blk, spi_addr, &value, size, 0); //TODO: Flags
    } else {
        error_report("%s: Cache not enabled. (Addr: 0x%" HWADDR_PRIx ")\n", __func__, addr);
    }
}

static const MemoryRegionOps esp8266_cache_ops = {
    .read = esp8266_cache_read,
    .write = esp8266_cache_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_cache_reset(Esp8266CacheState *cache)
{
    cache->enabled = false;
    cache->block = 0;
    cache->is_2MB_block = false;
    cache->is_1MB_offset = false;
    cache->target = 0;

    if (cache->dport->flash_blk) {
        memory_region_set_enabled(&cache->flash_cache.mem, true);
    }
}

static void esp8266_cache_init_region(Esp8266CacheState *cs,
                                    Esp8266CacheRegionState *crs,
                                    const char* name, hwaddr base) 
{
    crs->base = base;
    memory_region_init_io(&crs->mem, OBJECT(cs->dport), &esp8266_cache_ops, crs, "flash-cache", ESP8266_CACHE_SIZE);
                                    
}

static void esp8266_dport_reset(DeviceState *dev)
{
    Esp8266DportState *s = ESP8266_DPORT(dev);

    // Based on https://github.com/esp8266/Arduino/blob/da6ec83b5fdbd5b02f04cf143dcf8e158a8cfd36/doc/rgisters_dump.txt
    s->wdt_irq_enabled = 1;
    s->timer1_irq_enabled = 0;

    s->spi0_irq = 0;
    s->hspi_irq = 0;

    s->ioswap_uart = 0;
    s->ioswap_spi = 0;
    s->ioswap_uart0 = 0;
    s->ioswap_uart1 = 0;
    s->ioswap_hspi = 0;
    s->ioswap_cspi = 0;

    s->double_clk = 0;

    esp8266_cache_reset(&s->cache);
}

static void esp8266_dport_realize(DeviceState *dev, Error **errp)
{
    Esp8266DportState *s = ESP8266_DPORT(dev);
    //MachineState *ms = MACHINE(qdev_get_machine());

    esp8266_dport_reset(DEVICE(s));
}

static void esp8266_dport_init(Object *obj)
{
    Esp8266DportState *s = ESP8266_DPORT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->cache.dport = s;
    s->cache.flash_cache.cache = &s->cache;

    memory_region_init_io(&s->iomem, obj, &esp8266_dport_ops, s,
                          TYPE_ESP8266_DPORT, ESP8266_DPORT_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    esp8266_cache_init_region(&s->cache, &s->cache.flash_cache, "flash-cache", ESP8266_CACHE_BASE);


    /*qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_stall_req, ESP32_DPORT_APPCPU_STALL_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_reset_req, ESP32_DPORT_APPCPU_RESET_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->clk_update_req, ESP32_DPORT_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->cache_ill_irq, ESP32_DPORT_CACHE_ILL_IRQ_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->flash_enc_en_gpio, ESP32_DPORT_FLASH_ENC_EN_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->flash_dec_en_gpio, ESP32_DPORT_FLASH_DEC_EN_GPIO, 1);*/
}

static Property esp8266_dport_properties[] = {
    DEFINE_PROP_DRIVE("flash", Esp8266DportState, flash_blk),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp8266_dport_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp8266_dport_reset;
    dc->realize = esp8266_dport_realize;
    device_class_set_props(dc, esp8266_dport_properties);
}

static const TypeInfo esp8266_dport_info = {
    .name = TYPE_ESP8266_DPORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266DportState),
    .instance_init = esp8266_dport_init,
    .class_init = esp8266_dport_class_init
};

static void esp8266_dport_register_types(void)
{
    type_register_static(&esp8266_dport_info);
}

type_init(esp8266_dport_register_types)
