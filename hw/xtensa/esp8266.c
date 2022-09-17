/*
 * ESP8266 SoC and machine
 *
 * Copyright (c) 2022
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "exec/exec-all.h"
#include "hw/boards.h"
#include "hw/hw.h"
#include "hw/loader.h"
#include "hw/misc/esp8266_dport.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp8266.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "elf.h"
#include "qemu-common.h"

// Utils
static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

static qemu_irq get_cpu_irq(XtensaCPU *cpu, unsigned num)
{
    for (int int_index = 0; int_index < cpu->env.config->nextint; ++int_index) {
        if (cpu->env.config->extint[int_index] == num) {
            return xtensa_get_extints(&cpu->env)[int_index];
            break;
        }
    }
    return NULL;
}


// SoC
#define TYPE_ESP8266_SOC "xtensa.esp8266"
#define ESP8266_SOC(obj) OBJECT_CHECK(Esp8266SocState, (obj), TYPE_ESP8266_SOC)
#define TYPE_ESP8266_CPU XTENSA_CPU_TYPE_NAME("esp8266")

// See
// https://github.com/espressif/ESP8266_RTOS_SDK/blob/a192988b7906680440213eefc730d99dec0ce237/components/esp8266/include/esp8266/eagle_soc.h#L210
enum {
    ESP8266_MEMREGION_DRAM0,
    ESP8266_MEMREGION_IROM,
    ESP8266_MEMREGION_IRAM1,
    ESP8266_MEMREGION_ICACHE0,
    ESP8266_MEMREGION_ICACHE1,
    ESP8266_MEMREGION_FLASH_CACHE,
    // TODO: Those should be persisten on deep sleep (deep sleep is not
    // implemented yet)
    ESP8266_MEMREGION_RTC_SYS,
    ESP8266_MEMREGION_RTC_USER
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp8266_memmap[] = {
    [ESP8266_MEMREGION_DRAM0] = { 0x3ffE8000, (96 * 1024) },
    [ESP8266_MEMREGION_IROM] = { 0x40000000, 0x10000 },
    [ESP8266_MEMREGION_IRAM1] = { 0x40100000, 0x8000 },
    [ESP8266_MEMREGION_ICACHE0] = { 0x40108000, 0x4000 },
    [ESP8266_MEMREGION_ICACHE1] = { 0x4010C000, 0x4000 },
    [ESP8266_MEMREGION_FLASH_CACHE] = { 0x40200000, (1 * 1024 * 1024) },
    [ESP8266_MEMREGION_RTC_SYS] = { 0x60001000, 0x200 },
    [ESP8266_MEMREGION_RTC_USER] = { 0x60001200, 0x200 },
};

static void esp8266_soc_reset(DeviceState *dev)
{
    Esp8266SocState *s = ESP8266_SOC(dev);

    xtensa_select_static_vectors(&s->cpu.env, 1);
    cpu_reset(CPU(&s->cpu));

    device_cold_reset(DEVICE(&s->rtc_cntl));
    device_cold_reset(DEVICE(&s->gpio));
    for (int i = 0; i < 2; ++i) {
        device_cold_reset(DEVICE(&s->uart[i]));
    }
    for (int i = 0; i < 2; ++i) {
        device_cold_reset(DEVICE(&s->spi[i]));
    }
    device_cold_reset(DEVICE(&s->regi2c));
    device_cold_reset(DEVICE(&s->pm));
    device_cold_reset(DEVICE(&s->adc));
    device_cold_reset(DEVICE(&s->efuse));

    s->requested_reset = 0;
}

static void esp8266_soc_add_periph_device(MemoryRegion *dest, void *dev,
                                          hwaddr base_addr)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, base_addr, mr, 0);
}

static void esp8266_soc_realize(DeviceState *dev, Error **errp)
{
    Esp8266SocState *s = ESP8266_SOC(dev);

    // Init memory map
    const struct MemmapEntry *memmap = esp8266_memmap;
    MemoryRegion *sys_mem = get_system_memory();

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *icache0 = g_new(MemoryRegion, 1);
    MemoryRegion *icache1 = g_new(MemoryRegion, 1);
    MemoryRegion *flash_cache = g_new(MemoryRegion, 1);
    MemoryRegion *brom = g_new(MemoryRegion, 1);
    MemoryRegion *rtc_sys = g_new(MemoryRegion, 1);
    MemoryRegion *rtc_user = g_new(MemoryRegion, 1);

    memory_region_init_rom(brom, NULL, "esp8266.irom.cpu",
                           memmap[ESP8266_MEMREGION_IROM].size, &error_fatal);
    memory_region_add_subregion(&s->cpu_specific_mem,
                                memmap[ESP8266_MEMREGION_IROM].base, brom);

    memory_region_init_ram(dram, NULL, "esp8266.dram0",
                           memmap[ESP8266_MEMREGION_DRAM0].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP8266_MEMREGION_DRAM0].base,
                                dram);

    memory_region_init_ram(iram, NULL, "esp8266.iram1",
                           memmap[ESP8266_MEMREGION_IRAM1].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP8266_MEMREGION_IRAM1].base,
                                iram);

    memory_region_init_ram(icache0, NULL, "esp8266.icache0",
                           memmap[ESP8266_MEMREGION_ICACHE0].size,
                           &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP8266_MEMREGION_ICACHE0].base,
                                icache0);

    memory_region_init_ram(icache1, NULL, "esp8266.icache1",
                           memmap[ESP8266_MEMREGION_ICACHE1].size,
                           &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP8266_MEMREGION_ICACHE1].base,
                                icache1);

    memory_region_init_ram(flash_cache, NULL, "esp8266.flash_cache",
                           memmap[ESP8266_MEMREGION_FLASH_CACHE].size,
                           &error_fatal);
    memory_region_add_subregion(
        sys_mem, memmap[ESP8266_MEMREGION_FLASH_CACHE].base, flash_cache);

    memory_region_init_ram(rtc_sys, NULL, "esp8266.rtc_sys",
                           memmap[ESP8266_MEMREGION_RTC_SYS].size,
                           &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP8266_MEMREGION_RTC_SYS].base,
                                rtc_sys);

    memory_region_init_ram(rtc_user, NULL, "esp8266.rtc_user",
                           memmap[ESP8266_MEMREGION_RTC_USER].size,
                           &error_fatal);
    memory_region_add_subregion(
        sys_mem, memmap[ESP8266_MEMREGION_RTC_USER].base, rtc_user);


    // Init devices
    qdev_realize(DEVICE(&s->cpu), NULL, &error_fatal);

    qdev_realize(DEVICE(&s->dport), &s->periph_bus, &error_fatal);
    MemoryRegion *dport_mem =
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dport), 0);

    memory_region_add_subregion(sys_mem, DR_REG_DPORT_BASE, dport_mem);

    if (s->dport.flash_blk) {
        Esp8266CacheRegionState *flash_cache = &(s->dport.cache.flash_cache);
        memory_region_add_subregion_overlap(sys_mem,
                                            flash_cache->base,
                                            &flash_cache->mem, 0);
    }

    // Device: RTC
    qdev_realize(DEVICE(&s->rtc_cntl), &s->rtc_bus, &error_fatal);
    esp8266_soc_add_periph_device(sys_mem, &s->rtc_cntl, DR_REG_RTC_BASE);

    // Device: GPIO
    qdev_realize(DEVICE(&s->gpio), &s->periph_bus, &error_fatal);
    esp8266_soc_add_periph_device(sys_mem, &s->gpio, DR_REG_GPIO_BASE);

    // Device: UART
    for (int i = 0; i < 2; ++i) {
        const hwaddr uart_base[] = { DR_REG_UART0_BASE, DR_REG_UART1_BASE };
        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        esp8266_soc_add_periph_device(sys_mem, &s->uart[i], uart_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           get_cpu_irq(&s->cpu, ETS_GPIO_INUM));
    }

    // Device: SPI
    for (int i = 0; i < 2; ++i) {
        const hwaddr spi_base[] = { DR_REG_SPI0_BASE, DR_REG_SPI1_BASE };
        qdev_realize(DEVICE(&s->spi[i]), &s->periph_bus, &error_fatal);

        esp8266_soc_add_periph_device(sys_mem, &s->spi[i], spi_base[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi[i]), 0,
                           get_cpu_irq(&s->cpu, ETS_SPI_INUM));
    }

    // Device: HDRF
    qdev_realize(DEVICE(&s->hdrf), &s->periph_bus, &error_fatal);
    esp8266_soc_add_periph_device(sys_mem, &s->hdrf, DR_REG_HDRF_BASE);

    // Device: Internal i2c bus
    qdev_realize(DEVICE(&s->regi2c), &s->periph_bus, &error_fatal);
    esp8266_soc_add_periph_device(sys_mem, &s->regi2c, DR_REG_REGI2C_BASE);

    // Device: PM
    qdev_realize(DEVICE(&s->pm), &s->periph_bus, &error_fatal);
    esp8266_soc_add_periph_device(sys_mem, &s->pm, DR_REG_PM_BASE);

    // Device: ADC
    qdev_realize(DEVICE(&s->adc), &s->periph_bus, &error_fatal);
    esp8266_soc_add_periph_device(sys_mem, &s->adc, DR_REG_SAR_BASE);

    // Device: eFuse
    qdev_realize(DEVICE(&s->efuse), &s->periph_bus, &error_fatal);
    esp8266_soc_add_periph_device(sys_mem, &s->efuse, DR_REG_EFUSE_BASE);


    create_unimplemented_device("esp8266.RNG", DR_REG_RNG_BASE, 0x1);
    create_unimplemented_device("esp8266.GPIO", DR_REG_GPIO_BASE, 0x300);
    create_unimplemented_device("esp8266.timer", DR_REG_TIMER_BASE, 0x100);
    create_unimplemented_device("esp8266.RTC", DR_REG_RTC_BASE, 0x100);
    create_unimplemented_device("esp8266.IOMUX", DR_REG_IO_MUX_BASE, 0x100);
    create_unimplemented_device("esp8266.watchdog", DR_REG_WATCHDOG_BASE,
                                0x200);
    create_unimplemented_device("esp8266.SLC", DR_REG_SLC_BASE, 0x200);
    create_unimplemented_device("esp8266.I2S", DR_REG_I2S_BASE, 0x100);
    create_unimplemented_device("esp8266.WIFI", DR_REG_WIFI_BASE, 0x800);
    create_unimplemented_device("esp8266.WDEV", DR_REG_WDEV_BASE, 0x1800);


    qemu_register_reset((QEMUResetHandler *)esp8266_soc_reset, dev);
}

static void esp8266_soc_init(Object *obj)
{
    Esp8266SocState *s = ESP8266_SOC(obj);
    // MachineState *ms = MACHINE(qdev_get_machine());
    char name[16];

    MemoryRegion *system_memory = get_system_memory();

    qbus_init(&s->periph_bus, sizeof(s->periph_bus), TYPE_SYSTEM_BUS, DEVICE(s),
              "esp8266-periph-bus");
    qbus_init(&s->rtc_bus, sizeof(s->rtc_bus), TYPE_SYSTEM_BUS, DEVICE(s),
              "esp8266-rtc-bus");

    object_initialize_child(obj, "cpu", &s->cpu, TYPE_ESP8266_CPU);

    s->cpu.env.sregs[PRID] = 0; // TODO: get the correct value

    memory_region_init(&s->cpu_specific_mem, NULL, "cpu-mem", UINT32_MAX);

    CPUState *cs = CPU(&s->cpu);
    cs->num_ases = 1;
    cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem);

    MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
    memory_region_init_alias(cpu_view_sysmem, NULL, "cpu-sysmem", system_memory,
                             0, UINT32_MAX);
    memory_region_add_subregion_overlap(&s->cpu_specific_mem, 0,
                                        cpu_view_sysmem, 0);
    cs->memory = &s->cpu_specific_mem;

    for (int i = 0; i < 2; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32_UART);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart[0]), "chardev");
    object_property_add_alias(obj, "serial1", OBJECT(&s->uart[1]), "chardev");
    // object_property_add_alias(obj, "serial2", OBJECT(&s->uart[2]),
    // "chardev");

    object_initialize_child(obj, "gpio", &s->gpio, TYPE_ESP8266_GPIO);

    object_initialize_child(obj, "dport", &s->dport, TYPE_ESP8266_DPORT);

    // object_initialize_child(obj, "intmatrix", &s->intmatrix,
    // TYPE_ESP32_INTMATRIX);

    // object_initialize_child(obj, "crosscore_int", &s->crosscore_int,
    // TYPE_ESP32_CROSSCORE_INT);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl,
                            TYPE_ESP8266_RTC_CNTL);

    // for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
    //     snprintf(name, sizeof(name), "frc%d", i);
    //     object_initialize_child(obj, name, &s->frc_timer[i],
    //     TYPE_ESP32_FRC_TIMER);
    // }

    // for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
    //     snprintf(name, sizeof(name), "timg%d", i);
    //     object_initialize_child(obj, name, &s->timg[i], TYPE_ESP32_TIMG);
    // }

    for (int i = 0; i < 2; ++i) {
        snprintf(name, sizeof(name), "spi%d", i);
        object_initialize_child(obj, name, &s->spi[i], TYPE_ESP32_SPI);
        object_property_set_bool(OBJECT(&s->spi[i]), "is_esp8266", true,
                                 &error_abort);
    }

    // for (int i = 0; i < ESP32_I2C_COUNT; ++i) {
    //     snprintf(name, sizeof(name), "i2c%d", i);
    //     object_initialize_child(obj, name, &s->i2c[i], TYPE_ESP32_I2C);
    // }

    // object_initialize_child(obj, "rng", &s->rng, TYPE_ESP32_RNG);

    // object_initialize_child(obj, "sha", &s->sha, TYPE_ESP32_SHA);

    // object_initialize_child(obj, "rsa", &s->rsa, TYPE_ESP32_RSA);

    object_initialize_child(obj, "hdrf", &s->hdrf, TYPE_ESP8266_HDRF);

    object_initialize_child(obj, "regi2c", &s->regi2c, TYPE_ESP8266_REGI2C);

    object_initialize_child(obj, "pm", &s->pm, TYPE_ESP8266_PM);

    object_initialize_child(obj, "adc", &s->adc, TYPE_ESP8266_ADC);

    object_initialize_child(obj, "efuse", &s->efuse, TYPE_ESP8266_EFUSE);

    // object_initialize_child(obj, "flash_enc", &s->flash_enc,
    // TYPE_ESP32_FLASH_ENCRYPTION);

    // object_initialize_child(obj, "sdmmc", &s->sdmmc, TYPE_DWC_SDMMC);

    // qdev_init_gpio_in_named(DEVICE(s), esp32_dig_reset,
    // ESP32_RTC_DIG_RESET_GPIO, 1); qdev_init_gpio_in_named(DEVICE(s),
    // esp32_cpu_reset, ESP32_RTC_CPU_RESET_GPIO, ESP32_CPU_COUNT);
    // qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_stall,
    // ESP32_RTC_CPU_STALL_GPIO, ESP32_CPU_COUNT);
    // qdev_init_gpio_in_named(DEVICE(s), esp32_clk_update,
    // ESP32_RTC_CLK_UPDATE_GPIO, 1); qdev_init_gpio_in_named(DEVICE(s),
    // esp32_timg_cpu_reset, ESP32_TIMG_WDT_CPU_RESET_GPIO, 2);
    // qdev_init_gpio_in_named(DEVICE(s), esp32_timg_sys_reset,
    // ESP32_TIMG_WDT_SYS_RESET_GPIO, 2);
}

static Property esp8266_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp8266_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp8266_soc_realize;
    device_class_set_props(dc, esp8266_soc_properties);
}

static const TypeInfo esp8266_soc_info = { .name = TYPE_ESP8266_SOC,
                                           .parent = TYPE_DEVICE,
                                           .instance_size =
                                               sizeof(Esp8266SocState),
                                           .instance_init = esp8266_soc_init,
                                           .class_init =
                                               esp8266_soc_class_init };

static void esp8266_soc_register_types(void)
{
    type_register_static(&esp8266_soc_info);
}

type_init(esp8266_soc_register_types)


// Machine
#define TYPE_ESP8266_MACHINE MACHINE_TYPE_NAME("esp8266")

    static void esp8266_machine_init_spi_flash(Esp8266SocState *ss,
                                               BlockBackend *blk)
{
    /* "main" flash chip is attached to SPI0, CS0 */
    DeviceState *spi_master = DEVICE(&ss->spi[0]);
    BusState *spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *flash_dev = qdev_new("gd25q32"); // sst25vf032b gd25q32
    qdev_prop_set_drive(flash_dev, "drive", blk);
    qdev_realize_and_unref(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(
        spi_master, SSI_GPIO_CS, 0,
        qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}

static void esp8266_machine_init(MachineState *machine)
{
    // Object creation
    Esp8266MachineState *ms = ESP8266_MACHINE(machine);
    object_initialize_child(OBJECT(ms), "soc", &ms->soc, TYPE_ESP8266_SOC);
    Esp8266SocState *ss = ESP8266_SOC(&ms->soc);

    // Devices creation
    BlockBackend *blk = NULL;
    DriveInfo *dinfo = drive_get_next(IF_MTD);
    if (dinfo) {
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    if (blk) {
        ss->dport.flash_blk = blk;
        esp8266_machine_init_spi_flash(ss, blk);
    }
    qdev_prop_set_chr(DEVICE(ss), "serial0", serial_hd(0));
    qdev_prop_set_chr(DEVICE(ss), "serial1", serial_hd(1));
    // qdev_prop_set_chr(DEVICE(ss), "serial2", serial_hd(2));
    // if (machine->ram_size > 0) {
    //     qdev_prop_set_bit(DEVICE(&ss->dport), "has_psram", true);
    // }

    qdev_realize(DEVICE(ss), NULL, &error_fatal);


    // if (machine->ram_size > 0) {
    //     esp8266_machine_init_psram(ss, (uint32_t) (machine->ram_size / MiB));
    // }

    // esp32_machine_init_i2c(ss);

    // esp32_machine_init_openeth(ss);

    // esp32_machine_init_sd(ss);

    // Program loading
    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&ss->cpu));

    const char *load_elf_filename = NULL;
    if (machine->firmware) {
        // TODO: check if firmware is an elf or a binary
        load_elf_filename = machine->firmware;
    }
    if (machine->kernel_filename) {
        qemu_log("Warning: both -bios and -kernel arguments specified. Only "
                 "loading the the -kernel file.\n");
        load_elf_filename = machine->kernel_filename;
    }

    if (load_elf_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        // FIX: not working (failing to load)
        int success = load_elf_as(load_elf_filename, NULL, translate_phys_addr,
                                  &ss->cpu, &elf_entry, &elf_lowaddr, NULL,
                                  NULL, 0, EM_XTENSA, 0, 0, CPU(&ss->cpu)->as);
        if (success > 0) {
            ss->cpu.env.pc = elf_entry;
        } else {
            error_report("Error: loading elf specified in -bios");
            exit(1);
        }

    } else {
        char *rom_binary =
            qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp8266-rom.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary "
                         "not found (1)");
            exit(1);
        }

        int size = load_image_targphys_as(
            rom_binary, esp8266_memmap[ESP8266_MEMREGION_IROM].base,
            esp8266_memmap[ESP8266_MEMREGION_IROM].size, CPU(&ss->cpu)->as);
        if (size < 0) {
            error_report("Error: could not load ROM binary '%s'", rom_binary);
            exit(1);
        }
        g_free(rom_binary);
    }
}


/* Initialize machine type */
static void esp8266_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP8266 machine";
    mc->init = esp8266_machine_init;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
    mc->default_ram_size = 0;
    // TODO
    //  mc->fixup_ram_size = esp32_fixup_ram_size;
}

static const TypeInfo esp8266_info = {
    .name = TYPE_ESP8266_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp8266MachineState),
    .class_init = esp8266_machine_class_init,
};

static void esp8266_machine_type_init(void)
{
    type_register_static(&esp8266_info);
}

type_init(esp8266_machine_type_init);
