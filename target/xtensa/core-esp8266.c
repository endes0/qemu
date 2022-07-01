#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-esp8266/core-isa.h"
#include "core-esp8266/core-matmap.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_esp8266
#include "core-esp8266/xtensa-modules.c.inc"

static XtensaConfig esp8266 __attribute__((unused)) = {
    .name = "esp8266",
    .gdb_regmap = {
        .reg = {
#include "core-esp8266/gdb-config.c.inc"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(esp8266)
