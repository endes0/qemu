#pragma once

#include "qemu/osdep.h"
#include "hw/adc/esp8266_adc.h"
#include "hw/boards.h"
#include "hw/char/esp32_uart.h"
#include "hw/gpio/esp8266_gpio.h"
#include "hw/hw.h"
#include "hw/i2c/esp8266_regi2c.h"
#include "hw/misc/esp8266_dport.h"
#include "hw/misc/esp8266_hdrf.h"
#include "hw/misc/esp8266_pm.h"
#include "hw/misc/esp8266_rtc_cntl.h"
#include "hw/nvram/esp8266_efuse.h"
#include "hw/ssi/esp32_spi.h"
#include "qemu-common.h"
#include "target/xtensa/cpu.h"

typedef struct Esp8266SocState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    XtensaCPU cpu;
    Esp8266DportState dport;
    ESP32UARTState uart[2];
    Esp8266GpioState gpio;
    Esp8266RtcCntlState rtc_cntl;
    Esp32SpiState spi[2];
    Esp8266HdrfState hdrf;
    Esp8266RegI2CState regi2c;
    Esp8266PmState pm;
    Esp8266AdcState adc;
    Esp8266EfuseState efuse;

    BusState rtc_bus;
    BusState periph_bus;

    MemoryRegion cpu_specific_mem;

    uint32_t requested_reset;
} Esp8266SocState;

struct Esp8266MachineState {
    MachineState parent;

    Esp8266SocState soc;
    DeviceState *flash_dev;
};

#define TYPE_ESP8266_MACHINE MACHINE_TYPE_NAME("esp8266")
OBJECT_DECLARE_SIMPLE_TYPE(Esp8266MachineState, ESP8266_MACHINE)
