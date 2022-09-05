#pragma once

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/misc/esp8266_dport.h"
#include "hw/misc/esp8266_rtc_cntl.h"
#include "hw/misc/esp8266_hdrf.h"
#include "hw/misc/esp8266_pm.h"
#include "hw/char/esp32_uart.h"
#include "hw/nvram/esp8266_efuse.h"
#include "hw/gpio/esp8266_gpio.h"
#include "hw/ssi/esp32_spi.h"
#include "hw/adc/esp8266_adc.h"
#include "target/xtensa/cpu.h"

typedef struct Esp8266SocState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    XtensaCPU cpu;
    Esp8266DportState dport;
    //Esp32IntMatrixState intmatrix;
    //Esp32CrosscoreInt crosscore_int;
    ESP32UARTState uart[2];
    Esp8266GpioState gpio;
    //Esp32RngState rng;
    Esp8266RtcCntlState rtc_cntl;
    //Esp32FrcTimerState frc_timer[ESP32_FRC_COUNT];
    //Esp32TimgState timg[ESP32_TIMG_COUNT];
    Esp32SpiState spi[2];
    Esp8266HdrfState hdrf;
    Esp8266PmState pm;
    Esp8266AdcState adc;
    //Esp32I2CState i2c[ESP32_I2C_COUNT];
    //Esp32ShaState sha;
    //Esp32RsaState rsa;
    Esp8266EfuseState efuse;
    //Esp32FlashEncryptionState flash_enc;
    //DWCSDMMCState sdmmc;
    //DeviceState *eth;

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