#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#include "hw/misc/esp8266_reg.h"
#define TYPE_ESP8266_RTC_CNTL "misc.esp8266.rtc_cntl"
#define ESP8266_RTC_CNTL(obj) OBJECT_CHECK(Esp8266RtcCntlState, (obj), TYPE_ESP8266_RTC_CNTL)

// From: https://github.com/espressif/ESP8266_RTOS_SDK/blob/d45071563cebe9ca520cbed2537dc840b4d6a1e6/components/esp8266/include/esp8266/rtc_register.h
typedef enum Esp8266ResetCause {
    ESP8266_NO_MEAN                =  0,
    ESP8266_POWERON_RESET          =  1,    /**<1, Vbat power on reset*/
    ESP8266_EXT_RESET              =  2,    /**<2, external system reset*/
    ESP8266_SW_RESET               =  3,    /**<3, Software reset digital core*/
    ESP8266_OWDT_RESET             =  4,    /**<4, Legacy watch dog reset digital core*/
    ESP8266_DEEPSLEEP_RESET        =  5,    /**<5, Deep Sleep reset digital core*/
    ESP8266_SDIO_RESET             =  6,    /**<6, Reset by SLC module, reset digital core*/
} Esp8266ResetCause;

typedef struct Esp8266RtcCntlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    Esp8266ResetCause reset_cause;
} Esp8266RtcCntlState;

REG32(RTC_SLEEP_TARGET, 0x4)
REG32(RTC_UNK_REG, 0x8) // Unknown register. The bootrom put low (if high) the bit 21 after reading RTC_STATE2.
REG32(RTC_STATE1, 0x14)
    FIELD(RTC_STATE1, HW_RESET_CAUSE, 0, 3)
REG32(RTC_STATE2, 0x18)
    FIELD(RTC_STATE2, HW_WAKEUP_CAUSE, 8, 5)
REG32(RTC_SLEEP_VALUE, 0x1c)
REG32(RTC_INT_STATUS, 0x20)
REG32(RTC_INT_CLEAR, 0x24)
REG32(RTC_INT_ENABLE, 0x28)
REG32(RTC_STORE0, 0x30)
REG32(RTC_STORE1, 0x34)
REG32(RTC_STORE2, 0x38)
REG32(RTC_STORE3, 0x3c)


// GPIO 16 (WAKE) regsiters
REG32(RTC_GPIO_OUT, 0x68)
REG32(RTC_GPIO_ENABLE, 0x74)
REG32(RTC_GPIO_IN_DATA, 0x8c)
REG32(RTC_GPIO_CONF, 0x90)
REG32(PAD_XPD_DCDC_CONF, 0xA0)

#define ESP8266_RTC_CNTL_SIZE (A_PAD_XPD_DCDC_CONF + 4)
