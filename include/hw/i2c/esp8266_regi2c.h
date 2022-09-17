#ifndef ESP8266_REGI2C_H
#define ESP8266_REGI2C_H

#include "hw/i2c/i2c.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"

#define TYPE_ESP8266_REGI2C "esp8266.regi2c"
#define ESP8266_REGI2C(obj) \
    OBJECT_CHECK(Esp8266RegI2CState, (obj), TYPE_ESP8266_REGI2C)

#define ESP8266_REGI2C_HOSTS 5
#define ESP8266_REGI2C_MEM_SIZE 0x40

typedef struct Esp8266RegI2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;

    bool trans_done[ESP8266_REGI2C_HOSTS];
    uint8_t data[ESP8266_REGI2C_HOSTS];
    uint8_t block[ESP8266_REGI2C_HOSTS];
    uint8_t reg[ESP8266_REGI2C_HOSTS];

} Esp8266RegI2CState;

REG32(HOST_0, 0x0);
FIELD(HOST_0, BLOCK, 0, 8);
FIELD(HOST_0, REG, 7, 8);
FIELD(HOST_0, DATA, 15, 8);
FIELD(HOST_0, WRITE, 24, 1);
FIELD(HOST_0, TRANSMISSION_DONE, 25, 1);

#endif /* ESP8266_REGI2C_H */