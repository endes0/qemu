#include "qemu/osdep.h"
#include "hw/i2c/esp8266_regi2c.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

static void esp8266_i2c_reset(DeviceState *dev)
{
    Esp8266RegI2CState *s = ESP8266_REGI2C(dev);

    for (size_t i = 0; i < ESP8266_REGI2C_HOSTS; i++) {
        s->trans_done[i] = 0;
        s->data[i] = 0;
        s->block[i] = 0;
        s->reg[i] = 0;
    }
}

static uint64_t esp8266_i2c_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266RegI2CState *s = ESP8266_REGI2C(opaque);
    uint32_t res = 0;

    switch (addr) {
    case A_HOST_0 ... A_HOST_0 + 4 * (ESP8266_REGI2C_HOSTS - 1):
        res = FIELD_DP32(res, HOST_0, DATA, s->data[(addr - A_HOST_0) / 4]);
        res = FIELD_DP32(res, HOST_0, BLOCK, s->block[(addr - A_HOST_0) / 4]);
        res = FIELD_DP32(res, HOST_0, REG, s->reg[(addr - A_HOST_0) / 4]);
        res = FIELD_DP32(res, HOST_0, TRANSMISSION_DONE,
                         s->trans_done[(addr - A_HOST_0) / 4]);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: Unimplemented register read 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    return res;
}

static void esp8266_i2c_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned int size)
{
    Esp8266RegI2CState *s = ESP8266_REGI2C(opaque);

    switch (addr) {
    case A_HOST_0 ... A_HOST_0 + 4 * (ESP8266_REGI2C_HOSTS - 1):
        s->data[(addr - A_HOST_0) / 4] = FIELD_EX32(value, HOST_0, DATA);
        s->block[(addr - A_HOST_0) / 4] = FIELD_EX32(value, HOST_0, BLOCK);
        s->reg[(addr - A_HOST_0) / 4] = FIELD_EX32(value, HOST_0, REG);
        s->trans_done[(addr - A_HOST_0) / 4] = false;
        if (!FIELD_EX32(value, HOST_0, WRITE)) {
            i2c_start_recv(s->bus, s->block[(addr - A_HOST_0) / 4]);
            i2c_send(s->bus, s->reg[(addr - A_HOST_0) / 4]);
            s->data[(addr - A_HOST_0) / 4] = i2c_recv(s->bus);
            i2c_end_transfer(s->bus);
            trace_esp8266_i2c_recive(s->block[(addr - A_HOST_0) / 4],
                                     s->reg[(addr - A_HOST_0) / 4]);
        } else {
            i2c_start_send(s->bus, s->block[(addr - A_HOST_0) / 4]);
            i2c_send(s->bus, s->reg[(addr - A_HOST_0) / 4]);
            i2c_send(s->bus, s->data[(addr - A_HOST_0) / 4]);
            i2c_end_transfer(s->bus);
            trace_esp8266_i2c_send(s->block[(addr - A_HOST_0) / 4],
                                   s->reg[(addr - A_HOST_0) / 4],
                                   s->data[(addr - A_HOST_0) / 4]);
        }
        s->trans_done[(addr - A_HOST_0) / 4] = true;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps esp8266_i2c_ops = {
    .read = esp8266_i2c_read,
    .write = esp8266_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp8266_i2c_init(Object *obj)
{
    Esp8266RegI2CState *s = ESP8266_REGI2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp8266_i2c_ops, s,
                          TYPE_ESP8266_REGI2C, ESP8266_REGI2C_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    s->bus = i2c_init_bus(DEVICE(s), "i2c");
}

static void esp8266_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = esp8266_i2c_reset;
}

static const TypeInfo esp8266_i2c_type_info = {
    .name = TYPE_ESP8266_REGI2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266RegI2CState),
    .instance_init = esp8266_i2c_init,
    .class_init = esp8266_i2c_class_init,
};

static void esp8266_i2c_register_types(void)
{
    type_register_static(&esp8266_i2c_type_info);
}

type_init(esp8266_i2c_register_types)