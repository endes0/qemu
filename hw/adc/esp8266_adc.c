
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/adc/esp8266_adc.h"


static uint64_t esp8266_adc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266AdcState *s = ESP8266_ADC(opaque);
    uint32_t r = 0;

    switch (addr) {
    case A_TOS_CAL_DATA:
        //TODO: implement
        r = s->calibration_reg;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read from 0x%" HWADDR_PRIx "\n",
                  __func__, addr);
        break;
    }
    
    return r;
}

static void esp8266_adc_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp8266AdcState *s = ESP8266_ADC(opaque);

    switch (addr) {
    case A_TOS_CAL_DATA:
        s->calibration_reg = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented write to 0x%" HWADDR_PRIx "\n",
                  __func__, addr);
        break;
    }
}

static const MemoryRegionOps esp8266_adc_ops = {
    .read =  esp8266_adc_read,
    .write = esp8266_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp8266_adc_init(Object *obj)
{
    Esp8266AdcState *s = ESP8266_ADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp8266_adc_ops, s,
                          TYPE_ESP8266_ADC, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}


static const TypeInfo esp8266_adc_info = {
    .name = TYPE_ESP8266_ADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266AdcState),
    .instance_init = esp8266_adc_init,
};

static void esp8266_adc_register_types(void)
{
    type_register_static(&esp8266_adc_info);
}

type_init(esp8266_adc_register_types)
