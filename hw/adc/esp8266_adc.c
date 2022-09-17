
#include "qemu/osdep.h"
#include "hw/adc/esp8266_adc.h"
#include "hw/hw.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "trace.h"


#define CORRECTION_FACTOR 11 / 12
#define DEFAULT_VDD 3.3
#define DEFAULT_TOUT 0.0

static uint16_t esp8266_adc_to_reg_value(uint16_t value)
{
    uint16_t x = value & 0x700;
    uint16_t y = value & 0x0FF;
    if (y > 0) {
        x += (y / 279) << 8;
    }

    return ~(x + 21);
}

static uint64_t esp8266_adc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266AdcState *s = ESP8266_ADC(opaque);
    uint32_t r = 0;

    switch (addr) {
    case A_SAR_CFG:
        r = FIELD_DP32(r, SAR_CFG, ADC_ENABLE, s->enable);
        r = FIELD_DP32(r, SAR_CFG, START, 1);
        r = FIELD_DP32(r, SAR_CFG, READS, s->reads);
        r = FIELD_DP32(r, SAR_CFG, UNK, 0);
        r = FIELD_DP32(r, SAR_CFG, CLK_DIV, 0); // TODO: implement
        r = FIELD_DP32(r, SAR_CFG, READY_STATE, 0);
        break;
    case A_SAR_CFG1:
        r = FIELD_DP32(r, SAR_CFG1, CLK_DIV, 0); // TODO: implement
        r = FIELD_DP32(r, SAR_CFG1, READ_TOUT, s->tout_mode);
        r = FIELD_DP32(r, SAR_CFG1, READ_VDD, s->vdd_mode);
        break;
    case A_SAR_W0 ... A_SAR_W7:
        if (s->enable) {
            if (s->vdd_mode) {
                // TODO: custom VDD
                // min 0,001 max
                r = esp8266_adc_to_reg_value(((DEFAULT_VDD * 512) - 1 / 4) *
                                             CORRECTION_FACTOR);
            } else if (s->tout_mode) {
                // TODO: custom TOUT
                //  min 0 v == r=0
                //  max 1 v == r=2047
                r = esp8266_adc_to_reg_value(((DEFAULT_TOUT * 2047)) *
                                             CORRECTION_FACTOR);
                ;
            } else {
                // TODO: implement
                qemu_log_mask(LOG_UNIMP, "%s: Unimplemented ADC mode",
                              __func__);
            }

            if (s->reads < (addr - A_SAR_W0) / 4) {
                qemu_log_mask(
                    LOG_GUEST_ERROR,
                    "%s: read from an output register (0x%" HWADDR_PRIx
                    ") that exceeds the adc dout window (%u reads)\n",
                    __func__, addr, s->reads);
            }
        } else {
            // TODO: implement
            qemu_log_mask(
                LOG_UNIMP,
                "%s: Unimplemented read from ADC when it's disabled\n",
                __func__);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read from 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_esp8266_adc_read(addr, r);
    return r;
}

static void esp8266_adc_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned int size)
{
    Esp8266AdcState *s = ESP8266_ADC(opaque);

    switch (addr) {
    case A_SAR_CFG:
        s->enable = FIELD_EX32(value, SAR_CFG, ADC_ENABLE);
        s->reads = FIELD_EX32(value, SAR_CFG, READS);
        if (FIELD_EX32(value, SAR_CFG, START) == 1) {
            trace_esp8266_adc_start_sampling(s->reads);
        }
        break;
    case A_SAR_CFG1:
        s->tout_mode = FIELD_EX32(value, SAR_CFG1, READ_TOUT);
        s->vdd_mode = FIELD_EX32(value, SAR_CFG1, READ_VDD);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write to 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_esp8266_adc_write(addr, value);
}

static const MemoryRegionOps esp8266_adc_ops = {
    .read = esp8266_adc_read,
    .write = esp8266_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp8266_adc_init(Object *obj)
{
    Esp8266AdcState *s = ESP8266_ADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp8266_adc_ops, s, TYPE_ESP8266_ADC,
                          0xB0);
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
