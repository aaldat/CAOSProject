/*
 * QEMU Machine Emulation for NXP S32K3X8EVB (Cortex-M7)
 * UART e memoria su QEMU 7.2.0
 */

#include "qemu/osdep.h"
#include "qemu/typedefs.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"

#include "hw/char/stm32l4x5_usart.h"

#include "hw/char/serial.h"
#include "hw/qdev-properties.h"

#include "hw/core/split-irq.h"
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/arm/boot.h"
#include "hw/arm/armv7m.h"
#include "sysemu/sysemu.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define KiB                1024ULL
#define MiB               (1024ULL * KiB)

#define FLASH_SIZE         (8 * MiB)
#define FLASH_OFFSET       0x00400000ULL

#define RAM_SIZE           (768 * KiB)
#define RAM_OFFSET         0x20400000ULL

#define ITCM_SIZE          (64 * KiB)
#define ITCM_OFFSET        0x00000000ULL

#define DFLASH_SIZE        (128 * KiB)
#define DFLASH_OFFSET      0x10000000ULL

#define DTCM_SIZE          (128 * KiB)
#define DTCM_OFFSET        0x20000000ULL

#define UART_BASE_ADDR     0x4006A000ULL

#define FLASH_BLOCK0_BASE_ADDR FLASH_OFFSET
#define FLASH_BLOCK0_SIZE      (2 * MiB)
#define FLASH_BLOCK1_SIZE      (2 * MiB)
#define FLASH_BLOCK2_SIZE      (2 * MiB)
#define FLASH_BLOCK3_SIZE      (2 * MiB)

/*================================================================
 * System Controller (ssys_state)
 *================================================================*/
#define TYPE_S32K3X8EVB_SYS "s32k3x8evb-sys"
OBJECT_DECLARE_SIMPLE_TYPE(ssys_state, S32K3X8EVB_SYS);

typedef struct ssys_state {
    SysBusDevice parent_obj;
    Clock *sysclk;
    Clock *refclk;
    Clock *aips_plat_clk;
    Clock *aips_slow_clk;
} ssys_state;

static void s32k3x8evb_sys_class_init(ObjectClass *oc, void *data)
{
    /* Nessuna proprietà aggiuntiva */
}

static const TypeInfo s32k3x8evb_sys_info = {
    .name          = TYPE_S32K3X8EVB_SYS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ssys_state),
    .class_init    = s32k3x8evb_sys_class_init,
};

/*================================================================
 * SoC Memory Device (S32K3State)
 *================================================================*/
#define TYPE_S32K3 "s32k3"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3State, S32K3);

typedef struct S32K3State {
    SysBusDevice parent_obj;
    MemoryRegion flash;
    MemoryRegion ram;
    MemoryRegion itcm;
    MemoryRegion dflash;
    MemoryRegion dtcm;
} S32K3State;

typedef struct S32K3Class {
    DeviceClass parent_class;
    uint32_t flash_size;
    uint32_t ram_size;
} S32K3Class;

#define S32K3_GET_CLASS(obj) \
    OBJECT_GET_CLASS(S32K3Class, obj, TYPE_S32K3)

static void s32k3_memory(DeviceState *dev, Error **errp)
{
    S32K3State *s = S32K3(dev);
    S32K3Class *c = S32K3_GET_CLASS(dev);

    memory_region_init_rom(&s->flash, OBJECT(dev),
                           "s32k3.flash", c->flash_size, errp);
    memory_region_add_subregion(get_system_memory(),
                                FLASH_OFFSET, &s->flash);

    memory_region_init_ram(&s->ram, NULL,
                           "s32k3.ram", c->ram_size, errp);
    memory_region_add_subregion(get_system_memory(),
                                RAM_OFFSET, &s->ram);

    memory_region_init_ram(&s->itcm, NULL,
                           "s32k3.itcm", ITCM_SIZE, errp);
    memory_region_add_subregion(get_system_memory(),
                                ITCM_OFFSET, &s->itcm);

    memory_region_init_rom(&s->dflash, OBJECT(dev),
                           "s32k3.dflash", DFLASH_SIZE, errp);
    memory_region_add_subregion(get_system_memory(),
                                DFLASH_OFFSET, &s->dflash);

    memory_region_init_ram(&s->dtcm, NULL,
                           "s32k3.dtcm", DTCM_SIZE, errp);
    memory_region_add_subregion(get_system_memory(),
                                DTCM_OFFSET, &s->dtcm);
}

static void s32k3_class_init(ObjectClass *klass, void *data)
{
    S32K3Class *k = S32K3_GET_CLASS(klass);
    k->flash_size = FLASH_SIZE;
    k->ram_size   = RAM_SIZE;
}

static const TypeInfo s32k3_info = {
    .name          = TYPE_S32K3,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(S32K3State),
    .class_size    = sizeof(S32K3Class),
    .class_init    = s32k3_class_init,
};

/*================================================================
 * Machine State
 *================================================================*/
typedef struct S32K3X8MachineState {
    MachineState *parent_obj;
    ssys_state sys;
} S32K3X8MachineState;

#define TYPE_S32K3X8_MACHINE MACHINE_TYPE_NAME("s32k3x8evb")
DECLARE_INSTANCE_CHECKER(S32K3X8MachineState, S32K3X8_MACHINE, TYPE_S32K3X8_MACHINE);

/* UART Initialization */
static void initialize_lpuarts(S32K3X8MachineState *m_state,
                               DeviceState *nvic, int num_lpuarts)
{
    for (int i = 0; i < num_lpuarts; i++) {
        DeviceState *lpuart = qdev_new(TYPE_STM32L4X5_LPUART);
        qdev_prop_set_chr(lpuart, "chardev", serial_hd(i));

        if (i == 0 || i == 1 || i == 8) {
            qdev_connect_clock_in(lpuart, "clk",
                                  m_state->sys.aips_plat_clk);
        } else {
            qdev_connect_clock_in(lpuart, "clk",
                                  m_state->sys.aips_slow_clk);
        }

        sysbus_realize_and_unref(SYS_BUS_DEVICE(lpuart), &error_fatal);

        hwaddr base = UART_BASE_ADDR + (i * 0x1000);
        sysbus_mmio_map(SYS_BUS_DEVICE(lpuart), 0, base);
        sysbus_connect_irq(SYS_BUS_DEVICE(lpuart), 0,
                           qdev_get_gpio_in(nvic, i));
    }
}

/* Machine Initialization */
static void s32k3x8_init(MachineState *ms)
{
    S32K3X8MachineState *m = g_new0(S32K3X8MachineState, 1);
    m->parent_obj = ms;

    /* SoC container */
    Object *soc = object_new("container");
    object_property_add_child(OBJECT(ms), "soc", soc);

    /* System controller */
    object_initialize_child(OBJECT(ms), "sys", &m->sys,
                            TYPE_S32K3X8EVB_SYS);
    sysbus_realize(SYS_BUS_DEVICE(&m->sys), &error_abort);

    /* Memoria SoC */
    DeviceState *mem = qdev_new(TYPE_S32K3);
    object_property_add_child(soc, "mem", OBJECT(mem));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mem), &error_fatal);
    s32k3_memory(mem, &error_fatal);

    /* Clocks */
    m->sys.sysclk        = clock_new(OBJECT(DEVICE(&m->sys)), "sysclk");
    clock_set_ns(m->sys.sysclk, 4.16);

    m->sys.refclk        = clock_new(OBJECT(DEVICE(&m->sys)), "refclk");
    clock_set_hz(m->sys.refclk, 1000000);

    m->sys.aips_plat_clk = clock_new(OBJECT(DEVICE(&m->sys)), "aips_plat_clk");
    clock_set_hz(m->sys.aips_plat_clk, 80000000);

    m->sys.aips_slow_clk = clock_new(OBJECT(DEVICE(&m->sys)), "aips_slow_clk");
    clock_set_hz(m->sys.aips_slow_clk, 40000000);

    /* NVIC / Cortex-M7 */
    DeviceState *nvic = qdev_new(TYPE_ARMV7M);
    object_property_add_child(soc, "nvic", OBJECT(nvic));
    qdev_prop_set_uint32(nvic, "num-irq",       256);
    qdev_prop_set_uint8 (nvic, "num-prio-bits", 4);
    qdev_connect_clock_in(nvic, "cpuclk", m->sys.sysclk);
    qdev_connect_clock_in(nvic, "refclk", m->sys.refclk);
    qdev_prop_set_string(nvic, "cpu-type",      ms->cpu_type);
    qdev_prop_set_bit   (nvic, "enable-bitband", true);
    object_property_set_link(OBJECT(nvic), "memory",
                             OBJECT(get_system_memory()),
                             &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(nvic), &error_fatal);

    /* UARTs */
    initialize_lpuarts(m, nvic, 16);

    /* Carica firmware in FLASH */
    armv7m_load_kernel(ARM_CPU(first_cpu),
                       ms->kernel_filename,
                       FLASH_BLOCK0_BASE_ADDR,
                       FLASH_BLOCK0_SIZE +
                       FLASH_BLOCK1_SIZE +
                       FLASH_BLOCK2_SIZE +
                       FLASH_BLOCK3_SIZE);
}

/* MachineClass registration */
static void s32k3x8_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->name             = g_strdup("s32k3x8evb");
    mc->desc             = "NXP S32K3X8 EVB (Cortex-M7)";
    mc->init             = s32k3x8_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m7");
    mc->default_cpus     = 1;
    mc->min_cpus         = 1;
    mc->max_cpus         = 1;
    mc->no_floppy        = 1;
    mc->no_cdrom         = 1;
    mc->no_parallel      = 1;
}

static const TypeInfo s32k3x8_machine_info = {
    .name       = TYPE_S32K3X8_MACHINE,
    .parent     = TYPE_MACHINE,
    .class_init = s32k3x8_class_init,
};

static void s32k3x8evb_machine_init(void)
{
    type_register_static(&s32k3x8_machine_info);
    type_register_static(&s32k3x8evb_sys_info);
    type_register_static(&s32k3_info);
}

type_init(s32k3x8evb_machine_init);
