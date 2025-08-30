#include "qemu/osdep.h"
#include "hw/boards.h"
#include "exec/address-spaces.h"
#include "target/arm/cpu.h"
#include "qemu/error-report.h"

typedef struct S32K3X8EVBState {
    MemoryRegion ram;
} S32K3X8EVBState;

static void s32k3x8evb_init(MachineState *machine)
{
    S32K3X8EVBState *s = g_new0(S32K3X8EVBState, 1);

    memory_region_init_ram(&s->ram, NULL, "s32k3x8evb.ram", 512 * 1024, NULL);
    memory_region_add_subregion(get_system_memory(), 0x20000000, &s->ram);
}

static void s32k3x8evb_machine_class_init(MachineClass *mc)
{
    mc->desc = "NXP S32K3X8EVB Evaluation Board";
    mc->init = s32k3x8evb_init;
    mc->default_ram_size = 512 * 1024;
    mc->default_cpu_type = "cortex-m7";
}

DEFINE_MACHINE("s32k3x8evb", s32k3x8evb_machine_class_init)
