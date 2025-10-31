#include <device.h>
#include "clllc.h"
#include "Ai_CLLLC_DAC_settings.h"

DACA_scope DACA_Target;
DACB_scope DACB_Target;
unsigned char dactest=0;

void DAC_init(uint32_t dac_base)
{
    // Set DAC reference voltage.
    DAC_setReferenceVoltage(dac_base, DAC_REF_VDAC);

    // Set DAC load mode.
    DAC_setLoadMode(dac_base, DAC_LOAD_SYSCLK);

    // Enable the DAC output
    DAC_enableOutput(dac_base);

    // Set the DAC shadow output
    DAC_setShadowValue(dac_base, 0U);

    // Delay for buffered DAC to power up.
    DEVICE_DELAY_US(10);
}

void DAC_targets_switching()
{
    switch (DACA_Target)
    {
        case TEST_A:
//            DAC_setShadowValue(DACA_BASE, (dactest++)&0x0fff);
            break;
        default :
            break;
    }

    switch (DACB_Target)
    {
        case TEST_B:
//            DAC_setShadowValue(DACB_BASE, (dactest++)&0x0fff);
            break;
        default:
            break;
    }
}
