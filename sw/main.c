/*
 * main.c
 */

#include "soc_C6748.h"
#include "psc.h"
#include "c674x/c6748/lcdkC6748.h"
#include "gpio.h"

#define GPIO_PIN(b, p) (b*16 + p + 1)

void delay(unsigned t)
{
    while(t--);
}

int main(void)
{
    GPIOBank6Pin12PinMuxSetup();

    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);

    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_PIN(6, 12), GPIO_DIR_OUTPUT);

    while(1) {

        GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_PIN(6, 12), GPIO_PIN_HIGH);
        delay(1000000);
        GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_PIN(6, 12), GPIO_PIN_LOW);

    }
	
	return 0;
}
