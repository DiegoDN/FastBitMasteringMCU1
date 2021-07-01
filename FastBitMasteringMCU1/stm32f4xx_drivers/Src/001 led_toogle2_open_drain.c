#include <stdio.h>
#include <stdint.h>

#include "stm32f446xx.h"

void delay(void)
{
	for (uint32_t i = 0; i < 500000; i++);
}

int main (void)
{
    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_05;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOA, ENABLE);

    GPIO_Init(&GpioLed);

    while(1)
    {
    	GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_05);
    	delay();
    }

    return 0;
}
