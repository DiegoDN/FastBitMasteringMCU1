#include <stdio.h>
#include <stdint.h>
#include "stm32f446xx.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW


int main(void)
{

	GPIO_Handle_t GpioLed;

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_05;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);



	while(1)
	{
		GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_05);
	}

	return 0;
}
