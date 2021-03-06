#include <stdio.h>
#include <stdint.h>
#include "stm32f446xx.h"

#define BTN_PRESSED   0


void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

int main (void)
{

    /* ############################################################################################
     *                                                                           LED CONFIGURATIONS
     * ############################################################################################
     */

    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_05;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOA, ENABLE);

    GPIO_Init(&GpioLed);


    /* ############################################################################################
     *                                                                        BUTTON CONFIGURATIONS
     * ############################################################################################
     */

    GPIO_Handle_t GpioButton;

    GpioButton.pGPIOx = GPIOC;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOC, ENABLE);

    GPIO_Init(&GpioButton);



    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BTN_PRESSED)
		{
        	delay();   /* debounce */
            GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_05);
    	}

    }

    return 0;
}
