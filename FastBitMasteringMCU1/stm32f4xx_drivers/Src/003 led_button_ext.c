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
     * 
     * This program uses Leds on PA05 and PA09 and Buttons on PB12 and PC13.
     */

    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_05;
    GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIOPinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;


    GPIO_Handle_t GpioLedExt;
    GpioLedExt.pGPIOx = GPIOA;
    GpioLedExt.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_09;
    GpioLedExt.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLedExt.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLedExt.GPIOPinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLedExt.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOA, ENABLE);

    GPIO_Init(&GpioLedExt);
    GPIO_Init(&GpioLed);


    /* ############################################################################################
     *                                                                        BUTTON CONFIGURATIONS
     * ############################################################################################
     */

    GPIO_Handle_t GpioButton;
    GpioButton.pGPIOx = GPIOC;
    GpioButton.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_13;
    GpioButton.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioButton.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButton.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;


    GPIO_Handle_t GpioButtonExt;
    GpioButtonExt.pGPIOx = GPIOB;
    GpioButtonExt.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_12;
    GpioButtonExt.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioButtonExt.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButtonExt.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeripheralClockControl(GPIOB, ENABLE);
    GPIO_PeripheralClockControl(GPIOC, ENABLE);

    GPIO_Init(&GpioButtonExt);
    GPIO_Init(&GpioButton);



    /* ############################################################################################
     *                                                                            THE INFINITE LOOP
     * ############################################################################################
     */

    while(1)
    {
        if((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BTN_PRESSED) || 
           (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_12) == BTN_PRESSED))
		{
        	delay();   /* debounce */
            GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_05);
            GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_09);
    	}

    }

    return 0;
}
