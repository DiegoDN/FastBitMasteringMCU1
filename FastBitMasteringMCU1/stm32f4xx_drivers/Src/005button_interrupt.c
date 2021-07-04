#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

void delay (void)
{
    for (uint32_t i = 0; i < 500000/2 ; i++);                  /* aprox 200ms of delay at 16MHz */
}

int main(void)
{

    /* ############################################################################################
     *                                                                           LED CONFIGURATIONS
     * ############################################################################################
     *
     * This program use Led on PA05 and the onboard button on  PC13.
     */

    GPIO_Handle_t GpioLed;
    memset(&GpioLed, 0, sizeof(GpioLed));

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_05;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);
    GPIO_WritetoOutputPin(GPIOA,GPIO_PIN_05,GPIO_PIN_RESET);


    /* ############################################################################################
     *                                                                        BUTTON CONFIGURATIONS
     * ############################################################################################
     */


    GPIO_Handle_t GpioButton;
    memset(&GpioButton, 0, sizeof(GpioButton));

    GpioButton.pGPIOx = GPIOC;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ITFT;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
    GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

    GPIO_PeripheralClockControl(GPIOC, ENABLE);
    GPIO_Init(&GpioButton);


    /* ############################################################################################
     *                                                                           IRQ CONFIGURATIONS
     * ############################################################################################
     */

	//IRQ Configurations
    GPIO_IRQPriorityConfig (IRQ_NO_EXTI15_10, NVIC_IRQ_PRI_15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1)
	{
		/* infinite loop */
	}

	return 0;
}


void EXTI15_10_IRQHandler(void)
{
    delay();
	GPIO_IRQHandling(GPIO_PIN_13);
    delay();
	GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_05);
}




