#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

#define BTN_PRESSED   0

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

/* ############################################################################################
 *                                                                           SPI CONFIGURATIONS
 * ############################################################################################
 *
 * This program use SPI1.
 *
 * PB5 -> SPI1_MOSI
 * PB4 -> SPI1_MISO
 * PB3 -> SPI1_SCLK
 * PA4 -> NSS
 * ALT function mode: 5
 */

void SPI1_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* SCLK */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_03;
    GPIO_Init(&SPIPins);

    /* MOSI */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_05;
    GPIO_Init(&SPIPins);

    /* MISO */
    /* SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_04; */
    /* GPIO_Init(&SPIPins); */

    /* NSS */
    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_04;
    GPIO_Init(&SPIPins);

}

void SPI1_Inits(void)
{
    SPI_Handle_t SPI1handle;
    SPI1handle.pSPIx = SPI1;
    SPI1handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
    SPI1handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV8;  /* 2MHz */
    SPI1handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
    SPI1handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    SPI1handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    SPI1handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI1handle);
}

void GPIO_ButtonInit(void)
{
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
}

void GPIO_LedInit(void)
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
}

int main (void)
{

    /* ############################################################################################
     *                                                                           SPI CONFIGURATIONS
     * ############################################################################################
     */

    //char user_data[] = "Hello World";
	char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";


    /* This Function will initialize GPIOPins as SPI1 Pins */
    SPI1_GPIOInits();

    /* This Function will initialize the SPI1 Peripheral */
    SPI1_Inits();

    /* This function will enable the NSS and SSOEN */
    SPI_SSOEConfig(SPI1, ENABLE);

    GPIO_ButtonInit();
    GPIO_LedInit();


    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BTN_PRESSED)
		{
        	delay();   /* debounce */

            GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_05);

            /* Enable the SPI1 peripheral */
            SPI_PeripheralControl(SPI1, ENABLE);

            /* Send length Information */
            uint8_t dataLen = strlen(user_data);
            SPI_SendData(SPI1, &dataLen, 1);

            /* Send the Data */
            SPI_SendData(SPI1, (uint8_t *) user_data, strlen(user_data));

            /* Confirm SPI not busy */
            while ( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

            /* Disable the SPI1 peripheral */
            SPI_PeripheralControl(SPI1, DISABLE);
    	}

    }

	return 0;
}
