#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

#define BTN_PRESSED                     0
#define LED_ON                          1
#define LED_OFF                         0

/* MACROS FOR CMD CODES */
#define COMMAND_LED_CTRL                0x50
#define COMMAND_SENSOR_READ             0x51
#define COMMAND_LED_READ                0x52
#define COMMAND_PRINT                   0x53
#define COMMAND_ID_READ                 0x54

/* MACROS FOR ARDUINO PINS */
#define ANALOG_PIN_O                    0
#define ANALOG_PIN_1                    1
#define ANALOG_PIN_2                    2
#define ANALOG_PIN_3                    3
#define ANALOG_PIN_4                    4

#define LED_PIN                         9


void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}


uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
    if(ackByte == 0xF5) /* ACK */
    {
        return 1;
    }
    else
    {
        return 0; /* NACK */
    }
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
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_04;
    GPIO_Init(&SPIPins);

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
    uint8_t dummyWrite = 0xFF;
    uint8_t dummyRead;
    uint8_t ackByte = 0xFF;
    uint8_t args[2];

    /* ############################################################################################
     *                                                                           SPI CONFIGURATIONS
     * ############################################################################################
     */

    
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
        while ( ! (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BTN_PRESSED));

        delay();   /* debounce */

		GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_05);

		/* Enable the SPI1 peripheral */
		SPI_PeripheralControl(SPI1, ENABLE);


		/* 1. CMD_LED_CRTL  <PIN NO(1)>  <VALUE(1)> */
		uint8_t commandCode = COMMAND_LED_CTRL;

		/* Send the Data */
		SPI_SendData(SPI1, &commandCode, 1);
		SPI_ReceiveData(SPI1, &dummyRead, 1);   /*do a dummy read to clear RXNE buffer */

		/* Send dummy bits to fetch response from slave */
		SPI_SendData(SPI1, &dummyWrite, 1);
		SPI_ReceiveData(SPI1, &ackByte, 1);

		if(SPI_VerifyResponse (ackByte))
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI1, args, 2);
		}


        while ( ! (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BTN_PRESSED));

        /* 2. CMD_SENSOR_READ  <ANALOG PIN NUMBER(1)> */
        commandCode = COMMAND_SENSOR_READ;
        uint8_t analogRead;

        /* Send the Data */
        SPI_SendData(SPI1, &commandCode, 1);
        SPI_ReceiveData(SPI1, &dummyRead, 1);   /*do a dummy read to clear RXNE buffer */

        /* Send dummy bits to fetch response from slave */
        SPI_SendData(SPI1, &dummyWrite, 1);
        SPI_ReceiveData(SPI1, &ackByte, 1);

        if(SPI_VerifyResponse (ackByte))
        {
            //send arguments
            args[0] = ANALOG_PIN_O;
            SPI_SendData(SPI1, args, 1);
            SPI_ReceiveData(SPI1, &dummyRead, 1);   /*do a dummy read to clear RXNE buffer */

            /* Send dummy bits to fetch response from slave */
            delay();  /*wait for the slave to do analog read */
            SPI_SendData(SPI1, &dummyWrite, 1);
            SPI_ReceiveData(SPI1, &analogRead, 1);
        }


        /* Confirm SPI not busy */
        while ( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

        /* Disable the SPI1 peripheral */
        SPI_PeripheralControl(SPI1, DISABLE);

    }

	return 0;
}


