#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"


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
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
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
    /* SPIPins.pGPIOx = GPIOA; */
    /* SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_04; */
    /* GPIO_Init(&SPIPins); */

}

void SPI1_Inits(void)
{
    SPI_Handle_t SPI1handle;
    SPI1handle.pSPIx = SPI1;
    SPI1handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
    SPI1handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV2;  /* 8MHz */
    SPI1handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
    SPI1handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    SPI1handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    SPI1handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

    SPI_Init(&SPI1handle);
}


int main (void)
{
    char user_data[] = "Hello World";

    /* This Function will initialize GPIOPins as SPI1 Pins */
    SPI1_GPIOInits();

    /* This Function will initialize the SPI1 Peripheral */
    SPI1_Inits();

    /* Enable the SPI1 SSI */
    SPI_SSIConfig(SPI1, ENABLE);

    /* Enable the SPI1 peripheral */
    SPI_PeripheralControl(SPI1, ENABLE);

    /* Send the Data */
    SPI_SendData(SPI1, (uint8_t *) user_data, strlen(user_data));

    /* Disable the SPI1 peripheral */
    SPI_PeripheralControl(SPI1, DISABLE);

    while(1);

	return 0;
}
