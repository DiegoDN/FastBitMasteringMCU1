#include <stdio.h>
#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"


/* ################################################################################################
 *                                                           FUNCTION APIS SUPPORTED BY THIS DRIVER
 * ################################################################################################
 */
/*
 * DS = DATASHEET   @ = PAGE NUMBER
 * DATASHEET = dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

/* ################################################################################################
 *                                                                      SPI_PeripheralClockControl
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_PeripheralClockControl
 * FUNCION BRIEF: THIS FUNCTION ENABLE OR DISABLE PERIPHERAL CLOCK ON GIVEN SPI PORT
 * PARAMETERS:    BASE ADDRESS OF SPI PERIPHERAL
 * PARAMETERS:    ENABLE OR DISABLE MACROS
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if      (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		} 
        else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
        } 
        else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
        }
	} 
    
    else
	{
		if      (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		} 
        else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
        } 
        else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
        }
	}
}


/* ################################################################################################
 *                                                                                         SPI_Init
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_Init
 * FUNCION BRIEF: INITIALIZE THE GIVEN SPI
 * PARAMETERS:    SPI HANDLE ADDR
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void SPI_Init  (SPI_Handle_t *pSPIHandle)
{
	/* 1. CONFIGURE SPI_CR1 */
	uint32_t tempreg = 0;

	/* 1a. CONFIGURE DEVICE MODE */
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	/* 1b. CONFIGURE BUS CONFIG */
	if     (( pSPIHandle->SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX ))
	{
		/*BIDI MODE = 0 */
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (( pSPIHandle->SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_HALFDUPLEX ))
	{
		/*BIDI MODE = 1 */
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (( pSPIHandle->SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_HALFDUPLEX ))
	{
		/*BIDI MODE = 0, RXONLY = 1 */
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |=  (1 << SPI_CR1_RXONLY);
	}
	
	/* 1c. CONFIGURE CLOCK SPEED (BAUD RATE) */
	tempreg |= pSPIHandle->SPI_Config.SPI_SCLKSpeed << SPI_CR1_BR;

	/* 1d. CONFIGURE THE DFF */
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/* 1e. CONFIGURE THE CPOL */
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* 1d. CONFIGURE THE CPHA */
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	/* 1e. SAVE tempreg on SPI_CR1	 */
	pSPIHandle->pSPIx->CR1 = tempreg;
}


/* ################################################################################################
 *                                                                                       SPI_DeInit
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_DeInit
 * FUNCION BRIEF: DE-INITIALIZE THE GIVEN SPI
 * PARAMETERS:    BASE ADDR SPI 
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if      (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	} 
    else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	} 
    else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	} 
    else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	} 
}
