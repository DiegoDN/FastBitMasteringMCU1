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
	/* 0. ENABLE Peripheral Clock */
	SPI_PeripheralClockControl(pSPIHandle->pSPIx, ENABLE);

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

	/* 1e. CONFIGURE THE SSM */
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	/* 1f. SAVE tempreg on SPI_CR1	 */
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


/* ################################################################################################
 *                                                                                SPI_GetFlagStatus
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_GetFlagStatus
 * FUNCION BRIEF: TEST IF A BIT POSITION IS SET OR RESET
 * PARAMETERS:    SPI NUMBER, FLAG NAME
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;	
	}
	return FLAG_RESET;
}


/* ################################################################################################
 *                                                                                     SPI_SendData
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_SendData
 * FUNCION BRIEF: SEND DATA FROM THE GIVER SPI.
 * PARAMETERS:    SPI NUMBER, TXBUFFER, LENGTH
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          THIS IS A BLOCKING CALL;
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* 1. Wait until TXE is set */
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		/* 2. Check the DFF Bit in CR1 */
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			/* 16 bit DFF */
			/* 1. Load data into the DR */
			pSPIx->DR = *((uint16_t *)pTXBuffer);
			Len--;
			Len--;
			(uint16_t *)pTXBuffer++;
		}
		else
		{	/* 8 bit DFF */
			/* 1. Load data into the DR */
			pSPIx->DR = *((uint8_t *)pTXBuffer);
			Len--;
			pTXBuffer++;
		}
	}
}


/* ################################################################################################
 *                                                                            SPI_PeripheralControl
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_PeripheralControl
 * FUNCION BRIEF: ENABLE THE PERIPHERAL OF THE GIVER SPI.
 * PARAMETERS:    SPI NUMBER, ENABLE OR DISABLE
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/* ################################################################################################
 *                                                                                    SPI_SSIConfig
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_SSIConfig
 * FUNCION BRIEF: ENABLE THE SSI OF THE GIVER SPI as 1 IN ORDER TO AVOID MODE FAULT ERROR MODF.
 * PARAMETERS:    SPI NUMBER, ENABLE OR DISABLE
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/* ################################################################################################
 *                                                                                   SPI_SSOEConfig
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_SSOEConfig
 * FUNCION BRIEF: ENABLE THE SSOE OF THE GIVER SPI as 1 IN ORDER TO enable NSS.
 * PARAMETERS:    SPI NUMBER, ENABLE OR DISABLE
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOEN);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOEN);
	}
}