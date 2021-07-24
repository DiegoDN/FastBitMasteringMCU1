#include <stdio.h>
#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"

/* ################################################################################################
 *                                           Prototypes for Helper Functions for SPI Interrupt Mode
 * ################################################################################################
 *	
 */

static void spi_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle);


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
 *                                                                                  SPI_ReceiveData
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_ReceiveData
 * FUNCION BRIEF: RECEIVE DATA FROM THE GIVER SPI.
 * PARAMETERS:    SPI NUMBER, TXBUFFER, LENGTH
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          THIS IS A BLOCKING CALL;
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* 1. Wait until TXE is set */
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		/* 2. Check the DFF Bit in CR1 */
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			/* 16 bit DFF */
			/* 1. Load data from the the DR to the RXBuffer*/
			*((uint16_t *)pRXBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRXBuffer++;
		}
		else
		{	/* 8 bit DFF */
			/* 1. Load data from the DR to the RXBuffer*/
			*((uint8_t *)pRXBuffer) = pSPIx->DR;
			Len--;
			pRXBuffer++;
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


/* ################################################################################################
 *                                                                           SPI_IRQInterruptConfig
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_IRQInterruptConfig
 * FUNCION BRIEF: CONFIGURE THE INTERRUPT ON THE GIVEN SPI.
 * PARAMETERS:    IRQNUMBER, ENABLE OR DISABLE
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			//program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if((IRQNumber >= 32) && (IRQNumber < 64))
		{
			//program ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			//program ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}		
	}
	else
	{
		if(IRQNumber < 32)
		{
			//program ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if((IRQNumber >= 32) && (IRQNumber < 64))
		{
			//program ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			//program ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}		

	}

}


/* ################################################################################################
 *                                                                            SPI_IRQPriorityConfig
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_IRQPriorityConfig
 * FUNCION BRIEF: CONFIGURE THE PRIORITY ON THE GIVE IRQ NUMBER
 * PARAMETERS:    IRQNUMBER, IRQPRIORITY
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	/* 1. first lets find out the ipr register */
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}


/* ################################################################################################
 *                                                                                   SPI_SendDataIT
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_SendDataIT
 * FUNCION BRIEF: SEND DATA FROM THE GIVER SPI using interruptions.
 * PARAMETERS:    SPI HANDLE ADDR, TXBUFFER, LENGTH
 * PARAMETERS:    
 * RETURN TYPE:   uint8_t;
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TXState;

	if (state != SPI_BUSY_IN_TX)
	{
		/* 1. Save the TX Buffer address and lenght information on global variables */
		pSPIHandle->pTXBuffer = pTXBuffer;
		pSPIHandle->TXLen = Len;

		/* 2. Mark SPI state as busy so no other code can take control over SPI peripheral */
		pSPIHandle->TXState = SPI_BUSY_IN_TX;

		/* 3. Enable the TXEIE control bit to the the Interrupt whenever TXE FLag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIEN);
	}

	return state;
}


/* ################################################################################################
 *                                                                                SPI_ReceiveDataIT
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_ReceiveDataIT
 * FUNCION BRIEF: RECEIVE DATA FROM THE GIVER SPI using interruptions.
 * PARAMETERS:    SPI HANDLE ADDR, RXBUFFER, LENGTH
 * PARAMETERS:    
 * RETURN TYPE:   uint8_t;
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RXState;

	if (state != SPI_BUSY_IN_RX)
	{
		/* 1. Save the RX Buffer address and lenght information on global variables */
		pSPIHandle->pRXBuffer = pRXBuffer;
		pSPIHandle->RXLen = Len;

		/* 2. Mark SPI state as busy so no other code can take control over SPI peripheral */
		pSPIHandle->RXState = SPI_BUSY_IN_TX;

		/* 3. Enable the TXEIE control bit to the the Interrupt whenever TXE FLag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXEIEN);
	}

	return state;
}


/* ################################################################################################
 *                                                                                  SPI_IRQHandling
 * ################################################################################################
 *	
 * FUNCTION NAME: SPI_IRQHandling
 * FUNCION BRIEF: 
 * PARAMETERS:    SPI HANDLE ADDR
 * PARAMETERS:    NONE;
 * RETURN TYPE:   NONE;
 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1 , temp2;

	/* Check for TXE */
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIEN);

	if (temp1 && temp2)
	{
		/* Handle TXE */
		spi_TXE_interrupt_handle(pSPIHandle);
	}

	/* Check for RXNE */
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXEIEN);

	if (temp1 && temp2)
	{
		/* Handle RXNE */
		spi_RXNE_interrupt_handle(pSPIHandle);
	}

	/* Check for OVR Flag */
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		/* Handle OVR  */
		spi_OVR_interrupt_handle(pSPIHandle);
	}
}



/* ################################################################################################
 *                                                          Helper Functions for SPI Interrupt Mode
 * ################################################################################################
 *	
 */

static void spi_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	/* 1. Check for DFF bit in CR1 */
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		/* 16Bits DFF - Load data into DR*/  
		pSPIHandle->pSPIx->DR = *((uint16_t *) pSPIHandle->pTXBuffer);
		pSPIHandle->TXLen--;
		pSPIHandle->TXLen--;
		(uint16_t *) pSPIHandle->pTXBuffer++;
	}
	else
	{
		/* 8Bit DFF - Load data into DR*/  
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTXBuffer);
		pSPIHandle->TXLen--;
        pSPIHandle->pTXBuffer++;
	}

	if(! (pSPIHandle->TXLen))
	{
		/*TXLen is Zero, close the SPI transmission and inform the app that TX is over */
		SPI_CloseTransmissiong(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	/* 1. Check for DFF bit in CR1 */
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		/* 16Bits DFF - Load data into DR*/  
		*((uint16_t *) pSPIHandle->pRXBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RXLen--;
		pSPIHandle->RXLen--;
		pSPIHandle->pRXBuffer--;
		pSPIHandle->pRXBuffer--;
	}
	else
	{
		/* 8Bit DFF - Load data into DR*/  
		*(pSPIHandle->pRXBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->TXLen--;
        pSPIHandle->pRXBuffer--;
	}

	if(! (pSPIHandle->RXLen))
	{
		/*RXLen is Zero, close the SPI reception and inform the app that RX is over */
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	/* 1. Clear OVR Flag */
	if(pSPIHandle->TXState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void) temp;

	/* 2. Inform the APP */
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}


void SPI_CloseTransmissiong(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIEN);
	pSPIHandle->pTXBuffer = NULL;
	pSPIHandle->TXLen = 0;
	pSPIHandle->TXState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXEIEN);
	pSPIHandle->pRXBuffer = NULL;
	pSPIHandle->RXLen = 0;
	pSPIHandle->RXState = SPI_READY;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{

}
