#include <stdio.h>
#include <stdint.h>

#include "stm32f446xx.h"
#include "../Inc/stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"


/* ################################################################################################
 *                                                           FUNCTION APIS SUPPORTED BY THIS DRIVER
 * ################################################################################################
 */

/* ################################################################################################
 *                                                                      GPIO_PeripheralClockControl
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_PeripheralClockControl
 * FUNCION BRIEF: THIS FUNCTION ENABLE OR DISABLE PERIPHERAL CLOCK ON GIVEN GPIO PORT
 * PARAMETERS:    BASE ADDRESS OF GPIO PERIPHERAL
 * PARAMETERS:    ENABLE OR DISABLE MACROS
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	} else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/* ################################################################################################
 *					                                                                      GPIO_Init
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_Init
 * FUNCION BRIEF: 
 * PARAMETERS:    
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_Init  (GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	/* 1 Configure the MODE of the GPIO Pin */

	if(pGPIOHandle->GPIOPinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/* THIS IS THE NON-INTERRUPT MODE */

		temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); //2 because the datasheet says that each Pin take 2 positions.
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER = temp;													//setting
	}

	else
	{
		/*INTERRUPT MODE. THIS PART WILL BE CODED LATER */
	}

	temp = 0;


	/* 2 Configure the SPEED of the GPIO Pin */
	temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); //2 because the datasheet says that each Pin take 2 positions.
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;                                                 //setting
	temp = 0;

	/* 3 Configure the PUPD of the GPIO Pin */
	temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); //2 because the datasheet says that each Pin take 2 positions.
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;													//setting
	temp = 0;


	/* 4 Configure the OPTYPE of the GPIO Pin */
	temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); //1 because the datasheet says that each Pin take 1 positions.
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;			                                     //setting
	temp = 0;

	/* 5 Configure the ALT FUNCT of the GPIO Pin */
	temp = pGPIOHandle->GPIOPinConfig.GPIO_PinAltFuncMode;
	
	if(pGPIOHandle->GPIOPinConfig.GPIO_PinNumber < 8)
	{
		uint8_t temp2 = (pGPIOHandle->GPIOPinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFRL &= ~( 0xF << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->AFRL |= temp << (4 * temp2);								   //setting
		temp2 = 0;										    
	}
	else
	{
		uint8_t temp2 = (pGPIOHandle->GPIOPinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFRH &= ~( 0xF << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->AFRH |= temp << (4 * temp2);								   //setting
		temp2 = 0;						    				
	}
	
	temp = 0;

}


/* ################################################################################################
 *					                                                                    GPIO_DeInit
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_DeInit
 * FUNCION BRIEF: 
 * PARAMETERS:    
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}









/* Data Read and Write */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return 0;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return 0;
}
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint16_t Value)
{

}
void GPIO_ToogleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/* IRQ Handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriotity, uint8_t EnorDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
