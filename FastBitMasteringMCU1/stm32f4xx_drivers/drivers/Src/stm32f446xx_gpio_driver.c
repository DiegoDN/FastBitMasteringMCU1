#include <stdio.h>
#include <stdint.h>

#include "stm32f446xx.h"
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
 *                                                                                        GPIO_Init
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_Init
 * FUNCION BRIEF: INITIALIZE THE GIVEN GPIO
 * PARAMETERS:    GPIO HANDLE ADDR
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

		temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); 
		//2 because the datasheet says that each Pin take 2 positions.
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clear
		pGPIOHandle->pGPIOx->MODER |= temp;                                                  //set
	}

	else
	{
		if(pGPIOHandle->GPIOPinConfig.GPIO_PinMode == GPIO_MODE_ITFT)
		{
			/* 1a. configure the FTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);   /* set   the FTSR BIT */
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);  /* clear the RTSR BIT */
		}
		else if(pGPIOHandle->GPIOPinConfig.GPIO_PinMode == GPIO_MODE_ITRT)
		{
			/* 1b. configure the RTSR */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);   /* set   the RTSR BIT */
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);  /* clear the FTSR BIT */

		}
		else if(pGPIOHandle->GPIOPinConfig.GPIO_PinMode == GPIO_MODE_ITFRT)
		{
			/* 1b. configure the FTSR and RTSR*/
			EXTI->FTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);   /* set   the FTSR BIT */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);   /* set   the RTSR BIT */
		}

			/* 2. Configure the GPIO Port selection in SYSCFG_EXTICR */


			/* 3. Enable the EXTI interrupt delivery using IMR */
			EXTI->IMR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	}

	temp = 0;


	/* 2 Configure the SPEED of the GPIO Pin */
	temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); 
	//2 because the datasheet says that each Pin take 2 positions.
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;                                                 //set
	temp = 0;

	/* 3 Configure the PUPD of the GPIO Pin */
	temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); 
	//2 because the datasheet says that each Pin take 2 positions.
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->PUPDR |= temp;                                                 //set
	temp = 0;


	/* 4 Configure the OPTYPE of the GPIO Pin */
	temp = ( pGPIOHandle->GPIOPinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber) ); 
	//1 because the datasheet says that each Pin take 1 positions.
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->OTYPER |= temp;                                                  //set
	temp = 0;

	/* 5 Configure the ALT FUNCT of the GPIO Pin */
	temp = pGPIOHandle->GPIOPinConfig.GPIO_PinAltFuncMode;
	
	if(pGPIOHandle->GPIOPinConfig.GPIO_PinNumber < 8)
	{
		uint8_t temp2 = (pGPIOHandle->GPIOPinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFRL &= ~( 0xF << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clear
		pGPIOHandle->pGPIOx->AFRL |= temp << (4 * temp2);                                  //set
		temp2 = 0;										    
	}
	else
	{
		uint8_t temp2 = (pGPIOHandle->GPIOPinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFRH &= ~( 0xF << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); //clear
		pGPIOHandle->pGPIOx->AFRH |= temp << (4 * temp2);                                  //set
		temp2 = 0;						    				
	}
	
	temp = 0;

}


/* ################################################################################################
 *                                                                                      GPIO_DeInit
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_DeInit
 * FUNCION BRIEF: DE-INITIALIZE THE GIVEN GPIO
 * PARAMETERS:    BASE ADDR GPIO 
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


/* ################################################################################################
 *                                                                            GPIO_ReadFromInputPin
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_ReadFromInputPin
 * FUNCION BRIEF: READ THE VALUE OF A GPIO PIN
 * PARAMETERS:    GPIO PORT
 * PARAMETERS:    GPIO PIN
 * RETURN TYPE:   0 or 1;
 * NOTE:          NONE;
 */

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value = (uint8_t) (( pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;	
}


/* ################################################################################################
 *                                                                           GPIO_ReadFromInputPort
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_ReadFromInputPort
 * FUNCION BRIEF: READ THE VALUE OF A GPIO PORT (ALL PINS)
 * PARAMETERS:    GPIO PORT
 * PARAMETERS:    
 * RETURN TYPE:   value of register port;
 * NOTE:          NONE;
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = (uint16_t) ( pGPIOx->IDR);
	return value;
}


/* ################################################################################################
 *                                                                            GPIO_WritetoOutputPin
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_WritetoOutputPin
 * FUNCION BRIEF: WRITE A VALUE TO A GIVEN GPIO PIN
 * PARAMETERS:    GPIO PORT
 * PARAMETERS:    GPIO PIN
 * PARAMETERS:    VALUE TO WRITE
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/* ################################################################################################
 *                                                                           GPIO_WritetoOutputPort
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_WritetoOutputPort
 * FUNCION BRIEF: WRITE A VALUE TO A GIVEN GPIO PORT
 * PARAMETERS:    GPIO PORT
 * PARAMETERS:    VALUE TO WRITE
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/* ################################################################################################
 *                                                                             GPIO_ToogleOutputPin
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_ToogleOutputPin
 * FUNCION BRIEF: TOOGLE THE STATE OF A GIVE GPIO PIN
 * PARAMETERS:    GPIO PORT
 * PARAMETERS:    GPIO PIN
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_ToogleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}


/* ################################################################################################
 *                                                                          GPIO_IRQInterruptConfig
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_IRQInterruptConfig
 * FUNCION BRIEF: 
 * PARAMETERS:    
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 *                                                                           GPIO_IRQPriorityConfig
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_IRQPriorityConfig
 * FUNCION BRIEF: 
 * PARAMETERS:    
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriotity)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_ammount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriotity << shift_ammount);

}


/* ################################################################################################
 *                                                                                 GPIO_IRQHandling
 * ################################################################################################
 *	
 * FUNCTION NAME: GPIO_IRQHandling
 * FUNCION BRIEF: 
 * PARAMETERS:    
 * PARAMETERS:    
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}

}
