#include <stdio.h>
#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_i2c_driver.h"

/* ################################################################################################
 *                                                           FUNCTION APIS SUPPORTED BY THIS DRIVER
 * ################################################################################################
 */
/*
 * DS = DATASHEET   @ = PAGE NUMBER
 * DATASHEET = dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

/* ################################################################################################
 *                                                                       I2C_PeripheralClockControl
 * ################################################################################################
 *
 * FUNCTION NAME: I2C_PeripheralClockControl
 * FUNCION BRIEF: THIS FUNCTION ENABLE OR DISABLE PERIPHERAL CLOCK ON GIVEN I2C PORT
 * PARAMETERS:    BASE ADDRESS OF I2C PERIPHERAL
 * PARAMETERS:    ENABLE OR DISABLE MACROS
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if      (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
        else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
        }
	}

    else
	{
		if      (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
        else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
        }
	}
}


/* ################################################################################################
 *                                                                                         I2C_Init
 * ################################################################################################
 *
 * FUNCTION NAME: I2C_Init
 * FUNCION BRIEF: INITIALIZE THE GIVEN I2C
 * PARAMETERS:    I2C HANDLE ADDR
 * PARAMETERS:
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void I2C_Init  (I2C_Handle_t *pI2CHandle)
{

}


/* ################################################################################################
 *                                                                                       I2C_DeInit
 * ################################################################################################
 *
 * FUNCTION NAME: I2C_DeInit
 * FUNCION BRIEF: DE-INITIALIZE THE GIVEN I2C
 * PARAMETERS:    BASE ADDR I2C
 * PARAMETERS:
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if      (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
    else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
    else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/* ################################################################################################
 *                                                                                I2C_GetFlagStatus
 * ################################################################################################
 *
 * FUNCTION NAME: I2C_GetFlagStatus
 * FUNCION BRIEF: TEST IF A BIT POSITION IS SET OR RESET
 * PARAMETERS:    I2C NUMBER, FLAG NAME
 * PARAMETERS:
 * RETURN TYPE:   NONE;
 * NOTE:          NONE;
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	/*
	if (pI2Cx->SR & FlagName)
	{
		return FLAG_SET;
	}
	*/
	return FLAG_RESET;

}
