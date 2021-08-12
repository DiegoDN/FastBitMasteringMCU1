#ifndef STM32F446XX_I2C_DRIVER_H_
#define STM32F446XX_I2C_DRIVER_H_


#include <stdio.h>
#include <stdint.h>
#include "stm32f446xx.h"

/*
 * DS = DATASHEET   @ = PAGE NUMBER
 * DATASHEET = dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

/* ################################################################################################
 *                                                             CONFIGURATION STRUCTURE FOR I2C PINs
 * ################################################################################################
 */

typedef struct 
{
    uint8_t I2C_SCLSpeed;    
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;   
    uint8_t I2C_FMDutyCycle;  
} I2C_Config_t;


/* ################################################################################################
 *                                                                    HANDLE STRUCTURE FOR I2C PINs
 * ################################################################################################
 */

typedef struct 
{
    I2C_RegDef_t *pI2Cx;                    /* holds base addr of I2C PORT to which this belongs */
    I2C_Config_t I2C_Config;                                 /* holds I2C configuration settings */
} I2C_Handle_t;



/* I2C Speed possible macros */
/* @I2C_SCLSpeed */
#define I2C_SCL_SPEED_SM                100000
#define I2C_SCL_SPEED_FM2k              200000
#define I2C_SCL_SPEED_FM4k              400000

/* I2C ACK CTRL possible macros */
/* @I2C_ACKControl */
#define I2C_ACK_ENABLE                  1
#define I2C_ACK_DISABLE                 0

/* I2C Duty Cycle possible macros */
/* @I2C_FMDutyCycle */
#define I2C_FM_DUTY_2                   0
#define I2C_FM_DUTY_16_9                1




/* ################################################################################################
 *                                                                    APIS SUPPORTED BY THIS DRIVER
 * ################################################################################################
 */

/* Peripheral Clock Setup */
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/* Init / DeInit */
void I2C_Init  (I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/* Data Send and Receive */


/* IRQ Handling */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriotity);



/* Other Peripheral Control APIS */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


/* Application Callbacks */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* STM32F446XX_I2C_DRIVER_H_ */
