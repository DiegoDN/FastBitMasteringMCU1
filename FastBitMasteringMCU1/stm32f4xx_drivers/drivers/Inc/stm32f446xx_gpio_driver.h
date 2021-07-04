#ifndef STM32F446XX_GPIO_DRIVER_H_
#define STM32F446XX_GPIO_DRIVER_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f446xx.h"

/*
 * DS = DATASHEET   @ = PAGE NUMBER
 * DATASHEET = dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

/* ################################################################################################
 *                                                            CONFIGURATION STRUCTURE FOR GPIO PINs
 * ################################################################################################
 */

typedef struct 
{
    uint8_t GPIO_PinNumber;              /* possible values from @GPIO_PIN_NUMBER - DS 187 - 192 */
    uint8_t GPIO_PinMode;                 /* possible values from @GPIO_PIN_MODES - DS 187 - 192 */
    uint8_t GPIO_PinSpeed;                /* possible values from @GPIO_PIN_SPEED - DS 187 - 192 */
    uint8_t GPIO_PinPuPdControl;           /* possible values from @GPIO_PIN_PUPD - DS 187 - 192 */
    uint8_t GPIO_PinOPType;             /* possible values from @GPIO_PIN_OP_TYPE - DS 187 - 192 */
    uint8_t GPIO_PinAltFuncMode;          /* possible values from @GPIO_PIN_MODES - DS 187 - 192 */
} GPIO_PinConfig_t;


/* GPIO PIN Possible PINNUMBER macros */ 
/* @GPIO_PIN_NUMBER */
#define GPIO_PIN_00                     0
#define GPIO_PIN_01                     1
#define GPIO_PIN_02                     2
#define GPIO_PIN_03                     3
#define GPIO_PIN_04                     4
#define GPIO_PIN_05                     5
#define GPIO_PIN_06                     6
#define GPIO_PIN_07                     7
#define GPIO_PIN_08                     8
#define GPIO_PIN_09                     9
#define GPIO_PIN_10                     10
#define GPIO_PIN_11                     11
#define GPIO_PIN_12                     12
#define GPIO_PIN_13                     13
#define GPIO_PIN_14                     14
#define GPIO_PIN_15                     15

/* GPIO PIN Possible MODE macros */ 
/* @GPIO_PIN_MODES */
#define GPIO_MODE_IN                    0
#define GPIO_MODE_OUT                   1
#define GPIO_MODE_ALTFN                 2
#define GPIO_MODE_ANALOG                3
#define GPIO_MODE_ITFT                  4
#define GPIO_MODE_ITRT                  5
#define GPIO_MODE_ITFRT                 6


/* GPIO PIN Possible SPEEDS macros */ 
/* @GPIO_PIN_SPEED */
#define GPIO_SPEED_LOW                  0
#define GPIO_SPEED_MEDIUM               1
#define GPIO_SPEED_FAST                 2
#define GPIO_SPEED_HIGH                 3


/* GPIO PIN Possible PULL-UP PULL-DOWN configuration macros */ 
/* @GPIO_PIN_PUPD */
#define GPIO_PIN_NO_PUPD                0
#define GPIO_PIN_PU                     1
#define GPIO_PIN_PD                     2


/* GPIO PIN Possible OUTPUT Types macros */ 
/* @GPIO_PIN_OP_TYPE */
#define GPIO_OP_TYPE_PP                 0
#define GPIO_OP_TYPE_OD                 1


/* ################################################################################################
 *                                                                   HANDLE STRUCTURE FOR GPIO PINs
 * ################################################################################################
 */

typedef struct 
{
    GPIO_RegDef_t *pGPIOx;             /* holds base addr of GPIO PORT to which this pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig;                     /* holds GPIO Pin configuration settings */
} GPIO_Handle_t;


/* ################################################################################################
 *                                                                    APIS SUPPORTED BY THIS DRIVER
 * ################################################################################################
 */

/* Peripheral Clock Setup */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Init / DeInit */
void GPIO_Init  (GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Data Read and Write */
uint8_t  GPIO_ReadFromInputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx);
void GPIO_WritetoOutputPin		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritetoOutputPort		(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToogleOutputPin		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriotity);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* STM32F446XX_GPIO_DRIVER_H_ */
