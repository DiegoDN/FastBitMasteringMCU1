#ifndef STM32F446XX_GPIO_DRIVER_H_
#define STM32F446XX_GPIO_DRIVER_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f446xx.h"


/* ################################################################################################
 *                                                            CONFIGURATION STRUCTURE FOR GPIO PINs
 * ################################################################################################
 */

typedef struct 
{
	uint8_t GPIO_PinNumber; 													/* DS 187 - 192 */
	uint8_t GPIO_PinMode; 														/* DS 187 - 192 */
	uint8_t GPIO_PinSpeed; 														/* DS 187 - 192 */
	uint8_t GPIO_PinPuPdControl; 												/* DS 187 - 192 */
	uint8_t GPIO_PinOPType; 													/* DS 187 - 192 */
	uint8_t GPIO_PinAltFuncMode; 												/* DS 187 - 192 */
} GPIO_PinConfig_t;


/* ################################################################################################
 *                                                                   HANDLE STRUCTURE FOR GPIO PINs
 * ################################################################################################
 */

typedef struct 
{
	GPIO_RegDef_t *pGPIOx;             /* holds base addr of GPIO PORT to which this pin belongs */
	GPIO_PinConfig_t GPIOPinConfig;                     /* holds GPIO Pin configuration settings */
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
void GPIO_WritetoOutputPort		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint16_t Value);
void GPIO_ToogleOutputPin		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriotity, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* STM32F446XX_GPIO_DRIVER_H_ */
