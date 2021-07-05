#ifndef STM32F446XX_SPI_DRIVER_H_
#define STM32F446XX_SPI_DRIVER_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f446xx.h"

/*
 * DS = DATASHEET   @ = PAGE NUMBER
 * DATASHEET = dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

/* ################################################################################################
 *                                                             CONFIGURATION STRUCTURE FOR SPI PINs
 * ################################################################################################
 */


typedef struct 
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SCLKSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t; 


/* DS PG 886*/ 
#define SPI_DEVICE_MODE_MASTER          1                                       /* MSTR DS PG 886*/
#define SPI_DEVICE_MODE_SLAVE           0                                       /* MSTR DS PG 886*/

#define SPI_BUS_CONFIG_FULLDUPLEX       1                                      /* BIDI DS PG 886 */
#define SPI_BUS_CONFIG_HALFDUPLEX       2                                      /* BIDI DS PG 886 */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3                                      /* BIDI DS PG 886 */

#define SPI_SCLK_SPEED_DIV2             0                                        /* BR DS PG 886 */
#define SPI_SCLK_SPEED_DIV4             1                                        /* BR DS PG 886 */
#define SPI_SCLK_SPEED_DIV8             2                                        /* BR DS PG 886 */
#define SPI_SCLK_SPEED_DIV16            3                                        /* BR DS PG 886 */
#define SPI_SCLK_SPEED_DIV32            4                                        /* BR DS PG 886 */
#define SPI_SCLK_SPEED_DIV64            5                                        /* BR DS PG 886 */
#define SPI_SCLK_SPEED_DIV128           6                                        /* BR DS PG 886 */
#define SPI_SCLK_SPEED_DIV256           7                                        /* BR DS PG 886 */

#define SPI_DFF_8BITS                   0                                       /* DFF DS PG 886 */
#define SPI_DFF_16BITS                  1                                       /* DFF DS PG 886 */

#define SPI_CPHA_HIGH                   1                                      /* CPHA DS PG 886 */
#define SPI_CPHA_LOW                    0                                      /* CPHA DS PG 886 */

#define SPI_SSM_EN                      1                                       /* SSM DS PG 886 */
#define SPI_SSM_DI                      0                                       /* SSM DS PG 886 */




/* ################################################################################################
 *                                                                    HANDLE STRUCTURE FOR SPI PINs
 * ################################################################################################
 */

typedef struct 
{
    SPI_RegDef_t *pSPIx;                /* holds base addr of SPI PORT to which this pin belongs */
    SPI_Config_t SPI_Config;                             /* holds SPI Pin configuration settings */
} SPI_Handle_t;



/* ################################################################################################
 *                                                                    APIS SUPPORTED BY THIS DRIVER
 * ################################################################################################
 */

/* Peripheral Clock Setup */
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* Init / DeInit */
void SPI_Init  (SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* Data Send and Receive */
void SPI_SendData(SPI_Handle_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_Handle_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len );

/* IRQ Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriotity);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/* Other Peripheral Control APIS */









#endif /* STM32F446XX_SPI_DRIVER_H_ */