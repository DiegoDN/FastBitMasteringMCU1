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

#define SPI_CPOL_HIGH                   1                                      /* CPOL DS PG 886 */
#define SPI_CPOL_LOW                    0                                      /* CPOL DS PG 886 */

#define SPI_SSM_EN                      1                                       /* SSM DS PG 886 */
#define SPI_SSM_DI                      0                                       /* SSM DS PG 886 */

#define SPI_TXE_FLAG                    (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG                   (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG                   (1 << SPI_SR_BSY)

#define SPI_READY                       0
#define SPI_BUSY_IN_RX                  1
#define SPI_BUSY_IN_TX                  2


/* ################################################################################################
 *                                                                    HANDLE STRUCTURE FOR SPI PINs
 * ################################################################################################
 */

typedef struct 
{
    SPI_RegDef_t *pSPIx;                /* holds base addr of SPI PORT to which this pin belongs */
    SPI_Config_t SPI_Config;                             /* holds SPI Pin configuration settings */
    uint8_t *pTXBuffer;
    uint8_t *pRXBuffer;
    uint8_t TXLen;                                               /* Stores app TX Buffer Address */
    uint8_t RXLen;                                               /* Stores app RX Buffer Address */
    uint8_t TXState;                                                     /* Stores app TX length */
    uint8_t RXState;                                                     /* Stores app TX length */

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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);

/* Data Send and Receive with Interrupts */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len);

/* IRQ Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriotity);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


/* Other Peripheral Control APIS */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmissiong(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/* Application Callbacks */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);



#endif /* STM32F446XX_SPI_DRIVER_H_ */
