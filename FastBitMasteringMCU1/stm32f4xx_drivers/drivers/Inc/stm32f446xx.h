#ifndef STM32F446XX_H_
#define STM32F446XX_H_
#include <stdint.h>
#include <stddef.h>

/*
 * DS = DATASHEET   @ = PAGE NUMBER
 * DATASHEET = dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

/* ################################################################################################   
 *                                                                              SOME GENERIC MACROS
 * ################################################################################################
 */

#define __vo                            volatile
#define __weak                          __attribute__((weak))
#define ENABLE                          1
#define DISABLE                         0
#define SET                             ENABLE
#define RESET                           DISABLE     
#define GPIO_PIN_SET                    ENABLE
#define GPIO_PIN_RESET                  DISABLE     
#define FLAG_RESET                      RESET
#define FLAG_SET                        SET

/* ################################################################################################
 *                                                                       PROCESSOR SPECIFIC DETAILS
 * ################################################################################################
 */

/* ################################################################################################
 *                                                                               PRIORITY REGISTERS
 * ################################################################################################
 */

#define NVIC_ISER0                      ((__vo uint32_t *) 0xE000E100)                 /* DS 219 */
#define NVIC_ISER1                      ((__vo uint32_t *) 0xE000E104)                 /* DS 219 */
#define NVIC_ISER2                      ((__vo uint32_t *) 0xE000E108)                 /* DS 219 */
#define NVIC_ISER3                      ((__vo uint32_t *) 0xE000E10C)                 /* DS 219 */

#define NVIC_ICER0                      ((__vo uint32_t *) 0xE000E180)                 /* DS 219 */
#define NVIC_ICER1                      ((__vo uint32_t *) 0xE000E184)                 /* DS 219 */
#define NVIC_ICER2                      ((__vo uint32_t *) 0xE000E188)                 /* DS 219 */
#define NVIC_ICER3                      ((__vo uint32_t *) 0xE000E18C)                 /* DS 219 */

#define NVIC_PR_BASE_ADDR               ((__vo uint32_t *) 0xE000E400)                 /* DS 219 */

#define NO_PR_BITS_IMPLEMENTED          4


/* ################################################################################################
 *                                                        BASE ADDRESSES OF FLASH AND SRAM MEMORIES
 * ################################################################################################
 */

#define FLASH_BASEADDR                  0x08000000U                                     /* DS 62 */
#define SRAM1_BASEADDR                  0x20000000U                                     /* DS 62 */
#define SRAM2_BASEADDR                  0x2001C000U                                     /* DS 62 */
#define SRAM                            SRAM1_BASEADDR                                  /* DS 62 */
#define ROM                             0x001FFFFFU                                     /* DS 62 */


/* ################################################################################################
 *                                                           BASE ADDRESSES OF AHBx AND APBx BUSSES
 * ################################################################################################
 */
    
#define PERIPH_BASEADDR	                0x40000000U                                     /* DS 59 */
#define APB1_PERIPH_BASEADDR            PERIPH_BASEADDR                                 /* DS 62 */
#define APB2_PERIPH_BASEADDR            0x40010000U                                     /* DS 58 */
#define AHB1_PERIPH_BASEADDR            0x40020000U                                     /* DS 57 */
#define AHB2_PERIPH_BASEADDR            0x50000000U                                     /* DS 57 */


/* ################################################################################################   
 *                                                        BASE ADDRESSES OF PERIPHERALS ON AHB1 BUS
 * ################################################################################################
 */

#define GPIOA_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x0000U)                /* DS 57 */
#define GPIOB_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x0400U)                /* DS 57 */
#define GPIOC_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x0800U)                /* DS 57 */
#define GPIOD_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x0C00U)                /* DS 57 */
#define GPIOE_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x1000U)                /* DS 57 */
#define GPIOF_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x0400U)                /* DS 57 */
#define GPIOG_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x0800U)                /* DS 57 */
#define GPIOH_BASEADDR                  (AHB1_PERIPH_BASEADDR + 0x1C00U)                /* DS 57 */

#define RCC_BASEADDR                    (AHB1_PERIPH_BASEADDR + 0x3800U)                /* DS 57 */

#define EXTI_BASEADDR                   (APB2_PERIPH_BASEADDR + 0x3C00U)                /* DS 57 */


/* ################################################################################################
 *                                                        BASE ADDRESSES OF PERIPHERALS ON APB1 BUS
 * ################################################################################################
 */

#define CAN1_BASEADDR                   (APB1_PERIPH_BASEADDR + 0x6400U)                /* DS 59 */
#define CAN2_BASEADDR                   (APB1_PERIPH_BASEADDR + 0x6800U)                /* DS 59 */
#define I2C1_BASEADDR                   (APB1_PERIPH_BASEADDR + 0x5400U)                /* DS 59 */
#define I2C2_BASEADDR                   (APB1_PERIPH_BASEADDR + 0x5800U)                /* DS 59 */
#define I2C3_BASEADDR                   (APB1_PERIPH_BASEADDR + 0x5C00U)                /* DS 59 */
#define SPI2_BASEADDR                   (APB1_PERIPH_BASEADDR + 0x3800U)                /* DS 59 */
#define SPI3_BASEADDR                   (APB1_PERIPH_BASEADDR + 0x3C00U)                /* DS 59 */
#define USART2_BASEADDR                 (APB1_PERIPH_BASEADDR + 0x4400U)                /* DS 59 */
#define USART3_BASEADDR                 (APB1_PERIPH_BASEADDR + 0x4800U)                /* DS 59 */
#define UART4_BASEADDR                  (APB1_PERIPH_BASEADDR + 0x4C00U)                /* DS 59 */
#define UART5_BASEADDR                  (APB1_PERIPH_BASEADDR + 0x5000U)                /* DS 59 */


/* ################################################################################################
 *                                                        BASE ADDRESSES OF PERIPHERALS ON APB2 BUS
 * ################################################################################################
 */

#define EXTI_BASEADDR                   (APB2_PERIPH_BASEADDR + 0x3C00U)                /* DS 57 */
#define SPI1_BASEADDR                   (APB2_PERIPH_BASEADDR + 0x3000U)                /* DS 58 */
#define SPI4_BASEADDR                   (APB2_PERIPH_BASEADDR + 0x3400U)                /* DS 58 */
#define SYSCFG_BASEADDR                 (APB2_PERIPH_BASEADDR + 0x3800U)                /* DS 58 */
#define USART1_BASEADDR                 (APB2_PERIPH_BASEADDR + 0x1000U)                /* DS 58 */
#define USART6_BASEADDR                 (APB2_PERIPH_BASEADDR + 0x1400U)                /* DS 58 */


/* ################################################################################################
 *                                                        PERIPHERAL REGISTER DEFINITION STRUCTURES
 * ################################################################################################
 */

typedef struct 
{
    __vo uint32_t MODER;                                /* MODE REG - ADDR OFFSET: 0x00 - DS 193 */
    __vo uint32_t OTYPER;                        /* OUTPUT TYPE REG - ADDR OFFSET: 0x04 - DS 193 */
    __vo uint32_t OSPEEDER;                    /* OUTPUT SPEED REG - ADDR OFFSET:  0x08 - DS 193 */
    __vo uint32_t PUPDR;                   /* PULLUP PULL DOWN REG - ADDR OFFSET:  0x0C - DS 193 */
    __vo uint32_t IDR;                           /* INPUT DATA REG - ADDR OFFSET:  0x10 - DS 194 */
    __vo uint32_t ODR;                          /* OUTPUT DATA REG - ADDR OFFSET:  0x14 - DS 194 */
    __vo uint32_t BSRR;                      /* BITSET / RESET REG - ADDR OFFSET:  0x18 - DS 194 */
    __vo uint32_t LCKR;                  /* CONFIGURATION LOCK REG - ADDR OFFSET:  0x1C - DS 194 */
    __vo uint32_t AFRL;              /* ALTERNATE FUNCTION REG LOW - ADDR OFFSET:  0x20 - DS 194 */
    __vo uint32_t AFRH;             /* ALTERNATE FUNCTION REG HIGH - ADDR OFFSET:  0x20 - DS 194 */

} GPIO_RegDef_t;


typedef struct 
{
    __vo uint32_t CR;                          /* CLOCK CONTROL REG - ADDR OFFSET: 0x00 - DS 172 */
    __vo uint32_t PLL_CFGR;                /* PLL CONFIGURATION REG - ADDR OFFSET: 0x04 - DS 172 */
    __vo uint32_t CFGR;                  /* CLOCK CONFIGURATION REG - ADDR OFFSET: 0x08 - DS 172 */
    __vo uint32_t CIR;                       /* CLOCK INTERRUPT REG - ADDR OFFSET: 0x0C - DS 172 */
    __vo uint32_t AHB1_RSTR;                      /* AHB1 RESET REG - ADDR OFFSET: 0x10 - DS 172 */
    __vo uint32_t AHB2_RSTR;                      /* AHB2 RESET REG - ADDR OFFSET: 0x14 - DS 172 */
    __vo uint32_t AHB3_RSTR;                      /* AHB3 RESET REG - ADDR OFFSET: 0x18 - DS 172 */
    uint32_t      RESERVED0;                            /* RESERVED - ADDR OFFSET: 0x1C - DS 172 */
    __vo uint32_t APB1_RSTR;                      /* APB1 RESET REG - ADDR OFFSET: 0x20 - DS 172 */
    __vo uint32_t APB2_RSTR;                      /* APB2 RESET REG - ADDR OFFSET: 0x24 - DS 173 */
    uint32_t      RESERVED1;                            /* RESERVED - ADDR OFFSET: 0x28 - DS 172 */
    uint32_t      RESERVED2;                            /* RESERVED - ADDR OFFSET: 0x2C - DS 172 */
    __vo uint32_t AHB1_ENR;                  /* AHB1 CLK ENABLE REG - ADDR OFFSET: 0x30 - DS 173 */
    __vo uint32_t AHB2_ENR;                  /* AHB2 CLK ENABLE REG - ADDR OFFSET: 0x34 - DS 173 */
    __vo uint32_t AHB3_ENR;                  /* AHB3 CLK ENABLE REG - ADDR OFFSET: 0x38 - DS 173 */
    uint32_t      RESERVED3;                            /* RESERVED - ADDR OFFSET: 0x3C - DS 173 */
    __vo uint32_t APB1_ENR;                  /* APB1 CLK ENABLE REG - ADDR OFFSET: 0x40 - DS 173 */
    __vo uint32_t APB2_ENR;                  /* APB2 CLK ENABLE REG - ADDR OFFSET: 0x44 - DS 173 */
    uint32_t      RESERVED4;                            /* RESERVED - ADDR OFFSET: 0x48 - DS 173 */
    uint32_t      RESERVED5;                            /* RESERVED - ADDR OFFSET: 0x4C - DS 173 */
    __vo uint32_t AHB1_LPENR;      /* AHB1 CLK ENABLE LOW POWER REG - ADDR OFFSET: 0x50 - DS 174 */
    __vo uint32_t AHB2_LPENR;      /* AHB2 CLK ENABLE LOW POWER REG - ADDR OFFSET: 0x54 - DS 174 */
    __vo uint32_t AHB3_LPENR;      /* AHB3 CLK ENABLE LOW POWER REG - ADDR OFFSET: 0x58 - DS 174 */
    __vo uint32_t APB1_LPENR;      /* APB1 CLK ENABLE LOW POWER REG - ADDR OFFSET: 0x60 - DS 174 */
    __vo uint32_t APB2_LPENR;      /* APB2 CLK ENABLE LOW POWER REG - ADDR OFFSET: 0x64 - DS 174 */
    __vo uint32_t BDCR;                /* BACKUP DOMAIN CONTROL REG - ADDR OFFSET: 0x70 - DS 174 */
    __vo uint32_t CSR;                /* CLK CONTROL AND STATUS REG - ADDR OFFSET: 0x74 - DS 174 */
    uint32_t      RESERVED6;                            /* RESERVED - ADDR OFFSET: 0x78 - DS 175 */
    uint32_t      RESERVED7;                            /* RESERVED - ADDR OFFSET: 0x7C - DS 175 */
    __vo uint32_t SSCGR;          /* SPREAD SPECTRUM CLK GENERATION - ADDR OFFSET: 0x80 - DS 175 */
    __vo uint32_t PLLI2SCFGR;           /* PLLI2S CONFIGURATION REG - ADDR OFFSET: 0x84 - DS 175 */
    __vo uint32_t PLLSAICFGR;              /* PLL CONFIGURATION REG - ADDR OFFSET: 0x88 - DS 175 */
    __vo uint32_t DCKCFGR;       /* DEDICATED CLK CONFIGURATION REG - ADDR OFFSET: 0x8C - DS 175 */
    __vo uint32_t CK_GATENR;                 /* CLK GATE ENABLE REG - ADDR OFFSET: 0x90 - DS 175 */
    __vo uint32_t DCKCFGR2;    /* DEDICATED CLK CONFIGURATION REG 2 - ADDR OFFSET: 0x94 - DS 175 */
    
 } RCC_RegDef_t;


typedef struct 
{
    __vo uint32_t IMR;                        /* INTERRUPT MASK REG - ADDR OFFSET: 0x00 - DS 246 */
    __vo uint32_t EMR;                            /* EVENT MASK REG - ADDR OFFSET: 0x04 - DS 246 */
    __vo uint32_t RTSR;            /* RISING TRIGGER SELECTION REG - ADDR OFFSET:  0x08 - DS 247 */
    __vo uint32_t FTSR;           /* FALLING TRIGGER SELECTION REG - ADDR OFFSET:  0x0C - DS 247 */
    __vo uint32_t SWIER;           /* SOFTWARE INTERRUPT EVENT REG - ADDR OFFSET:  0x10 - DS 248 */
    __vo uint32_t PR;                               /* PENDING REG - ADDR OFFSET:  0x14 - DS 248 */

} EXTI_RegDef_t;


typedef struct 
{
    __vo uint32_t MEMRMP;                       /* MEMORY REMAP REG - ADDR OFFSET: 0x00 - DS 195 */
    __vo uint32_t PMC;              /* PERIP. MODE CONFIGURATON REG - ADDR OFFSET: 0x04 - DS 197 */
    __vo uint32_t EXTICR1;             /* EXTERNAL INTERRUPT 1 REG - ADDR OFFSET:  0x08 - DS 197 */
    __vo uint32_t EXTICR2;             /* EXTERNAL INTERRUPT 2 REG - ADDR OFFSET:  0x0C - DS 198 */
    __vo uint32_t EXTICR3;                 /* EXTERNAL INTERRUPT 3 - ADDR OFFSET:  0x10 - DS 199 */
    __vo uint32_t EXTICR4;            /* EXTERNAL INTERRUPT 41 REG - ADDR OFFSET:  0x14 - DS 199 */
    __vo uint32_t CMPCR;             /* COMPENSATION CELL CTRL REG - ADDR OFFSET:  0x20 - DS 200 */
    __vo uint32_t CFGR;                       /* CONFIGURATION REG - ADDR OFFSET:  0x2C - DS 200 */

} SYSCFG_RegDef_t;


typedef struct 
{
    __vo uint32_t CR1;                             /* CONTROL REG 1 - ADDR OFFSET: 0x00 - DS 886 */
    __vo uint32_t CR2;                             /* CONTROL REG 2 - ADDR OFFSET: 0x04 - DS 888 */
    __vo uint32_t SR;                                /* STATUS REG - ADDR OFFSET:  0x08 - DS 889 */
    __vo uint32_t DR;                                  /* DATA REG - ADDR OFFSET:  0x0C - DS 890 */
    __vo uint32_t CRCPR;                     /* CRC POLYNOMIAL REG - ADDR OFFSET:  0x10 - DS 891 */
    __vo uint32_t RXCRCR;                            /* RX CRC REG - ADDR OFFSET:  0x14 - DS 892 */
    __vo uint32_t TXCRCR;                            /* TX CRC REG - ADDR OFFSET:  0x18 - DS 892 */
    __vo uint32_t I2SCFGR;                       /* I2S CONFIG REG - ADDR OFFSET:  0x1C - DS 893 */
    __vo uint32_t I2SPR;                      /* I2S PRESCALER REG - ADDR OFFSET:  0x20 - DS 894 */

} SPI_RegDef_t;


/* ################################################################################################
 *                                                                           PERIPHERAL DEFINITIONS 
 * ################################################################################################
 */

#define GPIOA                           ((GPIO_RegDef_t *) GPIOA_BASEADDR )
#define GPIOB                           ((GPIO_RegDef_t *) GPIOB_BASEADDR )
#define GPIOC                           ((GPIO_RegDef_t *) GPIOC_BASEADDR )
#define GPIOD                           ((GPIO_RegDef_t *) GPIOD_BASEADDR )
#define GPIOE                           ((GPIO_RegDef_t *) GPIOE_BASEADDR )
#define GPIOF                           ((GPIO_RegDef_t *) GPIOF_BASEADDR )
#define GPIOG                           ((GPIO_RegDef_t *) GPIOG_BASEADDR )
#define GPIOH                           ((GPIO_RegDef_t *) GPIOH_BASEADDR )

#define RCC                             ((RCC_RegDef_t *) RCC_BASEADDR )

#define EXTI                            ((EXTI_RegDef_t *) EXTI_BASEADDR )

#define SYSCFG                          ((SYSCFG_RegDef_t *) SYSCFG_BASEADDR )

#define SPI1                            ((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2                            ((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3                            ((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4                            ((SPI_RegDef_t *) SPI4_BASEADDR)


/* ################################################################################################   
 *                                                        CLOCK ENABLE MACROS FOR GPIOX PERIPHERALS
 * ################################################################################################
 */

#define GPIOA_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 0))                  /* DS 144 */
#define GPIOB_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 1))                  /* DS 144 */
#define GPIOC_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 2))                  /* DS 144 */
#define GPIOD_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 3))                  /* DS 144 */
#define GPIOE_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 4))                  /* DS 144 */
#define GPIOF_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 5))                  /* DS 144 */
#define GPIOG_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 6))                  /* DS 144 */
#define GPIOH_PCLK_EN()                 ((RCC->AHB1_ENR) |= (1 << 7))                  /* DS 144 */


/* ################################################################################################   
 *                                                          CLOCK ENABLE MACROS FOR I2C PERIPHERALS
 * ################################################################################################
 */

#define I2C1_PCLK_EN()                  ((RCC->APB1_ENR) |= (1 << 21))                 /* DS 147 */
#define I2C2_PCLK_EN()                  ((RCC->APB1_ENR) |= (1 << 22))                 /* DS 147 */
#define I2C3_PCLK_EN()                  ((RCC->APB1_ENR) |= (1 << 23))                 /* DS 147 */


/* ################################################################################################   
 *                                                          CLOCK ENABLE MACROS FOR SPI PERIPHERALS
 * ################################################################################################
 */

#define SPI1_PCLK_EN()                  ((RCC->APB2_ENR) |= (1 << 12))                 /* DS 149 */
#define SPI2_PCLK_EN()                  ((RCC->APB1_ENR) |= (1 << 14))                 /* DS 147 */
#define SPI3_PCLK_EN()                  ((RCC->APB1_ENR) |= (1 << 15))                 /* DS 147 */
#define SPI4_PCLK_EN()                  ((RCC->APB2_ENR) |= (1 << 13))                 /* DS 149 */


/* ################################################################################################   
 *                                                         CLOCK ENABLE MACROS FOR UART PERIPHERALS
 * ################################################################################################
 */

#define USART2_PCLK_EN()                ((RCC->APB1_ENR) |= (1 << 17))                 /* DS 147 */
#define USART3_PCLK_EN()                ((RCC->APB1_ENR) |= (1 << 18))                 /* DS 147 */
#define UART4_PCLK_EN()                 ((RCC->APB1_ENR) |= (1 << 19))                 /* DS 147 */
#define UART5_PCLK_EN()                 ((RCC->APB1_ENR) |= (1 << 20))                 /* DS 147 */


/* ################################################################################################   
 *                                                          CLOCK ENABLE MACROS FOR CAN PERIPHERALS
 * ################################################################################################
 */

#define CAN1_PCLK_EN()                  ((RCC->APB1_ENR) |= (1 << 25))                 /* DS 147 */
#define CAN2_PCLK_EN()                  ((RCC->APB1_ENR) |= (1 << 26))                 /* DS 147 */


/* ################################################################################################   
 *                                                       CLOCK ENABLE MACROS FOR SYSCFG PERIPHERALS
 * ################################################################################################
 */

#define SYSCFG_PCLK_EN()                ((RCC->APB2_ENR) |= (1 << 14))                 /* DS 149 */


/* ################################################################################################   
 *                                                       CLOCK DISABLE MACROS FOR GPIOX PERIPHERALS
 * ################################################################################################
 */

#define GPIOA_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 0))                 /* DS 144 */
#define GPIOB_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 1))                 /* DS 144 */
#define GPIOC_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 2))                 /* DS 144 */
#define GPIOD_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 3))                 /* DS 144 */
#define GPIOE_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 4))                 /* DS 144 */
#define GPIOF_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 5))                 /* DS 144 */
#define GPIOG_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 6))                 /* DS 144 */
#define GPIOH_PCLK_DI()                 ((RCC->AHB1_ENR) &= ~(1 << 7))                 /* DS 144 */


/* ################################################################################################   
 *                                                         CLOCK DISABLE MACROS FOR I2C PERIPHERALS
 * ################################################################################################
 */

#define I2C1_PCLK_DI()                  ((RCC->APB1_ENR) &= ~(1 << 21))                /* DS 147 */
#define I2C2_PCLK_DI()                  ((RCC->APB1_ENR) &= ~(1 << 22))                /* DS 147 */
#define I2C3_PCLK_DI()                  ((RCC->APB1_ENR) &= ~(1 << 23))                /* DS 147 */


/* ################################################################################################   
 *                                                         CLOCK DISABLE MACROS FOR SPI PERIPHERALS
 * ################################################################################################
 */
 
#define SPI1_PCLK_DI()                  ((RCC->APB2_ENR) &= ~(1 << 12))                /* DS 149 */
#define SPI2_PCLK_DI()                  ((RCC->APB1_ENR) &= ~(1 << 14))                /* DS 147 */
#define SPI3_PCLK_DI()                  ((RCC->APB1_ENR) &= ~(1 << 15))                /* DS 147 */
#define SPI4_PCLK_DI()                  ((RCC->APB2_ENR) &= ~(1 << 13))                /* DS 149 */


/* ################################################################################################   
 *                                                        CLOCK DISABLE MACROS FOR UART PERIPHERALS
 * ################################################################################################
 */

#define USART2_PCLK_DI()                ((RCC->APB1_ENR) &= ~(1 << 17))                /* DS 147 */
#define USART3_PCLK_DI()                ((RCC->APB1_ENR) &= ~(1 << 18))                /* DS 147 */
#define UART4_PCLK_DI()                 ((RCC->APB1_ENR) &= ~(1 << 19))                /* DS 147 */
#define UART5_PCLK_DI()                 ((RCC->APB1_ENR) &= ~(1 << 20))                /* DS 147 */


/* ################################################################################################   
 *                                                          CLOCK DIABLE MACROS FOR CAN PERIPHERALS
 * ################################################################################################
 */

#define CAN1_PCLK_DI()                  ((RCC->APB1_ENR) &= ~(1 << 25))                /* DS 147 */
#define CAN2_PCLK_DI()                  ((RCC->APB1_ENR) &= ~(1 << 26))                /* DS 147 */


/* ################################################################################################   
 *                                                      CLOCK DISABLE MACROS FOR SYSCFG PERIPHERALS
 * ################################################################################################
 */
 
#define SYSCFG_PCLK_DI()                ((RCC->APB2_ENR) &= ~(1 << 14))                /* DS 149 */


/* ################################################################################################   
 *                                                               RESET MACROS FOR GPIOX PERIPHERALS
 * ################################################################################################
 */

#define GPIOA_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 0)); ((RCC->AHB1_RSTR) &= ~(1 << 0));} while(0)  /* DS 136 */
#define GPIOB_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 1)); ((RCC->AHB1_RSTR) &= ~(1 << 1));} while(0)  /* DS 136 */
#define GPIOC_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 2)); ((RCC->AHB1_RSTR) &= ~(1 << 2));} while(0)  /* DS 136 */
#define GPIOD_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 3)); ((RCC->AHB1_RSTR) &= ~(1 << 3));} while(0)  /* DS 136 */
#define GPIOE_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 4)); ((RCC->AHB1_RSTR) &= ~(1 << 4));} while(0)  /* DS 136 */
#define GPIOF_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 5)); ((RCC->AHB1_RSTR) &= ~(1 << 5));} while(0)  /* DS 136 */
#define GPIOG_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 6)); ((RCC->AHB1_RSTR) &= ~(1 << 6));} while(0)  /* DS 136 */
#define GPIOH_REG_RESET()  do{((RCC->AHB1_RSTR) |= (1 << 7)); ((RCC->AHB1_RSTR) &= ~(1 << 7));} while(0)  /* DS 136 */


/* ################################################################################################   
 *                                                               RESET MACROS FOR SPIX PERIPHERALS
 * ################################################################################################
 */

#define SPI1_REG_RESET()  do{((RCC->APB2_RSTR) |= (1 << 12)); ((RCC->APB2_RSTR) &= ~(1 << 12));} while(0)  /* DS 142 */
#define SPI2_REG_RESET()  do{((RCC->APB1_RSTR) |= (1 << 14)); ((RCC->APB1_RSTR) &= ~(1 << 14));} while(0)  /* DS 139 */
#define SPI3_REG_RESET()  do{((RCC->APB1_RSTR) |= (1 << 15)); ((RCC->APB1_RSTR) &= ~(1 << 15));} while(0)  /* DS 139 */
#define SPI4_REG_RESET()  do{((RCC->APB2_RSTR) |= (1 << 13)); ((RCC->APB2_RSTR) &= ~(1 << 13));} while(0)  /* DS 142 */


/* ################################################################################################   
 *                                                                  MACRO FOR GPIO BASE ADDR RETURN
 * ################################################################################################
 */


#define GPIO_BASEADDR_TO_CODE(x)        ( (x == GPIOA) ? 0 : \
                                          (x == GPIOB) ? 1 : \
                                          (x == GPIOC) ? 2 : \
                                          (x == GPIOD) ? 3 : \
                                          (x == GPIOE) ? 4 : \
                                          (x == GPIOF) ? 5 : \
                                          (x == GPIOG) ? 6 : \
                                          (x == GPIOH) ? 7 : 0 )



/* ################################################################################################   
 *                                                                       MACRO FOR SPI BITS CONFIGS
 * ################################################################################################
 */

#define SPI_CR1_BIDIMODE                15                                          /* DS PG 886 */
#define SPI_CR1_BIDIOE                  14                                          /* DS PG 886 */
#define SPI_CR1_CRC_EN                  13                                          /* DS PG 886 */
#define SPI_CR1_CRC_NEXT                12                                          /* DS PG 886 */
#define SPI_CR1_DFF                     11                                          /* DS PG 886 */
#define SPI_CR1_RXONLY                  10                                          /* DS PG 886 */
#define SPI_CR1_SSM                     9                                           /* DS PG 886 */
#define SPI_CR1_SSI                     8                                           /* DS PG 886 */
#define SPI_CR1_LSB_FIRST               7                                           /* DS PG 886 */
#define SPI_CR1_SPE                     6                                           /* DS PG 886 */
#define SPI_CR1_BR                      3                                           /* DS PG 886 */
#define SPI_CR1_MSTR                    2                                           /* DS PG 886 */
#define SPI_CR1_CPOL                    1                                           /* DS PG 886 */
#define SPI_CR1_CPHA                    0                                           /* DS PG 886 */

#define SPI_CR2_TXEIEN                  7                                           /* DS PG 888 */
#define SPI_CR2_RXEIEN                  6                                           /* DS PG 888 */
#define SPI_CR2_ERRIE                   5                                           /* DS PG 888 */
#define SPI_CR2_FRF                     4                                           /* DS PG 888 */
#define SPI_CR2_SSOEN                   2                                           /* DS PG 888 */
#define SPI_CR2_TXDMAEN                 1                                           /* DS PG 888 */
#define SPI_CR2_RXDMAEN                 0                                           /* DS PG 888 */

#define SPI_SR_FRE                      8                                           /* DS PG 889 */
#define SPI_SR_BSY                      7                                           /* DS PG 889 */
#define SPI_SR_OVR                      6                                           /* DS PG 889 */
#define SPI_SR_MODF                     5                                           /* DS PG 889 */
#define SPI_SR_CRCERR                   4                                           /* DS PG 889 */
#define SPI_SR_UDR                      3                                           /* DS PG 889 */
#define SPI_SR_CHSIDE                   2                                           /* DS PG 889 */
#define SPI_SR_TXE                      1                                           /* DS PG 889 */
#define SPI_SR_RXNE                     0                                           /* DS PG 889 */


/* ################################################################################################   
 *                                                                 MACRO FOR SPI APPLICATION EVENTS
 * ################################################################################################
 */


#define SPI_EVENT_TX_CMPLT              1
#define SPI_EVENT_RX_CMPLT              2
#define SPI_EVENT_OVR_ERR               3
#define SPI_EVENT_CRC_ERR               4



/* ################################################################################################   
 *                                                                     INTERRUPT REQUEST IRQ MACROS
 * ################################################################################################
 */

#define IRQ_NO_EXTI0                    6                                       /* DS PG 239-240 */
#define IRQ_NO_EXTI1                    7                                       /* DS PG 239-240 */
#define IRQ_NO_EXTI2                    8                                       /* DS PG 239-240 */
#define IRQ_NO_EXTI3                    9                                       /* DS PG 239-240 */
#define IRQ_NO_EXTI4                    10                                      /* DS PG 239-240 */
#define IRQ_NO_EXTI9_5                  23                                      /* DS PG 239-240 */
#define IRQ_NO_EXTI15_10                40                                      /* DS PG 239-240 */
#define IRQ_NO_SPI1                     35                                      /* DS PG 239-240 */
#define IRQ_NO_SPI2                     36                                      /* DS PG 239-240 */
#define IRQ_NO_SPI3                     51                                      /* DS PG 239-240 */
#define IRQ_NO_SPI4                     84                                      /* DS PG 239-240 */
#define IRQ_NO_I2C1_EV                  31                                      /* DS PG 239-240 */
#define IRQ_NO_I2C1_ER                  32                                      /* DS PG 239-240 */
#define IRQ_NO_USART1                   37                                      /* DS PG 239-240 */
#define IRQ_NO_USART2                   38                                      /* DS PG 239-240 */
#define IRQ_NO_USART3                   39                                      /* DS PG 239-240 */
#define IRQ_NO_UART4                    52                                      /* DS PG 239-240 */
#define IRQ_NO_UART5                    53                                      /* DS PG 239-240 */
#define IRQ_NO_USART6                   71                                      /* DS PG 239-240 */

#define NVIC_IRQ_PRI_0                  0
#define NVIC_IRQ_PRI_1                  1
#define NVIC_IRQ_PRI_2                  2
#define NVIC_IRQ_PRI_3                  3
#define NVIC_IRQ_PRI_4                  4
#define NVIC_IRQ_PRI_5                  5
#define NVIC_IRQ_PRI_6                  6
#define NVIC_IRQ_PRI_7                  7
#define NVIC_IRQ_PRI_8                  8
#define NVIC_IRQ_PRI_9                  9
#define NVIC_IRQ_PRI_10                 10
#define NVIC_IRQ_PRI_11                 11
#define NVIC_IRQ_PRI_12                 12
#define NVIC_IRQ_PRI_13                 13
#define NVIC_IRQ_PRI_14                 14
#define NVIC_IRQ_PRI_15                 15




















/* ################################################################################################
 *                                                                                     HEADER FILES
 * ################################################################################################
 */

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

#endif /* STM32F446XX_H_ */


