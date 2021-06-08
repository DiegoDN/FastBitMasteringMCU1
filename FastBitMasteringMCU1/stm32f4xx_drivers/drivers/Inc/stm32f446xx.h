#ifndef STM32F446XX_H_
#define STM32F446XX_H_

/* ############################################################################
 *                   			      BASE ADDRESSES OF FLASH AND SRAM MEMORIES
 * ############################################################################
*/

#define FLASH_BASEADDR 					0x08000000UL   							/* DS 62 */
#define SRAM1_BASEADDR 					0x20000000UL   							/* DS 62 */
#define SRAM2_BASEADDR 					0x2001C000UL   							/* DS 62 */
#define SRAM 							SRAM1_BASEADDR   						/* DS 62 */
#define ROM 							0x001FFFFFUL   							/* DS 62 */


/* ############################################################################
 *                       		         BASE ADDRESSES OF AHBx AND APBx BUSSES
 * ############################################################################
*/

#define PERIPH_BASEADDR					0x04000000UL							/* DS 59 */
#define APB1_PERIPH_BASEADDR			PERIPH_BASE								/* DS 62 */
#define APB2_PERIPH_BASEADDR			0x04001000UL							/* DS 58 */
#define AHB1_PERIPH_BASEADDR			0x04002000UL							/* DS 57 */
#define AHB2_PERIPH_BASEADDR			0x05000000UL							/* DS 57 */


/* ############################################################################   
 * 								      BASE ADDRESSES OF PERIPHERALS ON AHB1 BUS
 * ############################################################################
*/

#define GPIOA_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0000)			/* DS 57 */
#define GPIOB_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0400)			/* DS 57 */
#define GPIOC_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0800)			/* DS 57 */
#define GPIOD_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0C00)			/* DS 57 */
#define GPIOE_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1000)			/* DS 57 */
#define GPIOF_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0400)			/* DS 57 */
#define GPIOG_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0800)			/* DS 57 */
#define GPIOH_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1C00)			/* DS 57 */


/* ############################################################################
 * 								      BASE ADDRESSES OF PERIPHERALS ON APB1 BUS
 * ############################################################################
*/

#define CAN1_BASEADDR					(APB1_PERIPH_BASEADDR + 0x6400)			/* DS 59 */
#define CAN2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x6800)			/* DS 59 */
#define I2C1_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5400)			/* DS 59 */
#define I2C2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5800)			/* DS 59 */
#define I2C3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5C00)			/* DS 59 */
#define SPI2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x3800)			/* DS 59 */
#define SPI3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x3C00)			/* DS 59 */
#define USART2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4400)			/* DS 59 */
#define USART3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4800)			/* DS 59 */
#define UART4_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4C00)			/* DS 59 */
#define UART5_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5000)			/* DS 59 */


/* ############################################################################
 * 								      BASE ADDRESSES OF PERIPHERALS ON APB2 BUS
 * ############################################################################
*/

#define EXTI_BASEADDR					(APB2_PERIPH_BASEADDR + 0x3C00)			/* DS 57 */
#define SPI1_BASEADDR					(APB2_PERIPH_BASEADDR + 0x3000)			/* DS 58 */
#define SYSCFG_BASEADDR					(APB2_PERIPH_BASEADDR + 0x5400)			/* DS 58 */
#define USART1_BASEADDR					(APB2_PERIPH_BASEADDR + 0x1000)			/* DS 58 */
#define USART6_BASEADDR					(APB2_PERIPH_BASEADDR + 0x1400)			/* DS 58 */




#endif /* STM32F446XX_H_ */
