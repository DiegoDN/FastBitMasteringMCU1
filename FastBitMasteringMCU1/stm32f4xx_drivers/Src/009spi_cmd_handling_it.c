
/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */
/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003_SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

#define BTN_PRESSED                     0
#define LED_ON                          1
#define LED_OFF                         0
#define MAX_LEN                         500
#define LED_PIN                         9

/*Global Variables */
SPI_Handle_t SPI1handle;
char RcvBuff[MAX_LEN];
char ReadByte;
volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;


void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}


uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
    if(ackByte == 0xF5) /* ACK */
    {
        return 1;
    }
    else
    {
        return 0; /* NACK */
    }
}


/* ############################################################################################
 *                                                                           SPI CONFIGURATIONS
 * ############################################################################################
 *
 * This program use SPI1.
 *
 * PB5 -> SPI1_MOSI
 * PB4 -> SPI1_MISO
 * PB3 -> SPI1_SCLK
 * PA4 -> NSS
 * ALT function mode: 5
 */

void SPI1_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* SCLK */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_03;
    GPIO_Init(&SPIPins);

    /* MOSI */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_05;
    GPIO_Init(&SPIPins);

    /* MISO */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_04;
    GPIO_Init(&SPIPins);

    /* NSS */
    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_04;
    GPIO_Init(&SPIPins);

}

void SPI1_Inits(void)
{
    SPI_Handle_t SPI1handle;
    SPI1handle.pSPIx = SPI1;
    SPI1handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
    SPI1handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV32;
    SPI1handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
    SPI1handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    SPI1handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    SPI1handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI1handle);
}


void Slave_GPIO_InterruptPin_Init(void)
{
    GPIO_Handle_t spiIntPin;
    memset(&spiIntPin,0,sizeof(spiIntPin));

    //this is led gpio configuration
    spiIntPin.pGPIOx = GPIOA;
    spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_09;
    spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ITFT;
    spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

    GPIO_Init(&spiIntPin);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI_15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
}


int main (void)
{
    uint8_t dummy = 0xff;

    /* ############################################################################################
     *                                                                           SPI CONFIGURATIONS
     * ############################################################################################
     */

    
    /* This Function will initialize GPIOPins as SPI1 Pins */
    SPI1_GPIOInits();

    /* This Function will initialize the SPI1 Peripheral */
    SPI1_Inits();

    /* This function will enable the NSS and SSOEN */
    SPI_SSOEConfig(SPI1, ENABLE);

    /* This function will configure the Interrupts IRQ */
    SPI_IRQInterruptConfig(IRQ_NO_SPI1,ENABLE);
    Slave_GPIO_InterruptPin_Init();

    while(1)
    {
        rcvStop = 0;

        while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

        //enable the SPI2 peripheral
        SPI_PeripheralControl(SPI1, ENABLE);


        while(!rcvStop)
        {
            /* fetch the data from the SPI peripheral byte by byte in interrupt mode */
            while ( SPI_SendDataIT(&SPI1handle, &dummy, 1) == SPI_BUSY_IN_TX);
            while ( SPI_ReceiveDataIT(&SPI1handle, (uint8_t *) &ReadByte, 1) == SPI_BUSY_IN_RX );
        }


        // confirm SPI is not busy
        while( SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG) );

        //Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI1, DISABLE);

        printf("Rcvd data = %s\n", RcvBuff);

        dataAvailable = 0;

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	}

    return 0;
}


/* Runs when a data byte is received from the peripheral over SPI*/
void SPI1_IRQHandler(void)
{

	SPI_IRQHandling(&SPI1handle);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent)
{
	static uint32_t i = 0;

	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
    if(AppEvent == SPI_EVENT_RX_CMPLT)
    {
        RcvBuff[i++] = ReadByte;
        if(ReadByte == '\0' || ( i == MAX_LEN))
        {
            rcvStop = 1;
            RcvBuff[i - 1] = '\0';
            i = 0;
        }
    }
}


/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_09);
	dataAvailable = 1;
}
