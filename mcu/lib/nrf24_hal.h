#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include "STM32L432KC_GPIO.h"

#define RF_CE PB7
#define SPI_CE PB1

// CE (chip enable) pin (PB11)
#define nRF24_CE_PIN               PB7
#define nRF24_CE_L                 digitalWrite(RF_CE, 0);
#define nRF24_CE_H                 digitalWrite(RF_CE, 1);

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_L                digitalWrite(SPI_CE, 0);
#define nRF24_CSN_H                digitalWrite(SPI_CE, 1);

//// IRQ pin (PB10)
//#define nRF24_IRQ_PORT             GPIOB
//#define nRF24_IRQ_PIN              GPIO_Pin_10


// Macros for the RX on/off
#define nRF24_RX_ON                nRF24_CE_H
#define nRF24_RX_OFF               nRF24_CE_L


// Function prototypes
//void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
