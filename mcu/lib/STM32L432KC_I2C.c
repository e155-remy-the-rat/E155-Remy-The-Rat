// STM32L432KC_I2C.c
// Source code for I2Cfunctions

#include "STM32L432KC.h"
#include "STM32L432KC_I2C.h"
#include "STM32L432KC_RCC.h"

void i2cConfig() {

    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN); // Turn on GPIOA clock domain (GPIOAEN bit in AHB1ENR)
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  // turn on clock to I2C1

    // Initially assigning SPI pins
    pinMode(SDA, GPIO_ALT); // SPI1_SDA
    pinMode(SCL, GPIO_ALT); // SPI1_SCL

    // Set output speed type to high for SCK
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED10);

    // Select open drain output
    GPIOA->OTYPER |= (GPIO_OTYPER_IDR_9 | GPIO_OTYPER_IDR_10);

    // Select pull up
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD9, 1);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD10, 1);

    // Set to AF04 for I2C alternate functions
    GPIOA->AFR[1] |= _VAL2FLD(GPIO_AFRL_AFSEL1, 4);
    GPIOA->AFR[1] |= _VAL2FLD(GPIO_AFRL_AFSEL2, 4);

    // Slow down clock to APB1 Peripherals -> we want 16 MHz to I2C1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // divide 64 MHz clk by 4 to get 16 MHz

    // clear PE bit
    I2C1->CR1 &= ~I2C_CR1_PE;

    // configure ANFOFF and DNF in CR1

    // configure PRESC SDADEL SCLDEL SCLH SCLL in TIMINGR
    I2C1->TIMINGR |= _VAL2FLD(I2C_TIMINGR_PRESC, 0);
    I2C1->TIMINGR |= _VAL2FLD(I2C_TIMINGR_SCLL, 0x4);
    I2C1->TIMINGR |= _VAL2FLD(I2C_TIMINGR_SCLH, 0x2);
    I2C1->TIMINGR |= _VAL2FLD(I2C_TIMINGR_SDADEL, 0x0);
    I2C1->TIMINGR |= _VAL2FLD(I2C_TIMINGR_SCLDEL, 0x2);
    

    // configure NOSTRETCH

    // set AUTOEND
    I2C1->CR2 |= I2C_CR2_AUTOEND;

    // set PE bit
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2cWrite(int address, uint8_t * TX_Buffer, int num_bytes) {
    // configure address, assumes 7 bit address
    I2C1->CR2 |= (address << 1); // put seven bits of address in starting in bit 1 of the CR2 register
    
    // configure size of data package
    I2C1->CR2 |= _VAL2FLD(I2C_CR2_NBYTES, num_bytes);

    // set the read/not write bit to 0 for write
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;

    // signal to start
    I2C1->CR2 |= I2C_CR2_START;

    // send over each byte in the data package
    for (int i = 0; i < num_bytes; i++) {
      while(!(I2C1->ISR & I2C_ISR_TXE)); // wait until transmit buffer is empty
      I2C1->TXDR = TX_Buffer[i];
    }
}


void i2cRead(int address, uint8_t * RX_Buffer, int num_bytes) {
    // configure address, assumes 7 bit address
    I2C1->CR2 |= (address << 1); // put seven bits of address in starting in bit 1 of the CR2 register
    
    // configure size of data package
    I2C1->CR2 |= _VAL2FLD(I2C_CR2_NBYTES, num_bytes);

    // set the read/not write bit to 1 for read
    I2C1->CR2 |= I2C_CR2_RD_WRN;

    // signal to start
    I2C1->CR2 |= I2C_CR2_START;

   // read in each byte desired
    for (int i = 0; i < num_bytes; i++) {
      while(!(I2C1->ISR & I2C_ISR_RXNE)); // wait until data is ready to be read
      RX_Buffer[num_bytes - i - 1] = I2C1->RXDR;
    }
}
