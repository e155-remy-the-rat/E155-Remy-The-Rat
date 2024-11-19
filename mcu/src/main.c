/** 
    Main: Contains main function for I2C communication with the IMUs, filtering of data, 
    and SPI communication with RF transmitter.
    @file main.c
    @author Marina Ring
    @version 1.0 11/17/2024
*/
#include <stdio.h>
#include <stm32l432xx.h>
#include "main.h"

////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////

float beta = 0.025;

////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

int main(void) {

  // Configure flash latency and set clock to run at 84 MHz
  configureFlash();
  configureClock();
  initTIM(TIM16);

  // Enable GPIO clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

  // Configure I2C communication
  i2cConfig();



  //configMPU();

  uint8_t read_temp_msb[1] = {0x41};
  uint8_t read_temp_lsb[1] = {0x42};
  uint8_t temp_msb[1] = {0};
  uint8_t temp_lsb[1] = {0};

  // read temp
  readMPU(temp_msb, temp_lsb);

  int temp_val = (temp_msb[0] << 8) | temp_lsb[0];
  float temp = ((float)temp_val) / 340 + 36.53;
  

  //uint8_t temp_lsb[1] = {};
  //uint8_t temp_msb[1] = {};

  //while(1) {
  //  readMPU(temp_msb, temp_lsb);
  //  int temp_val = (temp_msb[0] << 8) | temp_lsb[0];

  // // low pass filter
  //  float filtered_temp_val = filtered_temp_val - (beta * (filtered_temp_val - (float)temp_val))

  //  float temp = (filtered_temp_val) / 340 + 36.53;
  //}

}
