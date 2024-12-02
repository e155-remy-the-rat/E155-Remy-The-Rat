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
  uint8_t temp_msb;
  uint8_t temp_lsb;

  // Configure flash latency and set clock to run at 84 MHz
  configureFlash();
  configureClock();
  initTIM(TIM16);

  // Enable GPIO clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

  // Configure I2C communication
  i2cConfig();

  // Configure IMU chip
  configICM();

  //uint8_t read_who_am_i[1] = {0x00};
  //uint8_t who_am_i[1];

  //// read temp
  //readTempICM(&temp_msb, &temp_lsb);
  ////i2cWrite(ICM_ADDRESS, read_who_am_i, 1, 0);
  ////i2cRead(ICM_ADDRESS, who_am_i, 1);

  //int temp_val = (temp_msb << 8) | temp_lsb;
  //int temp_val_convert = (temp_val & (1 << 16)) ? (-1 * (float)(~temp_val + 1)) : temp_val;
  float room_temp_offset = 0.0;
  float temp_sensitivity = 333.87;
  //float temp = (((float)temp_val_convert - room_temp_offset)/(temp_sensitivity)) + 21.0;
  

  //uint8_t temp_lsb[1] = {};
  //uint8_t temp_msb[1] = {};

  float temp;
  int temp_val, temp_val_convert;

  while(1) {
    readTempICM(&temp_msb, &temp_lsb);
    temp_val = (temp_msb << 8) | temp_lsb;
    temp_val_convert = ((temp_msb >> 7) & 1) ? (temp_val | ~((1 << 16) - 1)) : temp_val;
    temp = (((float)temp_val_convert - room_temp_offset)/(temp_sensitivity)) + 21.0;
    //delay_millis(TIM16, 10);

   // low pass filter
    //float filtered_temp_val = filtered_temp_val - (beta * (filtered_temp_val - (float)temp_val))
  }

}
