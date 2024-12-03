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
// Functions
////////////////////////////////////////////////

// Function used by printf to send characters to the laptop
int _write(int file, char *ptr, int len) {
  int i = 0;
  for (i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}


////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

int main(void) {
  // Configure flash latency and set clock to run at 84 MHz
  configureFlash();
  configureClock();
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  initTIM(TIM2);

  // Enable GPIO clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

  // Configure I2C communication
  i2cConfig();

  // Configure IMU chips
  configICM(ICM_ADDRESS1);
  //configICM(ICM_ADDRESS2);

  // WHO AM I TEST 
  //uint8_t read_who_am_i[1] = {0x00};
  //uint8_t who_am_i[1];
  //i2cWrite(ICM_ADDRESS1, read_who_am_i, 1, 0);
  //i2cRead(ICM_ADDRESS1, who_am_i, 1);

  configAccelICM(ICM_ADDRESS1, 6, RANGE_2G);

  uint8_t accel_data[6] = {};
  uint16_t accelx_val, accely_val, accelz_val;
  float accelz, accely, accelx;

  while(1) {
    readAccelICM(ICM_ADDRESS1, accel_data);
    accelx_val = ((accel_data[0] << 8) | accel_data[1]);
    accelx = (float) (((accel_data[0] >> 7) & 1) ? (accelx_val | ~((1 << 16) - 1)) : accelx_val);
    
    accely_val = (float)((accel_data[2] << 8) | accel_data[3]);
    accely = (float) (((accel_data[2] >> 7) & 1) ? (accely_val | ~((1 << 16) - 1)) : accely_val);
    
    accelz_val = (float)((accel_data[4] << 8) | accel_data[5]);
    accelz = (float) (((accel_data[4] >> 7) & 1) ? (accelz_val | ~((1 << 16) - 1)) : accelz_val);

    accelx = accelx/RANGE_2G_CONVERSION;
    accely = accely/RANGE_2G_CONVERSION;
    accelz = accelz/RANGE_2G_CONVERSION;

    printf("AccelX: %f, AccelY: %f, AccelZ: %f \n", accelx, accely, accelz);
    delay_millis(TIM2, 100);

  }

  //float room_temp_offset = 0.0;
  //float temp_sensitivity = 333.87;
  
  //uint8_t temp_msb1, temp_msb2;
  //uint8_t temp_lsb1, temp_lsb2;
  //float temp1, temp2;
  //int temp_val1, temp_val2, temp_val_convert1, temp_val_convert2;

  //while(1) {
  //  // send/receive I2C for temp value
  //  readTempICM(ICM_ADDRESS1, &temp_msb1, &temp_lsb1);
  //  readTempICM(ICM_ADDRESS2, &temp_msb2, &temp_lsb2);
    
  //  // convert temp value
  //  temp_val1 = (temp_msb1 << 8) | temp_lsb1;
  //  temp_val_convert1 = ((temp_msb1 >> 7) & 1) ? (temp_val1 | ~((1 << 16) - 1)) : temp_val1;
  //  temp1 = (((float)temp_val_convert1 - room_temp_offset)/(temp_sensitivity)) + 21.0;
    
  //  temp_val2 = (temp_msb2 << 8) | temp_lsb2;
  //  temp_val_convert2 = ((temp_msb2 >> 7) & 1) ? (temp_val2 | ~((1 << 16) - 1)) : temp_val2;
  //  temp2 = (((float)temp_val_convert2 - room_temp_offset)/(temp_sensitivity)) + 21.0;
    
  //  printf("Temp 1: %f,  Temp 2: %f \n", temp1, temp2);
  //  delay_millis(TIM2, 100);

  // // low pass filter
  //  //float filtered_temp_val = filtered_temp_val - (beta * (filtered_temp_val - (float)temp_val))
  //}

}
