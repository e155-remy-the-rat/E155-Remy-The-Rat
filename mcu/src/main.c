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

float i2c_val_to_float(uint8_t msb, uint8_t lsb) {
  uint16_t int_value = ((msb << 8) | lsb);
  float value = (float) (((msb >> 7) & 1) ? (int_value | ~((1 << 16) - 1)) : int_value);
  return value;
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

  configAccelGyroICM(ICM_ADDRESS1, 6, 0, RANGE_2G, RANGE_2000DPS);

  uint8_t raw_data[12] = {};
  float accel_z, accel_y, accel_x, gyro_z, gyro_y, gyro_x;

  FusionAhrs ahrs;
  FusionAhrsInitialise(&ahrs);

  while(1) {
    readAccelGyroICM(ICM_ADDRESS1, raw_data);

    // accel values in g's
    accel_x = i2c_val_to_float(raw_data[0], raw_data[1])/RANGE_2G_CONVERSION;
    accel_y = i2c_val_to_float(raw_data[2], raw_data[3])/RANGE_2G_CONVERSION;
    accel_z = i2c_val_to_float(raw_data[4], raw_data[5])/RANGE_2G_CONVERSION;
    
    // gyro values in dps
    gyro_x = i2c_val_to_float(raw_data[6], raw_data[7]);
    gyro_y = i2c_val_to_float(raw_data[8], raw_data[9]);
    gyro_z = i2c_val_to_float(raw_data[10], raw_data[11]);

    //printf("AccelX: %f, AccelY: %f, AccelZ: %f \n", accel_x, accel_y, accel_z);
    //printf("GyroX: %f, GyroY: %f, GyroZ: %f \n", gyro_x, gyro_y, gyro_z);

    FusionVector accelerometer = {accel_x, accel_y, accel_z};
    FusionVector gyroscope = {gyro_x, gyro_y, gyro_z};
    
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.1);

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
   
    printf("GyroX: %f, GyroY: %f, GyroZ: %f \n", gyro_x, gyro_y, gyro_z);
    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n\n ", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    
    delay_millis(TIM2, 100);
  }

}
