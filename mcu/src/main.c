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

int delay_val = 100;
int num_config_vals = 100;

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

// function to concatenate two eight bit numbers and convert it from its two's compliment form
float i2c_val_to_float(uint8_t msb, uint8_t lsb) {
  uint16_t int_value = ((msb << 8) | lsb);
  float value = (float) (((msb >> 7) & 1) ? (int_value | ~((1 << 16) - 1)) : int_value);
  return value;
}


// function that collects data and calculates acceleration and rotation data
void collect_data(int address, int accel_conversion, int gyro_conversion, float * accel_x, float * accel_y, float * accel_z, float * gyro_x, float * gyro_y, float * gyro_z) {
  uint8_t raw_data[12] = {};

  readAccelGyroICM(address, raw_data);

  // accel values in g's
  *accel_x = i2c_val_to_float(raw_data[0], raw_data[1])/accel_conversion;
  *accel_y = i2c_val_to_float(raw_data[2], raw_data[3])/accel_conversion;
  *accel_z = i2c_val_to_float(raw_data[4], raw_data[5])/accel_conversion;
    
  // gyro values in dps
  *gyro_x = i2c_val_to_float(raw_data[6], raw_data[7])/gyro_conversion;
  *gyro_y = i2c_val_to_float(raw_data[8], raw_data[9])/gyro_conversion;
  *gyro_z = i2c_val_to_float(raw_data[10], raw_data[11])/gyro_conversion;
}

// function used for calibration
void average_data(int address, int accel_conversion, int gyro_conversion, float * accel_x_offset, float * accel_y_offset, float * accel_z_offset, float * gyro_x_offset, float * gyro_y_offset, float * gyro_z_offset) {
  float accel_z[num_config_vals], accel_y[num_config_vals], accel_x[num_config_vals], gyro_z[num_config_vals], gyro_y[num_config_vals], gyro_x[num_config_vals];
  float ax, ay, az, gx, gy, gz;

  *accel_x_offset = 0;
  *accel_y_offset = 0;
  *accel_z_offset = 0;
  *gyro_x_offset = 0;
  *gyro_y_offset = 0;
  *gyro_z_offset = 0;

  ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;
  
  for (int i = 0; i < num_config_vals; i++) {
    collect_data(ICM_ADDRESS1, RANGE_2G_CONVERSION, RANGE_250DPS_CONVERSION, &ax, &ay, &az, &gx, &gy, &gz);

    accel_x[i] = ax;
    accel_y[i] = ay;
    accel_z[i] = az;
    gyro_x[i] = gx;
    gyro_y[i] = gy;
    gyro_z[i] = gz;
  }

  for (int i = 0; i < num_config_vals; i++) {
    *accel_x_offset = *accel_x_offset + accel_x[i];
    *accel_y_offset = *accel_y_offset + accel_y[i];
    *accel_z_offset = *accel_z_offset + accel_z[i];
    *gyro_x_offset = *gyro_x_offset + gyro_x[i];
    *gyro_y_offset = *gyro_y_offset + gyro_y[i];
    *gyro_z_offset = *gyro_z_offset + gyro_z[i];
  }

  *accel_x_offset = *accel_x_offset/num_config_vals;
  *accel_y_offset = *accel_y_offset/num_config_vals;
  *accel_z_offset = *accel_z_offset/num_config_vals;
  *gyro_x_offset = *gyro_x_offset/num_config_vals;
  *gyro_y_offset = *gyro_y_offset/num_config_vals;
  *gyro_z_offset = *gyro_z_offset/num_config_vals;
}


////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

int main(void) {
  // Configure flash latency and set clock to run at 84 MHz
  configureFlash();
  configureClock();
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  RCC->APB1ENR1 |= RCC_APB2ENR_TIM16EN;
  initTIM(TIM2);
  initTIM(TIM16);

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

  configAccelGyroICM(ICM_ADDRESS1, 6, 6, RANGE_2G, RANGE_250DPS);

  //if (digitalRead(CONFIG_BUTTON)) {
    //  for (int i = 0; i < num_config_vals; i++) {
    //    collect_data(ICM_ADDRESS1, RANGE_2G_CONVERSION, RANGE_250DPS_CONVERSION, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

        
    //  }
    //}

  uint8_t raw_data[12] = {};
  float accel_z, accel_y, accel_x, gyro_z, gyro_y, gyro_x;
  float accel_z_offset, accel_y_offset, accel_x_offset, gyro_z_offset, gyro_y_offset, gyro_x_offset;
  int sample_rate = 1 / (delay_val * 0.001);

  // collect data from IMU
  average_data(ICM_ADDRESS1, RANGE_2G_CONVERSION, RANGE_250DPS_CONVERSION, &accel_x_offset, &accel_y_offset, &accel_z_offset, &gyro_x_offset, &gyro_y_offset, &gyro_z_offset);

  // Define calibration (replace with actual calibration data if available)
  const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector gyroscopeSensitivity = {1.0/RANGE_250DPS_CONVERSION, 1.0/RANGE_250DPS_CONVERSION, 1.0/RANGE_250DPS_CONVERSION}; //in dps per lsb
  const FusionVector gyroscopeOffset = {gyro_x_offset, gyro_y_offset, gyro_z_offset};
  const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector accelerometerSensitivity = {1.0/RANGE_2G_CONVERSION, 1.0/RANGE_2G_CONVERSION, 1.0/RANGE_2G_CONVERSION};
  const FusionVector accelerometerOffset = {accel_x_offset, accel_y_offset, accel_z_offset};


  // Initialise algorithms
  FusionOffset offset;
  FusionAhrs ahrs;

  FusionOffsetInitialise(&offset, sample_rate);
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 250, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 5.0f,
            .recoveryTriggerPeriod = 5 * sample_rate, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);


  while(1) {
    collect_data(ICM_ADDRESS1, RANGE_2G_CONVERSION, RANGE_250DPS_CONVERSION, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

    //printf("AccelX: %f, AccelY: %f, AccelZ: %f \n", accel_x, accel_y, accel_z);
    printf("GyroX: %f, GyroY: %f, GyroZ: %f \n", gyro_x, gyro_y, gyro_z);

    
    // Acquire latest sensor data
    const uint32_t counter = TIM16->CNT; // replace this with actual gyroscope timestamp
    FusionVector gyroscope = {gyro_x - gyro_x_offset, gyro_y - gyro_y_offset, gyro_z - gyro_z_offset}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = {accel_x, accel_y, accel_z}; // replace this with actual accelerometer data in g
  
    //// Apply calibration
    //gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    //accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
   
    //// Update gyroscope offset correction algorithm
    //gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    //// Calculate delta time (in seconds) to account for gyroscope sample clock error
    //const float delta_time = (float) (counter) * 0.001; // each counter count is a millisecond
    //TIM16->CNT = 0;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.1);

    // Print algorithm outputs
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);


    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n",
            euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    printf("AccelX: %f, AccelY: %f, AccelZ: %f \n", accel_x, accel_y, accel_z);
    printf("X %0.1f, Y %0.1f, Z %0.1f\n\n",
            earth.axis.x, earth.axis.y, earth.axis.z);
    //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
    //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
    //        earth.axis.x, earth.axis.y, earth.axis.z);



    delay_millis(TIM2, delay_val);
  }

}
