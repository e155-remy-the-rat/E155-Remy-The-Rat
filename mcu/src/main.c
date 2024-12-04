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
int gyro_range = RANGE_250DPS;
int gyro_range_val = 250;
float gyro_conversion = RANGE_250DPS_CONVERSION;
int accel_range = RANGE_2G;
float accel_conversion = RANGE_2G_CONVERSION;

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
void collect_data(int address, float a_conversion, float g_conversion, float * accel_x, float * accel_y, float * accel_z, float * gyro_x, float * gyro_y, float * gyro_z) {
  uint8_t raw_data[12] = {};

  readAccelGyroICM(address, raw_data);

  // accel values in g's
  *accel_x = i2c_val_to_float(raw_data[0], raw_data[1])/a_conversion;
  *accel_y = i2c_val_to_float(raw_data[2], raw_data[3])/a_conversion;
  *accel_z = i2c_val_to_float(raw_data[4], raw_data[5])/a_conversion;
    
  // gyro values in dps
  *gyro_x = i2c_val_to_float(raw_data[6], raw_data[7])/g_conversion;
  *gyro_y = i2c_val_to_float(raw_data[8], raw_data[9])/g_conversion;
  *gyro_z = i2c_val_to_float(raw_data[10], raw_data[11])/g_conversion;
}

// function used for calibration
void average_data(int address, float a_conversion, float g_conversion, float * accel_x_offset, float * accel_y_offset, float * accel_z_offset, float * gyro_x_offset, float * gyro_y_offset, float * gyro_z_offset) {
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
    collect_data(address, a_conversion, g_conversion, &ax, &ay, &az, &gx, &gy, &gz);

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
  // variables
  uint8_t raw_data[12] = {};

  float accel_z_left, accel_y_left, accel_x_left, gyro_z_left, gyro_y_left, gyro_x_left;
  float accel_z_right, accel_y_right, accel_x_right, gyro_z_right, gyro_y_right, gyro_x_right;

  float accel_z_offset_left, accel_y_offset_left, accel_x_offset_left, gyro_z_offset_left, gyro_y_offset_left, gyro_x_offset_left;
  float accel_z_offset_right, accel_y_offset_right, accel_x_offset_right, gyro_z_offset_right, gyro_y_offset_right, gyro_x_offset_right;

  int sample_rate = 1 / (delay_val * 0.001);

  uint8_t state_left, state_right;
  //float speed = 0;
  //float position = 0;


  // Configure flash latency and set clock to run at 84 MHz
  configureFlash();
  configureClock();
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
  initTIM(TIM2);
  initTIM(TIM6);

  // Enable GPIO clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

  // Configure onboard LED
  pinMode(LED_PIN, GPIO_OUTPUT);
  digitalWrite(LED_PIN, 0);

  // Configure I2C communication
  i2cConfig();

  // Configure IMU chips
  configICM(ICM_ADDRESS1);
  configICM(ICM_ADDRESS2);

  // configure acceleration/gyroscope readings
  configAccelGyroICM(ICM_ADDRESS1, 6, 6, accel_range, gyro_range);
  configAccelGyroICM(ICM_ADDRESS2, 6, 6, accel_range, gyro_range);


  // delay 10 seconds intially to allow user to hold sensors flat
  // onboard LED flashes while waiting
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, 1);
    delay_millis(TIM2, 500);
    digitalWrite(LED_PIN, 0);
    delay_millis(TIM2, 500);
  }

  // collect data from IMU
  average_data(ICM_ADDRESS1, accel_conversion, gyro_conversion, &accel_x_offset_left, &accel_y_offset_left, &accel_z_offset_left, &gyro_x_offset_left, &gyro_y_offset_left, &gyro_z_offset_left);
  average_data(ICM_ADDRESS2, accel_conversion, gyro_conversion, &accel_x_offset_right, &accel_y_offset_right, &accel_z_offset_right, &gyro_x_offset_right, &gyro_y_offset_right, &gyro_z_offset_right);

  // Initialise algorithms
  FusionOffset offset_left;
  FusionAhrs ahrs_left;
  FusionOffsetInitialise(&offset_left, sample_rate);
  FusionAhrsInitialise(&ahrs_left);

  FusionOffset offset_right;
  FusionAhrs ahrs_right;
  FusionOffsetInitialise(&offset_right, sample_rate);
  FusionAhrsInitialise(&ahrs_right);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = gyro_range_val, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 2.0f,
            .recoveryTriggerPeriod = 5 * sample_rate, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs_left, &settings);
  FusionAhrsSetSettings(&ahrs_right, &settings);


  while(1) {
    // collect data
    collect_data(ICM_ADDRESS1, accel_conversion, gyro_conversion, &accel_x_left, &accel_y_left, &accel_z_left, &gyro_x_left, &gyro_y_left, &gyro_z_left);
    collect_data(ICM_ADDRESS2, accel_conversion, gyro_conversion, &accel_x_right, &accel_y_right, &accel_z_right, &gyro_x_right, &gyro_y_right, &gyro_z_right);

    // apply configuration offsets
    accel_x_left = accel_x_left - accel_x_offset_left;
    accel_y_left = accel_y_left - accel_y_offset_left;
    accel_z_left = accel_z_left - accel_z_offset_left + 1;
    gyro_x_left = gyro_x_left - gyro_x_offset_left;
    gyro_y_left = gyro_y_left - gyro_y_offset_left;
    gyro_z_left = gyro_z_left - gyro_z_offset_left;

    accel_x_right = accel_x_right - accel_x_offset_right;
    accel_y_right = accel_y_right - accel_y_offset_right;
    accel_z_right = accel_z_right - accel_z_offset_right + 1;
    gyro_x_right = gyro_x_right - gyro_x_offset_right;
    gyro_y_right = gyro_y_right - gyro_y_offset_right;
    gyro_z_right = gyro_z_right - gyro_z_offset_right;

       
    // Acquire latest sensor data
    //uint32_t counter = TIM6->CNT; // replace this with actual gyroscope timestamp
    FusionVector gyroscope_left = {gyro_x_left, gyro_y_left, gyro_z_left}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer_left = {accel_x_left, accel_y_left, accel_z_left}; // replace this with actual accelerometer data in g
    FusionVector gyroscope_right = {gyro_x_right, gyro_y_right, gyro_z_right}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer_right = {accel_x_right, accel_y_right, accel_z_right}; // replace this with actual accelerometer data in g
  

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    //const float delta_time = (float) (counter) * 0.001; // each counter count is a millisecond
    //TIM6->EGR |= 1;     // Force update
    //TIM6->SR &= ~(0x1); // Clear UIF

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdateNoMagnetometer(&ahrs_left, gyroscope_left, accelerometer_left, 0.1);
    FusionAhrsUpdateNoMagnetometer(&ahrs_right, gyroscope_right, accelerometer_right, 0.1);

    // Print algorithm outputs
    const FusionEuler euler_left = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_left));
    const FusionVector earth_left = FusionAhrsGetEarthAcceleration(&ahrs_left);

    const FusionEuler euler_right = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_right));
    const FusionVector earth_right = FusionAhrsGetEarthAcceleration(&ahrs_right);


    // map angles of the left and right IMUs to states to send to motors
    // mappings are assuming initialization at horizontal position
    if (euler_left.angle.pitch >= 50.0) {
      state_left = 0b00;
    }
    else if (euler_left.angle.pitch < 50.0 && euler_left.angle.pitch >= 15.0) {
      state_left = 0b01;
    }
    else if (euler_left.angle.pitch < 15.0 && euler_left.angle.pitch >= -20.0) {
      state_left = 0b10;
    }
    else {
      state_left = 0b11;
    }

    if (euler_right.angle.pitch >= 50.0) { 
      state_right = 0b11;
    }
    else if (euler_right.angle.pitch < 50.0 && euler_right.angle.pitch >= 15.0) {
      state_right = 0b10;
    }
    else if (euler_left.angle.pitch < 15.0 && euler_left.angle.pitch >= -20.0) {
      state_right = 0b01;
    }
    else {
      state_right = 0b00;
    }

    // Print to monitor
    printf("LEFT:\n");
    printf("GyroX: % f, GyroY: % f, GyroZ: % f \n", gyro_x_left, gyro_y_left, gyro_z_left);
    printf("Roll % 0.1f, Pitch % 0.1f, Yaw % 0.1f\n",
            euler_left.angle.roll, euler_left.angle.pitch, euler_left.angle.yaw);
    printf("State: %u\n\n", state_left);

    //printf("RIGHT:\n");
    //printf("GyroX: % f, GyroY: % f, GyroZ: % f \n", gyro_x_right, gyro_y_right, gyro_z_right);
    //printf("Roll % 0.1f, Pitch % 0.1f, Yaw % 0.1f\n\n",
    //        euler_right.angle.roll, euler_right.angle.pitch, euler_right.angle.yaw);
    //printf("State: %u\n\n", state_right); 

    //printf("AccelX: % f, AccelY: % f, AccelZ: % f \n", accel_x, accel_y, accel_z);
    //printf("X % f, Y % f, Z % f\n\n",
    //        earth.axis.x, earth.axis.y, earth.axis.z);


    // delay for readability
    delay_millis(TIM2, delay_val);
  }

}



//-- CODE GRAVEYARD --//


  //// Define calibration (replace with actual calibration data if available)
  //const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  //const FusionVector gyroscopeSensitivity = {1.0/RANGE_250DPS_CONVERSION, 1.0/RANGE_250DPS_CONVERSION, 1.0/RANGE_250DPS_CONVERSION}; //in dps per lsb
  //const FusionVector gyroscopeOffset = {gyro_x_offset, gyro_y_offset, gyro_z_offset};
  //const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  //const FusionVector accelerometerSensitivity = {1.0/RANGE_2G_CONVERSION, 1.0/RANGE_2G_CONVERSION, 1.0/RANGE_2G_CONVERSION};
  //const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};


  //// Apply calibration
    //gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    //accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
   
    //// Update gyroscope offset correction algorithm
    //gyroscope = FusionOffsetUpdate(&offset, gyroscope);


        //speed = speed + earth.axis.z*delta_time;
    //position = position + speed*delta_time  + 0.5*earth.axis.z*delta_time*delta_time;
    //printf("Z Accel: % f, Z Vel: % f, Z Position: % f\n\n\n", earth.axis.z, speed, position);

    //if ( position < 0) {
    //  float gyro_x_rand, gyro_y_rand, gyro_z_rand, accel_x_rand, accel_y_rand;
    //  position = 0; 
    //  average_data(ICM_ADDRESS1, RANGE_2G_CONVERSION, RANGE_250DPS_CONVERSION, &accel_x_rand, &accel_y_rand, &accel_z_offset, &gyro_x_rand, &gyro_y_rand, &gyro_z_rand);
    //  TIM6->EGR |= 1;     // Force update
    //  TIM6->SR &= ~(0x1); // Clear UIF

    //}

