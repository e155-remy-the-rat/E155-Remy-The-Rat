#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "main.h"

uint8_t state_left, state_right;

int delay_val = 2; // delay value in milliseconds, used to calculate sample rate
int num_config_vals = 100; // number of values averaged to configure the IMU

// accelerometer and gyroscope setings
int gyro_range = RANGE_250DPS;
int gyro_range_val = 250;
float gyro_conversion = RANGE_250DPS_CONVERSION;
int accel_range = RANGE_2G;
float accel_conversion = RANGE_2G_CONVERSION;


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
  uint16_t int_value = ((msb << 8) | lsb); // concatenate 
  float value = (float) (((msb >> 7) & 1) ? (int_value | ~((1 << 16) - 1)) : int_value); // convert from two's compliment to float
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
  //float accel_z[num_config_vals], accel_y[num_config_vals], accel_x[num_config_vals], gyro_z[num_config_vals], gyro_y[num_config_vals], gyro_x[num_config_vals];
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

  // collect num_config_vals sets of data to be averaged
  for (int i = 0; i < num_config_vals; i++) {
    collect_data(address, a_conversion, g_conversion, &ax, &ay, &az, &gx, &gy, &gz);

    // add data to offset to be averaged
    *accel_x_offset = *accel_x_offset + ax;
    *accel_y_offset = *accel_y_offset +  ay;
    *accel_z_offset = *accel_z_offset + az;
    *gyro_x_offset = *gyro_x_offset + gx;
    *gyro_y_offset = *gyro_y_offset + gy;
    *gyro_z_offset = *gyro_z_offset + gz;
  }

  // take the average 
  *accel_x_offset = *accel_x_offset/num_config_vals;
  *accel_y_offset = *accel_y_offset/num_config_vals;
  *accel_z_offset = *accel_z_offset/num_config_vals;
  *gyro_x_offset = *gyro_x_offset/num_config_vals;
  *gyro_y_offset = *gyro_y_offset/num_config_vals;
  *gyro_z_offset = *gyro_z_offset/num_config_vals;
}


uint8_t state = 0;
int TX(void)
{
  uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the TX address
  nRF24_DisableAA(0xFF); // disable ShockBurst
  nRF24_SetRFChannel(90); // set RF channel to 2490MHz
  nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
  nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
  nRF24_SetAddrWidth(5); // address width is 5 bytes
  nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power
  nRF24_SetAddr(nRF24_PIPETX, ADDR); // program TX address
  nRF24_SetOperationalMode(nRF24_MODE_TX); // switch transceiver to the TX mode
  nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
  // the nRF24 is ready for transmission, upload a payload, then pull CE pin to HIGH and it will transmit a packet...

  uint8_t status;
  uint8_t pBuf[8] = {'h', 'e', 'l', 'l', 'o', '1', '2', '3'};

  pBuf[0] = state_left + 8;
  pBuf[1] = state_right + 8;

  uint8_t length = 8;
  nRF24_WritePayload(pBuf, length); // transfer payload data to transceiver
  nRF24_CE_H; // assert CE pin (transmission starts)

  while (1) {
      status = nRF24_GetStatus();
      if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
          // transmission ended, exit loop
          break;
      }
  }

  nRF24_CE_L; // de-assert CE pin (nRF24 goes to StandBy-I mode)
  nRF24_ClearIRQFlags(); // clear any pending IRQ flags
  if (status & nRF24_FLAG_MAX_RT) {
      // Auto retransmit counter exceeds the programmed maximum limit (payload in FIFO is not removed)
      // Also the software can flush the TX FIFO here...
      return 9;
  }
  if (status & nRF24_FLAG_TX_DS) {
      // Successful transmission
      return 1;
  }
  // In fact that should not happen
  return 10;

}



 int main_body(void) {

  GPIO_TypeDef * GPIO_PORT_PTR = gpioPinToBase(PB4);
  int pin_offset = gpioPinOffset(PB4);
  GPIO_PORT_PTR->PUPDR &= ~(0b11 << 8);

  // configure flash and latency and set clock to run at 84 MHz - this is important for I2C
  configureFlash();
  configureClock();

  // enable GPIO
  gpioEnable(GPIO_PORT_A);
  gpioEnable(GPIO_PORT_B);
  gpioEnable(GPIO_PORT_C);

  // configure SPI pins
  pinMode(RF_CE, GPIO_OUTPUT); 
  pinMode(RP_1, GPIO_OUTPUT); 
  pinMode(RP_2, GPIO_OUTPUT); 
  pinMode(RP_3, GPIO_OUTPUT); 
  pinMode(RP_4, GPIO_OUTPUT); 

  // Configure timers
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
  initTIM(TIM2);
  initTIM(TIM6);

  // Configure onboard LED
  pinMode(LED_PIN, GPIO_OUTPUT);
  digitalWrite(LED_PIN, 0);

  // Configure SPI communication  
  initSPI(0b100, 0, 0);

  // Configure I2C communication
  i2cConfig();

  // Configure IMU chips
  configICM(ICM_ADDRESS1);
  configICM(ICM_ADDRESS2);

  // configure acceleration/gyroscope readings
  configAccelGyroICM(ICM_ADDRESS1, 6, 6, accel_range, gyro_range);
  configAccelGyroICM(ICM_ADDRESS2, 6, 6, accel_range, gyro_range);


  // variables to store data and offsets
  uint8_t raw_data[12] = {};
  float accel_z_left, accel_y_left, accel_x_left, gyro_z_left, gyro_y_left, gyro_x_left;
  float accel_z_right, accel_y_right, accel_x_right, gyro_z_right, gyro_y_right, gyro_x_right;
  float accel_z_offset_left, accel_y_offset_left, accel_x_offset_left, gyro_z_offset_left, gyro_y_offset_left, gyro_x_offset_left;
  float accel_z_offset_right, accel_y_offset_right, accel_x_offset_right, gyro_z_offset_right, gyro_y_offset_right, gyro_x_offset_right;

  // sample rate in Hz
  int sample_rate = 1 / (delay_val * 0.001);

  // delay 10 seconds intially to allow user time to hold sensors flat
  // onboard LED flashes while waiting for visual cue
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, 1);
    delay_millis(TIM2, 500);
    digitalWrite(LED_PIN, 0);
    delay_millis(TIM2, 500);
  }

  // collect and average data from each IMU to configure all offsets
  average_data(ICM_ADDRESS1, accel_conversion, gyro_conversion, &accel_x_offset_left, &accel_y_offset_left, &accel_z_offset_left, &gyro_x_offset_left, &gyro_y_offset_left, &gyro_z_offset_left);
  average_data(ICM_ADDRESS2, accel_conversion, gyro_conversion, &accel_x_offset_right, &accel_y_offset_right, &accel_z_offset_right, &gyro_x_offset_right, &gyro_y_offset_right, &gyro_z_offset_right);

  // Initialise algorithms for each IMU 
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
            .gyroscopeRange = gyro_range_val, 
            .accelerationRejection = 2.0f,
            .recoveryTriggerPeriod = 5 * sample_rate, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs_left, &settings);
  FusionAhrsSetSettings(&ahrs_right, &settings);

  // counter used to calculate time between data readings
  uint32_t counter = 0;

  while (1) {
    // collect data from each IMU
    collect_data(ICM_ADDRESS1, accel_conversion, gyro_conversion, &accel_x_left, &accel_y_left, &accel_z_left, &gyro_x_left, &gyro_y_left, &gyro_z_left); // left
    collect_data(ICM_ADDRESS2, accel_conversion, gyro_conversion, &accel_x_right, &accel_y_right, &accel_z_right, &gyro_x_right, &gyro_y_right, &gyro_z_right); // right

    // read timer 
    counter = TIM6->CNT;

    // apply configuration offsets
    accel_x_left = accel_x_left - accel_x_offset_left;
    accel_y_left = accel_y_left - accel_y_offset_left;
    accel_z_left = accel_z_left - accel_z_offset_left + 1; // the algorithm expects neutral z acceleration to be 1g, so need to add back 1
    gyro_x_left = gyro_x_left - gyro_x_offset_left;
    gyro_y_left = gyro_y_left - gyro_y_offset_left;
    gyro_z_left = gyro_z_left - gyro_z_offset_left;

    accel_x_right = accel_x_right - accel_x_offset_right;
    accel_y_right = accel_y_right - accel_y_offset_right;
    accel_z_right = accel_z_right - accel_z_offset_right + 1; // the algorithm expects neutral z acceleration to be 1g, so need to add back 1
    gyro_x_right = gyro_x_right - gyro_x_offset_right;
    gyro_y_right = gyro_y_right - gyro_y_offset_right;
    gyro_z_right = gyro_z_right - gyro_z_offset_right;

    // format IMU data for each IMU into vector to be passed into the algorithm
    FusionVector gyroscope_left = {gyro_x_left, gyro_y_left, gyro_z_left};
    FusionVector accelerometer_left = {accel_x_left, accel_y_left, accel_z_left}; 
    FusionVector gyroscope_right = {gyro_x_right, gyro_y_right, gyro_z_right}; 
    FusionVector accelerometer_right = {accel_x_right, accel_y_right, accel_z_right}; 

    // Calculate delta time (in seconds) to account for sample clock error
    const float delta_time = (float) (counter) * 0.001; // each counter count is a millisecond
    
    // reset clock
    TIM6->EGR |= 1;     // Force update
    TIM6->SR &= ~(0x1); // Clear UIF

    // Update AHRS algorithm
    FusionAhrsUpdateNoMagnetometer(&ahrs_left, gyroscope_left, accelerometer_left, delta_time);
    FusionAhrsUpdateNoMagnetometer(&ahrs_right, gyroscope_right, accelerometer_right, delta_time);

    // Get algorithm outputs for each IMU
    const FusionEuler euler_left = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_left));
    const FusionVector earth_left = FusionAhrsGetEarthAcceleration(&ahrs_left);
    const FusionEuler euler_right = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_right));
    const FusionVector earth_right = FusionAhrsGetEarthAcceleration(&ahrs_right);

    // map angles of the left and right IMUs to states to send to motors
    // mappings are assuming initialization at horizontal position
    // LEFT IMU
    if (euler_left.angle.pitch >= 40.0) {
      state_left = 0b00;
    }
    else if (euler_left.angle.pitch < 40.0 && euler_left.angle.pitch >= 0.0) {
      state_left = 0b01;
    }
    else if (euler_left.angle.pitch < 0.0 && euler_left.angle.pitch >= -40.0) {
      state_left = 0b10;
    }
    else {
      state_left = 0b11;
    }

    // RIGHT IMU
    if (euler_right.angle.pitch >= 40.0) { 
      state_right = 0b11;
    }
    else if (euler_right.angle.pitch < 40.0 && euler_right.angle.pitch >= 0.0) {
      state_right = 0b10;
    }
    else if (euler_right.angle.pitch < 0.0 && euler_right.angle.pitch >= -40.0) {
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

    // Print to monitor
    printf("RIGHT:\n");
    printf("GyroX: % f, GyroY: % f, GyroZ: % f \n", gyro_x_right, gyro_y_right, gyro_z_right);
    printf("Roll % 0.1f, Pitch % 0.1f, Yaw % 0.1f\n",
            euler_right.angle.roll, euler_right.angle.pitch, euler_right.angle.yaw);
    printf("State: %u\n\n\n", state_right);


    // state transmitted to head is a combination of the left and right states
    state = (state_left<<2) | state_right; 


    // transmit data
    TX();
    
  }
}
