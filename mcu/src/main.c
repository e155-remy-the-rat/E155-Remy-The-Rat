#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "lib/STM32L432KC_GPIO.h"
#include "lib/STM32L432KC_RCC.h"
#include "lib/STM32L432KC_FLASH.h"
#include "lib/STM32L432KC_USART.h"
#include "lib/STM32L432KC_TIM.h"
#include "/Users/alishachulani/Desktop/E155-Remy-The-Rat-mcu/mcu/lib/ICM20948.h"
#include "/Users/alishachulani/Desktop/E155-Remy-The-Rat-mcu/mcu/lib/MPU6050.h"
#include "/Users/alishachulani/Desktop/E155-Remy-The-Rat-mcu/mcu/Fusion/Fusion.h"


#include "lib/STM32L432KC_SPI.h"

#include "lib/nrf24.h"


#define LED_PIN PB3 // LED pin for blinking on Port B pin 5
#define BUFF_LEN 32
#define SDA PA10
#define SCL PA9
#define CONFIG_BUTTON PB5

uint8_t state_left, state_right;

int delay_val = 2;
int num_config_vals = 100;
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



#define RP_1              PA2
#define RP_2              PA7
#define RP_3              PA6
#define RP_4              PA5

void updateRatState(uint8_t recievedState)
{
    digitalWrite(RP_1, (recievedState & (1 << 0)) ? 1 : 0); // Bit 0 -> RP_1
    digitalWrite(RP_2, (recievedState & (1 << 1)) ? 1 : 0); // Bit 1 -> RP_2
    digitalWrite(RP_3, (recievedState & (1 << 2)) ? 1 : 0); // Bit 2 -> RP_3
    digitalWrite(RP_4, (recievedState & (1 << 3)) ? 1 : 0); // Bit 3 -> RP_4
}


uint8_t state = 0;
void RX(void)
{
  uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the address for RX pipe
  nRF24_DisableAA(0xFF); // disable ShockBurst
  nRF24_SetRFChannel(90); // set RF channel to 2490MHz
  nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
  nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
  nRF24_SetAddrWidth(5); // address width is 5 bytes
  nRF24_SetAddr(nRF24_PIPE1, ADDR); // program pipe address
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 10); // enable RX pipe#1 with Auto-ACK: disabled, payload length: 10 bytes
  nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
  nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
  // then pull CE pin to HIGH, and the nRF24 will start a receive...

  uint8_t nRF24_received_payload[32]; // buffer for payload
  uint8_t payload_length; // variable to store a length of received payload
  uint8_t pipe; // pipe number
  nRF24_CE_H; // start receiving
  while (1) {
      // constantly poll the status of RX FIFO...
      if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
          // the RX FIFO have some data, take a note what nRF24 can hold up to three payloads of 32 bytes...
          pipe = nRF24_ReadPayload(nRF24_received_payload, &payload_length); // read a payload to buffer
          nRF24_ClearIRQFlags(); // clear any pending IRQ bits
          // now the nRF24_payload buffer holds received data
          // payload_length variable holds a length of received data
          // pipe variable holds a number of the pipe which has received the data
          // ... do something with received data ...

          // We assume the first byte in the payload contains the rat state to send to the fpga
          updateRatState(nRF24_received_payload[0]);
          for (volatile int i = 0; i < 10000000; i++); // Only update once a delay
      }
  }

}


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



 int main(void) {

  GPIO_TypeDef * GPIO_PORT_PTR = gpioPinToBase(PB4);
  int pin_offset = gpioPinOffset(PB4);
  GPIO_PORT_PTR->PUPDR &= ~(0b11 << 8);
  configureFlash();
  configureClock();

  gpioEnable(GPIO_PORT_A);
  gpioEnable(GPIO_PORT_B);
  gpioEnable(GPIO_PORT_C);


  pinMode(RF_CE, GPIO_OUTPUT); 

  pinMode(RP_1, GPIO_OUTPUT); 
  pinMode(RP_2, GPIO_OUTPUT); 
  pinMode(RP_3, GPIO_OUTPUT); 
  pinMode(RP_4, GPIO_OUTPUT); 


  // variables
  uint8_t raw_data[12] = {};

  float accel_z_left, accel_y_left, accel_x_left, gyro_z_left, gyro_y_left, gyro_x_left;
  float accel_z_right, accel_y_right, accel_x_right, gyro_z_right, gyro_y_right, gyro_x_right;

  float accel_z_offset_left, accel_y_offset_left, accel_x_offset_left, gyro_z_offset_left, gyro_y_offset_left, gyro_x_offset_left;
  float accel_z_offset_right, accel_y_offset_right, accel_x_offset_right, gyro_z_offset_right, gyro_y_offset_right, gyro_x_offset_right;

  int sample_rate = 1 / (delay_val * 0.001);
 // int sample_rate = 1000;

  

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

  initSPI(0b100, 0, 0);

  int i = delay_val - 1;
  uint32_t counter = 0;

  while (1) {

    //RX(); // set this to either RX or TX and then upload to corresponding MCU
    

  //  if (state == 15) {
  //    state = 0;
  //  }
  //  else {
  //    state = state + 1;
  //  }
  //}

      // collect data
      collect_data(ICM_ADDRESS1, accel_conversion, gyro_conversion, &accel_x_left, &accel_y_left, &accel_z_left, &gyro_x_left, &gyro_y_left, &gyro_z_left);
      collect_data(ICM_ADDRESS2, accel_conversion, gyro_conversion, &accel_x_right, &accel_y_right, &accel_z_right, &gyro_x_right, &gyro_y_right, &gyro_z_right);
      
      counter = TIM6->CNT;

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
      const float delta_time = (float) (counter) * 0.001; // each counter count is a millisecond
      TIM6->EGR |= 1;     // Force update
      TIM6->SR &= ~(0x1); // Clear UIF

      // Update gyroscope AHRS algorithm
      FusionAhrsUpdateNoMagnetometer(&ahrs_left, gyroscope_left, accelerometer_left, delta_time);
      FusionAhrsUpdateNoMagnetometer(&ahrs_right, gyroscope_right, accelerometer_right, delta_time);

      // Print algorithm outputs
      const FusionEuler euler_left = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_left));
      const FusionVector earth_left = FusionAhrsGetEarthAcceleration(&ahrs_left);

      const FusionEuler euler_right = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_right));
      const FusionVector earth_right = FusionAhrsGetEarthAcceleration(&ahrs_right);


      // map angles of the left and right IMUs to states to send to motors
      // mappings are assuming initialization at horizontal position
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

      //i = 0;
    //}

    //i++;


    // delay for readability
    //delay_millis(TIM2, 1);

    state = (state_left<<2) | state_right; 


  
    TX();
  
  }

}
