// ICM20948.c
// Source code for ICM 20948 functions

#include "ICM20948.h"


// function to configure the ICM, enabling accelerometer and gyroscope
void configICM(int address) {
    // reset chip, clear sleep mode, enable accel and gyro
    uint8_t reset[3] = {0x06, 0b00000010, 0b00000000};
    i2cWrite(address, reset, 3, 1);
};


// function to read a temperature from the ICM
void readTempICM(int address, uint8_t * temp_msb, uint8_t * temp_lsb) {
    uint8_t read_temp_msb[1] = {0x39}; // address of MSB
    uint8_t read_temp_lsb[1] = {0x3A}; // address of LSB

    // read MSB
    i2cWrite(address, read_temp_msb, 1, 0);
    i2cRead(address, temp_msb, 1);

    // read LSB
    i2cWrite(address, read_temp_lsb, 1, 0);
    i2cRead(address, temp_lsb, 1);
};


// function to confgure the accelerometer and gyroscope on the ICM
void configAccelGyroICM(int address, uint8_t accel_dlpfcfg, uint8_t gyro_dlpfcfg, uint8_t accel_fs_sel, uint8_t gyro_fs_sel) {
  // set sample rate divisor
  uint8_t accel_smplrt[3] = {0x10, 0x00, 0x00};
  i2cWrite(address, accel_smplrt, 3, 1);

  uint8_t gyro_smplrt[2] = {0x00, 0x00};
  i2cWrite(address, gyro_smplrt, 3, 1);

  // set low pass filtering properties
  uint8_t accel_dlp_config = 0b00000001 | (accel_dlpfcfg << 3) | accel_fs_sel << 1; 
  uint8_t accel_low_pass[2] = {0x14, accel_dlp_config};
  i2cWrite(address, accel_low_pass, 2, 1);

  uint8_t gyro_dlp_config = 0b00000001 | (gyro_dlpfcfg << 3) | gyro_fs_sel << 1; 
  uint8_t gyro_low_pass[2] = {0x01, accel_dlp_config};
  i2cWrite(address, gyro_low_pass, 2, 1);
};
 

// function to read accelerometer and gyroscope data
void readAccelGyroICM(int address, uint8_t * data) {
    uint8_t read_data = 0x2D; // address of first piece of data

    // read all accel data
    i2cWrite(address, &read_data, 1, 0);
    i2cRead(address, data, 12); // there are 12 bytes total of accelerometer and gyroscope data
}