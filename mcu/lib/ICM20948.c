// ICM20948.c
// Source code for ICM 20948 functions

#include "ICM20948.h"


void configICM(int address) {
    // reset chip, clear sleep mode, enable accel and disable gyro
    uint8_t reset[3] = {0x06, 0b00000010, 0b00000111};
    i2cWrite(address, reset, 3, 1);
};

void readTempICM(int address, uint8_t * temp_msb, uint8_t * temp_lsb) {
    uint8_t read_temp_msb[1] = {0x39};
    uint8_t read_temp_lsb[1] = {0x3A};

    // read msb
    i2cWrite(address, read_temp_msb, 1, 0);
    i2cRead(address, temp_msb, 1);

    // read lsb
    i2cWrite(address, read_temp_lsb, 1, 0);
    i2cRead(address, temp_lsb, 1);
};

void configAccelICM(int address, uint8_t dlpfcfg, uint8_t fs_sel) {
  // set offset for z acceleration
  uint8_t za_offs[3] = {0x1A, 0x00, 0b00000000};
  i2cWrite(address, za_offs, 3, 1);

  // set sample rate divisor
  uint8_t accel_smplrt[3] = {0x10, 0x00, 0x00};
  i2cWrite(address, accel_smplrt, 3, 1);

  // set low pass filtering properties
  uint8_t accel_dlp_config = 0b00000001 | (dlpfcfg << 3) | fs_sel << 1; 
  uint8_t accel_low_pass[2] = {0x14, accel_dlp_config};
  i2cWrite(address, accel_low_pass, 2, 1);
};

void readAccelICM(int address, uint8_t * accel) {
    uint8_t read_accel = 0x2D;

    // read all accel data
    i2cWrite(address, &read_accel, 1, 0);
    i2cRead(address, accel, 6);
}