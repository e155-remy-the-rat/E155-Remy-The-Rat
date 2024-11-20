// ICM20948.c
// Source code for ICM 20948 functions

#include "ICM20948.h"

void configICM(void) {
    // reset chip, clear sleep mode, enable accel and disable gyro
    uint8_t reset[3] = {0x06, 0b00000010, 0b00000111};
    i2cWrite(ICM_ADDRESS, reset, 3, 1);
};

void readTempICM(uint8_t * temp_msb, uint8_t * temp_lsb) {
    uint8_t read_temp_msb[1] = {0x39};
    uint8_t read_temp_lsb[1] = {0x3A};

    // read msb
    i2cWrite(ICM_ADDRESS, read_temp_msb, 1, 0);
    i2cRead(ICM_ADDRESS, temp_msb, 1);

    // read lsb
    i2cWrite(ICM_ADDRESS, read_temp_lsb, 1, 0);
    i2cRead(ICM_ADDRESS, temp_lsb, 1);
};

void readAccelZICM(uint8_t * accelz_msb, uint8_t * accelz_lsb) {
  uint8_t za_offs[3] = {0x1A, 0x00, 0b00000000};

  int accel_smplrt_div = 0b00000000000;
  uint8_t accel_smplrt[3] = {0x10, (accel_smplrt_div >> 8), 0x00};

  // TODO: made all of the configuration bits constants and OR together configurations
  uint8_t accel_config[2] = {0x14, 0b00100101};


}