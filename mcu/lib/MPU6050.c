// MPU6050.c
// Source code for MPU 6050 functions

#include "MPU6050.h"

void configMPU(void) {
    uint8_t temp_enable[2] = {0x23, 0x80};
    i2cWrite(ADDRESS, temp_enable, 2, 1);

    uint8_t sample_rate[2] = {0x19, 7};
    i2cWrite(ADDRESS, sample_rate, 2, 1);


};

void readMPU(uint8_t * temp_msb, uint8_t * temp_lsb) {
    uint8_t read_temp_msb[1] = {0x41};
    uint8_t read_temp_lsb[1] = {0x42};

    // read msb
    i2cWrite(ADDRESS, read_temp_msb, 0b01, 0);
    i2cRead(ADDRESS, temp_msb, 0b01);

    // read lsb
    i2cWrite(ADDRESS, read_temp_lsb, 0b01, 0);
    i2cRead(ADDRESS, temp_lsb, 0b01);
};