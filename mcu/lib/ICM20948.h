// ICM20948.h
// Header for interfacing with ICM 20948 chip 

#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h> // Include stdint header
#include "STM32L432KC_I2C.h"

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

#define ICM_ADDRESS1 0b1101001 // AD0 is logic high
#define ICM_ADDRESS2 0b1101000 // AD0 is logic low

#define RANGE_2G 0b00
#define RANGE_4G 0b01
#define RANGE_8G 0b10
#define RANGE_16G 0b11

#define RANGE_2G_CONVERSION 16384
#define RANGE_4G_CONVERSION 8192
#define RANGE_8G_CONVERSION 4096
#define RANGE_16G_CONVERSION 2048

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configICM(int address);
void readTempICM(int address, uint8_t * temp_msb, uint8_t * temp_lsb);
void configAccelICM(int address, uint8_t dlpfcfg, uint8_t fs_sel);
void readAccelICM(int address, uint8_t * accel);

#endif