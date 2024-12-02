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


///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configICM(int address);
void readTempICM(int address, uint8_t * temp_msb, uint8_t * temp_lsb);
void readAccelZICM(int address, uint8_t * accelz_msb, uint8_t * accelz_lsb);

#endif