/**
    Main Header: Contains general defines and selected portions of CMSIS files
    @file main.h
    @author Marina Ring
    @version 1.0 11/17/2024
*/

#ifndef MAIN_H
#define MAIN_H

#include "../lib/STM32L432KC.h"
#include "../lib/MPU6050.h"
#include "../lib/ICM20948.h"
#include "../Fusion/Fusion.h"

#define LED_PIN PB3 // LED pin for blinking on Port B pin 5
#define SDA PA10
#define SCL PA9
#define CONFIG_BUTTON PB5

#endif 