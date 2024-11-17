/**
    Main Header: Contains general defines and selected portions of CMSIS files
    @file main.h
    @author Marina Ring
    @version 1.0 11/17/2024
*/

#ifndef MAIN_H
#define MAIN_H

#include "../lib/STM32L432KC.h"
// #include "../lib/IMU something something.h"

#define LED_PIN PB3 // LED pin for blinking on Port B pin 5
#define BUFF_LEN 32
#define SPI SPI1
#define COPI PA12
#define CIPO PB4
#define SCK PA5
#define CS PB6
#define SDA PA10
#define SCL PA9

#endif 