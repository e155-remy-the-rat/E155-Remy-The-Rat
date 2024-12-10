/**
    Main Header: Contains general defines and selected portions of CMSIS files
    @file main.h
    @author Marina Ring
    @version 1.0 11/17/2024
*/

#ifndef MAIN_H
#define MAIN_H

#include "../lib/STM32L432KC.h"
#include "../lib/ICM20948.h"
#include "../Fusion/Fusion.h"
#include "../lib/nrf24.h"

#define LED_PIN           PB3 // LED pin for blinking on Port B pin 5
#define SDA               PA10
#define SCL               PA9
#define CONFIG_BUTTON     PB5
#define RP_1              PA1 // A1
#define RP_2              PA2 // A7
#define RP_3              PA0 // A0
#define RP_4              PA7 // A6

int main_head(void);
int main_body(void);

#endif 