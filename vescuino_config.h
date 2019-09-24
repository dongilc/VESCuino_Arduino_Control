/*
* vescuino_config.h
*
*  Created on: 19 April 2018
*      Author: CDI
*/

#ifndef VESCUINO_CONFIG_H_
#define VESCUINO_CONFIG_H_

/* ========================= Basic Setting ========================= */
// Select One of the Arduino (Arduino Uno doesn't supported)
#define USE_ARDUINO_MEGA_2560
//#define USE_ARDUINO_DUE
//#define USE_ARDUINO_TEENSY_3_2

// Select One of this
#define USE_VESC_CONTROL_SPI
//#define USE_VESC_CONTROL_UART

// Debug Print Setting
#define USE_LOOP_CNT_PRINT
//#define USE_SLOW_DEBUG_MODE
//define USE_PRINT_RX_RESULT
/* ========================= End of Basic Setting ========================= */

#endif