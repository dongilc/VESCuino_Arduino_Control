/*
* vescuino_uart_arduino.h
*
*  Created on: 19 April 2018
*      Author: CDI
*/

#ifndef VESCUINO_UART_ARDUINO_H_
#define VESCUINO_UART_ARDUINO_H_

//
#include <Arduino.h>
#include <HardwareSerial.h>
#include "bldc_interface.h"

uint8_t get_current_serial_number(void);

class VESC_UART {
public:
	VESC_UART(HardwareSerial* HwSerial, int baudrate) : vesc_Serial(HwSerial), vesc_Baudrate(baudrate) {};
	~VESC_UART() {};

	void init();

	// getter
	void get_fw(void);
	void get_values(void);

	// setter
	void set_terminal_cmd(char* cmd);
	void set_duty_cycle(float dutyCycle);
	void set_current(float current);
	void set_current_brake(float current);
	void set_rpm(int rpm);
	void set_pos(float pos);
	void set_servo_pos(float pos);
	void set_forward_can(int32_t vesc_id);

private:
	HardwareSerial* vesc_Serial;
	int vesc_Baudrate;
	int serial_number;
};

#endif /* VESCUINO_UART_ARDUINO_H_ */