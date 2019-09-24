/*
* vescuino_can_arduino.h
*
*  Created on: 19 April 2018
*      Author: CDI
*/

#ifndef VESCUINO_CAN_ARDUINO_H_
#define VESCUINO_CAN_ARDUINO_H_

//
#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"
#include "bldc_interface.h"

class VESC_CAN {
public:
	VESC_CAN(uint8_t cs_pin, uint8_t baudrate) : cs(cs_pin), bps(baudrate) {};
	~VESC_CAN() {};

	void init();

	MCP_CAN* canb;

	// getter
	void get_fw(uint8_t can_id);
	void get_values(uint8_t can_id);

	// setter
	void set_terminal_cmd(char* cmd);
	void set_duty_cycle(float dutyCycle);
	void set_current(float current);
	void set_current_brake(float current);
	void set_rpm(uint8_t controller_id, float rpm);
	void set_pos(float pos);
	void set_servo_pos(float pos);

private:
	int cs;
	int bps;
};

#endif /* VESCUINO_CAN_ARDUINO_H_ */