/*
* comm_uart_arduino.cpp
*
*  Created on: 19 April 2018
*      Author: CDI
*/

#include "vescuino_config.h"

#ifdef USE_VESC_CONTROL_UART

#include <vescuino_uart_arduino.h>
#include <packet.h>
#include <buffer.h>
#include <DueTimer.h>

// Uncomment to print debugging messages
//#define USE_DEBUG_PRINT

// Private variables
static unsigned char send_buffer[256];
static int32_t can_fwd_vesc = -1;
static bool is_data_ready = false;
static uint8_t serial_num_now = 0;

// Private Functions
static void timer3Handler();
static void fwd_can_append(uint8_t *data, int *ind);
static void process_packet_serial(unsigned char *data, int len);
static void send_packet_serial1(unsigned char *data, int len);
static void send_packet_serial2(unsigned char *data, int len);
static void send_packet_serial3(unsigned char *data, int len);

//
void VESC_UART::init()
{
	// Initialize UART
	this->vesc_Serial->begin(this->vesc_Baudrate);

	if (this->vesc_Serial == &Serial1) {
		Serial.println("VESC Uart Start - Serial1, Baudrate: " + String(this->vesc_Baudrate));
		packet_init(send_packet_serial1, process_packet_serial, 0);
		this->serial_number = 1;
	} 
	else if (this->vesc_Serial == &Serial2) {
		Serial.println("VESC Uart Start - Serial2, Baudrate: " + String(this->vesc_Baudrate));
		packet_init(send_packet_serial2, process_packet_serial, 1);
		this->serial_number = 2;
	}
	else if (this->vesc_Serial == &Serial3) {
		Serial.println("VESC Uart Start - Serial3, Baudrate: " + String(this->vesc_Baudrate));
		packet_init(send_packet_serial3, process_packet_serial, 2);
		this->serial_number = 3;
	}
	else {
		Serial.println("Error! Set Vesc Serial Among Serial1~3." + String(this->vesc_Baudrate));
	}

	// Due Timer 
	Timer3.attachInterrupt(timer3Handler);
	Timer3.start(1000); // Calls every 1ms
}

// Serial1 RX Interrupt 
void serialEvent1()
{
	bool isWork = false;
#ifdef USE_DEBUG_PRINT
	int len = 0;
#endif

	if (Serial1.available()) {
#ifdef USE_DEBUG_PRINT
		Serial.println(F("[ bldc ] Serial1_Event: Reading from serial..."));
#endif

		while (Serial1.available()) {
			serial_num_now = 0;
			packet_process_byte(Serial1.read(), serial_num_now);
#ifdef USE_DEBUG_PRINT
			len += sizeof(uint8_t);
#endif
		}

#ifdef USE_DEBUG_PRINT
		Serial.print(F("[ bldc ] Serial1_Event: Just read "));
		Serial.print(len);
		Serial.println(F(" bytes."));
#endif

		is_data_ready = true;

	}
	else {
#ifdef USE_DEBUG_PRINT
		Serial.println(F("[ bldc ] Serial1_Event: nothing to do... "));
#endif
	}
}

// Serial2 RX Interrupt
void serialEvent2()
{
	bool isWork = false;
#ifdef USE_DEBUG_PRINT
	int len = 0;
#endif

	if (Serial2.available()) {
#ifdef USE_DEBUG_PRINT
		Serial.println(F("[ bldc ] Serial2_Event: Reading from serial..."));
#endif

		while (Serial2.available()) {
			serial_num_now = 1;
			packet_process_byte(Serial2.read(), serial_num_now);
#ifdef USE_DEBUG_PRINT
			len += sizeof(uint8_t);
#endif
		}

#ifdef USE_DEBUG_PRINT
		Serial.print(F("[ bldc ] Serial2_Event: Just read "));
		Serial.print(len);
		Serial.println(F(" bytes."));
#endif

		is_data_ready = true;

	}
	else {
#ifdef USE_DEBUG_PRINT
		Serial.println(F("[ bldc ] Serial2_Event: nothing to do... "));
#endif
	}
}

// Serial3 RX Interrupt
void serialEvent3()
{
	bool isWork = false;
#ifdef USE_DEBUG_PRINT
	int len = 0;
#endif

	if (Serial3.available()) {
#ifdef USE_DEBUG_PRINT
		Serial.println(F("[ bldc ] Serial3_Event: Reading from serial..."));
#endif

		while (Serial3.available()) {
			serial_num_now = 2;
			packet_process_byte(Serial3.read(), serial_num_now);
#ifdef USE_DEBUG_PRINT
			len += sizeof(uint8_t);
#endif
		}

#ifdef USE_DEBUG_PRINT
		Serial.print(F("[ bldc ] Serial3_Event: Just read "));
		Serial.print(len);
		Serial.println(F(" bytes."));
#endif

		is_data_ready = true;

	}
	else {
#ifdef USE_DEBUG_PRINT
		Serial.println(F("[ bldc ] Serial3_Event: nothing to do... "));
#endif
	}
}

uint8_t get_current_serial_number(void)
{
	return serial_num_now;
}

/* Getter */
void VESC_UART::get_fw(void)
{
	int send_index = 0;
	send_buffer[send_index++] = COMM_FW_VERSION;
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::get_values(void)
{
	int send_index = 0;
	send_buffer[send_index++] = COMM_GET_VALUES;
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

/* Setter */
void VESC_UART::set_terminal_cmd(char* cmd) 
{	
	int send_index = 0;
	int len = strlen(cmd);
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_TERMINAL_CMD;
	memcpy(send_buffer + send_index, cmd, len);
	send_index += len;
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::set_duty_cycle(float dutyCycle) 
{
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_DUTY;
	buffer_append_float32(send_buffer, dutyCycle, 100000.0, &send_index);
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::set_current(float current) 
{
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::set_current_brake(float current) 
{
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::set_rpm(int rpm) 
{
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_RPM;
	buffer_append_int32(send_buffer, rpm, &send_index);
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::set_pos(float pos) 
{
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_POS;
	buffer_append_float32(send_buffer, pos, 1000000.0, &send_index);
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::set_servo_pos(float pos) 
{
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_SERVO_POS;
	buffer_append_float16(send_buffer, pos, 1000.0, &send_index);
	packet_send_packet(send_buffer, send_index, (this->serial_number - 1));
}

void VESC_UART::set_forward_can(int32_t vesc_id) {
	can_fwd_vesc = vesc_id;
}

static void fwd_can_append(uint8_t *data, int *ind)
{
	if (can_fwd_vesc >= 0) {
		data[(*ind)++] = COMM_FORWARD_CAN;
		data[(*ind)++] = can_fwd_vesc;

#ifdef USE_DEBUG_PRINT
		Serial.print(F("[ bldc ] CAN FORWARD ID : "));
		Serial.println(can_fwd_vesc);
#endif
	}
}

/*
* This thread is only for calling the timer function once
* per millisecond. Can also be implementer using interrupts
* if no RTOS is available.
*/
static void timer3Handler()
{
	packet_timerfunc();
}

// Packet RX Processing Function
static void process_packet_serial(unsigned char *data, int len) {
	// Let bldc_interface process the packet.
	bldc_interface_process_packet(data, len);
}

// Serial1 TX, Send Packet Function
static void send_packet_serial1(unsigned char *data, int len)
{
	// Send the data over UART
#ifdef USE_DEBUG_PRINT
	Serial.print(F("[ bldc ] Serial1 send_packet = len["));
	Serial.print(len);
	Serial.print(F("] write["));
	for (int i = 0; i < len; i++) {
		Serial.print(data[i]);
		Serial.print(F(":"));
	}

	Serial.println(F("]"));
#endif

	Serial1.write(data, len);
	delay(1);
}

// Serial2 TX, Send Packet Function
static void send_packet_serial2(unsigned char *data, int len)
{
	// Send the data over UART
#ifdef USE_DEBUG_PRINT
	Serial.print(F("[ bldc ] Serial2 send_packet = len["));
	Serial.print(len);
	Serial.print(F("] write["));
	for (int i = 0; i < len; i++) {
		Serial.print(data[i]);
		Serial.print(F(":"));
	}

	Serial.println(F("]"));
#endif

	Serial2.write(data, len);
	delay(1);
}

// Serial3 TX, Send Packet Function
static void send_packet_serial3(unsigned char *data, int len)
{
	// Send the data over UART
#ifdef USE_DEBUG_PRINT
	Serial.print(F("[ bldc ] Serial3 send_packet = len["));
	Serial.print(len);
	Serial.print(F("] write["));
	for (int i = 0; i < len; i++) {
		Serial.print(data[i]);
		Serial.print(F(":"));
	}

	Serial.println(F("]"));
#endif

	Serial3.write(data, len);
	delay(1);
}

#endif