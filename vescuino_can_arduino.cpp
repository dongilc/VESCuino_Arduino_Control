/*
* vescuino_can_arduino.cpp
*
*  Created on: 12 Feb 2018
*      Author: CDI
*/

#include <vescuino_can_arduino.h>
#include <packet.h>
#include <buffer.h>

#define USE_DEBUG_PRINT

// Private variables
static unsigned char send_buffer[256];
static int32_t can_fwd_vesc = -1;

// Private functions
static void send_packet_can(unsigned char *data, int len);
static void process_packet_can(unsigned char *data, int len);

//
void VESC_CAN::init(void)
{
	// initialize SPI:
	MCP_CAN CAN(this->cs);
	canb = &CAN;

START_INIT:

	switch (this->bps)
	{
	case CAN_500KBPS:
		if (CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
		{
			Serial.println("VESC Control Start - CAN Mode (500kbps)");
		}
		else
		{
			Serial.println("CAN BUS Shield init fail");
			Serial.println("Init CAN BUS Shield again");
			delay(100);
			goto START_INIT;
		}
		break;

	case CAN_1000KBPS:
		if (CAN_OK == CAN.begin(CAN_1000KBPS))
		{
			Serial.println("VESC Control Start - CAN Mode (1Mbps)");
		}
		else
		{
			Serial.println("CAN BUS Shield init fail");
			Serial.println("Init CAN BUS Shield again");
			delay(100);
			goto START_INIT;
		}
		break;

	default:
		Serial.println("VESC Control Start - CAN Mode Baudrate Error");
		while (true);
		break;

	}

	// Initialize the bldc interface and provide a send function
	packet_init(send_packet_can, process_packet_can, 0);
}

/**
* Callback that the packet handler uses to send an assembled packet.
*
* @param data
* Data array pointer
* @param len
* Data array length
*/
void send_packet_can(unsigned char *data, int len)
{
	// Send the data over CAN
#ifdef USE_DEBUG_PRINT
	Serial.print(F("[ bldc ] send_packet = len["));
	Serial.print(len);
	Serial.print(F("] write["));
	for (int i = 0; i < len; i++) {
		Serial.print(data[i]);
		Serial.print(F(":"));
	}

	Serial.println(F("]"));
#endif

	//
}

// Packet RX Processing Function
static void process_packet_can(unsigned char *data, int len) 
{
	// Let bldc_interface process the packet.
	bldc_interface_process_packet(data, len);
}

/* Getter */
void VESC_CAN::get_fw(uint8_t can_id)
{
	int send_index = 0;
	send_buffer[send_index++] = COMM_FW_VERSION;
	packet_send_packet(send_buffer, send_index, 0);
}

void VESC_CAN::get_values(uint8_t can_id)
{
	int send_index = 0;
	send_buffer[send_index++] = COMM_GET_VALUES;
	packet_send_packet(send_buffer, send_index, 0);
}

void VESC_CAN::set_rpm(uint8_t controller_id, float rpm) {
	int send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int)rpm, &send_index);

	this->canb->sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), 1, send_index, buffer);
	//        comm_can_transmit(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}