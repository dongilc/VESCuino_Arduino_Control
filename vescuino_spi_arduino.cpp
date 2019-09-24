/*
* vescuino_spi_arduino.cpp
*
*  Created on: 12 Feb 2018
*      Author: CDI
*/

#include "vescuino_config.h"

#ifdef USE_VESC_CONTROL_SPI

#include <vescuino_spi_arduino.h>
#include <packet.h>
#include <buffer.h>

//
#define SPI_FIXED_DATA_BYTE		128
//#define USE_DEBUG_PRINT	// Uncomment to print debugging messages. Only when debugging

// Private variables
static uint8_t tx_buffer[SPI_FIXED_DATA_BYTE];
static uint8_t rx_buffer[SPI_FIXED_DATA_BYTE];

// Private functions
static void send_packet_spi(unsigned char *data, int len);
static void process_packet_spi(unsigned char *data, int len);
static void send_packet_bldc_interface(unsigned char *data, int len);

//
void VESC_SPI::init(void)
{
	// initialize SPI:
	pinMode(this->cs, OUTPUT);
	SPI.begin();
	Serial.println("VESC Control Start - SPI Master Mode (" + String(this->clock) + "Hz), Arduino Model : " + String(this->arduino_model));

	// Initialize a send function and process funtion
	packet_init(send_packet_spi, process_packet_spi, 0);

	// Initialize data packing function
	bldc_interface_init(send_packet_bldc_interface);
}

void VESC_SPI::select() {
	//Set CS low to start transmission (interrupts conversion)
	SPI.beginTransaction(SPISettings(this->clock, MSBFIRST, SPI_MODE0));
	digitalWrite(this->cs, LOW);
}

void VESC_SPI::deselect() {
	//Set CS high to stop transmission (restarts conversion)
	digitalWrite(this->cs, HIGH);
	SPI.endTransaction();
}

void VESC_SPI::ExchangeBytes(void)
{
	unsigned int  i = 0;

#ifdef USE_DEBUG_PRINT
	// TX Packet
	Serial.print("[spi txd] ");
	for (int i = 0; i < SPI_FIXED_DATA_BYTE; i++) {
		Serial.print(tx_buffer[i]);
		Serial.print(":");
	}
	Serial.println("\r\n---------------------------------------------");
#endif

	// send tx
	select();
	SPI.transfer(tx_buffer, SPI_FIXED_DATA_BYTE);
	
	// save rx_buffer and reset tx_buffer
	for (i = 0; i < SPI_FIXED_DATA_BYTE; i++) {
		rx_buffer[i] = tx_buffer[i];
		tx_buffer[i] = 0;
	}

#ifdef USE_DEBUG_PRINT
	// TX Packet
	Serial.print("[spi rxd] ");
	for (int i = 0; i < SPI_FIXED_DATA_BYTE; i++) {
		Serial.print(rx_buffer[i]);
		Serial.print(":");
	}
	Serial.println("\r\n---------------------------------------------");
#endif

	// process rx_buffer
	int len = 0;
	if (rx_buffer[0] == 2) {
		len = rx_buffer[1] + 5;
	}
	else if (rx_buffer[0] == 3) {
		len = rx_buffer[1] + 6;
	}
#ifdef USE_DEBUG_PRINT
	Serial.println("rx length:" + String(len));
#endif
	for (i = 0; i < len; i++) {
		packet_process_byte(rx_buffer[i], 0);
	}
	deselect();
}

void VESC_SPI::spi_data_exchange(void)
{
	// prepare spi tx data (make packet and store to tx buffer)
	bldc_interface_communicate_custom(this->arduino_model, this->num_id, this->id, this->comm_set, this->value);
	
	// spi tx/rx data exchange
	this->ExchangeBytes();
}

/**
* Callback that the packet handler uses to send an assembled packet.
*
* @param data
* Data array pointer
* @param len
* Data array length
*/
void send_packet_spi(unsigned char *data, int len)
{
	for (int i = 0; i < SPI_FIXED_DATA_BYTE; i++) {
		if (i<len)	tx_buffer[i] = data[i];
		else		tx_buffer[i] = 0;
	}

	// Send the data over SPI
#ifdef USE_DEBUG_PRINT
	Serial.print(F("[ bldc ] send_packet = len["));
	Serial.print(len);
	Serial.print(F("] write["));
	for (int i = 0; i < len; i++) {
		Serial.print(tx_buffer[i]);
		Serial.print(F(":"));
	}

	Serial.println(F("]"));
#endif
}

// Packet RX Processing Function
static void process_packet_spi(unsigned char *data, int len) 
{
	// Let bldc_interface process the packet.
	bldc_interface_process_packet(data, len);
}

/*
* Callback that bldc_interface uses to send packets.
*
* @param data
* Data array pointer
* @param len
* Data array length
*/
static void send_packet_bldc_interface(unsigned char *data, int len) {
	// Pass the packet to the packet handler to add checksum, length, start and stop bytes.
	packet_send_packet(data, len, 0);
}

#endif