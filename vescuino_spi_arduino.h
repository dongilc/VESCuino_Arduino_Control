/*
* vescuino_spi_arduino.h
*
*  Created on: 12 Feb 2018
*      Author: CDI
*/

#ifndef VESCUINO_SPI_ARDUINO_H_
#define VESCUINO_SPI_ARDUINO_H_

//
#include <Arduino.h>
#include <SPI.h>
#include <bldc_interface.h>

class VESC_SPI {
public:
	uint8_t *id;
	uint8_t *comm_set;
	float *value;

	// constructor.
	VESC_SPI(long sck, uint8_t chip_select, uint8_t num_of_id, uint8_t model_num)
		: clock(sck), cs(chip_select), num_id(num_of_id), arduino_model(model_num)
	{
		//
		id = new uint8_t[num_id];
		comm_set = new uint8_t[num_id];
		value = new float[num_id];
	}

	unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData);
	unsigned int ReadReg(uint8_t WriteAddr, uint8_t WriteData);
	void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes);
	void ExchangeBytes(void);

	void init();
	void select();
	void deselect();
	void spi_data_exchange(void);

private:
	long clock;
	uint8_t cs;
	uint8_t num_id;
	uint8_t arduino_model;
};

#endif /* VESCUINO_SPI_ARDUINO_H_ */