/*
* vescuino_common.h
*
*  Created on: 19 April 2018
*      Author: CDI
*/

#ifndef VESCUINO_COMMON_H_
#define VESCUINO_COMMON_H_

#include "vescuino_config.h"

// Vesc Control Class
class VESC_CONTROL
{
public:
	int NO_VESC;
	int NO_DOF;

	//int ctm_state;	// 0:ready, 1:init, 2:run, 3:end

	float* erpm;
	int* tacho;
	float* enc_pos;
	float* enc_rps;
	float* enc_rad;
	float* current;
	float* brake;

	float* pos;
	float* vel;
	float* target_vel;

	VESC_CONTROL(int no_of_vesc, int no_of_dof)
	{
		NO_VESC = no_of_vesc;
		NO_DOF = no_of_dof;

		erpm = new float[NO_VESC];
		tacho = new int[NO_VESC];
		enc_pos = new float[NO_VESC];
		enc_rps = new float[NO_VESC];
		enc_rad = new float[NO_VESC];
		current = new float[NO_VESC];
		brake = new float[NO_VESC];

		pos = new float[NO_DOF];
		vel = new float[NO_DOF];
		target_vel = new float[NO_DOF];

		// initialize
		for (int i = 0; i < NO_VESC; i++) {
			erpm[i] = 0.;
			tacho[i] = 0;
			enc_pos[i] = 0;
			enc_rps[i] = 0;
			enc_rad[i] = 0;
			current[i] = 0.;
			brake[i] = 0.;
		}

		for (int i = 0; i < NO_DOF; i++) {
			pos[i] = 0.;
			vel[i] = 0.;
			target_vel[i] = 0.;
		}
	}

	void Init(void);
};

void VESC_CONTROL::Init(void) 
{
	// initialize
	for (int i = 0; i < NO_VESC; i++) {
		erpm[i] = 0.;
		tacho[i] = 0;
		enc_rps[i] = 0;
		enc_rad[i] = 0;
		current[i] = 0.;
		brake[i] = 0.;
	}

	for (int i = 0; i < NO_DOF; i++) {
		pos[i] = 0.;
		vel[i] = 0.;
		target_vel[i] = 0.;
	}
}

/* Arduino Model Number */
typedef enum {
	ARDUINO_UNO = 0,
	ARDUINO_MEGA_2560,
	ARDUINO_DUE
} ARDUINO_MODEL;

/* Loop delay time setting */
#ifdef USE_ARDUINO_MEGA_2560
#define ARDUINO_MODEL			ARDUINO_MEGA_2560
#ifdef USE_SLOW_DEBUG_MODE
#define LOOP_DELAY_MSEC			1000			// For slow debugging mode
#else
#define LOOP_DELAY_USEC_SPI 	1300			// spi, 400:1khz, 1500:500hz
#define LOOP_DELAY_USEC_UART	6000			// uart, 6000:100hz, 4400:150hz
#endif	
#define SPI_CLOCK				4000000 //8000000			// 8MHz clock works.
#elif defined USE_ARDUINO_DUE
#define ARDUINO_MODEL			ARDUINO_DUE		// DUE
#ifdef USE_SLOW_DEBUG_MODE
#define LOOP_DELAY_MSEC			1000			// For slow debugging mode
#else
#define LOOP_DELAY_USEC_SPI 	1300			// spi, 400:1khz, 1500:500hz
#define LOOP_DELAY_USEC_UART	6000			// uart, 6000:100hz, 4400:150hz
#endif
#define SPI_CLOCK				21000000		// 21MHz clock works.
#endif

/* VESC pin settings */
const int nCS_Pin = 10;// 40; //10;

/* global variable */
uint8_t spi_connected = 0;
float vesc_vin = 0.;
float vesc_temp = 0.;
uint8_t vesc_fault = 0;
uint8_t vesc_can_devs = 0;

/* Loop time cnt variable */
typedef struct {
	unsigned int dt_loop;
	float dt_loop_msec;
	float dt_loop_freq;
	float time_loop_now_sec;
	unsigned long time_loop_now;
	unsigned long time_loop_prev;
} loop_time;
loop_time lt1;
int loop_cnt = 0;
int loop_cnt_1sec = 0;

/* Loop time handle function - spi */
void loop_time_handle_spi()
{
	lt1.time_loop_prev = lt1.time_loop_now;
	lt1.time_loop_now = micros();
	lt1.time_loop_now_sec = lt1.time_loop_now / 1000000.;
	lt1.dt_loop = lt1.time_loop_now - lt1.time_loop_prev;
	lt1.dt_loop_msec = lt1.dt_loop / 1000.;  // msec
	lt1.dt_loop_freq = 1000. / lt1.dt_loop_msec;  // freq

#ifdef LOOP_DELAY_MSEC
	loop_cnt_1sec = 1000 / LOOP_DELAY_MSEC;
#elif defined LOOP_DELAY_USEC_SPI
	loop_cnt_1sec = 1000000 / LOOP_DELAY_USEC_SPI;
#endif

#ifdef USE_LOOP_CNT_PRINT
	if (loop_cnt == loop_cnt_1sec)
	{
		Serial.print("[ loop ] Freq :");
		Serial.print(lt1.dt_loop_freq);
		Serial.print("Hz, ");
		Serial.print(lt1.dt_loop_msec);
		Serial.print("ms, t_now :");
		Serial.print(lt1.time_loop_now_sec);
		Serial.print("s");
		if (spi_connected == 1) {
			Serial.println(", SPI Ok. Vin:" + String(vesc_vin) + "V, Temp:" + String(vesc_temp) + "C, Can_devs:" + String(vesc_can_devs) + ", Fault:" + String(vesc_fault));
		}
		else {
			Serial.println(", Trying to connect SPI Slave...");
		}
		loop_cnt = 0;
		spi_connected = 0;
	}
#endif
}

/* Loop time handle function - uart */
void loop_time_handle_uart()
{
	lt1.time_loop_prev = lt1.time_loop_now;
	lt1.time_loop_now = micros();
	lt1.time_loop_now_sec = lt1.time_loop_now / 1000000.;
	lt1.dt_loop = lt1.time_loop_now - lt1.time_loop_prev;
	lt1.dt_loop_msec = lt1.dt_loop / 1000.;  // msec
	lt1.dt_loop_freq = 1000. / lt1.dt_loop_msec;  // freq

#ifdef LOOP_DELAY_MSEC
	loop_cnt_1sec = 1000 / LOOP_DELAY_MSEC;
#elif defined LOOP_DELAY_USEC_UART
	loop_cnt_1sec = 1000000 / LOOP_DELAY_USEC_UART;
#endif

#ifdef USE_LOOP_CNT_PRINT
	if (loop_cnt == loop_cnt_1sec)
	{
		Serial.print("[ loop ] Freq :");
		Serial.print(lt1.dt_loop_freq);
		Serial.print("Hz, ");
		Serial.print(lt1.dt_loop_msec);
		Serial.print("ms, t_now :");
		Serial.print(lt1.time_loop_now_sec);
		Serial.println("s");
		loop_cnt = 0;
	}
#endif
}

/* loop time delay function - spi */
void loop_time_delay_spi()
{
#ifdef LOOP_DELAY_MSEC
	delay(LOOP_DELAY_MSEC);
#elif defined LOOP_DELAY_USEC_SPI
	delayMicroseconds(LOOP_DELAY_USEC_SPI);
#endif

	loop_cnt++;
}

/* loop time delay function - uart */
void loop_time_delay_uart()
{
#ifdef LOOP_DELAY_MSEC
	delay(LOOP_DELAY_MSEC);
#elif defined LOOP_DELAY_USEC_UART
	delayMicroseconds(LOOP_DELAY_USEC_UART);
#endif

	loop_cnt++;
}

#endif /* VESCUINO_COMMON_H_ */
