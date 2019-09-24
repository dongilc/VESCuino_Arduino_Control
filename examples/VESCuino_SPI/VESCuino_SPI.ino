/*
  MJVESC SPI
  VESC - Arduino SPI Control
  Myongji Univ., Dongil Choi. 20190924
  -- GPL V3 licensed --
  The software is released under the GNU General Public License version 3.0
*/

#include "vescuino_spi_arduino.h"
#include "vescuino_common.h"
#ifdef USE_ARDUINO_MEGA_2560
#include <TimerOne.h>
#elif defined USE_ARDUINO_DUE
#include "DueTimer.h"
#endif

// Connection Checking
int check_pin = 41;
bool pinOn = false;
int dt_us = 1000;

/* =========== SPI Setting =========== */
#define NUM_OF_VESC		1	// Set Number of VESC (1~6)
/* =========== End of SPI Setting =========== */

// Vesc SPI Communication Class
VESC_SPI vesc_spi(SPI_CLOCK, nCS_Pin, NUM_OF_VESC, ARDUINO_MODEL);

// Vesc Control Class
const int num_of_wheel = NUM_OF_VESC;
const int degree_of_freedom = NUM_OF_VESC;
VESC_CONTROL vesc_ctrl(num_of_wheel, degree_of_freedom);

// 1Khz Control loop
void Timer1_INT() 
{
	/* add main program code here */
	if (spi_connected == 1)
	{
		//vesc_ctrl.current[0] = 2.0;
		//vesc_ctrl.current[1] = 2.0;
		//vesc_ctrl.current[2] = 2.0;
		//vesc_ctrl.current[3] = 2.0;

		// SPI Command Set
		for (int i = 0; i < NUM_OF_VESC; i++)
		{
			vesc_spi.id[i] = i;
			vesc_spi.comm_set[i] = COMM_SET_DUTY; //COMM_SET_CURRENT_BRAKE;// COMM_SET_CURRENT;// COMM_SET_CURRENT_BRAKE;
			vesc_spi.value[i] = 0.05;//vesc_ctrl.current[i];
		}

		pinOn = !pinOn;
		digitalWrite(check_pin, pinOn); // pin on, off, on, off...
	}
}

void setup()
{
	// Debug Serial
	Serial.begin(115200);

	// vesc spi class init
	vesc_spi.init();

	// set rx post fuctions
	bldc_interface_set_rx_custom_app_data_func(bldc_custom_received);

	// Timer Interrupt. 1ms
#ifdef USE_ARDUINO_MEGA_2560
	Timer1.initialize(dt_us);
	Timer1.attachInterrupt(Timer1_INT);
	pinMode(check_pin, OUTPUT);
#elif defined USE_ARDUINO_DUE
	Timer1.attachInterrupt(Timer1_INT);
	Timer1.start(dt_us); 
	pinMode(check_pin, OUTPUT);
#endif
}

void loop()
{
	loop_time_handle_spi();

	// spi data exchange over 1kHz
	vesc_spi.spi_data_exchange();

	loop_time_delay_spi();
}

// SPI RX Callback Function
void bldc_custom_received(custom_values *values)
{
	vesc_vin = values->v_in;
	vesc_temp = values->temp_mos1;
	vesc_fault = values->fault_code;
	vesc_can_devs = values->num_of_id; 

	if (vesc_vin >= 5.0 && vesc_vin <=63.0)
	{
		spi_connected = 1;
	}

#ifdef USE_PRINT_RX_RESULT
	// Get Value Return
	Serial.println("======================================");
	Serial.println("FW Version: " + String(values->fw_major) + "." + String(values->fw_minor));
	Serial.println("temperature Mosfet: " + String(values->temp_mos1) + " C");
	Serial.println("temperature Motor: " + String(values->temp_pcb) + " C");
	Serial.println("current_motor: " + String(values->current_motor) + " A");
	Serial.println("current_in: " + String(values->current_in) + " A");
	Serial.println("duty_now: " + String(values->duty_now));
	Serial.println("rpm: " + String(values->rpm) + " (erpm)");
	Serial.println("v_in: " + String(values->v_in) + " V");
	Serial.println("amp_hours: " + String(values->amp_hours) + " Ah");
	Serial.println("amp_hours_charged: " + String(values->amp_hours_charged) + " Ah");
	Serial.println("watt_hours: " + String(values->watt_hours) + " Wh");
	Serial.println("watt_hours_charged: " + String(values->watt_hours_charged) + " Wh");
	Serial.println("tachometer: " + String(values->tachometer));
	Serial.println("tachometer_abs: " + String(values->tachometer_abs));
	Serial.print("fault_code: ");
	Serial.println(bldc_interface_fault_to_string((mc_fault_code)values->fault_code));
	Serial.println("encoder_read_deg: " + String(values->enc_pos));
	Serial.println("======================================");

	// Can Status MSGs
	Serial.println("can_devs: " + String(values->num_of_id));
	for (int i = 0; i < values->num_of_id; i++)
	{
		Serial.println("-------------------------------------");
		Serial.println("can_id: " + String(values->id_can[i]));
		Serial.println("rpm: " + String(values->rpm_can[i]) + " (erpm)");
		//Serial.println("current: " + String(values->current_can[i]) + " A");
		//Serial.println("duty: " + String(values->duty_can[i]));
		Serial.println("tachometer: " + String(values->tachometer_can[i]) + " (rev/6ratio)");
	}
#endif

	/* add program code here */
	vesc_vin = values->v_in;
	vesc_temp = values->temp_mos1;
	vesc_fault = values->fault_code;
	vesc_can_devs = values->num_of_id;

	if (vesc_vin >= 5.0 && vesc_vin <= 63.0)
	{
		spi_connected = 1;
	}

	// VESC ID = 0
	vesc_ctrl.erpm[0] = values->rpm;
	vesc_ctrl.tacho[0] = values->tachometer;
	vesc_ctrl.pos[0] = values->enc_pos;
	//
	vesc_ctrl.enc_rps[0] = values->enc_rps;
	vesc_ctrl.enc_rad[0] = values->enc_rad;

	// VESC ID = 1~
	for (int i = 0; i < values->num_of_id; i++) {
		vesc_ctrl.enc_rps[values->id_can[i]] = (float)values->rps_can[i];
		vesc_ctrl.enc_rad[values->id_can[i]] = (float)values->rad_can[i];
	}
	/* ---------------------------- */


}
