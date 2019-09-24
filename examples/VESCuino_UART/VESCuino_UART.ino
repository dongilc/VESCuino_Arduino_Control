/*
  VESCuino UART
  VESC - Arduino Uart Control
  Naverlabs, CDI. 20180419
*/

#include "vescuino_uart_arduino.h"
#include "vescuino_common.h"

#ifdef USE_ARDUINO_DUE
#include "DueTimer.h"
int dt_us = 1000;	// 1ms
int timer_cnt = 0;
#endif

/* =========== UART Setting =========== */
#define NUM_OF_VESC		2		// Set Number of VESC (1~3)
/* =========== End of UART Setting =========== */

// Use Hardware Serial For Vesc (Serial1, Serial2, Serial2)
VESC_UART vesc1(&Serial1, 115200);
VESC_UART vesc2(&Serial2, 115200);

// Vesc Control Class
const int num_of_wheel = NUM_OF_VESC;
const int degree_of_freedom = NUM_OF_VESC;
VESC_CONTROL vesc_ctrl(num_of_wheel, degree_of_freedom);
int control_en_flag = 0;

// 1Khz Control loop
void Timer1_INT()
{
	/* add main program code here */
	if (timer_cnt <= 10000)	{
		control_en_flag = 1;
	}
	else {
		control_en_flag = 0;
	}

	if (timer_cnt == 15000) {
		timer_cnt = 0;
	}
	/* --------------------------- */

	timer_cnt++;
}

void setup()
{
	// default : Serial = Debugging Print (Serial0)
	Serial.begin(115200);
	
	// vesc uart class init
	vesc1.init();
	//vesc2.init();
	
	// set rx post fuctions
	bldc_interface_set_rx_fw_func(bldc_fw_received);
	bldc_interface_set_rx_value_func(bldc_val_received);

	// Timer Interrupt. 1ms
	Timer1.attachInterrupt(Timer1_INT);
	Timer1.start(dt_us); // Calls every us
}

void loop()
{
	loop_time_handle_uart();

	/* add main program code here */
	vesc1.get_values();
	//vesc2.get_values();
	
	if (control_en_flag)
	{
		vesc1.set_forward_can(-1);
		vesc1.set_rpm(500.0);
		vesc1.set_forward_can(1);
		vesc1.set_rpm(1000.0);
	}
	/* ---------------------------- */

	loop_time_delay_uart();
}

void bldc_fw_received(int major, int minor)
{
	/* add program code here */


	/* ---------------------------- */

#ifdef USE_PRINT_RX_RESULT
	Serial.print(F("[ bldc ] COMM_FW_VERSION = "));
	Serial.print(major);
	Serial.print(F("."));
	Serial.println(minor);
#endif
}

void bldc_val_received(mc_values *val)
{
	/* add program code here */
	uint8_t id = 0;
	id = get_current_serial_number();
	vesc_ctrl.erpm[id] = val->rpm;
	vesc_ctrl.tacho[id] = val->tachometer;
	vesc_ctrl.enc_pos[id] = val->pid_pos_now;

	//Serial.println("id:" + String(id) + ", erpm=" + String(vesc_ctrl.erpm[id]) + ", tacho=" + String(vesc_ctrl.tacho[id]));
	/* ---------------------------- */


#ifdef USE_PRINT_RX_RESULT
	Serial.println(F("[ bldc ] Received 'COMM_GET_VALUES'"));

	Serial.print(F("[ bldc ] COMM_GET_VALUES = temp_fet["));
	Serial.print(val->temp_mos1);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = temp_motor["));
	Serial.print(val->temp_pcb);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = current_motor["));
	Serial.print(val->current_motor);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = current_in["));
	Serial.print(val->current_in);
	Serial.println("]");

	Serial.print(F("[ bldc ] COMM_GET_VALUES = duty_now["));
	Serial.print(val->duty_now);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = erpm["));
	Serial.print(val->rpm);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = v_in["));
	Serial.print(val->v_in);
	Serial.println("]");

	Serial.print(F("[ bldc ] COMM_GET_VALUES = amp_hours["));
	Serial.print(val->amp_hours);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = amp_hours_charged["));
	Serial.print(val->amp_hours_charged);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = watt_hours["));
	Serial.print(val->watt_hours);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = watt_hours_charged["));
	Serial.print(val->watt_hours_charged);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = tachometer["));
	Serial.print(val->tachometer);
	Serial.println("]");
	Serial.print(F("[ bldc ] COMM_GET_VALUES = tachometer_abs["));
	Serial.print(val->tachometer_abs);
	Serial.println("]");

	Serial.print(F("[ bldc ] COMM_GET_VALUES = fault_code["));
	Serial.print(bldc_interface_fault_to_string(val->fault_code));
	Serial.println("]");
#endif
}