/*
  VESCuino CAN
  VESC - Arduino CAN Control
  Naverlabs, CDI. 20180423
*/

#include "vescuino_can_arduino.h"
#include "vescuino_common.h"

/* =========== UART Setting =========== */
#define NUM_OF_VESC		2		// Set Number of VESC (1~3)
/* =========== End of UART Setting =========== */

// Use Seed Studio Canbus Shield v1.2  
const int SPI_CS_PIN = 9;	// default CS_PIN=9
VESC_CAN vesc_can(SPI_CS_PIN, CAN_500KBPS);	// default:CAN_500KBPS, highspeed:CAN_1000KBPS
unsigned char flagRecv = 0;

// Vesc Control Class
const int num_of_wheel = NUM_OF_VESC;
const int degree_of_freedom = NUM_OF_VESC;
VESC_CONTROL vesc_ctrl(num_of_wheel, degree_of_freedom);

void setup()
{
	// default : Serial = Debugging Print (Serial0)
	Serial.begin(115200);
	
	// vesc can class init
	vesc_can.init();
	attachInterrupt(2, MCP2515_ISR, FALLING);	// start interrupt

	// set rx post fuctions
	bldc_interface_set_rx_fw_func(bldc_fw_received);
	bldc_interface_set_rx_value_func(bldc_val_received);

	//vesc_can.set_rpm(0, 1000);
}

void loop()
{
	loop_time_handle_uart();

	/* add main program code here */
	//vesc_can.get_values(0);
	//vesc2.get_values();
	
	//vesc1.set_current(2.0);
	/* ---------------------------- */

	if (flagRecv) {
		can_rx();
	}
	

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
	//id = get_current_serial_number();
	vesc_ctrl.erpm[id] = val->rpm;
	vesc_ctrl.tacho[id] = val->tachometer;
	
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

void MCP2515_ISR()
{
	flagRecv = 1;
}

void can_rx()
{
	unsigned char len = 0;
	unsigned char buf[8];

	flagRecv = 0;

	while (CAN_MSGAVAIL == vesc_can.canb->checkReceive())
	{
		// read data,  len: data length, buf: data buf
		vesc_can.canb->readMsgBuf(&len, buf);

		Serial.print("Get Data From id: ");
		Serial.println(vesc_can.canb->getCanId());
		if (vesc_can.canb->isExtendedFrame())
		{
			Serial.println("IsExtendedFrame");
		}
		else
		{
			Serial.println("Is NOT ExtendedFrame");
		}

		// print the data
		for (int i = 0; i<len; i++)
		{
			Serial.print(buf[i]); Serial.print("\t");
		}
		Serial.println();
	}
}