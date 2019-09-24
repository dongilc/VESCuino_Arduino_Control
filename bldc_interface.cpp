/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * bldc_interface.c
 *
 * Compatible Firmware Versions
 * 2.16
 *
 */

//#define DEBUG_BLDC

#include <bldc_interface.h>
#include <buffer.h>

// Private variables
//static unsigned char send_buffer[256]; //512/3];
//static int32_t can_fwd_vesc = -1;

// Private variables for received data
static mc_values values;
static int fw_major;
static int fw_minor;
static float rotor_pos;
static mc_configuration mcconf;
static app_configuration appconf;
static float detect_cycle_int_limit;
static float detect_coupling_k;
static signed char detect_hall_table[8];
static signed char detect_hall_res;
static float dec_ppm;
static float dec_ppm_len;
static float dec_adc;
static float dec_adc_voltage;
static float dec_chuk;

// Private functions
void send_packet_no_fwd(unsigned char *data, int len);
//static void fwd_can_append(uint8_t *data, int *ind);

// Function pointers
static void(*send_func)(unsigned char *data, int len) = 0;
static void(*forward_func)(unsigned char *data, int len) = 0;

// Function pointers for received data
static void(*rx_value_func)(mc_values *values) = 0;
static void(*rx_printf_func)(char *str) = 0;
static void(*rx_fw_func)(int major, int minor) = 0;
static void(*rx_rotor_pos_func)(float pos) = 0;
static void(*rx_mcconf_func)(mc_configuration *conf) = 0;
static void(*rx_appconf_func)(app_configuration *conf) = 0;
static void(*rx_detect_func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res) = 0;
static void(*rx_dec_ppm_func)(float val, float ms) = 0;
static void(*rx_dec_adc_func)(float val, float voltage) = 0;
static void(*rx_dec_chuk_func)(float val) = 0;
static void(*rx_mcconf_received_func)(void) = 0;
static void(*rx_appconf_received_func)(void) = 0;
static void(*rx_custom_app_data_func)(custom_values *c_values) = 0;	//cdi

/**
 * Initialize bldc_interface.
 *
 * @param func
 * A function to be used when sending packets. Null (0) means that no packets will be sent.
 */
void bldc_interface_init(void(*func)(unsigned char *data, int len)) {
	//can_fwd_vesc = -1;
	send_func = func;
}

/**
 * Enable or disable can forwarding to other VESCs.
 *
 * @param vesc_id
 * The VESC ID to forward to. Setting this to -1 disables this feature.
 */
/*
void bldc_interface_set_forward_can(int32_t vesc_id) {
	can_fwd_vesc = vesc_id;
}
*/

/**
 * Provide a function to forward received data to instead of processing it and calling handlers.
 * This will also prevent data from being sent.
 *
 * @param func
 * The forward function. Null (0) to disable forwarding.
 */
void bldc_interface_set_forward_func(void(*func)(unsigned char *data, int len)) {
	forward_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void bldc_interface_send_packet(unsigned char *data, int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void bldc_interface_process_packet(unsigned char *data, int len) {
	#ifdef DEBUG_BLDC
		Serial.println(F("[ bldc ] New packet to process!"));
	#endif
	if (!len) {
		return;
	}

	if (forward_func) {
		forward_func(data, len);
		return;
	}

	int ind = 0;
	int i = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_FW_VERSION:
		//if (len == 2) {
			ind = 0;
			fw_major = data[ind++];
			fw_minor = data[ind++];
		//} else {
		//	fw_major = -1;
		//	fw_minor = -1;
		//}
#ifdef DEBUG_BLDC
		Serial.print(F("[ bldc ] COMM_FW_VERSION = "));
		Serial.print(fw_major);
		Serial.print(F("."));
		Serial.println(fw_minor);
#endif
		if (rx_fw_func) {
			rx_fw_func(fw_major, fw_minor);
		}
		break;

	case COMM_ERASE_NEW_APP:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_ERASE_NEW_APP = ?"));
		#endif
		// TODO
		break;
	case COMM_WRITE_NEW_APP_DATA:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_WRITE_NEW_APP_DATA = ?"));
		#endif
		// TODO
		break;

	case COMM_GET_VALUES:
		ind = 0;
		/*
		// Old Version
		values.temp_mos1 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos2 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos3 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos4 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos5 = buffer_get_float16(data, 10.0, &ind);
		values.temp_mos6 = buffer_get_float16(data, 10.0, &ind);
		values.temp_pcb = buffer_get_float16(data, 10.0, &ind);
		values.current_motor = buffer_get_float32(data, 100.0, &ind);
		values.current_in = buffer_get_float32(data, 100.0, &ind);
		values.duty_now = buffer_get_float16(data, 1000.0, &ind);
		values.rpm = buffer_get_float32(data, 1.0, &ind);
		values.v_in = buffer_get_float16(data, 10.0, &ind);
		values.amp_hours = buffer_get_float32(data, 10000.0, &ind);
		values.amp_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
		values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		values.tachometer = buffer_get_int32(data, &ind);
		values.tachometer_abs = buffer_get_int32(data, &ind);
		values.fault_code = (mc_fault_code)data[ind++];
		*/

		values.temp_mos1 = buffer_get_float16(data, 1e1, &ind);
		values.temp_pcb = buffer_get_float16(data, 1e1, &ind);
		values.current_motor = buffer_get_float32(data, 1e2, &ind);
		values.current_in = buffer_get_float32(data, 1e2, &ind);
		ind += 8;
		values.duty_now = buffer_get_float16(data, 1e3, &ind);
		values.rpm = buffer_get_float32(data, 1e0, &ind);
		values.v_in = buffer_get_float16(data, 1e1, &ind);
		values.amp_hours = buffer_get_float32(data, 10000.0, &ind);
		values.amp_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
		values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		values.tachometer = buffer_get_int32(data, &ind);
		values.tachometer_abs = buffer_get_int32(data, &ind);
		values.fault_code = (mc_fault_code)data[ind++];
		values.pid_pos_now = buffer_get_float32(data, 1e6, &ind);	//cdi

#ifdef DEBUG_BLDC
		Serial.print(F("[ bldc ] COMM_GET_VALUES = temp_fet["));
		Serial.print(values.temp_mos1);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = temp_motor["));
		Serial.print(values.temp_pcb);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = current_motor["));
		Serial.print(values.current_motor);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = current_in["));
		Serial.print(values.current_in);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = duty_now["));
		Serial.print(values.duty_now);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = erpm["));
		Serial.print(values.rpm);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = v_in["));
		Serial.print(values.v_in);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = amp_hours["));
		Serial.print(values.amp_hours);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = amp_hours_charged["));
		Serial.print(values.amp_hours_charged);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = watt_hours["));
		Serial.print(values.watt_hours);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = watt_hours_charged["));
		Serial.print(values.watt_hours_charged);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = tachometer["));
		Serial.print(values.tachometer);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = tachometer_abs["));
		Serial.print(values.tachometer_abs);
		Serial.println("]");
		Serial.print(F("[ bldc ] COMM_GET_VALUES = fault_code["));
		Serial.print(values.fault_code);
		Serial.println("]");
#endif

		if (rx_value_func) {
			rx_value_func(&values);
		}
		break;

	case COMM_PRINT:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_PRINT = ?"));
		#endif
		if (rx_printf_func) {
			data[len] = '\0';
			rx_printf_func((char*)data);
		}
		break;

	case COMM_SAMPLE_PRINT:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_SAMPLE_PRINT = ?"));
		#endif
		// TODO
		break;

	case COMM_ROTOR_POSITION:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_ROTOR_POSITION = ?"));
		#endif
		ind = 0;
		rotor_pos = buffer_get_float32(data, 100000.0, &ind);

		if (rx_rotor_pos_func) {
			rx_rotor_pos_func(rotor_pos);
		}
		break;

	case COMM_EXPERIMENT_SAMPLE:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_EXPERIMENT_SAMPLE = ?"));
		#endif
		// TODO
		break;

	case COMM_GET_MCCONF:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_GET_MCCONF = ?"));
		#endif
		//TODO
		break;
	case COMM_GET_MCCONF_DEFAULT:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_GET_MCCONF_DEFAULT = ?"));
		#endif
		ind = 0;
		mcconf.pwm_mode = (mc_pwm_mode)data[ind++];
		mcconf.comm_mode = (mc_comm_mode)data[ind++];
		mcconf.motor_type = (mc_motor_type)data[ind++];
		mcconf.sensor_mode = (mc_sensor_mode)data[ind++];

		mcconf.l_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_current_min = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_in_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_in_current_min = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_abs_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm_fbrake = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm_fbrake_cc = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_vin = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_vin = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_battery_cut_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_battery_cut_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_slow_abs_current = data[ind++];
		mcconf.l_rpm_lim_neg_torque = data[ind++];
		mcconf.l_temp_fet_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_fet_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_motor_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_motor_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_duty = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.l_max_duty = buffer_get_float32(data, 1000000.0, &ind);

		mcconf.sl_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_phase_advance_at_br = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_cycle_int_rpm_br = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_bemf_coupling_k = buffer_get_float32(data, 1000.0, &ind);

		memcpy(mcconf.hall_table, data + ind, 8);
		ind += 8;
		mcconf.hall_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.foc_current_kp = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_current_ki = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_f_sw = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_dt_us = buffer_get_float32(data, 1e6, &ind);
		mcconf.foc_encoder_inverted = data[ind++];
		mcconf.foc_encoder_offset = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_encoder_ratio = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sensor_mode = (mc_foc_sensor_mode)data[ind++];
		mcconf.foc_pll_kp = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_pll_ki = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_motor_l = buffer_get_float32(data, 1e8, &ind);
		mcconf.foc_motor_r = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_motor_flux_linkage = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_observer_gain = buffer_get_float32(data, 1e0, &ind);
		mcconf.foc_duty_dowmramp_kp = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_duty_dowmramp_ki = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_openloop_rpm = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_openloop_hyst = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_openloop_time = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_d_current_duty = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_d_current_factor = buffer_get_float32(data, 1e3, &ind);
		memcpy(mcconf.foc_hall_table, data + ind, 8);
		ind += 8;
		mcconf.foc_hall_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.s_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_min_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.p_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_ang_div = buffer_get_float32(data, 1e5, &ind);

		mcconf.cc_startup_boost_duty = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.cc_min_current = buffer_get_float32(data, 1000.0, &ind);
		mcconf.cc_gain = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.cc_ramp_step_max = buffer_get_float32(data, 1000000.0, &ind);

		mcconf.m_fault_stop_time_ms = buffer_get_int32(data, &ind);
		mcconf.m_duty_ramp_step = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_duty_ramp_step_rpm_lim = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_current_backoff_gain = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_encoder_counts = buffer_get_uint32(data, &ind);

		if (rx_mcconf_func) {
			rx_mcconf_func(&mcconf);
		}
		break;

	case COMM_GET_APPCONF:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_GET_APPCONF = ?"));
		#endif
		// TODO
		break;
	case COMM_GET_APPCONF_DEFAULT:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_GET_APPCONF_DEFAULT = ?"));
		#endif
		ind = 0;
		appconf.controller_id = data[ind++];
		appconf.timeout_msec = buffer_get_uint32(data, &ind);
		appconf.timeout_brake_current = buffer_get_float32(data, 1000.0, &ind);
		appconf.send_can_status = data[ind++];
		appconf.send_can_status_rate_hz = buffer_get_uint16(data, &ind);

		appconf.app_to_use = (app_use)data[ind++];

		appconf.app_ppm_conf.ctrl_type = (ppm_control_type)data[ind++];
		appconf.app_ppm_conf.pid_max_erpm = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.hyst = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.pulse_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.pulse_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.median_filter = data[ind++];
		appconf.app_ppm_conf.safe_start = data[ind++];
		appconf.app_ppm_conf.rpm_lim_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.rpm_lim_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_ppm_conf.multi_esc = data[ind++];
		appconf.app_ppm_conf.tc = data[ind++];
		appconf.app_ppm_conf.tc_max_diff = buffer_get_float32(data, 1000.0, &ind);

		appconf.app_adc_conf.ctrl_type = (adc_control_type)data[ind++];
		appconf.app_adc_conf.hyst = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.voltage_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.voltage_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.use_filter = data[ind++];
		appconf.app_adc_conf.safe_start = data[ind++];
		appconf.app_adc_conf.cc_button_inverted = data[ind++];
		appconf.app_adc_conf.rev_button_inverted = data[ind++];
		appconf.app_adc_conf.voltage_inverted = data[ind++];
		appconf.app_adc_conf.rpm_lim_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.rpm_lim_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.multi_esc = data[ind++];
		appconf.app_adc_conf.tc = data[ind++];
		appconf.app_adc_conf.tc_max_diff = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_adc_conf.update_rate_hz = buffer_get_uint16(data, &ind);

		appconf.app_uart_baudrate = buffer_get_uint32(data, &ind);

		appconf.app_chuk_conf.ctrl_type = (chuk_control_type)data[ind++];
		appconf.app_chuk_conf.hyst = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.rpm_lim_start = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.rpm_lim_end = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.ramp_time_pos = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.ramp_time_neg = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.stick_erpm_per_s_in_cc = buffer_get_float32(data, 1000.0, &ind);
		appconf.app_chuk_conf.multi_esc = data[ind++];
		appconf.app_chuk_conf.tc = data[ind++];
		appconf.app_chuk_conf.tc_max_diff = buffer_get_float32(data, 1000.0, &ind);

		appconf.app_nrf_conf.speed = (NRF_SPEED)data[ind++];
		appconf.app_nrf_conf.power = (NRF_POWER)data[ind++];
		appconf.app_nrf_conf.crc_type = (NRF_CRC)data[ind++];
		appconf.app_nrf_conf.retry_delay = (NRF_RETR_DELAY)data[ind++];
		appconf.app_nrf_conf.retries = data[ind++];
		appconf.app_nrf_conf.channel = data[ind++];
		memcpy(appconf.app_nrf_conf.address, data + ind, 3);
		ind += 3;
		appconf.app_nrf_conf.send_crc_ack = data[ind++];

		if (rx_appconf_func) {
			rx_appconf_func(&appconf);
		}
		break;

	case COMM_DETECT_MOTOR_PARAM:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_DETECT_MOTOR_PARAM = ?"));
		#endif
		ind = 0;
		detect_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		detect_coupling_k = buffer_get_float32(data, 1000.0, &ind);
		for (i = 0;i < 8;i++) {
			detect_hall_table[i] = (const signed char)(data[ind++]);
		}
		detect_hall_res = (const signed char)(data[ind++]);

		if (rx_detect_func) {
			rx_detect_func(detect_cycle_int_limit, detect_coupling_k,
					detect_hall_table, detect_hall_res);
		}
		break;

	case COMM_DETECT_MOTOR_R_L: {
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_DETECT_MOTOR_R_L = ?"));
		#endif
		// TODO!
	} break;

	case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_DETECT_MOTOR_FLUX_LINKAGE = ?"));
		#endif
		// TODO!
	} break;

	case COMM_DETECT_ENCODER: {
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_DETECT_ENCODER = ?"));
		#endif
		// TODO!
	} break;

	case COMM_DETECT_HALL_FOC: {
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_DETECT_HALL_FOC = ?"));
		#endif
		// TODO!
	} break;

	case COMM_GET_DECODED_PPM:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_GET_DECODED_PPM = ?"));
		#endif
		ind = 0;
		dec_ppm = buffer_get_float32(data, 1000000.0, &ind);
		dec_ppm_len = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_ppm_func) {
			rx_dec_ppm_func(dec_ppm, dec_ppm_len);
		}
		break;

	case COMM_GET_DECODED_ADC:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_GET_DECODED_ADC = ?"));
		#endif
		ind = 0;
		dec_adc = buffer_get_float32(data, 1000000.0, &ind);
		dec_adc_voltage = buffer_get_float32(data, 1000000.0, &ind);
		// TODO for adc2

		if (rx_dec_adc_func) {
			rx_dec_adc_func(dec_adc, dec_adc_voltage);
		}
		break;

	case COMM_GET_DECODED_CHUK:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_GET_DECODED_CHUK = ?"));
		#endif
		ind = 0;
		dec_chuk = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_chuk_func) {
			rx_dec_chuk_func(dec_chuk);
		}
		break;

	case COMM_SET_MCCONF:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_SET_MCCONF = ?"));
		#endif
		// This is a confirmation that the new mcconf is received.
		if (rx_mcconf_received_func) {
			rx_mcconf_received_func();
		}
		break;

	case COMM_SET_APPCONF:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] COMM_SET_APPCONF = ?"));
		#endif
		// This is a confirmation that the new appconf is received.
		if (rx_appconf_received_func) {
			rx_appconf_received_func();
		}
		break;

	//cdi
	// spi rx data process
	case COMM_CUSTOM_APP_DATA:
#ifdef DEBUG_BLDC
		Serial.println(F("[ bldc ] COMM_CUSTOM_APP_DATA = ?"));
#endif
		ind = 0;
		c_values.send_mode = data[ind++];
		{
			// fw
			c_values.fw_major = data[ind++];
			c_values.fw_minor = data[ind++];
			// get value
			c_values.temp_mos1 = buffer_get_float16(data, 1e1, &ind);
			c_values.temp_pcb = buffer_get_float16(data, 1e1, &ind);
			c_values.current_motor = buffer_get_float32(data, 1e2, &ind);
			c_values.current_in = buffer_get_float32(data, 1e2, &ind);
			c_values.duty_now = buffer_get_float16(data, 1e3, &ind);
			c_values.rpm = buffer_get_float32(data, 1e0, &ind);
			c_values.v_in = buffer_get_float16(data, 1e1, &ind);
			c_values.amp_hours = buffer_get_float32(data, 10000.0, &ind);
			c_values.amp_hours_charged = buffer_get_float32(data, 10000.0, &ind);
			c_values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
			c_values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);			
			c_values.tachometer = buffer_get_int32(data, &ind);
			c_values.tachometer_abs = buffer_get_int32(data, &ind);
			c_values.fault_code = (mc_fault_code)data[ind++];
			c_values.enc_pos = buffer_get_float32(data, 1e6, &ind);
			//
			c_values.enc_rps = buffer_get_float32(data, 1e5, &ind);
			c_values.enc_rad = buffer_get_float32(data, 1e2, &ind);
		}
		c_values.send_mode = data[ind++];
		{
			// can status msgs
			c_values.num_of_id = data[ind++];
			for (int i = 0; i < c_values.num_of_id; i++)
			{
				c_values.id_can[i] = data[ind++];
				//c_values.rpm_can[i] = buffer_get_float32(data, 1e0, &ind);
				//c_values.current_can[i] = buffer_get_float32(data, 1e2, &ind);
				//c_values.duty_can[i] = buffer_get_float16(data, 1e3, &ind);
				c_values.rps_can[i] = buffer_get_float32(data, 1e5, &ind);
				c_values.rad_can[i] = buffer_get_float32(data, 1e2, &ind);
			}
		}

		// run rx data post function
		if (rx_custom_app_data_func) {
			rx_custom_app_data_func(&c_values);
		}
		break;

	default:
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] default = ?"));
		#endif
		break;
	}
}

/**
 * Function pointer setters. When data that is requested with the get functions
 * is received, the corresponding function pointer will be called with the
 * received data.
 *
 * @param func
 * A function to be called when the corresponding data is received.
 */

void bldc_interface_set_rx_value_func(void(*func)(mc_values *values)) {
	rx_value_func = func;
}

void bldc_interface_set_rx_printf_func(void(*func)(char *str)) {
	rx_printf_func = func;
}

void bldc_interface_set_rx_fw_func(void(*func)(int major, int minor)) {
	rx_fw_func = func;
}

void bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos)) {
	rx_rotor_pos_func = func;
}

void bldc_interface_set_rx_mcconf_func(void(*func)(mc_configuration *conf)) {
	rx_mcconf_func = func;
}

void bldc_interface_set_rx_appconf_func(void(*func)(app_configuration *conf)) {
	rx_appconf_func = func;
}

void bldc_interface_set_rx_detect_func(void(*func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res)) {
	rx_detect_func = func;
}

void bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms)) {
	rx_dec_ppm_func = func;
}

void bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage)) {
	rx_dec_adc_func = func;
}

void bldc_interface_set_rx_dec_chuk_func(void(*func)(float val)) {
	rx_dec_chuk_func = func;
}

void bldc_interface_set_rx_mcconf_received_func(void(*func)(void)) {
	rx_mcconf_received_func = func;
}

void bldc_interface_set_rx_appconf_received_func(void(*func)(void)) {
	rx_appconf_received_func = func;
}

//cdi
void bldc_interface_set_rx_custom_app_data_func(void(*func)(custom_values *values)) {
	rx_custom_app_data_func = func;
}

// cdi
void bldc_interface_communicate_custom(uint8_t arduino_model, uint8_t num_of_id, uint8_t *id, uint8_t *comm, float *value)
{
	unsigned char send_buffer[512];
	int send_index = 0;

	uint8_t id_set = 0;
	uint8_t comm_set = 0;
	float value_set;

	//fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_CUSTOM_APP_DATA;	// +1
	send_buffer[send_index++] = arduino_model;			// +1
	send_buffer[send_index++] = num_of_id;				// +1
	for (int i = 0; i < num_of_id; i++) // +6
	{
		id_set = id[i];
		comm_set = comm[i];
		value_set = value[i];

		send_buffer[send_index++] = id_set;
		send_buffer[send_index++] = comm_set;
		switch (comm_set) {
		case COMM_SET_DUTY:
			buffer_append_float32(send_buffer, value_set, 100000.0, &send_index);
			break;
		case COMM_SET_CURRENT:
			buffer_append_float32(send_buffer, value_set, 1000.0, &send_index);
			break;
		case COMM_SET_CURRENT_BRAKE:
			buffer_append_float32(send_buffer, value_set, 1000.0, &send_index);
			break;
		case COMM_SET_RPM:
			buffer_append_int32(send_buffer, (int)value_set, &send_index);
			break;
		case COMM_SET_POS:
			buffer_append_float32(send_buffer, value_set, 1000000.0, &send_index);
			break;
		default:
			break;
		}
	}
	send_packet_no_fwd(send_buffer, send_index);
}

// Setters
/*
void bldc_interface_terminal_cmd(char* cmd) {
	int send_index = 0;
	int len = strlen(cmd);
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_TERMINAL_CMD;
	memcpy(send_buffer + send_index, cmd, len);
	send_index += len;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_duty_cycle(float dutyCycle) {
	uint8_t send_buffer[5];
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_DUTY;
	buffer_append_float32(send_buffer, dutyCycle, 100000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_current(float current) {
	uint8_t send_buffer[5];
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_current_brake(float current) {
	uint8_t send_buffer[5];
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_rpm(int rpm) {
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_RPM;
	buffer_append_int32(send_buffer, rpm, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_pos(float pos) {
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_POS;
	buffer_append_float32(send_buffer, pos, 1000000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_servo_pos(float pos) {
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_SERVO_POS;
	buffer_append_float16(send_buffer, pos, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_uartchuck_data(const chuck_data *chuck_d_remote) {
	uint8_t send_buffer[13];
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_CUSTOM_APP_DATA;
	send_buffer[send_index++] = (uint8_t) chuck_d_remote->js_x;
	send_buffer[send_index++] = (uint8_t) chuck_d_remote->js_y;
	send_buffer[send_index++] = (uint8_t) chuck_d_remote->bt_c;
	send_buffer[send_index++] = (uint8_t) chuck_d_remote->bt_z;
	buffer_append_int16(send_buffer, chuck_d_remote->acc_x, &send_index);
	buffer_append_int16(send_buffer, chuck_d_remote->acc_y, &send_index);
	buffer_append_int16(send_buffer, chuck_d_remote->acc_z, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_mcconf(const mc_configuration *mcconf) {
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_MCCONF;

	send_buffer[send_index++] = mcconf->pwm_mode;
	send_buffer[send_index++] = mcconf->comm_mode;
	send_buffer[send_index++] = mcconf->motor_type;
	send_buffer[send_index++] = mcconf->sensor_mode;

	buffer_append_float32(send_buffer, mcconf->l_current_max, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_current_min, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_in_current_max, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_in_current_min, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_abs_current_max, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_min_erpm, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_max_erpm, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_max_erpm_fbrake, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_max_erpm_fbrake_cc, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_min_vin, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_max_vin, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_battery_cut_start, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_battery_cut_end, 1000, &send_index);
	send_buffer[send_index++] = mcconf->l_slow_abs_current;
	send_buffer[send_index++] = mcconf->l_rpm_lim_neg_torque;
	buffer_append_float32(send_buffer,mcconf->l_temp_fet_start, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_temp_fet_end, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_temp_motor_start, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_temp_motor_end, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_min_duty, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->l_max_duty, 1000000, &send_index);

	buffer_append_float32(send_buffer,mcconf->sl_min_erpm, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->sl_min_erpm_cycle_int_limit, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->sl_max_fullbreak_current_dir_change, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->sl_cycle_int_limit, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->sl_phase_advance_at_br, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->sl_cycle_int_rpm_br, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->sl_bemf_coupling_k, 1000, &send_index);

	memcpy(send_buffer + send_index, mcconf->hall_table, 8);
	send_index += 8;
	buffer_append_float32(send_buffer,mcconf->hall_sl_erpm, 1000, &send_index);

	buffer_append_float32(send_buffer, mcconf->foc_current_kp, 1e5, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_current_ki, 1e5, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_f_sw, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_dt_us, 1e6, &send_index);
	send_buffer[send_index++] = mcconf->foc_encoder_inverted;
	buffer_append_float32(send_buffer, mcconf->foc_encoder_offset, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_encoder_ratio, 1e3, &send_index);
	send_buffer[send_index++] = mcconf->foc_sensor_mode;
	buffer_append_float32(send_buffer, mcconf->foc_pll_kp, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_pll_ki, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_motor_l, 1e8, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_motor_r, 1e5, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_motor_flux_linkage, 1e5, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_observer_gain, 1e0, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_duty_dowmramp_kp, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_duty_dowmramp_ki, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_openloop_rpm, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_sl_openloop_hyst, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_sl_openloop_time, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_sl_d_current_duty, 1e3, &send_index);
	buffer_append_float32(send_buffer, mcconf->foc_sl_d_current_factor, 1e3, &send_index);
	memcpy(send_buffer + send_index, mcconf->foc_hall_table, 8);
	send_index += 8;
	buffer_append_float32(send_buffer,mcconf->foc_hall_sl_erpm, 1000, &send_index);

	buffer_append_float32(send_buffer,mcconf->s_pid_kp, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->s_pid_ki, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->s_pid_kd, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->s_pid_min_erpm, 1000, &send_index);

	buffer_append_float32(send_buffer,mcconf->p_pid_kp, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->p_pid_ki, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->p_pid_kd, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->p_pid_ang_div, 1e5, &send_index);

	buffer_append_float32(send_buffer,mcconf->cc_startup_boost_duty, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->cc_min_current, 1000, &send_index);
	buffer_append_float32(send_buffer,mcconf->cc_gain, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->cc_ramp_step_max, 1000000, &send_index);

	buffer_append_int32(send_buffer, mcconf->m_fault_stop_time_ms, &send_index);
	buffer_append_float32(send_buffer,mcconf->m_duty_ramp_step, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->m_duty_ramp_step_rpm_lim, 1000000, &send_index);
	buffer_append_float32(send_buffer,mcconf->m_current_backoff_gain, 1000000, &send_index);
	buffer_append_uint32(send_buffer, mcconf->m_encoder_counts, &send_index);

	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_appconf(const app_configuration *appconf) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_SET_APPCONF;
	send_buffer[send_index++] = appconf->controller_id;
	buffer_append_uint32(send_buffer, appconf->timeout_msec, &send_index);
	buffer_append_float32(send_buffer, appconf->timeout_brake_current, 1000.0, &send_index);
	send_buffer[send_index++] = appconf->send_can_status;
	buffer_append_uint16(send_buffer, appconf->send_can_status_rate_hz, &send_index);

	send_buffer[send_index++] = appconf->app_to_use;

	send_buffer[send_index++] = appconf->app_ppm_conf.ctrl_type;
	buffer_append_float32(send_buffer, appconf->app_ppm_conf.pid_max_erpm, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_ppm_conf.hyst, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_ppm_conf.pulse_start, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_ppm_conf.pulse_end, 1000.0, &send_index);
	send_buffer[send_index++] = appconf->app_ppm_conf.median_filter;
	send_buffer[send_index++] = appconf->app_ppm_conf.safe_start;
	buffer_append_float32(send_buffer, appconf->app_ppm_conf.rpm_lim_start, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_ppm_conf.rpm_lim_end, 1000.0, &send_index);
	send_buffer[send_index++] = appconf->app_ppm_conf.multi_esc;
	send_buffer[send_index++] = appconf->app_ppm_conf.tc;
	buffer_append_float32(send_buffer, appconf->app_ppm_conf.tc_max_diff, 1000.0, &send_index);

	send_buffer[send_index++] = appconf->app_adc_conf.ctrl_type;
	buffer_append_float32(send_buffer, appconf->app_adc_conf.hyst, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_adc_conf.voltage_start, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_adc_conf.voltage_end, 1000.0, &send_index);
	send_buffer[send_index++] = appconf->app_adc_conf.use_filter;
	send_buffer[send_index++] = appconf->app_adc_conf.safe_start;
	send_buffer[send_index++] = appconf->app_adc_conf.cc_button_inverted;
	send_buffer[send_index++] = appconf->app_adc_conf.rev_button_inverted;
	send_buffer[send_index++] = appconf->app_adc_conf.voltage_inverted;
	buffer_append_float32(send_buffer, appconf->app_adc_conf.rpm_lim_start, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_adc_conf.rpm_lim_end, 1000.0, &send_index);
	send_buffer[send_index++] = appconf->app_adc_conf.multi_esc;
	send_buffer[send_index++] = appconf->app_adc_conf.tc;
	buffer_append_float32(send_buffer, appconf->app_adc_conf.tc_max_diff, 1000.0, &send_index);
	buffer_append_uint16(send_buffer, appconf->app_adc_conf.update_rate_hz, &send_index);

	buffer_append_uint32(send_buffer, appconf->app_uart_baudrate, &send_index);

	send_buffer[send_index++] = appconf->app_chuk_conf.ctrl_type;
	buffer_append_float32(send_buffer, appconf->app_chuk_conf.hyst, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_chuk_conf.rpm_lim_start, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_chuk_conf.rpm_lim_end, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_chuk_conf.ramp_time_pos, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_chuk_conf.ramp_time_neg, 1000.0, &send_index);
	buffer_append_float32(send_buffer, appconf->app_chuk_conf.stick_erpm_per_s_in_cc, 1000.0, &send_index);
	send_buffer[send_index++] = appconf->app_chuk_conf.multi_esc;
	send_buffer[send_index++] = appconf->app_chuk_conf.tc;
	buffer_append_float32(send_buffer, appconf->app_chuk_conf.tc_max_diff, 1000.0, &send_index);

	send_buffer[send_index++] = appconf->app_nrf_conf.speed;
	send_buffer[send_index++] = appconf->app_nrf_conf.power;
	send_buffer[send_index++] = appconf->app_nrf_conf.crc_type;
	send_buffer[send_index++] = appconf->app_nrf_conf.retry_delay;
	send_buffer[send_index++] = appconf->app_nrf_conf.retries;
	send_buffer[send_index++] = appconf->app_nrf_conf.channel;
	memcpy(send_buffer + send_index, appconf->app_nrf_conf.address, 3);
	send_index += 3;
	send_buffer[send_index++] = appconf->app_nrf_conf.send_crc_ack;

	send_packet_no_fwd(send_buffer, send_index);
}
*/

// Getters
/*
void bldc_interface_get_fw_version(void) {
	unsigned char send_buffer[1];
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_FW_VERSION;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_values(void) {
	unsigned char send_buffer[3];
	int send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_GET_VALUES;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_mcconf(void) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_GET_MCCONF;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_appconf(void) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_GET_APPCONF;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_ppm(void) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_GET_DECODED_PPM;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_adc(void) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_GET_DECODED_ADC;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_chuk(void) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_GET_DECODED_CHUK;
	send_packet_no_fwd(send_buffer, send_index);
}

// Other functions
void bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_DETECT_MOTOR_PARAM;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	buffer_append_float32(send_buffer, min_rpm, 1000.0, &send_index);
	buffer_append_float32(send_buffer, low_duty, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_reboot(void) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_REBOOT;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_send_alive(void) {
	int send_index = 0;
	//fwd_can_append(send_buffer, &send_index);
	fwd_can_append();
	send_buffer[send_index++] = COMM_ALIVE;
	send_packet_no_fwd(send_buffer, send_index);
}
*/

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV8302: return "FAULT_CODE_DRV8302";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	default: return "Unknown fault";
	}
}

// Private functions
void send_packet_no_fwd(unsigned char *data, int len) {
	if (!forward_func) {
		bldc_interface_send_packet(data, len);
	}
}

/*
static void fwd_can_append(uint8_t *data, int *ind) {
	if (can_fwd_vesc >= 0) {
		data[*ind++] = COMM_FORWARD_CAN;
		data[*ind++] = can_fwd_vesc;
	}
}
*/