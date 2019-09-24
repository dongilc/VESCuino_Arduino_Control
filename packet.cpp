/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

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
 * packet.c
 *
 *  Created on: 21 mar 2013
 *      Author: benjamin
 */

#include <string.h>
#include <packet.h>
#include <crc.h>

//#define DEBUG_BLDC_P

typedef struct {
	volatile unsigned char rx_state;
	volatile unsigned char rx_timeout;
	void(*send_func)(unsigned char *data, int len);
	void(*process_func)(unsigned char *data, int len);
	unsigned int payload_length;
	unsigned char rx_buffer[PACKET_MAX_PL_LEN];
	unsigned char tx_buffer[PACKET_MAX_PL_LEN + 6];
	unsigned int rx_data_ptr;
	unsigned char crc_low;
	unsigned char crc_high;
} PACKET_STATE_t;

static PACKET_STATE_t handler_states[PACKET_HANDLERS];

void packet_init(void (*s_func)(unsigned char *data, int len),
		void (*p_func)(unsigned char *data, int len), int handler_num) {
	handler_states[handler_num].send_func = s_func;
	handler_states[handler_num].process_func = p_func;
}

void packet_send_packet(unsigned char *data, int len, int handler_num) {
	if (len > PACKET_MAX_PL_LEN) {
		return;
	}

	int b_ind = 0;

	if (len <= 256) {
		handler_states[handler_num].tx_buffer[b_ind++] = 2;
		handler_states[handler_num].tx_buffer[b_ind++] = len;
	} else {
		handler_states[handler_num].tx_buffer[b_ind++] = 3;
		handler_states[handler_num].tx_buffer[b_ind++] = len >> 8;
		handler_states[handler_num].tx_buffer[b_ind++] = len & 0xFF;
	}

	memcpy(handler_states[handler_num].tx_buffer + b_ind, data, len);
	b_ind += len;

	unsigned short crc = crc16(data, len);
	handler_states[handler_num].tx_buffer[b_ind++] = (uint8_t)(crc >> 8);
	handler_states[handler_num].tx_buffer[b_ind++] = (uint8_t)(crc & 0xFF);
	handler_states[handler_num].tx_buffer[b_ind++] = 3;

	if (handler_states[handler_num].send_func) {
		handler_states[handler_num].send_func(handler_states[handler_num].tx_buffer, b_ind);
	}
}

/**
 * Call this function every millisecond.
 */
void packet_timerfunc(void) {
	int i = 0;
	for (i = 0;i < PACKET_HANDLERS;i++) {
		if (handler_states[i].rx_timeout) {
			handler_states[i].rx_timeout--;
		} else {
			handler_states[i].rx_state = 0;
			
			#ifdef DEBUG_BLDC_P
				Serial.println(F("[ bldc ] packet_timerfunc: Timeout!"));
			#endif
		}
	}
}

void packet_process_byte(uint8_t rx_data, int handler_num) {

	#ifdef DEBUG_BLDC_P
		Serial.print(F("[ bldc ] packet_process_byte = rx_data[ "));
		Serial.print(rx_data);
		Serial.print(F(" ] handler_states.rx_data[ "));
		Serial.print(handler_states[handler_num].rx_state);
		Serial.println(F(" ]"));
	#endif

	switch (handler_states[handler_num].rx_state) {
	case 0:
		if (rx_data == 2) {
			#ifdef DEBUG_BLDC_P
				Serial.println(F("[ bldc ] packet_process_byte: 1 byte PL len"));
			#endif
			// 1 byte PL len
			handler_states[handler_num].rx_state += 2;
			handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
			handler_states[handler_num].rx_data_ptr = 0;
			handler_states[handler_num].payload_length = 0;
		} else if (rx_data == 3) {
			#ifdef DEBUG_BLDC_P
				Serial.println(F("[ bldc ] packet_process_byte: 2 byte PL len"));
			#endif
			// 2 byte PL len
			handler_states[handler_num].rx_state++;
			handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
			handler_states[handler_num].rx_data_ptr = 0;
			handler_states[handler_num].payload_length = 0;
		} else {
			handler_states[handler_num].rx_state = 0;
		}
		break;

	case 1:
		handler_states[handler_num].payload_length = (unsigned int)rx_data << 8;

		#ifdef DEBUG_BLDC_P
			Serial.print(F("[ bldc ] packet_process_byte = payload_length[ "));
			Serial.print( handler_states[handler_num].payload_length );
			Serial.println(F(" ]"));
		#endif

		handler_states[handler_num].rx_state++;
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 2:
		handler_states[handler_num].payload_length |= (unsigned int)rx_data;

		#ifdef DEBUG_BLDC_P
			Serial.print(F("[ bldc ] packet_process_byte = payload_length[ "));
			Serial.print( handler_states[handler_num].payload_length );
			Serial.println(F(" ]"));
		#endif
		if (handler_states[handler_num].payload_length > 0 &&
				handler_states[handler_num].payload_length <= PACKET_MAX_PL_LEN) {
			handler_states[handler_num].rx_state++;
			handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		} else {
			handler_states[handler_num].rx_state = 0;
		}
		break;

	case 3:
		handler_states[handler_num].rx_buffer[handler_states[handler_num].rx_data_ptr++] = rx_data;
		if (handler_states[handler_num].rx_data_ptr == handler_states[handler_num].payload_length) {
			handler_states[handler_num].rx_state++;
		}
		#ifdef DEBUG_BLDC_P
			Serial.print(F("[ bldc ] packet_process_byte = Received "));
			Serial.print( handler_states[handler_num].rx_data_ptr );
			Serial.print(F(" bytes of "));
			Serial.print( handler_states[handler_num].payload_length );
			Serial.println(F("."));
		#endif
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 4:
		handler_states[handler_num].crc_high = rx_data;
		handler_states[handler_num].rx_state++;
		#ifdef DEBUG_BLDC_P
			Serial.println(F("[ bldc ] packet_process_byte = Received crc_high byte."));
		#endif
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 5:
		handler_states[handler_num].crc_low = rx_data;
		handler_states[handler_num].rx_state++;
		#ifdef DEBUG_BLDC_P
			Serial.println(F("[ bldc ] packet_process_byte = Received crc_low byte."));
		#endif
		handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 6:
		if (rx_data == 3) {
			#ifdef DEBUG_BLDC_P
				Serial.print(F("[ bldc ] packet_process_byte = rx_data["));
				Serial.print(rx_data);
				Serial.print("] rx_state[");
				Serial.print(handler_states[handler_num].rx_state);
				Serial.print("] payload_length[");
				Serial.print(handler_states[handler_num].payload_length);
				Serial.println("]");
			#endif
			if (crc16(handler_states[handler_num].rx_buffer, handler_states[handler_num].payload_length)
					== ((unsigned short)handler_states[handler_num].crc_high << 8
						| (unsigned short)handler_states[handler_num].crc_low)) {
				//Serial.println("Packet received!");
				// Packet received!
				if (handler_states[handler_num].process_func) {
					handler_states[handler_num].process_func(handler_states[handler_num].rx_buffer,
							handler_states[handler_num].payload_length);
				}
			}
		}
		handler_states[handler_num].rx_state = 0;
		break;

	default:
		handler_states[handler_num].rx_state = 0;
		break;
	}
}
