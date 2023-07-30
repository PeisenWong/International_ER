/*
	Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef VESC_CAN_BLDC_INTERFACE_H_
#define VESC_CAN_BLDC_INTERFACE_H_

#include <stdbool.h>
#include "datatypes.h"
#include "vesc_interface.h"

#define MCCONF_SIGNATURE		503309878
#define APPCONF_SIGNATURE		783041200

// Global variables
bool rx_value_selective_print;
bool rx_value_complete;
mc_values rx_value_buf;
char zzz[100];

void bldc_interface_init(void(*func)(uint32_t controller_id, unsigned char *data, unsigned int len));
void bldc_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len));
void bldc_interface_send_packet(uint32_t controller_id, unsigned char *data, unsigned int len);
void bldc_interface_process_packet(unsigned char *data, unsigned int len);

// Function pointer setters
void bldc_interface_set_rx_value_func(void(*func)(mc_values *values));
void bldc_interface_set_rx_value_selective_func(void(*func)(uint32_t mode, mc_values *values));
void bldc_interface_set_rx_printf_func(void(*func)(char *str));
void bldc_interface_set_rx_fw_func(void(*func)(int major, int minor));
void bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos));
void bldc_interface_set_rx_mcconf_func(void(*func)(mc_configuration *conf));
void bldc_interface_set_rx_appconf_func(void(*func)(app_configuration *conf));
void bldc_interface_set_rx_detect_func(void(*func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res));
void bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms));
void bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage));
void bldc_interface_set_rx_dec_chuk_func(void(*func)(float val));
void bldc_interface_set_rx_mcconf_received_func(void(*func)(void));
void bldc_interface_set_rx_appconf_received_func(void(*func)(void));

void bldc_interface_set_sim_control_function(void(*func)(uint32_t controller_id, motor_control_mode mode, float value));
void bldc_interface_set_sim_values_func(void(*func)(uint32_t controller_id));

// Setters
void bldc_interface_terminal_cmd(uint32_t controller_id, char* cmd);
void bldc_interface_set_duty_cycle(uint32_t controller_id, float dutyCycle);
void bldc_interface_set_current(uint32_t controller_id, float current);
void bldc_interface_set_current_brake(uint32_t controller_id, float current);
void bldc_interface_set_rpm(uint32_t controller_id, int rpm);
void bldc_interface_set_pos(uint32_t controller_id, float pos);
void bldc_interface_set_handbrake(uint32_t controller_id, float current);
void bldc_interface_set_servo_pos(uint32_t controller_id, float pos);
void bldc_interface_set_mcconf(uint32_t controller_id, const mc_configuration *mcconf);
void bldc_interface_set_appconf(uint32_t controller_id, const app_configuration *appconf);

// Getters
void bldc_interface_get_fw_version(uint32_t controller_id);
void bldc_interface_get_values(uint32_t controller_id);
void bldc_interface_get_values_selective(uint32_t controller_id, COMM_GET_VALUE_SELECTIVE_t value_bit_pos);
void bldc_interface_get_mcconf(uint32_t controller_id);
void bldc_interface_get_appconf(uint32_t controller_id);
void bldc_interface_get_decoded_ppm(uint32_t controller_id);
void bldc_interface_get_decoded_adc(uint32_t controller_id);
void bldc_interface_get_decoded_chuk(uint32_t controller_id);

// Other functions
void bldc_interface_detect_motor_param(uint32_t controller_id, float current, float min_rpm, float low_duty);
void bldc_interface_reboot(uint32_t controller_id);
void bldc_interface_send_alive(uint32_t controller_id);
void send_values_to_receiver(mc_values *values);

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault);

// User Callbacks
void bldc_val_selective_received_cb(uint32_t mode, mc_values *val);
#endif /* VESC_CAN_BLDC_INTERFACE_H_ */
