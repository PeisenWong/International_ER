#include "vesc_interface.h"
#include <math.h>

static void bldc_send_packet(uint8_t controller_id, uint8_t *data, unsigned int len);

/*
 * Function Name		: VESCInit
 * Function Description : Initialize the bldc parameters
 * Function Remarks		: NONE
 * Function Arguments	: max_rpm			maximum rpm of the bldc
 * 						  pole_pairs		number of pole pairs of the bldc
 * 						  wheel_diameter 	wheel diameter
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCInit(40000,1,0.0037,&csev);
 */
void VESCInit(float max_rpm, float pole_pairs, float wheel_diameter,uint16_t Instance, VESC_t* vesc){
	bldc_interface_init(bldc_send_packet);
//	bldc_interface_set_rx_value_selective_func(bldc_val_selective_received_cb);

	vesc->max_rpm = max_rpm;
	vesc->pole_pairs = pole_pairs;
	vesc->wheel_diameter = wheel_diameter;
	vesc->Instance = Instance;
}

void VESCPOS(float arg, VESC_t* vesc){
	comm_can_set_pos(vesc->Instance, arg);
}

/*
 * Function Name		: VESCVelocity
 * Function Description : Command the VESC to move with specified velocity without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: FLeftVel		speed of front left motor in meter per second
 * 						  FRightVel		speed of front right motor in meter per second
 * 						  BLeftVel 		speed of back left motor in meter per second
 * 						  BRightVel		speed of back right motor in meter per second
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCVelocity(1.0, 1.5 , 1.5 , 1.0, &csev);
 */
void VESCVelocity(float Velocity, VESC_t* vesc) {

	float vel;
	vel = ((Velocity * 60) / (M_PI * vesc->wheel_diameter)) * vesc->pole_pairs;

	VESCRPM(vel, vesc);
}

/*
 * Function Name		: VESCRPM
 * Function Description : Command the VESC to move with specified RPM without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: FLeftRPM		speed of front left motor in revolution per minute
 * 						  FRightRPM		speed of front right motor in revolution per minute
 * 						  BLeftRPM 		speed of back left motor in revolution per minute
 * 						  BRightRPM		speed of back right motor in revolution per minute
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCRPM(2000, 2000, 2000, 2000, &csev);
 */
void VESCRPM(float RPM, VESC_t* vesc) {

	if(RPM > vesc->max_rpm){
		RPM = vesc->max_rpm;
	}

	RPM = RPM * vesc->pole_pairs;

	comm_can_set_rpm(vesc->Instance, RPM);

	vesc->rpm_flag = 1;
}

/*
 * Function Name		: VESCPDC
 * Function Description : Command the VESC to move with specified duty cycle without any position control.
 * Function Remarks		: The range of duty cycle : 0.0-0.9
 * Function Arguments	: FLeftPDC		duty cycle of front left motor
 * 						  FRightPDC		duty cycle of front right motor
 * 						  BLeftPDC 		duty cycle of back left motor
 * 						  BRightPDC		duty cycle of back right motor
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCPDC(0.5, 0.5, 0.5, 0.5);
 */
void VESCPDC(float Duty, VESC_t* vesc) {

	if(Duty > 0.9){
		Duty = 0.9;
	}

	comm_can_set_duty(vesc->Instance, Duty);

	vesc->pdc_flag = 1;
}

/*
 * Function Name		: VESCStop
 * Function Description : Command the VESC to stop
 * Function Remarks		: NONE
 * Function Arguments	: vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCStop(&csev);
 */
void VESCStop(VESC_t* vesc) {

	if(vesc->pdc_flag) {
		comm_can_set_duty(vesc->Instance, 0.0);
	}

	if(vesc->rpm_flag) {
		comm_can_set_rpm(vesc->Instance, 0.0);
	}

	if(vesc->current_flag){
		comm_can_set_current(vesc->Instance, 0.0);
	}

	vesc->flags = 0;
}

/*
 * Function Name		: VESCEnquire
 * Function Description : Enquire the VESC board and the motor parameters
 * Function Remarks		: NONE
 * Function Arguments	: controller_id		CAN id of the target VESC board
 * 						  parameter			Enumeration to the enquired parameters
 * 						  print				True to print the result
 * Function Return		: None
 * Function Example		: VESCEnquire(VESC1, AVG_ID, true);
 */
mc_values VESCEnquire(VESC_t* vesc, COMM_GET_VALUE_SELECTIVE_t parameter) {
//	rx_value_complete = false;
	rx_value_selective_print = false;
	if(parameter < ALL)
		bldc_interface_get_values_selective(vesc->Instance, parameter);
	else
		bldc_interface_get_values(vesc->Instance);
//	while(!rx_value_complete);
//	rx_value_complete = false;
	return rx_value_buf;
}

/*	Private Function	*/
void bldc_send_packet(uint8_t controller_id, uint8_t *data, unsigned int len) {
	comm_can_send_buffer(controller_id, data, len, 0);
}

float VESC_calVel(float rpm)
{
	float vel;
//	rpm /= (MCCONF_SI_MOTOR_POLES / 2);
	vel = (((rpm / (14 / 2.0)) * 1) * (M_PI * 0.127)) / 60;
	return vel;
}

