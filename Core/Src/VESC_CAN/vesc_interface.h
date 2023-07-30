#ifndef VESC_CAN_VESC_INTERFACE_H_
#define VESC_CAN_VESC_INTERFACE_H_

#include "vesc_can.h"
#include "../adapter.h"


typedef struct{
	float max_rpm;
	float pole_pairs;
	float wheel_diameter;
	uint16_t Instance;
	INFO Info;
	float vel;

	union{
		uint8_t flags;
		struct{
			unsigned current_flag	: 1;
			unsigned rpm_flag		: 1;
			unsigned pdc_flag		: 1;
			unsigned flag4			: 1;
			unsigned flag5			: 1;
			unsigned flag6			: 1;
			unsigned flag7			: 1;
			unsigned flag8			: 1;
		};

	};

}VESC_t;

void VESCInit(float max_rpm, float pole_pairs, float wheel_diameter,uint16_t Instance, VESC_t* vesc);
void VESCVelocity(float Velocity, VESC_t* vesc);
void VESCRPM(float RPM, VESC_t* vesc);
void VESCPDC(float Duty, VESC_t* vesc);
void VESCStop(VESC_t* vesc);
float VESC_calVel(float rpm);

VESC_t vesc1, vesc2;

mc_values VESCEnquire(VESC_t* vesc, COMM_GET_VALUE_SELECTIVE_t parameter);


#endif /* VESC_CAN_VESC_INTERFACE_H_ */
