#include "Fan_Controller.h"

 uint32_t *INA;
 uint32_t *INB;
 uint32_t *INC;
 uint32_t *IND;


void init_fan(uint32_t* motor_PWM_INA, uint32_t* motor_PWM_INB, uint32_t* motor_PWM_INC, uint32_t* motor_PWM_IND)
{
	INA = motor_PWM_INA;
	INB = motor_PWM_INB;
	INC = motor_PWM_INC;
	IND = motor_PWM_IND;
	*INA = 0;
	*INB = 0;
	*INC = 0;
	*IND = 0;
}

// speed has to be integer from -1000 to 1000
void set_fan_speed(int16_t speed){
	if (0 <= speed && speed <= 1000){
		*INB = 0;
		*INA = speed;
	}else if (-1000 <= speed && speed < 0){
		*INA = 0;
		*INB = -1*speed;
	}
}

void set_fan_speed2(int16_t speed2){
	if (0 <= speed2 && speed2 <= 1000){
		*IND = 0;
		*INC = speed2;
	}else if (-1000 <= speed2 && speed2 < 0){
		*INC = 0;
		*IND = -1*speed2;
	}
}



