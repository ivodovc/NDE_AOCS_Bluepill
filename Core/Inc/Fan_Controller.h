
#ifndef INC_FAN_CONTROLLER_H_
#define INC_FAN_CONTROLLER_H_

#include <stdint.h>

void init_fan(uint32_t* motor_PWM_INA, uint32_t* motor_PWM_INB, uint32_t* motor_PWM_INC, uint32_t* motor_PWM_IND);
void set_fan_speed(int16_t speed);
void set_fan_speed2(int16_t speed);

#endif /* INC_FAN_CONTROLLER_H_ */
