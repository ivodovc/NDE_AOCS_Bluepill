#ifndef FAN_LOGIC_H
#define FAN_LOGIC_H

#include <stdint.h>
extern uint16_t pwmValue;
extern uint16_t pwmValue1;
extern uint16_t pwmValue2;
extern uint16_t pwmValue3;
void FanLogic_Update(uint16_t rawValues[4], int16_t *fan_speed1, int16_t *fan_speed2);
void light_tracking_logic(uint16_t rawValues[4], int16_t *fan_speed1, int16_t *fan_speed2);

#endif
