#include "fan_logic.h"
#include <string.h>
uint16_t pwmValue = 0;
uint16_t pwmValue1 = 0;
uint16_t pwmValue2 = 0;
uint16_t pwmValue3 = 0;
void FanLogic_Update(uint16_t rawValue[4], int16_t *fan_speed1, int16_t *fan_speed2) {

	uint16_t caseValue = 3;

	  if(rawValue[0]>rawValue[1]&&rawValue[0]>rawValue[2]&&rawValue[0]>rawValue[3])
	  		{
	  			caseValue = 3;
	  		}
	      if(rawValue[1]>rawValue[0]&&rawValue[1]>rawValue[2]&&rawValue[1]>rawValue[3])
	          {
	          	caseValue = 2;
	          }
	      if(rawValue[2]>rawValue[0]&&rawValue[2]>rawValue[3]&&rawValue[2]>rawValue[1])
	  		{
	  			caseValue = 2;
	  		}
	      if(rawValue[3]>rawValue[0]&&rawValue[3]>rawValue[1]&&rawValue[3]>rawValue[2])
	  		{
	  			caseValue = 1;
	  		}

	      switch (caseValue) {
	          case 1:
	              *fan_speed1 = 1000;
	              *fan_speed2 = 1000;
	              break;
	          case 2:
	              *fan_speed1 = -1000;
	              *fan_speed2 = -1000;
	              break;
	          case 3:
	          default:
	              *fan_speed1 = 1;
	              *fan_speed2 = 1;
	              break;
	      }
}

int findMaxIndex(uint16_t arr[], uint16_t size) {
	uint16_t max = arr[0];
	uint16_t max_i = 0;
    for (int i = 1; i < size; i++) {
        if (arr[i] > max) {
            max = arr[i];
            max_i = i;
        }
    }
    return max_i;
}

int findMinIndex(uint16_t arr[], uint16_t size) {
	uint16_t min = arr[0];
	uint16_t min_i = 0;
    for (int i = 1; i < size; i++) {
        if (arr[i] < min) {
            min = arr[i];
            min_i = i;
        }
    }
    return min_i;
}

void light_tracking_logic(uint16_t rawValues[4], float *required_rotation){
	uint16_t max_index = findMaxIndex(rawValues, 4);
	uint16_t min_index = findMinIndex(rawValues, 4);
	const uint16_t THRESHOLD_VALUE = 400;
	if (rawValues[max_index] - rawValues[min_index] < THRESHOLD_VALUE){
		// if max is not better than min then do nothing
		//return;
	}

	switch (max_index)
	{
		case 0:
			*required_rotation = -60;
			break; // do nothing, already pointed at maximum

		case 1:
			*required_rotation = 0; // deg/s
			break; // do nothing, already pointed at maximum

		case 2:
			*required_rotation = 60;
			break; // do nothing, already pointed at maximum

		case 3:
			*required_rotation = 100;
			break; // do nothing, already pointed at maximum

	}

}
