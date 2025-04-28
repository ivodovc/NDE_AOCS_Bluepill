/*
 * command_processor.h
 *
 *  Created on: May 5, 2023
 *      Author: Ivo Dovicak
 */

#ifndef APPLICATION_USER_CORE_COMMAND_PROCESSOR_H_
#define APPLICATION_USER_CORE_COMMAND_PROCESSOR_H_

typedef enum{AMS_NONE, AMS_SWEEP, AMS_SWEEP_CONT, AMS_REGISTER, AMS_SWEEP_NDIV,
			AMS_LOWPOWER, AMS_WAKEUP, AMS_GETPOWERSTATUS,
			AMS_VERSION, AMS_SINGLE, AMS_HOWAREYOU,
			AMS_STOP, AMS_CHECK}
command_t;

#define MAX_ARG_LEN 10

void process_command_string(char* string, command_t* global_command, uint32_t* global_args);


#endif /* APPLICATION_USER_CORE_COMMAND_PROCESSOR_H_ */
