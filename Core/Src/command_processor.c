/*
 * command_processor.c
 *
 *  Created on: May 5, 2023
 *      Author: Ivo Dovicak
 */
#include <string.h>
#include "main.h"
#include "command_processor.h"
#include <stdlib.h>


void process_arguments_int(char* args, uint32_t* global_args){
	uint32_t size = strlen(args);
	uint32_t global_i = 0;
	// '()' is minimum string, if not return
	if (size<2){
		return;
	}
	// if first char is not (, something is wrong
	if (args[0] != '('){
		return;
	}
	// last separation symbol
	uint32_t last_symbol_i = 0;
	// main loop, go char by char and add to args
	for (uint32_t i=1; i<size; i++){
		// if closing parenthessis ) is found, return
		uint8_t c = args[i];

		// parse int
		if (c == ',' || c == ')'){
			// if not long enough string is found, e.g. (,,)
			if (i-last_symbol_i-1 <= 0){
				last_symbol_i = i;
				continue;
			}
			char arg_as_str[i - last_symbol_i];
			// add delimiting symbol
			arg_as_str[i - last_symbol_i-1] = '\0';
			memcpy(arg_as_str, args+last_symbol_i+1, i - last_symbol_i-1);
			last_symbol_i = i;
			// add to global_args
			uint32_t parsed_int = atoi(arg_as_str);
			global_args[global_i] = parsed_int;
			global_i++;
			if (global_i > MAX_ARG_LEN){
				return;
			}
		}
		if (c == ')'){
					return;
		}
	}
}

void process_arguments_hex(char* args, uint32_t* global_args){
	uint32_t size = strlen(args);
	uint32_t global_i = 0;
	// '()' is minimum string, if not return
	if (size<2){
		return;
	}
	// if first char is not (, something is wrong
	if (args[0] != '('){
		return;
	}
	// last separation symbol
	uint32_t last_symbol_i = 0;
	// main loop, go char by char and add to args
	for (uint32_t i=1; i<size; i++){
		// if closing parenthessis ) is found, return
		uint8_t c = args[i];

		// parse int
		if (c == ',' || c == ')'){
			// if not long enough string is found, e.g. (,,)
			if (i-last_symbol_i-1 <= 0){
				last_symbol_i = i;
				continue;
			}
			char arg_as_str[i - last_symbol_i];
			// add delimiting symbol
			arg_as_str[i - last_symbol_i-1] = '\0';
			memcpy(arg_as_str, args+last_symbol_i+1, i - last_symbol_i-1);
			last_symbol_i = i;
			// add to global_args
			// convert from HEX string
			// ueed to use strtoul instead of strtol, as we are converting to an unsigned int
			uint32_t parsed_int = strtoul(arg_as_str, NULL, 16);
			global_args[global_i] = parsed_int;
			global_i++;
			if (global_i > MAX_ARG_LEN){
				return;
			}
		}
		if (c == ')'){
					return;
		}
	}
}

// Process string commands
void process_command_string(char* string, command_t* global_command, uint32_t* global_args){
	char AMS_substr[4];
	AMS_substr[3] = '\0';
	memcpy(AMS_substr, &string[0], 3);
	// If command doesnt begin with AMS set global_command to NONE and return
	if (strcmp(AMS_substr, "AMS") != 0){
		*global_command = AMS_NONE;
		return;
	}
	// find string between _ and (
	uint8_t start_index = strchr(string, '_') - string;
	uint8_t end_index = strchr(string, '(') - string;
	char command[end_index - start_index];
	command[end_index - start_index - 1] = '\0';
	memcpy(command, &string[start_index+1], end_index-start_index-1);
	// set command based on string

	if (strcmp(command, "VERSION") == 0){
		*global_command = AMS_VERSION;
	}else if (strcmp(command, "SWEEP") == 0){
		*global_command = AMS_SWEEP;
	}else if (strcmp(command, "SWEEP_NDIV") == 0){
		*global_command = AMS_SWEEP_NDIV;
	}else if (strcmp(command, "SWEEP_CONT") == 0){
		*global_command = AMS_SWEEP_CONT;
	}else if (strcmp(command, "REGISTER")== 0){
		*global_command = AMS_REGISTER;
	}else if (strcmp(command, "SINGLE")== 0){
		*global_command = AMS_SINGLE;
	}else if (strcmp(command, "HOWAREYOU")== 0){
		*global_command = AMS_HOWAREYOU;
	}else if (strcmp(command, "STOP")== 0){
		*global_command = AMS_STOP;
	}else if (strcmp(command, "CHECK")== 0){
		*global_command = AMS_CHECK;
	}else if (strcmp(command, "LOWPOWER")== 0){
		*global_command = AMS_LOWPOWER;
	}else if (strcmp(command, "WAKEUP")== 0){
		*global_command = AMS_WAKEUP;
	}else if (strcmp(command, "GETPOWERSTATUS")== 0){
		*global_command = AMS_GETPOWERSTATUS;
	}else{
		*global_command = AMS_NONE;
	}
	// process arguments (max length is 10)
	memcpy(command, &string[start_index], end_index-start_index+1);
	uint8_t total_end_index = strchr(string, ')') - string;
	char args[total_end_index - end_index + 2];
	args[total_end_index - end_index + 1] = '\0';
	memcpy(args, &string[end_index], total_end_index-end_index+1);
	// for command AMS_REGISTER use hex version of string processor
	if (*global_command==AMS_REGISTER){
		process_arguments_hex(args, global_args);
	} else {
		process_arguments_int(args, global_args);
	}
}

