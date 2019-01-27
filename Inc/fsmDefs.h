/*
 * fsmDefs.h
 *
 *  Created on: 16.01.2019
 *      Author: achim
 */

#ifndef FSMDEFS_H_
#define FSMDEFS_H_
#include "FwSmConstants.h"

typedef struct {
	FwSmBool_t switcher_raised;
	uint32_t switcher_value;
	uint32_t started;
	uint32_t prog_mode;
	uint32_t bPumpOff;
	uint32_t bPumpEn;
	//uint32_t varWaterLevel;
	//uint32_t waterLevelMin;
	//uint32_t waterLevelHas2Run;//this is set only after watering a plant
	uint32_t lowPower;
	uint32_t sw1;
	uint32_t sw1_value;
	uint32_t eepromAbsMax;
	uint32_t eepromMin;
	uint32_t eepromMax;
	uint32_t eepromPmax;
	uint32_t mittelwert;
	uint32_t hx;
	uint32_t timeOut;
	uint32_t startTimWater;
	uint32_t timeOutWater;
	uint32_t startTimLED;
	uint32_t timeLED;
	uint32_t startTimLDR;
	uint32_t timeLDR;
	uint32_t startTimHx;
	uint32_t timeHx;
	uint32_t time_value;
	uint32_t timeNextRun_value;
	uint32_t ldr;
	uint32_t ldr_switch;
	uint32_t sleepMode;
	uint32_t run_mode;//indicate the End of a substate
	//                           restarts the statemachine

	uint32_t menuFsmExitState;
	char tmp[32];
} FsmIFace ;
FsmIFace iface;

//typedef struct {
//	FsmIFace iface;
//} Fsm;
//Fsm handle;

#endif /* FSMDEFS_H_ */
