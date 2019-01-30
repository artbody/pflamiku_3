/**
 * @file MainFsm.c
 *
 * @author FW Profile code generator version 5.22
 * @date Created on: Jan 30 2019 11:4:52
 */

/** MainFsm function definitions */
#include "MainFsm.h"

/** FW Profile function definitions */
#include "FwSmSCreate.h"
#include "FwSmConfig.h"

#include <stdlib.h>

/** Entry Action for the state S_setRGB. */
void f_setRGB(FwSmDesc_t smDesc)
{
	LED_RGB_Set(1.0);
	piface->run_mode = 3;
}

/** Entry Action for the state S_saveEeprom_pumpOff. */
void f_pump_off(FwSmDesc_t smDesc)
{
	pumpOFF();
	piface->mittelwert=piface->hx;
	piface->bPumpOn=0;
}

/** Entry Action for the state S_pumpOn. */
void f_pumpOn(FwSmDesc_t smDesc)
{
	piface->startTimWater=HAL_GetTick();
	pumpON();
	piface->bPumpOn=1;
}

/** Entry Action for the state S_measureHX. */
void f_readHX712(FwSmDesc_t smDesc)
{
	//measure the weight
	HX712_run();
	piface->timeOut=HAL_GetTick();
	// set the color of the RGBLED according to measurement value
	piface->mittelwert=piface->hx;
	LED_RGB_Set(1.0);
}

/** Entry Action for the state S_waterLow. */
void f_water_low(FwSmDesc_t smDesc)
{
	pumpOFF();
	Error_Handler();
}

/** Action on the transition from Initial State to C_mwKleinerMin. */
void A_readHX712(FwSmDesc_t smDesc)
{
	HX712_run();
}

/** Guard on the transition from C_mwKleinerMin to S_pumpOn. */
FwSmBool_t G_mwKleinerMin(FwSmDesc_t smDesc)
{
	if((piface->mittelwert <= piface->eepromMin) && (piface->bPumpOn==0)){
	return 1;
	}
	return 0;
}

/** Guard on the transition from C_mwKleinerMin to S_measureHX. */
FwSmBool_t G_mwKlMaxUpumpOn(FwSmDesc_t smDesc)
{
	if((piface->mittelwert <= piface->eepromMax) && (piface->bPumpOn==1)){
	return 1;
	}
	return 0;
}

/** Guard on the transition from C_timeout_exit to S_setRGB. */
FwSmBool_t G_switch_on(FwSmDesc_t smDesc)
{
	if(piface->sw1_value>0){
	return 1;
	}
	return 0;
}

/** Guard on the transition from C_timeout_exit to S_waterLow. */
FwSmBool_t G_pumpTimGrTmax(FwSmDesc_t smDesc)
{
	//test if pump has run to long
	if (piface->timeOut - piface->startTimWater > piface->eepromPmax) {
	//piface->startTimWater=HAL_GetTick();
		return 1;
	}
		return 0;
}

/** Guard on the transition from C_timeout_exit to S_saveEeprom_pumpOff. */
FwSmBool_t G_mwGrMax(FwSmDesc_t smDesc)
{
	if((piface->hx > piface->eepromMax) && (piface->bPumpOn==1)){
	return 1;
	}
	return 0;
}

/** Entry Action for the state ES_waterProg_enter. */
void f_es_menuFsm_waterProg_enter(FwSmDesc_t smDesc)
{
	HX712_run();
	
	piface->eepromMin=piface->hx;
}

/** Entry Action for the state ES_waterProg_run. */
void f_waterProg_run(FwSmDesc_t smDesc)
{
	
	HX712_run();
	pumpON();// start pump
	piface->bPumpOn=1;
	//start timer
	piface->timeOutWater=HAL_GetTick();
}

/** Entry Action for the state ES_waterProg_exit. */
void f_waterProg_exit(FwSmDesc_t smDesc)
{
	piface->run_mode = 3;
}

/** Entry Action for the state ES_waterProg_runing. */
void f_getSwitch(FwSmDesc_t smDesc)
{
	piface->sw1_value=shift_test_switch_is_ON();
}

/** Entry Action for the state ES_waterProg_stopping. */
void f_waterProg_stop(FwSmDesc_t smDesc)
{
	pumpOFF();
		piface->bPumpOn=0;
		piface->eepromPmax=HAL_GetTick()-piface->timeOutWater;
		
		HX712_run();
		//this is set only after //watering a plant
		piface->eepromMax=piface->hx;
		if(piface->eepromMax>(piface->eepromMin+3000)){
		//Save_2_Eeprom();
		}else{
			Error_Handler();
		};
}

/** Action on the transition from Initial State to ES_waterProg_enter. */
void f_waterProg_preenter(FwSmDesc_t smDesc)
{
	piface->prog_mode = 0;
}

/** Guard on the transition from C_waterProg_run to ES_waterProg_run. */
FwSmBool_t G_waterProg_run(FwSmDesc_t smDesc)
{
	if(piface->sw1_value==1){
	
	return 1;
	
	}
	return 0;
}

/** Guard on the transition from C_waterProg_run to ES_waterProg_exit. */
FwSmBool_t G_waterProg_exit(FwSmDesc_t smDesc)
{
	if(piface->sw1_value==0){
	
	return 1;
	
	}
	return 0;
}

/** Guard on the transition from ES_waterProg_runing to ES_waterProg_stopping. */
FwSmBool_t G_switchOff(FwSmDesc_t smDesc)
{
	if(piface->sw1_value==0){
	
	return 1;
	
	}
	return 0;
}

/** Entry Action for the state S_START. */
void f_getTime(FwSmDesc_t smDesc)
{
	//piface->time_value=HAL_GetTick();
	
	piface->timeOut=HAL_GetTick();
	
	//TODO for debug only
	//piface->sw1_value=1;
	
	piface->sw1_value=shift_test_switch_is_ON();
	piface->started =1;
	reducePwr();
}

/** Entry Action for the state S_progLdrSwitchValue. */
void f_setNewLdrSwitchValue(FwSmDesc_t smDesc)
{
	progLdrSwitchValue();
}

/** Entry Action for the state S_getLdrValue. */
void f_getLdrValue(FwSmDesc_t smDesc)
{
	LDR_Value();
	if(piface->ldr_value>piface->ldr_switch){
	piface->sleepMode=1;
	}else{
	piface->sleepMode=0;
	};
}

/** Entry Action for the state S_sleepMode. */
void f_sleepMode(FwSmDesc_t smDesc)
{
	sleepMode();
}

/** Action on the transition from Initial State to C_prog_ldr_switch. */
void A_INIT(FwSmDesc_t smDesc)
{
	piface->run_mode = 1;
}

/** Guard on the transition from C_HX_prog_or_run to S_progHX712. */
FwSmBool_t G_progHX712Mode(FwSmDesc_t smDesc)
{
	/*do only enter this State when sw1 is on */
	if(piface->sw1_value > 0){
	fullPwr();
	return 1;
	}
	return 0;
}

/** Guard on the transition from C_HX_prog_or_run to S_runHX712. */
FwSmBool_t G_runHX712_atTime(FwSmDesc_t smDesc)
{
	/*do only enter this State when time has passed */
	
	
	if (piface->timeOut - piface->startTimHx > piface->timeHx) {
		piface->run_mode = 2;
	fullPwr();
	piface->startTimHx=HAL_GetTick();
		return 1;
	}
		return 0;
}

/** Guard on the transition from C_HX_prog_or_run to S_getLdrValue. */
FwSmBool_t G_runLDR_atTime(FwSmDesc_t smDesc)
{
	/*do only enter this State when time has passed */
	/*do only enter this State when time has passed */
	
	
	if (piface->timeOut - piface->startTimLDR > piface->timeLDR) {
		fullPwr();	piface->startTimLDR=HAL_GetTick();
		return 1;
	}
		return 0;
}

/** Guard on the transition from C_prog_ldr_switch to S_progLdrSwitchValue. */
FwSmBool_t G_run_ldr_switch_value_update(FwSmDesc_t smDesc)
{
	/*do only enter this State when sw1 is on */
	if(piface->sw1_value > 0){
	return 1;
	}
	return 0;
}

/** Guard on the transition from C_prog_ldr_switch to S_getLdrValue. */
FwSmBool_t G_enter_all(FwSmDesc_t smDesc)
{
	/*do not enter the FSM when sw1 is on */
	if(piface->sw1_value == 0){
	return 1;
	}
	return 0;
}

/** Guard on the transition from C_sleepmode to S_sleepMode. */
FwSmBool_t G_sleepMode(FwSmDesc_t smDesc)
{
	/*do only enter this State when sleepMode is set by f_getLdrValue on */
	if(piface->sleepMode == 1){
	return 1;
	}
	return 0;
}

/* ----------------------------------------------------------------------------------------------------------------- */
FwSmDesc_t MainFsmCreate(void* smData)
{
	const FwSmCounterU2_t N_OUT_OF_S_setRGB = 1;	/* The number of transitions out of state S_setRGB */
	const FwSmCounterU2_t C_mwKleinerMin = 1;		/* The identifier of choice pseudo-state C_mwKleinerMin in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_C_mwKleinerMin = 3;	/* The number of transitions out of the choice-pseudo state C_mwKleinerMin */
	const FwSmCounterU2_t N_OUT_OF_S_saveEeprom_pumpOff = 1;	/* The number of transitions out of state S_saveEeprom_pumpOff */
	const FwSmCounterU2_t N_OUT_OF_S_pumpOn = 1;	/* The number of transitions out of state S_pumpOn */
	const FwSmCounterU2_t N_OUT_OF_S_measureHX = 1;	/* The number of transitions out of state S_measureHX */
	const FwSmCounterU2_t C_timeout_exit = 2;		/* The identifier of choice pseudo-state C_timeout_exit in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_C_timeout_exit = 4;	/* The number of transitions out of the choice-pseudo state C_timeout_exit */
	const FwSmCounterU2_t N_OUT_OF_S_waterLow = 1;	/* The number of transitions out of state S_waterLow */

	/** Create state machine EsmDescId2940, which is embedded in S_runHX712 */
	FW_SM_INST(EsmDescId2940,
		5,	/* NSTATES - The number of states */
		2,	/* NCPS - The number of choice pseudo-states */
		13,	/* NTRANS - The number of transitions */
		6,	/* NACTIONS - The number of state and transition actions */
		5	/* NGUARDS - The number of transition guards */
	);
	FwSmInit(&EsmDescId2940);

	/** Configure the state machine EsmDescId2940, which is embedded in S_runHX712 */
	FwSmSetData(&EsmDescId2940, smData);
	FwSmAddState(&EsmDescId2940, MainFsm_S_setRGB, N_OUT_OF_S_setRGB, &f_setRGB, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&EsmDescId2940, C_mwKleinerMin, N_OUT_OF_C_mwKleinerMin);
	FwSmAddState(&EsmDescId2940, MainFsm_S_saveEeprom_pumpOff, N_OUT_OF_S_saveEeprom_pumpOff, &f_pump_off, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2940, MainFsm_S_pumpOn, N_OUT_OF_S_pumpOn, &f_pumpOn, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2940, MainFsm_S_measureHX, N_OUT_OF_S_measureHX, &f_readHX712, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&EsmDescId2940, C_timeout_exit, N_OUT_OF_C_timeout_exit);
	FwSmAddState(&EsmDescId2940, MainFsm_S_waterLow, N_OUT_OF_S_waterLow, &f_water_low, NULL, NULL, NULL);
	FwSmAddTransIpsToCps(&EsmDescId2940, C_mwKleinerMin, &A_readHX712);
	FwSmAddTransStaToFps(&EsmDescId2940, TCLK, MainFsm_S_setRGB, NULL, NULL);
	FwSmAddTransCpsToSta(&EsmDescId2940, C_mwKleinerMin, MainFsm_S_pumpOn, NULL, &G_mwKleinerMin);
	FwSmAddTransCpsToSta(&EsmDescId2940, C_mwKleinerMin, MainFsm_S_measureHX, NULL, &G_mwKlMaxUpumpOn);
	FwSmAddTransCpsToSta(&EsmDescId2940, C_mwKleinerMin, MainFsm_S_setRGB, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2940, TCLK, MainFsm_S_saveEeprom_pumpOff, MainFsm_S_setRGB, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2940, TCLK, MainFsm_S_pumpOn, MainFsm_S_measureHX, NULL, NULL);
	FwSmAddTransStaToCps(&EsmDescId2940, TCLK, MainFsm_S_measureHX, C_timeout_exit, NULL, NULL);
	FwSmAddTransCpsToSta(&EsmDescId2940, C_timeout_exit, MainFsm_S_setRGB, NULL, &G_switch_on);
	FwSmAddTransCpsToSta(&EsmDescId2940, C_timeout_exit, MainFsm_S_waterLow, NULL, &G_pumpTimGrTmax);
	FwSmAddTransCpsToSta(&EsmDescId2940, C_timeout_exit, MainFsm_S_saveEeprom_pumpOff, NULL, &G_mwGrMax);
	FwSmAddTransCpsToSta(&EsmDescId2940, C_timeout_exit, MainFsm_S_measureHX, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2940, TCLK, MainFsm_S_waterLow, MainFsm_S_waterLow, NULL, NULL);

	const FwSmCounterU2_t N_OUT_OF_ES_waterProg_enter = 1;	/* The number of transitions out of state ES_waterProg_enter */
	const FwSmCounterU2_t N_OUT_OF_ES_waterProg_run = 1;	/* The number of transitions out of state ES_waterProg_run */
	const FwSmCounterU2_t C_waterProg_run = 1;		/* The identifier of choice pseudo-state C_waterProg_run in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_C_waterProg_run = 3;	/* The number of transitions out of the choice-pseudo state C_waterProg_run */
	const FwSmCounterU2_t N_OUT_OF_ES_waterProg_exit = 1;	/* The number of transitions out of state ES_waterProg_exit */
	const FwSmCounterU2_t N_OUT_OF_ES_waterProg_runing = 2;	/* The number of transitions out of state ES_waterProg_runing */
	const FwSmCounterU2_t N_OUT_OF_ES_waterProg_stopping = 1;	/* The number of transitions out of state ES_waterProg_stopping */

	/** Create state machine EsmDescId2939, which is embedded in S_progHX712 */
	FW_SM_INST(EsmDescId2939,
		5,	/* NSTATES - The number of states */
		1,	/* NCPS - The number of choice pseudo-states */
		10,	/* NTRANS - The number of transitions */
		6,	/* NACTIONS - The number of state and transition actions */
		3	/* NGUARDS - The number of transition guards */
	);
	FwSmInit(&EsmDescId2939);

	/** Configure the state machine EsmDescId2939, which is embedded in S_progHX712 */
	FwSmSetData(&EsmDescId2939, smData);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_enter, N_OUT_OF_ES_waterProg_enter, &f_es_menuFsm_waterProg_enter, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_run, N_OUT_OF_ES_waterProg_run, &f_waterProg_run, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&EsmDescId2939, C_waterProg_run, N_OUT_OF_C_waterProg_run);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_exit, N_OUT_OF_ES_waterProg_exit, &f_waterProg_exit, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_runing, N_OUT_OF_ES_waterProg_runing, &f_getSwitch, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_stopping, N_OUT_OF_ES_waterProg_stopping, &f_waterProg_stop, NULL, NULL, NULL);
	FwSmAddTransIpsToSta(&EsmDescId2939, MainFsm_ES_waterProg_enter, &f_waterProg_preenter);
	FwSmAddTransStaToCps(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_enter, C_waterProg_run, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_run, MainFsm_ES_waterProg_runing, NULL, NULL);
	FwSmAddTransCpsToSta(&EsmDescId2939, C_waterProg_run, MainFsm_ES_waterProg_run, NULL, &G_waterProg_run);
	FwSmAddTransCpsToSta(&EsmDescId2939, C_waterProg_run, MainFsm_ES_waterProg_exit, NULL, &G_waterProg_exit);
	FwSmAddTransCpsToSta(&EsmDescId2939, C_waterProg_run, MainFsm_ES_waterProg_enter, NULL, NULL);
	FwSmAddTransStaToFps(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_exit, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_runing, MainFsm_ES_waterProg_stopping, NULL, &G_switchOff);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_runing, MainFsm_ES_waterProg_runing, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_stopping, MainFsm_ES_waterProg_exit, NULL, NULL);

	const FwSmCounterU2_t N_OUT_OF_S_START = 1;	/* The number of transitions out of state S_START */
	const FwSmCounterU2_t C_HX_prog_or_run = 1;		/* The identifier of choice pseudo-state C_HX_prog_or_run in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_C_HX_prog_or_run = 4;	/* The number of transitions out of the choice-pseudo state C_HX_prog_or_run */
	const FwSmCounterU2_t N_OUT_OF_S_runHX712 = 1;	/* The number of transitions out of state S_runHX712 */
	const FwSmCounterU2_t N_OUT_OF_S_progHX712 = 1;	/* The number of transitions out of state S_progHX712 */
	const FwSmCounterU2_t C_prog_ldr_switch = 2;		/* The identifier of choice pseudo-state C_prog_ldr_switch in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_C_prog_ldr_switch = 2;	/* The number of transitions out of the choice-pseudo state C_prog_ldr_switch */
	const FwSmCounterU2_t N_OUT_OF_S_progLdrSwitchValue = 1;	/* The number of transitions out of state S_progLdrSwitchValue */
	const FwSmCounterU2_t N_OUT_OF_S_getLdrValue = 1;	/* The number of transitions out of state S_getLdrValue */
	const FwSmCounterU2_t C_sleepmode = 3;		/* The identifier of choice pseudo-state C_sleepmode in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_C_sleepmode = 2;	/* The number of transitions out of the choice-pseudo state C_sleepmode */
	const FwSmCounterU2_t N_OUT_OF_S_sleepMode = 0;	/* The number of transitions out of state S_sleepMode */

	/** Create state machine smDesc */
	FW_SM_INST(smDesc,
		6,	/* NSTATES - The number of states */
		3,	/* NCPS - The number of choice pseudo-states */
		14,	/* NTRANS - The number of transitions */
		5,	/* NACTIONS - The number of state and transition actions */
		6	/* NGUARDS - The number of transition guards */
	);
	FwSmInit(&smDesc);

	/** Configure the state machine smDesc */
	FwSmSetData(&smDesc, smData);
	FwSmAddState(&smDesc, MainFsm_S_START, N_OUT_OF_S_START, &f_getTime, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&smDesc, C_HX_prog_or_run, N_OUT_OF_C_HX_prog_or_run);
	FwSmAddState(&smDesc, MainFsm_S_runHX712, N_OUT_OF_S_runHX712, NULL, NULL, NULL, &EsmDescId2940);
	FwSmAddState(&smDesc, MainFsm_S_progHX712, N_OUT_OF_S_progHX712, NULL, NULL, NULL, &EsmDescId2939);
	FwSmAddChoicePseudoState(&smDesc, C_prog_ldr_switch, N_OUT_OF_C_prog_ldr_switch);
	FwSmAddState(&smDesc, MainFsm_S_progLdrSwitchValue, N_OUT_OF_S_progLdrSwitchValue, &f_setNewLdrSwitchValue, NULL, NULL, NULL);
	FwSmAddState(&smDesc, MainFsm_S_getLdrValue, N_OUT_OF_S_getLdrValue, &f_getLdrValue, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&smDesc, C_sleepmode, N_OUT_OF_C_sleepmode);
	FwSmAddState(&smDesc, MainFsm_S_sleepMode, N_OUT_OF_S_sleepMode, &f_sleepMode, NULL, NULL, NULL);
	FwSmAddTransIpsToCps(&smDesc, C_prog_ldr_switch, &A_INIT);
	FwSmAddTransStaToCps(&smDesc, TCLK, MainFsm_S_START, C_HX_prog_or_run, NULL, NULL);
	FwSmAddTransCpsToSta(&smDesc, C_HX_prog_or_run, MainFsm_S_progHX712, NULL, &G_progHX712Mode);
	FwSmAddTransCpsToSta(&smDesc, C_HX_prog_or_run, MainFsm_S_runHX712, NULL, &G_runHX712_atTime);
	FwSmAddTransCpsToSta(&smDesc, C_HX_prog_or_run, MainFsm_S_getLdrValue, NULL, &G_runLDR_atTime);
	FwSmAddTransCpsToSta(&smDesc, C_HX_prog_or_run, MainFsm_S_START, NULL, NULL);
	FwSmAddTransStaToSta(&smDesc, TCLAK, MainFsm_S_runHX712, MainFsm_S_START, NULL, NULL);
	FwSmAddTransStaToSta(&smDesc, TCLAK, MainFsm_S_progHX712, MainFsm_S_START, NULL, NULL);
	FwSmAddTransCpsToSta(&smDesc, C_prog_ldr_switch, MainFsm_S_progLdrSwitchValue, NULL, &G_run_ldr_switch_value_update);
	FwSmAddTransCpsToSta(&smDesc, C_prog_ldr_switch, MainFsm_S_getLdrValue, NULL, &G_enter_all);
	FwSmAddTransStaToSta(&smDesc, TCLK, MainFsm_S_progLdrSwitchValue, MainFsm_S_getLdrValue, NULL, NULL);
	FwSmAddTransStaToCps(&smDesc, TCLK, MainFsm_S_getLdrValue, C_sleepmode, NULL, NULL);
	FwSmAddTransCpsToSta(&smDesc, C_sleepmode, MainFsm_S_sleepMode, NULL, &G_sleepMode);
	FwSmAddTransCpsToSta(&smDesc, C_sleepmode, MainFsm_S_START, NULL, NULL);

	return &smDesc;
}