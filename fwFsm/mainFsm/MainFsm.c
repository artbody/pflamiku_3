/**
 * @file MainFsm.c
 *
 * @author FW Profile code generator version 5.22
 * @date Created on: Jan 27 2019 21:28:55
 */

/** MainFsm function definitions */
#include "MainFsm.h"

/** FW Profile function definitions */
#include "FwSmSCreate.h"
#include "FwSmConfig.h"

#include <stdlib.h>

/** Guard on the transition from CHOICE1 to S_pumpOn. */
FwSmBool_t G_mwKleinerMin(FwSmDesc_t smDesc)
{
	if((piface->mittelwert <= piface->eepromMin) && (piface->bPumpOff==0)){
	return 1;
	}
	return 0;
}

/** Guard on the transition from CHOICE2 to S_saveEeprom_pumpOff. */
FwSmBool_t G_mwGrMax(FwSmDesc_t smDesc)
{
	if((piface->mittelwert > piface->eepromMax) && (piface->bPumpOff==0)){
	return 1;
	}
	return 0;
}

/** Entry Action for the state ES_waterProg_enter. */
void f_es_menuFsm_waterProg_enter(FwSmDesc_t smDesc)
{
	piface->enc = TIM5->CNT;
}

/** Entry Action for the state ES_waterProg_run. */
void f_waterProg_run(FwSmDesc_t smDesc)
{
	
	main_HX712();
	main_pump_on();// start pump
	//start timer
	piface->timeOutWater=HAL_GetTick();
}

/** Entry Action for the state ES_waterProg_exit. */
void f_waterProg_exit(FwSmDesc_t smDesc)
{
	piface->run_mode = 0;
	
	//piface->sw1=1;
}

/** Entry Action for the state ES_waterProg_stopping. */
void f_waterProg_stop(FwSmDesc_t smDesc)
{
	main_pump_off();
	piface->eepromPmax=HAL_GetTick()-piface->timeOutWater;
	
	main_HX712();
	//this is set only after //watering a plant;
}

/** Action on the transition from Initial State to ES_waterProg_enter. */
void f_waterProg_preenter(FwSmDesc_t smDesc)
{
	TIM5->CNT = 0;
	piface->enc=0;
	piface->sw1 = 0;
}

/** Guard on the transition from C_waterProg_run to ES_waterProg_run. */
FwSmBool_t G_waterProg_run(FwSmDesc_t smDesc)
{
	if(piface->sw1==1){
	
	return 1;
	
	}
	return 0;
}

/** Guard on the transition from C_waterProg_run to ES_waterProg_exit. */
FwSmBool_t G_waterProg_exit(FwSmDesc_t smDesc)
{
	if(piface->sw1==0){
	
	return 1;
	
	}
	return 0;
}

/** Guard on the transition from ES_waterProg_runing to ES_waterProg_stopping. */
FwSmBool_t G_switchOff(FwSmDesc_t smDesc)
{
	if(piface->sw1==1){
	
	return 1;
	
	}
	return 0;
}

/** Entry Action for the state S_sleepMode. */
void f_sleepMode(FwSmDesc_t smDesc)
{
	/*do only enter this State when sw1 is on */
	if(piface->sleepMode = 1){
	return 1;
	}
	return 0;
}

/** Action on the transition from Initial State to CHOICE2. */
void A_INIT(FwSmDesc_t smDesc)
{
	piface->run_mode = 1;
}

/** Guard on the transition from CHOICE1 to S_progHX712. */
FwSmBool_t G_progHX712Mode(FwSmDesc_t smDesc)
{
	/*do only enter this State when sw1 is on */
	if(piface->sw1_value > 0){
	return 1;
	}
	return 0;
}

/** Guard on the transition from CHOICE1 to S_runHX712. */
FwSmBool_t G_runHX712_atTime(FwSmDesc_t smDesc)
{
	/*do only enter this State when time has passed */
	if(piface->time_value > piface->timeNextRun_value){
	return 1;
	}
	return 0;
}

/** Guard on the transition from CHOICE1 to S_getLdrValue. */
FwSmBool_t G_runLDR_atTime(FwSmDesc_t smDesc)
{
	/*do only enter this State when time has passed */
	if(piface->time_value > piface->timeNextRun_value){
	return 1;
	}
	return 0;
}

/** Guard on the transition from CHOICE2 to S_progLdrSwitchValue. */
FwSmBool_t G_run_ldr_switch_value_update(FwSmDesc_t smDesc)
{
	/*do only enter this State when sw1 is on */
	if(piface->sw1_value > 0){
	return 1;
	}
	return 0;
}

/** Guard on the transition from CHOICE2 to S_getLdrValue. */
FwSmBool_t G_enter_all(FwSmDesc_t smDesc)
{
	/*do not enter the FSM when sw1 is on */
	if(piface->sw1_value == 0){
	return 1;
	}
	return 0;
}

/* ----------------------------------------------------------------------------------------------------------------- */
FwSmDesc_t MainFsmCreate(void* smData)
{
	const FwSmCounterU2_t N_OUT_OF_S_setRGB = 1;	/* The number of transitions out of state S_setRGB */
	const FwSmCounterU2_t CHOICE1 = 1;		/* The identifier of choice pseudo-state CHOICE1 in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_CHOICE1 = 2;	/* The number of transitions out of the choice-pseudo state CHOICE1 */
	const FwSmCounterU2_t N_OUT_OF_S_saveEeprom_pumpOff = 1;	/* The number of transitions out of state S_saveEeprom_pumpOff */
	const FwSmCounterU2_t N_OUT_OF_S_pumpOn = 1;	/* The number of transitions out of state S_pumpOn */
	const FwSmCounterU2_t N_OUT_OF_S_measureHX = 1;	/* The number of transitions out of state S_measureHX */
	const FwSmCounterU2_t CHOICE2 = 2;		/* The identifier of choice pseudo-state CHOICE2 in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_CHOICE2 = 3;	/* The number of transitions out of the choice-pseudo state CHOICE2 */
	const FwSmCounterU2_t N_OUT_OF_S_waterLow = 1;	/* The number of transitions out of state S_waterLow */

	/** Create state machine EsmDescId2940, which is embedded in S_runHX712 */
	FW_SM_INST(EsmDescId2940,
		5,	/* NSTATES - The number of states */
		2,	/* NCPS - The number of choice pseudo-states */
		11,	/* NTRANS - The number of transitions */
		2,	/* NACTIONS - The number of state and transition actions */
		3	/* NGUARDS - The number of transition guards */
	);
	FwSmInit(&EsmDescId2940);

	/** Configure the state machine EsmDescId2940, which is embedded in S_runHX712 */
	FwSmSetData(&EsmDescId2940, smData);
	FwSmAddState(&EsmDescId2940, MainFsm_S_setRGB, N_OUT_OF_S_setRGB, NULL, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&EsmDescId2940, CHOICE1, N_OUT_OF_CHOICE1);
	FwSmAddState(&EsmDescId2940, MainFsm_S_saveEeprom_pumpOff, N_OUT_OF_S_saveEeprom_pumpOff, &f_save2Eeprom_pump_off, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2940, MainFsm_S_pumpOn, N_OUT_OF_S_pumpOn, NULL, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2940, MainFsm_S_measureHX, N_OUT_OF_S_measureHX, NULL, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&EsmDescId2940, CHOICE2, N_OUT_OF_CHOICE2);
	FwSmAddState(&EsmDescId2940, MainFsm_S_waterLow, N_OUT_OF_S_waterLow, NULL, NULL, NULL, NULL);
	FwSmAddTransIpsToCps(&EsmDescId2940, CHOICE1, &A_readHX712);
	FwSmAddTransStaToFps(&EsmDescId2940, TCLK, MainFsm_S_setRGB, NULL, NULL);
	FwSmAddTransCpsToSta(&EsmDescId2940, CHOICE1, MainFsm_S_pumpOn, NULL, &G_mwKleinerMin);
	FwSmAddTransCpsToSta(&EsmDescId2940, CHOICE1, MainFsm_S_setRGB, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2940, TCLK, MainFsm_S_saveEeprom_pumpOff, MainFsm_S_setRGB, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2940, TCLK, MainFsm_S_pumpOn, MainFsm_S_measureHX, NULL, NULL);
	FwSmAddTransStaToCps(&EsmDescId2940, TCLK, MainFsm_S_measureHX, CHOICE2, NULL, NULL);
	FwSmAddTransCpsToSta(&EsmDescId2940, CHOICE2, MainFsm_S_waterLow, NULL, &G_pumpTimGrTmax);
	FwSmAddTransCpsToSta(&EsmDescId2940, CHOICE2, MainFsm_S_saveEeprom_pumpOff, NULL, &G_mwGrMax);
	FwSmAddTransCpsToSta(&EsmDescId2940, CHOICE2, MainFsm_S_measureHX, NULL, NULL);
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
		5,	/* NACTIONS - The number of state and transition actions */
		3	/* NGUARDS - The number of transition guards */
	);
	FwSmInit(&EsmDescId2939);

	/** Configure the state machine EsmDescId2939, which is embedded in S_progHX712 */
	FwSmSetData(&EsmDescId2939, smData);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_enter, N_OUT_OF_ES_waterProg_enter, &f_es_menuFsm_waterProg_enter, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_run, N_OUT_OF_ES_waterProg_run, &f_waterProg_run, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&EsmDescId2939, C_waterProg_run, N_OUT_OF_C_waterProg_run);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_exit, N_OUT_OF_ES_waterProg_exit, &f_waterProg_exit, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_runing, N_OUT_OF_ES_waterProg_runing, NULL, NULL, NULL, NULL);
	FwSmAddState(&EsmDescId2939, MainFsm_ES_waterProg_stopping, N_OUT_OF_ES_waterProg_stopping, &f_waterProg_stop, NULL, NULL, NULL);
	FwSmAddTransIpsToSta(&EsmDescId2939, MainFsm_ES_waterProg_enter, &f_waterProg_preenter);
	FwSmAddTransStaToCps(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_enter, C_waterProg_run, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_run, MainFsm_ES_waterProg_runing, NULL, NULL);
	FwSmAddTransCpsToSta(&EsmDescId2939, C_waterProg_run, MainFsm_ES_waterProg_run, NULL, &G_waterProg_run);
	FwSmAddTransCpsToSta(&EsmDescId2939, C_waterProg_run, MainFsm_ES_waterProg_exit, NULL, &G_waterProg_exit);
	FwSmAddTransCpsToSta(&EsmDescId2939, C_waterProg_run, MainFsm_ES_waterProg_enter, NULL, NULL);
	FwSmAddTransStaToFps(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_exit, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_runing, MainFsm_ES_waterProg_runing, NULL, NULL);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_runing, MainFsm_ES_waterProg_stopping, NULL, &G_switchOff);
	FwSmAddTransStaToSta(&EsmDescId2939, TCLK, MainFsm_ES_waterProg_stopping, MainFsm_ES_waterProg_exit, NULL, NULL);

	const FwSmCounterU2_t N_OUT_OF_S_START = 1;	/* The number of transitions out of state S_START */
	const FwSmCounterU2_t CHOICE1 = 1;		/* The identifier of choice pseudo-state CHOICE1 in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_CHOICE1 = 4;	/* The number of transitions out of the choice-pseudo state CHOICE1 */
	const FwSmCounterU2_t N_OUT_OF_S_runHX712 = 1;	/* The number of transitions out of state S_runHX712 */
	const FwSmCounterU2_t N_OUT_OF_S_progHX712 = 1;	/* The number of transitions out of state S_progHX712 */
	const FwSmCounterU2_t CHOICE2 = 2;		/* The identifier of choice pseudo-state CHOICE2 in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_CHOICE2 = 2;	/* The number of transitions out of the choice-pseudo state CHOICE2 */
	const FwSmCounterU2_t N_OUT_OF_S_progLdrSwitchValue = 1;	/* The number of transitions out of state S_progLdrSwitchValue */
	const FwSmCounterU2_t N_OUT_OF_S_getLdrValue = 1;	/* The number of transitions out of state S_getLdrValue */
	const FwSmCounterU2_t CHOICE3 = 3;		/* The identifier of choice pseudo-state CHOICE3 in State Machine MainFsm */
	const FwSmCounterU2_t N_OUT_OF_CHOICE3 = 2;	/* The number of transitions out of the choice-pseudo state CHOICE3 */
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
	FwSmAddChoicePseudoState(&smDesc, CHOICE1, N_OUT_OF_CHOICE1);
	FwSmAddState(&smDesc, MainFsm_S_runHX712, N_OUT_OF_S_runHX712, NULL, NULL, NULL, &EsmDescId2940);
	FwSmAddState(&smDesc, MainFsm_S_progHX712, N_OUT_OF_S_progHX712, NULL, NULL, NULL, &EsmDescId2939);
	FwSmAddChoicePseudoState(&smDesc, CHOICE2, N_OUT_OF_CHOICE2);
	FwSmAddState(&smDesc, MainFsm_S_progLdrSwitchValue, N_OUT_OF_S_progLdrSwitchValue, &f_setNewLdrSwitchValue, NULL, NULL, NULL);
	FwSmAddState(&smDesc, MainFsm_S_getLdrValue, N_OUT_OF_S_getLdrValue, &f_getLdrValue, NULL, NULL, NULL);
	FwSmAddChoicePseudoState(&smDesc, CHOICE3, N_OUT_OF_CHOICE3);
	FwSmAddState(&smDesc, MainFsm_S_sleepMode, N_OUT_OF_S_sleepMode, &f_sleepMode, NULL, NULL, NULL);
	FwSmAddTransIpsToCps(&smDesc, CHOICE2, &A_INIT);
	FwSmAddTransStaToCps(&smDesc, TCLK, MainFsm_S_START, CHOICE1, NULL, NULL);
	FwSmAddTransCpsToSta(&smDesc, CHOICE1, MainFsm_S_progHX712, NULL, &G_progHX712Mode);
	FwSmAddTransCpsToSta(&smDesc, CHOICE1, MainFsm_S_runHX712, NULL, &G_runHX712_atTime);
	FwSmAddTransCpsToSta(&smDesc, CHOICE1, MainFsm_S_getLdrValue, NULL, &G_runLDR_atTime);
	FwSmAddTransCpsToSta(&smDesc, CHOICE1, MainFsm_S_START, NULL, NULL);
	FwSmAddTransStaToSta(&smDesc, TCLAK, MainFsm_S_runHX712, MainFsm_S_START, NULL, NULL);
	FwSmAddTransStaToSta(&smDesc, TCLAK, MainFsm_S_progHX712, MainFsm_S_START, NULL, NULL);
	FwSmAddTransCpsToSta(&smDesc, CHOICE2, MainFsm_S_progLdrSwitchValue, NULL, &G_run_ldr_switch_value_update);
	FwSmAddTransCpsToSta(&smDesc, CHOICE2, MainFsm_S_getLdrValue, NULL, &G_enter_all);
	FwSmAddTransStaToSta(&smDesc, TCLK, MainFsm_S_progLdrSwitchValue, MainFsm_S_getLdrValue, NULL, NULL);
	FwSmAddTransStaToCps(&smDesc, TCLK, MainFsm_S_getLdrValue, CHOICE3, NULL, NULL);
	FwSmAddTransCpsToSta(&smDesc, CHOICE3, MainFsm_S_sleepMode, NULL, &G_sleepMode);
	FwSmAddTransCpsToSta(&smDesc, CHOICE3, MainFsm_S_START, NULL, NULL);

	return &smDesc;
}