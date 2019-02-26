/**
 * @file
 * This header file declares the function to create one instance of the MainFsm state machine.
 * The state machine is configured with a set of function pointers representing the non-default
 * actions and guards of the state machine. Some of these functions may also be declared in
 * this header file in accordance with the configuration of the state machine in the FW Profile
 * Editor. In the latter case, the user has to provide an implementation for these functions
 * in a user-supplied body file.
 *
 * This header file has been automatically generated by the FW Profile Editor.
 * The state machine created by this file is shown in the figure below.
 * @image html MainFsm.png
 *
 * @author FW Profile code generator version 5.22
 * @date Created on: Feb 1 2019 19:18:44
 */

/* Make sure to include this header file only once */
#ifndef MAINFSM_H_
#define MAINFSM_H_

/* FW Profile function definitions */
#include "FwSmConstants.h"

/* State identifiers */
#define MainFsm_S_measureHX (1)		/**< The identifier of state S_measureHX in State Machine MainFsm */
#define MainFsm_S_pumpOn (2)		/**< The identifier of state S_pumpOn in State Machine MainFsm */
#define MainFsm_S_saveEeprom_pumpOff (3)		/**< The identifier of state S_saveEeprom_pumpOff in State Machine MainFsm */
#define MainFsm_S_setRGB (4)		/**< The identifier of state S_setRGB in State Machine MainFsm */
#define MainFsm_S_waterLow (5)		/**< The identifier of state S_waterLow in State Machine MainFsm */
#define MainFsm_ES_waterProg_enter (1)		/**< The identifier of state ES_waterProg_enter in State Machine MainFsm */
#define MainFsm_ES_waterProg_exit (2)		/**< The identifier of state ES_waterProg_exit in State Machine MainFsm */
#define MainFsm_ES_waterProg_run (3)		/**< The identifier of state ES_waterProg_run in State Machine MainFsm */
#define MainFsm_ES_waterProg_runing (4)		/**< The identifier of state ES_waterProg_runing in State Machine MainFsm */
#define MainFsm_ES_waterProg_stopping (5)		/**< The identifier of state ES_waterProg_stopping in State Machine MainFsm */
#define MainFsm_S_getLdrValue (1)		/**< The identifier of state S_getLdrValue in State Machine MainFsm */
#define MainFsm_S_progHX712 (2)		/**< The identifier of state S_progHX712 in State Machine MainFsm */
#define MainFsm_S_progLdrSwitchValue (3)		/**< The identifier of state S_progLdrSwitchValue in State Machine MainFsm */
#define MainFsm_S_runHX712 (4)		/**< The identifier of state S_runHX712 in State Machine MainFsm */
#define MainFsm_S_sleepMode (5)		/**< The identifier of state S_sleepMode in State Machine MainFsm */
#define MainFsm_S_START (6)		/**< The identifier of state S_START in State Machine MainFsm */

/* User-defined includes */
#include "stdint.h"
//#include "menu.h"
#include "fsmDefs.h"
#include "stm32l0xx_hal.h"
extern FsmIFace* piface;
extern void HX712_run(void);
extern void LDR_Value(void);
extern void LED_RGB_Set(float HSV_value);
extern void Read_from_Eeprom(void);
extern void Save_2_Eeprom(void);
extern void pumpOFF(void);
extern void pumpON(void);
extern void progLdrSwitchValue(void);
extern void sleepMode(void);
extern void Error_Handler(void);
extern uint32_t shift_test_switch_is_ON(void);
extern void reducePwr(void);
extern void fullPwr(void);
extern void afterPOff_newMw(void);

/* The identifiers of transition commands (triggers) */
#define Execute (0) /**< The identifier of the Execute transition trigger */
#define TCLK (8)/**< The identifier of the TCLK transition trigger */
#define TCLAK (9)/**< The identifier of the TCLAK transition trigger */

/**
 * Create a new state machine descriptor.
 * This interface creates the state machine descriptor statically.
 * It creates a single static instance of the state machine.
 * The function should only be called once.
 * If it is called several times, it always reconfigures and returns the same instance.
 * @param smData the pointer to the state machine data.
 * A value of NULL is legal (note that the default value of the pointer
 * to the state machine data when the state machine is created is NULL).
 * @return the pointer to the state machine descriptor
 */
FwSmDesc_t MainFsmCreate(void* smData);

/**
 * Entry Action for the state S_setRGB.
 * @param smDesc the state machine descriptor
 */
void f_setRGB(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_saveEeprom_pumpOff.
 * @param smDesc the state machine descriptor
 */
void f_pump_off(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_pumpOn.
 * @param smDesc the state machine descriptor
 */
void f_pumpOn(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_measureHX.
 * @param smDesc the state machine descriptor
 */
void f_readHX712(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_waterLow.
 * @param smDesc the state machine descriptor
 */
void f_water_low(FwSmDesc_t smDesc);

/**
 * Action on the transition from Initial State to C_mwKleinerMin.
 * @param smDesc the state machine descriptor
 */
void A_readHX712(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_mwKleinerMin to S_pumpOn.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_mwKleinerMin(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_mwKleinerMin to S_measureHX.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_mwKlMaxUpumpOn(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_timeout_exit to S_setRGB.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_switch_on(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_timeout_exit to S_waterLow.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_pumpTimGrTmax(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_timeout_exit to S_saveEeprom_pumpOff.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_mwGrMax(FwSmDesc_t smDesc);

/**
 * Entry Action for the state ES_waterProg_enter.
 * @param smDesc the state machine descriptor
 */
void f_es_menuFsm_waterProg_enter(FwSmDesc_t smDesc);

/**
 * Entry Action for the state ES_waterProg_run.
 * @param smDesc the state machine descriptor
 */
void f_waterProg_run(FwSmDesc_t smDesc);

/**
 * Entry Action for the state ES_waterProg_exit.
 * @param smDesc the state machine descriptor
 */
void f_waterProg_exit(FwSmDesc_t smDesc);

/**
 * Entry Action for the state ES_waterProg_runing.
 * @param smDesc the state machine descriptor
 */
void f_getSwitch(FwSmDesc_t smDesc);

/**
 * Entry Action for the state ES_waterProg_stopping.
 * @param smDesc the state machine descriptor
 */
void f_waterProg_stop(FwSmDesc_t smDesc);

/**
 * Action on the transition from Initial State to ES_waterProg_enter.
 * @param smDesc the state machine descriptor
 */
void f_waterProg_preenter(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_waterProg_run to ES_waterProg_run.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_waterProg_run(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_waterProg_run to ES_waterProg_exit.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_waterProg_exit(FwSmDesc_t smDesc);

/**
 * Guard on the transition from ES_waterProg_runing to ES_waterProg_stopping.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_switchOff(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_START.
 * @param smDesc the state machine descriptor
 */
void f_getTime(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_progLdrSwitchValue.
 * @param smDesc the state machine descriptor
 */
void f_setNewLdrSwitchValue(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_getLdrValue.
 * @param smDesc the state machine descriptor
 */
void f_getLdrValue(FwSmDesc_t smDesc);

/**
 * Entry Action for the state S_sleepMode.
 * @param smDesc the state machine descriptor
 */
void f_sleepMode(FwSmDesc_t smDesc);

/**
 * Action on the transition from Initial State to C_prog_ldr_switch.
 * @param smDesc the state machine descriptor
 */
void A_INIT(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_HX_prog_or_run to S_progHX712.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_progHX712Mode(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_HX_prog_or_run to S_runHX712.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_runHX712_atTime(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_HX_prog_or_run to S_getLdrValue.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_runLDR_atTime(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_prog_ldr_switch to S_progLdrSwitchValue.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_run_ldr_switch_value_update(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_prog_ldr_switch to S_getLdrValue.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_enter_all(FwSmDesc_t smDesc);

/**
 * Guard on the transition from C_sleepmode to S_sleepMode.
 * @param smDesc the state machine descriptor
 * @return 1 if the guard is fulfilled, otherwise 0.
 */
FwSmBool_t G_sleepMode(FwSmDesc_t smDesc);

#endif /* MainFsm_H_ */