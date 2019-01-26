
#ifndef FSM_H_
#define FSM_H_

#include "../Inc/sc_types.h"

#ifdef __cplusplus
extern "C" { 
#endif 

/*! \file Header of the state machine 'fsm'.
*/

/*! Define number of states in the state enum */

#define FSM_STATE_COUNT 14

/*! Define dimension of the state configuration vector for orthogonal states. */
#define FSM_MAX_ORTHOGONAL_STATES 1
/*! Define dimension of the state configuration vector for history states. */
#define FSM_MAX_HISTORY_STATES 2

/*! Define maximum number of time events that can be active at once */
#define FSM_MAX_PARALLEL_TIME_EVENTS 1

/*! Define indices of states in the StateConfVector */
#define SCVI_FSM_MAIN_REGION_STATEA 0
#define SCVI_FSM_MAIN_REGION_PROGMODE 0
#define SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_HXREAD 0
#define SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_PUMPON 0
#define SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_HX712TIME 0
#define SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_HXEXIT 0
#define SCVI_FSM_MAIN_REGION_RUNMODE 0
#define SCVI_FSM_MAIN_REGION_RUNMODE_RUN_LDRANDLED 0
#define SCVI_FSM_MAIN_REGION_RUNMODE_RUN_HXREAD 0
#define SCVI_FSM_MAIN_REGION_RUNMODE_RUN_PUMPRMON 0
#define SCVI_FSM_MAIN_REGION_RUNMODE_RUN_PUMPRMOFF 0
#define SCVI_FSM_MAIN_REGION_RUNMODE_RUN_LOWPOWERMODE 0
#define SCVI_FSM_MAIN_REGION_RUNMODE_RUN_ERRORSTATE 0
#define SCVI_FSM_MAIN_REGION__FINAL_ 0

/*! Enumeration of all states */ 
typedef enum
{
	Fsm_last_state,
	Fsm_main_region_StateA,
	Fsm_main_region_progMode,
	Fsm_main_region_progMode_progM_hxRead,
	Fsm_main_region_progMode_progM_pumpON,
	Fsm_main_region_progMode_progM_HX712Time,
	Fsm_main_region_progMode_progM_hxExit,
	Fsm_main_region_runMode,
	Fsm_main_region_runMode_run_LDRandLED,
	Fsm_main_region_runMode_run_HXRead,
	Fsm_main_region_runMode_run_pumpRMOn,
	Fsm_main_region_runMode_run_pumpRMOff,
	Fsm_main_region_runMode_run_lowPowerMode,
	Fsm_main_region_runMode_run_errorState,
	Fsm_main_region__final_
} FsmStates;

/*! Type definition of the data structure for the FsmIface interface scope. */
typedef struct
{
	sc_boolean switcher_raised;
	sc_integer switcher_value;
	sc_integer started;
	sc_integer prog_mode;
	sc_integer pumping;
	sc_integer lowPower;
	sc_integer sw1;
	sc_integer eepromMin;
	sc_integer eepromMax;
	sc_integer eepromPmax;
	sc_integer mittelwert;
	sc_integer timeOut;
	sc_integer ldr;
	sc_integer ldr_switch;
	sc_integer run_mode;
} FsmIface;



/*! Type definition of the data structure for the FsmTimeEvents interface scope. */
typedef struct
{
	sc_boolean fsm_main_region_StateA_tev0_raised;
} FsmTimeEvents;




/*! 
 * Type definition of the data structure for the Fsm state machine.
 * This data structure has to be allocated by the client code. 
 */
typedef struct
{
	FsmStates stateConfVector[FSM_MAX_ORTHOGONAL_STATES];
	FsmStates historyVector[FSM_MAX_HISTORY_STATES];
	sc_ushort stateConfVectorPosition; 
	
	FsmIface iface;
	FsmTimeEvents timeEvents;
} Fsm;



/*! Initializes the Fsm state machine data structures. Must be called before first usage.*/
extern void fsm_init(Fsm* handle);

/*! Activates the state machine */
extern void fsm_enter(Fsm* handle);

/*! Deactivates the state machine */
extern void fsm_exit(Fsm* handle);

/*! Performs a 'run to completion' step. */
extern void fsm_runCycle(Fsm* handle);

/*! Raises a time event. */
extern void fsm_raiseTimeEvent(Fsm* handle, sc_eventid evid);

/*! Raises the in event 'switcher' that is defined in the default interface scope. */ 
extern void fsmIface_raise_switcher(Fsm* handle, sc_integer value);

/*! Gets the value of the variable 'started' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_started(const Fsm* handle);
/*! Sets the value of the variable 'started' that is defined in the default interface scope. */ 
extern void fsmIface_set_started(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'prog_mode' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_prog_mode(const Fsm* handle);
/*! Sets the value of the variable 'prog_mode' that is defined in the default interface scope. */ 
extern void fsmIface_set_prog_mode(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'pumping' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_pumping(const Fsm* handle);
/*! Sets the value of the variable 'pumping' that is defined in the default interface scope. */ 
extern void fsmIface_set_pumping(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'lowPower' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_lowPower(const Fsm* handle);
/*! Sets the value of the variable 'lowPower' that is defined in the default interface scope. */ 
extern void fsmIface_set_lowPower(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'sw1' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_sw1(const Fsm* handle);
/*! Sets the value of the variable 'sw1' that is defined in the default interface scope. */ 
extern void fsmIface_set_sw1(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'eepromMin' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_eepromMin(const Fsm* handle);
/*! Sets the value of the variable 'eepromMin' that is defined in the default interface scope. */ 
extern void fsmIface_set_eepromMin(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'eepromMax' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_eepromMax(const Fsm* handle);
/*! Sets the value of the variable 'eepromMax' that is defined in the default interface scope. */ 
extern void fsmIface_set_eepromMax(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'eepromPmax' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_eepromPmax(const Fsm* handle);
/*! Sets the value of the variable 'eepromPmax' that is defined in the default interface scope. */ 
extern void fsmIface_set_eepromPmax(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'mittelwert' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_mittelwert(const Fsm* handle);
/*! Sets the value of the variable 'mittelwert' that is defined in the default interface scope. */ 
extern void fsmIface_set_mittelwert(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'timeOut' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_timeOut(const Fsm* handle);
/*! Sets the value of the variable 'timeOut' that is defined in the default interface scope. */ 
extern void fsmIface_set_timeOut(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'ldr' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_ldr(const Fsm* handle);
/*! Sets the value of the variable 'ldr' that is defined in the default interface scope. */ 
extern void fsmIface_set_ldr(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'ldr_switch' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_ldr_switch(const Fsm* handle);
/*! Sets the value of the variable 'ldr_switch' that is defined in the default interface scope. */ 
extern void fsmIface_set_ldr_switch(Fsm* handle, sc_integer value);
/*! Gets the value of the variable 'run_mode' that is defined in the default interface scope. */ 
extern sc_integer fsmIface_get_run_mode(const Fsm* handle);
/*! Sets the value of the variable 'run_mode' that is defined in the default interface scope. */ 
extern void fsmIface_set_run_mode(Fsm* handle, sc_integer value);

/*!
 * Checks whether the state machine is active (until 2.4.1 this method was used for states).
 * A state machine is active if it was entered. It is inactive if it has not been entered at all or if it has been exited.
 */
extern sc_boolean fsm_isActive(const Fsm* handle);

/*!
 * Checks if all active states are final. 
 * If there are no active states then the state machine is considered being inactive. In this case this method returns false.
 */
extern sc_boolean fsm_isFinal(const Fsm* handle);

/*! Checks if the specified state is active (until 2.4.1 the used method for states was called isActive()). */
extern sc_boolean fsm_isStateActive(const Fsm* handle, FsmStates state);


#ifdef __cplusplus
}
#endif 

#endif /* FSM_H_ */
