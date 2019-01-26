
#include "Fsm.h"
#include "../Inc/sc_types.h"
#include "FsmRequired.h"

#include <stdlib.h>
#include <string.h>
/*! \file Implementation of the state machine 'fsm'
*/

/* prototypes of all internal functions */
static void effect_main_region_progMode_tr0(Fsm* handle);
static void effect_main_region_runMode_tr0(Fsm* handle);
static void enact_main_region_StateA(Fsm* handle);
static void enact_main_region_progMode_progM_hxRead(Fsm* handle);
static void enact_main_region_progMode_progM_pumpON(Fsm* handle);
static void enact_main_region_progMode_progM_HX712Time(Fsm* handle);
static void enact_main_region_progMode_progM_hxExit(Fsm* handle);
static void enact_main_region_runMode_run_LDRandLED(Fsm* handle);
static void enact_main_region_runMode_run_HXRead(Fsm* handle);
static void enact_main_region_runMode_run_pumpRMOff(Fsm* handle);
static void enact_main_region_runMode_run_lowPowerMode(Fsm* handle);
static void enact_main_region_runMode_run_errorState(Fsm* handle);
static void exact_main_region_StateA(Fsm* handle);
static void exact_main_region_runMode(Fsm* handle);
static void enseq_main_region_StateA_default(Fsm* handle);
static void enseq_main_region_progMode_default(Fsm* handle);
static void enseq_main_region_progMode_progM_hxRead_default(Fsm* handle);
static void enseq_main_region_progMode_progM_pumpON_default(Fsm* handle);
static void enseq_main_region_progMode_progM_HX712Time_default(Fsm* handle);
static void enseq_main_region_progMode_progM_hxExit_default(Fsm* handle);
static void enseq_main_region_runMode_default(Fsm* handle);
static void enseq_main_region_runMode_run_LDRandLED_default(Fsm* handle);
static void enseq_main_region_runMode_run_HXRead_default(Fsm* handle);
static void enseq_main_region_runMode_run_pumpRMOn_default(Fsm* handle);
static void enseq_main_region_runMode_run_pumpRMOff_default(Fsm* handle);
static void enseq_main_region_runMode_run_lowPowerMode_default(Fsm* handle);
static void enseq_main_region_runMode_run_errorState_default(Fsm* handle);
static void enseq_main_region__final__default(Fsm* handle);
static void enseq_main_region_default(Fsm* handle);
static void enseq_main_region_progMode_progM_default(Fsm* handle);
static void shenseq_main_region_progMode_progM(Fsm* handle);
static void enseq_main_region_runMode_run_default(Fsm* handle);
static void shenseq_main_region_runMode_run(Fsm* handle);
static void exseq_main_region_StateA(Fsm* handle);
static void exseq_main_region_progMode(Fsm* handle);
static void exseq_main_region_progMode_progM_hxRead(Fsm* handle);
static void exseq_main_region_progMode_progM_pumpON(Fsm* handle);
static void exseq_main_region_progMode_progM_HX712Time(Fsm* handle);
static void exseq_main_region_progMode_progM_hxExit(Fsm* handle);
static void exseq_main_region_runMode(Fsm* handle);
static void exseq_main_region_runMode_run_LDRandLED(Fsm* handle);
static void exseq_main_region_runMode_run_HXRead(Fsm* handle);
static void exseq_main_region_runMode_run_pumpRMOn(Fsm* handle);
static void exseq_main_region_runMode_run_pumpRMOff(Fsm* handle);
static void exseq_main_region_runMode_run_lowPowerMode(Fsm* handle);
static void exseq_main_region_runMode_run_errorState(Fsm* handle);
static void exseq_main_region__final_(Fsm* handle);
static void exseq_main_region(Fsm* handle);
static void exseq_main_region_progMode_progM(Fsm* handle);
static void exseq_main_region_runMode_run(Fsm* handle);
static void react_main_region__entry_Default(Fsm* handle);
static void react_main_region_progMode_progM__entry_Default(Fsm* handle);
static void react_main_region_progMode_progM_HistProg(Fsm* handle);
static void react_main_region_runMode_run__entry_Default(Fsm* handle);
static void react_main_region_runMode_run_runH(Fsm* handle);
static void react_main_region_progMode_progM_progExit(Fsm* handle);
static void react_main_region_runMode_run_RMExit(Fsm* handle);
static sc_boolean react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_StateA_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_progMode_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_progMode_progM_hxRead_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_progMode_progM_pumpON_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_progMode_progM_HX712Time_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_progMode_progM_hxExit_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_runMode_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_runMode_run_LDRandLED_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_runMode_run_HXRead_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_runMode_run_pumpRMOn_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_runMode_run_pumpRMOff_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_runMode_run_lowPowerMode_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region_runMode_run_errorState_react(Fsm* handle, const sc_boolean try_transition);
static sc_boolean main_region__final__react(Fsm* handle, const sc_boolean try_transition);
static void clearInEvents(Fsm* handle);
static void clearOutEvents(Fsm* handle);


void fsm_init(Fsm* handle)
{
	sc_integer i;
	
	for (i = 0; i < FSM_MAX_ORTHOGONAL_STATES; ++i)
	{
		handle->stateConfVector[i] = Fsm_last_state;
	}
	
	for (i = 0; i < FSM_MAX_HISTORY_STATES; ++i)
	{
		handle->historyVector[i] = Fsm_last_state;
	}
	
	handle->stateConfVectorPosition = 0;
	
	clearInEvents(handle);
	clearOutEvents(handle);
	
	/* Default init sequence for statechart fsm */
	handle->iface.started = 0;
	handle->iface.prog_mode = 0;
	handle->iface.pumping = 0;
	handle->iface.lowPower = 0;
	handle->iface.sw1 = 0;
	handle->iface.eepromMin = 0;
	handle->iface.eepromMax = 0;
	handle->iface.eepromPmax = 0;
	handle->iface.mittelwert = 0;
	handle->iface.timeOut = 0;
	handle->iface.ldr = 0;
	handle->iface.ldr_switch = 0;
	handle->iface.run_mode = 0;
}

void fsm_enter(Fsm* handle)
{
	/* Default enter sequence for statechart fsm */
	enseq_main_region_default(handle);
}

void fsm_runCycle(Fsm* handle)
{
	clearOutEvents(handle);
	for (handle->stateConfVectorPosition = 0;
		handle->stateConfVectorPosition < FSM_MAX_ORTHOGONAL_STATES;
		handle->stateConfVectorPosition++)
		{
			
		switch (handle->stateConfVector[handle->stateConfVectorPosition])
		{
		case Fsm_main_region_StateA:
		{
			main_region_StateA_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_progMode_progM_hxRead:
		{
			main_region_progMode_progM_hxRead_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_progMode_progM_pumpON:
		{
			main_region_progMode_progM_pumpON_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_progMode_progM_HX712Time:
		{
			main_region_progMode_progM_HX712Time_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_progMode_progM_hxExit:
		{
			main_region_progMode_progM_hxExit_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_runMode_run_LDRandLED:
		{
			main_region_runMode_run_LDRandLED_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_runMode_run_HXRead:
		{
			main_region_runMode_run_HXRead_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOn:
		{
			main_region_runMode_run_pumpRMOn_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOff:
		{
			main_region_runMode_run_pumpRMOff_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_runMode_run_lowPowerMode:
		{
			main_region_runMode_run_lowPowerMode_react(handle, bool_true);
			break;
		}
		case Fsm_main_region_runMode_run_errorState:
		{
			main_region_runMode_run_errorState_react(handle, bool_true);
			break;
		}
		case Fsm_main_region__final_:
		{
			main_region__final__react(handle, bool_true);
			break;
		}
		default:
			break;
		}
	}
	
	clearInEvents(handle);
}

void fsm_exit(Fsm* handle)
{
	/* Default exit sequence for statechart fsm */
	exseq_main_region(handle);
}

sc_boolean fsm_isActive(const Fsm* handle)
{
	sc_boolean result = bool_false;
	sc_integer i;
	
	for(i = 0; i < FSM_MAX_ORTHOGONAL_STATES; i++)
	{
		result = result || handle->stateConfVector[i] != Fsm_last_state;
	}
	
	return result;
}

sc_boolean fsm_isFinal(const Fsm* handle)
{
	return (handle->stateConfVector[0] == Fsm_main_region__final_);

}

void fsm_raiseTimeEvent(Fsm* handle, sc_eventid evid)
{
	if ( ((sc_intptr_t)evid) >= ((sc_intptr_t)&(handle->timeEvents))
		&&  ((sc_intptr_t)evid) < ((sc_intptr_t)&(handle->timeEvents)) + sizeof(FsmTimeEvents))
		{
		*(sc_boolean*)evid = bool_true;
	}		
}

sc_boolean fsm_isStateActive(const Fsm* handle, FsmStates state)
{
	sc_boolean result = bool_false;
	switch (state)
	{
		case Fsm_main_region_StateA :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_STATEA] == Fsm_main_region_StateA
			);
			break;
		case Fsm_main_region_progMode :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_PROGMODE] >= Fsm_main_region_progMode
				&& handle->stateConfVector[SCVI_FSM_MAIN_REGION_PROGMODE] <= Fsm_main_region_progMode_progM_hxExit);
			break;
		case Fsm_main_region_progMode_progM_hxRead :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_HXREAD] == Fsm_main_region_progMode_progM_hxRead
			);
			break;
		case Fsm_main_region_progMode_progM_pumpON :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_PUMPON] == Fsm_main_region_progMode_progM_pumpON
			);
			break;
		case Fsm_main_region_progMode_progM_HX712Time :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_HX712TIME] == Fsm_main_region_progMode_progM_HX712Time
			);
			break;
		case Fsm_main_region_progMode_progM_hxExit :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_PROGMODE_PROGM_HXEXIT] == Fsm_main_region_progMode_progM_hxExit
			);
			break;
		case Fsm_main_region_runMode :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE] >= Fsm_main_region_runMode
				&& handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE] <= Fsm_main_region_runMode_run_errorState);
			break;
		case Fsm_main_region_runMode_run_LDRandLED :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE_RUN_LDRANDLED] == Fsm_main_region_runMode_run_LDRandLED
			);
			break;
		case Fsm_main_region_runMode_run_HXRead :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE_RUN_HXREAD] == Fsm_main_region_runMode_run_HXRead
			);
			break;
		case Fsm_main_region_runMode_run_pumpRMOn :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE_RUN_PUMPRMON] == Fsm_main_region_runMode_run_pumpRMOn
			);
			break;
		case Fsm_main_region_runMode_run_pumpRMOff :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE_RUN_PUMPRMOFF] == Fsm_main_region_runMode_run_pumpRMOff
			);
			break;
		case Fsm_main_region_runMode_run_lowPowerMode :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE_RUN_LOWPOWERMODE] == Fsm_main_region_runMode_run_lowPowerMode
			);
			break;
		case Fsm_main_region_runMode_run_errorState :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION_RUNMODE_RUN_ERRORSTATE] == Fsm_main_region_runMode_run_errorState
			);
			break;
		case Fsm_main_region__final_ :
			result = (sc_boolean) (handle->stateConfVector[SCVI_FSM_MAIN_REGION__FINAL_] == Fsm_main_region__final_
			);
			break;
		default:
			result = bool_false;
			break;
	}
	return result;
}

static void clearInEvents(Fsm* handle)
{
	handle->iface.switcher_raised = bool_false;
	handle->timeEvents.fsm_main_region_StateA_tev0_raised = bool_false;
}

static void clearOutEvents(Fsm* handle)
{
}

void fsmIface_raise_switcher(Fsm* handle, sc_integer value)
{
	handle->iface.switcher_value = value;
	handle->iface.switcher_raised = bool_true;
}


sc_integer fsmIface_get_started(const Fsm* handle)
{
	return handle->iface.started;
}
void fsmIface_set_started(Fsm* handle, sc_integer value)
{
	handle->iface.started = value;
}
sc_integer fsmIface_get_prog_mode(const Fsm* handle)
{
	return handle->iface.prog_mode;
}
void fsmIface_set_prog_mode(Fsm* handle, sc_integer value)
{
	handle->iface.prog_mode = value;
}
sc_integer fsmIface_get_pumping(const Fsm* handle)
{
	return handle->iface.pumping;
}
void fsmIface_set_pumping(Fsm* handle, sc_integer value)
{
	handle->iface.pumping = value;
}
sc_integer fsmIface_get_lowPower(const Fsm* handle)
{
	return handle->iface.lowPower;
}
void fsmIface_set_lowPower(Fsm* handle, sc_integer value)
{
	handle->iface.lowPower = value;
}
sc_integer fsmIface_get_sw1(const Fsm* handle)
{
	return handle->iface.sw1;
}
void fsmIface_set_sw1(Fsm* handle, sc_integer value)
{
	handle->iface.sw1 = value;
}
sc_integer fsmIface_get_eepromMin(const Fsm* handle)
{
	return handle->iface.eepromMin;
}
void fsmIface_set_eepromMin(Fsm* handle, sc_integer value)
{
	handle->iface.eepromMin = value;
}
sc_integer fsmIface_get_eepromMax(const Fsm* handle)
{
	return handle->iface.eepromMax;
}
void fsmIface_set_eepromMax(Fsm* handle, sc_integer value)
{
	handle->iface.eepromMax = value;
}
sc_integer fsmIface_get_eepromPmax(const Fsm* handle)
{
	return handle->iface.eepromPmax;
}
void fsmIface_set_eepromPmax(Fsm* handle, sc_integer value)
{
	handle->iface.eepromPmax = value;
}
sc_integer fsmIface_get_mittelwert(const Fsm* handle)
{
	return handle->iface.mittelwert;
}
void fsmIface_set_mittelwert(Fsm* handle, sc_integer value)
{
	handle->iface.mittelwert = value;
}
sc_integer fsmIface_get_timeOut(const Fsm* handle)
{
	return handle->iface.timeOut;
}
void fsmIface_set_timeOut(Fsm* handle, sc_integer value)
{
	handle->iface.timeOut = value;
}
sc_integer fsmIface_get_ldr(const Fsm* handle)
{
	return handle->iface.ldr;
}
void fsmIface_set_ldr(Fsm* handle, sc_integer value)
{
	handle->iface.ldr = value;
}
sc_integer fsmIface_get_ldr_switch(const Fsm* handle)
{
	return handle->iface.ldr_switch;
}
void fsmIface_set_ldr_switch(Fsm* handle, sc_integer value)
{
	handle->iface.ldr_switch = value;
}
sc_integer fsmIface_get_run_mode(const Fsm* handle)
{
	return handle->iface.run_mode;
}
void fsmIface_set_run_mode(Fsm* handle, sc_integer value)
{
	handle->iface.run_mode = value;
}

/* implementations of all internal functions */

static void effect_main_region_progMode_tr0(Fsm* handle)
{
	exseq_main_region_progMode(handle);
	enseq_main_region_StateA_default(handle);
}

static void effect_main_region_runMode_tr0(Fsm* handle)
{
	exseq_main_region_runMode(handle);
	enseq_main_region_StateA_default(handle);
}

/* Entry action for state 'StateA'. */
static void enact_main_region_StateA(Fsm* handle)
{
	/* Entry action for state 'StateA'. */
	fsm_setTimer(handle, (sc_eventid) &(handle->timeEvents.fsm_main_region_StateA_tev0_raised) , 130, bool_true);
}

/* Entry action for state 'hxRead'. */
static void enact_main_region_progMode_progM_hxRead(Fsm* handle)
{
	/* Entry action for state 'hxRead'. */
	handle->iface.eepromMin = fsmIface_hX712(handle);
}

/* Entry action for state 'pumpON'. */
static void enact_main_region_progMode_progM_pumpON(Fsm* handle)
{
	/* Entry action for state 'pumpON'. */
	fsmIface_pumpOn(handle);
	handle->iface.pumping = 1;
}

/* Entry action for state 'HX712Time'. */
static void enact_main_region_progMode_progM_HX712Time(Fsm* handle)
{
	/* Entry action for state 'HX712Time'. */
	handle->iface.eepromMax = fsmIface_hX712Time(handle);
}

/* Entry action for state 'hxExit'. */
static void enact_main_region_progMode_progM_hxExit(Fsm* handle)
{
	/* Entry action for state 'hxExit'. */
	fsmIface_pumpOff(handle);
	fsmIface_hXExit(handle, handle->iface.eepromMin, handle->iface.eepromMax);
	handle->iface.pumping = 0;
}

/* Entry action for state 'LDRandLED'. */
static void enact_main_region_runMode_run_LDRandLED(Fsm* handle)
{
	/* Entry action for state 'LDRandLED'. */
	handle->iface.ldr = fsmIface_getLdrValue(handle);
}

/* Entry action for state 'HXRead'. */
static void enact_main_region_runMode_run_HXRead(Fsm* handle)
{
	/* Entry action for state 'HXRead'. */
	handle->iface.mittelwert = fsmIface_hX712mw(handle);
}

/* Entry action for state 'pumpRMOff'. */
static void enact_main_region_runMode_run_pumpRMOff(Fsm* handle)
{
	/* Entry action for state 'pumpRMOff'. */
	fsmIface_pumpOff(handle);
}

/* Entry action for state 'lowPowerMode'. */
static void enact_main_region_runMode_run_lowPowerMode(Fsm* handle)
{
	/* Entry action for state 'lowPowerMode'. */
	fsmIface_lowPmode(handle, 1);
}

/* Entry action for state 'errorState'. */
static void enact_main_region_runMode_run_errorState(Fsm* handle)
{
	/* Entry action for state 'errorState'. */
	fsmIface_pumpOff(handle);
	fsmIface_pumpError(handle);
}

/* Exit action for state 'StateA'. */
static void exact_main_region_StateA(Fsm* handle)
{
	/* Exit action for state 'StateA'. */
	fsm_unsetTimer(handle, (sc_eventid) &(handle->timeEvents.fsm_main_region_StateA_tev0_raised) );		
}

/* Exit action for state 'runMode'. */
static void exact_main_region_runMode(Fsm* handle)
{
	/* Exit action for state 'runMode'. */
	handle->iface.started = 0;
}

/* 'default' enter sequence for state StateA */
static void enseq_main_region_StateA_default(Fsm* handle)
{
	/* 'default' enter sequence for state StateA */
	enact_main_region_StateA(handle);
	handle->stateConfVector[0] = Fsm_main_region_StateA;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state progMode */
static void enseq_main_region_progMode_default(Fsm* handle)
{
	/* 'default' enter sequence for state progMode */
	enseq_main_region_progMode_progM_default(handle);
}

/* 'default' enter sequence for state hxRead */
static void enseq_main_region_progMode_progM_hxRead_default(Fsm* handle)
{
	/* 'default' enter sequence for state hxRead */
	enact_main_region_progMode_progM_hxRead(handle);
	handle->stateConfVector[0] = Fsm_main_region_progMode_progM_hxRead;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[0] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state pumpON */
static void enseq_main_region_progMode_progM_pumpON_default(Fsm* handle)
{
	/* 'default' enter sequence for state pumpON */
	enact_main_region_progMode_progM_pumpON(handle);
	handle->stateConfVector[0] = Fsm_main_region_progMode_progM_pumpON;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[0] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state HX712Time */
static void enseq_main_region_progMode_progM_HX712Time_default(Fsm* handle)
{
	/* 'default' enter sequence for state HX712Time */
	enact_main_region_progMode_progM_HX712Time(handle);
	handle->stateConfVector[0] = Fsm_main_region_progMode_progM_HX712Time;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[0] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state hxExit */
static void enseq_main_region_progMode_progM_hxExit_default(Fsm* handle)
{
	/* 'default' enter sequence for state hxExit */
	enact_main_region_progMode_progM_hxExit(handle);
	handle->stateConfVector[0] = Fsm_main_region_progMode_progM_hxExit;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[0] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state runMode */
static void enseq_main_region_runMode_default(Fsm* handle)
{
	/* 'default' enter sequence for state runMode */
	enseq_main_region_runMode_run_default(handle);
}

/* 'default' enter sequence for state LDRandLED */
static void enseq_main_region_runMode_run_LDRandLED_default(Fsm* handle)
{
	/* 'default' enter sequence for state LDRandLED */
	enact_main_region_runMode_run_LDRandLED(handle);
	handle->stateConfVector[0] = Fsm_main_region_runMode_run_LDRandLED;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[1] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state HXRead */
static void enseq_main_region_runMode_run_HXRead_default(Fsm* handle)
{
	/* 'default' enter sequence for state HXRead */
	enact_main_region_runMode_run_HXRead(handle);
	handle->stateConfVector[0] = Fsm_main_region_runMode_run_HXRead;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[1] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state pumpRMOn */
static void enseq_main_region_runMode_run_pumpRMOn_default(Fsm* handle)
{
	/* 'default' enter sequence for state pumpRMOn */
	handle->stateConfVector[0] = Fsm_main_region_runMode_run_pumpRMOn;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[1] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state pumpRMOff */
static void enseq_main_region_runMode_run_pumpRMOff_default(Fsm* handle)
{
	/* 'default' enter sequence for state pumpRMOff */
	enact_main_region_runMode_run_pumpRMOff(handle);
	handle->stateConfVector[0] = Fsm_main_region_runMode_run_pumpRMOff;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[1] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state lowPowerMode */
static void enseq_main_region_runMode_run_lowPowerMode_default(Fsm* handle)
{
	/* 'default' enter sequence for state lowPowerMode */
	enact_main_region_runMode_run_lowPowerMode(handle);
	handle->stateConfVector[0] = Fsm_main_region_runMode_run_lowPowerMode;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[1] = handle->stateConfVector[0];
}

/* 'default' enter sequence for state errorState */
static void enseq_main_region_runMode_run_errorState_default(Fsm* handle)
{
	/* 'default' enter sequence for state errorState */
	enact_main_region_runMode_run_errorState(handle);
	handle->stateConfVector[0] = Fsm_main_region_runMode_run_errorState;
	handle->stateConfVectorPosition = 0;
	handle->historyVector[1] = handle->stateConfVector[0];
}

/* Default enter sequence for state null */
static void enseq_main_region__final__default(Fsm* handle)
{
	/* Default enter sequence for state null */
	handle->stateConfVector[0] = Fsm_main_region__final_;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for region main region */
static void enseq_main_region_default(Fsm* handle)
{
	/* 'default' enter sequence for region main region */
	react_main_region__entry_Default(handle);
}

/* 'default' enter sequence for region progM */
static void enseq_main_region_progMode_progM_default(Fsm* handle)
{
	/* 'default' enter sequence for region progM */
	react_main_region_progMode_progM__entry_Default(handle);
}

/* shallow enterSequence with history in child progM */
static void shenseq_main_region_progMode_progM(Fsm* handle)
{
	/* shallow enterSequence with history in child progM */
	/* Handle shallow history entry of progM */
	switch(handle->historyVector[ 0 ])
	{
		case Fsm_main_region_progMode_progM_hxRead :
		{
			enseq_main_region_progMode_progM_hxRead_default(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_pumpON :
		{
			enseq_main_region_progMode_progM_pumpON_default(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_HX712Time :
		{
			enseq_main_region_progMode_progM_HX712Time_default(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_hxExit :
		{
			enseq_main_region_progMode_progM_hxExit_default(handle);
			break;
		}
		default: break;
	}
}

/* 'default' enter sequence for region run */
static void enseq_main_region_runMode_run_default(Fsm* handle)
{
	/* 'default' enter sequence for region run */
	react_main_region_runMode_run__entry_Default(handle);
}

/* shallow enterSequence with history in child run */
static void shenseq_main_region_runMode_run(Fsm* handle)
{
	/* shallow enterSequence with history in child run */
	/* Handle shallow history entry of run */
	switch(handle->historyVector[ 1 ])
	{
		case Fsm_main_region_runMode_run_LDRandLED :
		{
			enseq_main_region_runMode_run_LDRandLED_default(handle);
			break;
		}
		case Fsm_main_region_runMode_run_HXRead :
		{
			enseq_main_region_runMode_run_HXRead_default(handle);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOn :
		{
			enseq_main_region_runMode_run_pumpRMOn_default(handle);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOff :
		{
			enseq_main_region_runMode_run_pumpRMOff_default(handle);
			break;
		}
		case Fsm_main_region_runMode_run_lowPowerMode :
		{
			enseq_main_region_runMode_run_lowPowerMode_default(handle);
			break;
		}
		case Fsm_main_region_runMode_run_errorState :
		{
			enseq_main_region_runMode_run_errorState_default(handle);
			break;
		}
		default: break;
	}
}

/* Default exit sequence for state StateA */
static void exseq_main_region_StateA(Fsm* handle)
{
	/* Default exit sequence for state StateA */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
	exact_main_region_StateA(handle);
}

/* Default exit sequence for state progMode */
static void exseq_main_region_progMode(Fsm* handle)
{
	/* Default exit sequence for state progMode */
	exseq_main_region_progMode_progM(handle);
}

/* Default exit sequence for state hxRead */
static void exseq_main_region_progMode_progM_hxRead(Fsm* handle)
{
	/* Default exit sequence for state hxRead */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state pumpON */
static void exseq_main_region_progMode_progM_pumpON(Fsm* handle)
{
	/* Default exit sequence for state pumpON */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state HX712Time */
static void exseq_main_region_progMode_progM_HX712Time(Fsm* handle)
{
	/* Default exit sequence for state HX712Time */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state hxExit */
static void exseq_main_region_progMode_progM_hxExit(Fsm* handle)
{
	/* Default exit sequence for state hxExit */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state runMode */
static void exseq_main_region_runMode(Fsm* handle)
{
	/* Default exit sequence for state runMode */
	exseq_main_region_runMode_run(handle);
	exact_main_region_runMode(handle);
}

/* Default exit sequence for state LDRandLED */
static void exseq_main_region_runMode_run_LDRandLED(Fsm* handle)
{
	/* Default exit sequence for state LDRandLED */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state HXRead */
static void exseq_main_region_runMode_run_HXRead(Fsm* handle)
{
	/* Default exit sequence for state HXRead */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state pumpRMOn */
static void exseq_main_region_runMode_run_pumpRMOn(Fsm* handle)
{
	/* Default exit sequence for state pumpRMOn */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state pumpRMOff */
static void exseq_main_region_runMode_run_pumpRMOff(Fsm* handle)
{
	/* Default exit sequence for state pumpRMOff */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state lowPowerMode */
static void exseq_main_region_runMode_run_lowPowerMode(Fsm* handle)
{
	/* Default exit sequence for state lowPowerMode */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state errorState */
static void exseq_main_region_runMode_run_errorState(Fsm* handle)
{
	/* Default exit sequence for state errorState */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for final state. */
static void exseq_main_region__final_(Fsm* handle)
{
	/* Default exit sequence for final state. */
	handle->stateConfVector[0] = Fsm_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for region main region */
static void exseq_main_region(Fsm* handle)
{
	/* Default exit sequence for region main region */
	/* Handle exit of all possible states (of fsm.main_region) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Fsm_main_region_StateA :
		{
			exseq_main_region_StateA(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_hxRead :
		{
			exseq_main_region_progMode_progM_hxRead(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_pumpON :
		{
			exseq_main_region_progMode_progM_pumpON(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_HX712Time :
		{
			exseq_main_region_progMode_progM_HX712Time(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_hxExit :
		{
			exseq_main_region_progMode_progM_hxExit(handle);
			break;
		}
		case Fsm_main_region_runMode_run_LDRandLED :
		{
			exseq_main_region_runMode_run_LDRandLED(handle);
			exact_main_region_runMode(handle);
			break;
		}
		case Fsm_main_region_runMode_run_HXRead :
		{
			exseq_main_region_runMode_run_HXRead(handle);
			exact_main_region_runMode(handle);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOn :
		{
			exseq_main_region_runMode_run_pumpRMOn(handle);
			exact_main_region_runMode(handle);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOff :
		{
			exseq_main_region_runMode_run_pumpRMOff(handle);
			exact_main_region_runMode(handle);
			break;
		}
		case Fsm_main_region_runMode_run_lowPowerMode :
		{
			exseq_main_region_runMode_run_lowPowerMode(handle);
			exact_main_region_runMode(handle);
			break;
		}
		case Fsm_main_region_runMode_run_errorState :
		{
			exseq_main_region_runMode_run_errorState(handle);
			exact_main_region_runMode(handle);
			break;
		}
		case Fsm_main_region__final_ :
		{
			exseq_main_region__final_(handle);
			break;
		}
		default: break;
	}
}

/* Default exit sequence for region progM */
static void exseq_main_region_progMode_progM(Fsm* handle)
{
	/* Default exit sequence for region progM */
	/* Handle exit of all possible states (of fsm.main_region.progMode.progM) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Fsm_main_region_progMode_progM_hxRead :
		{
			exseq_main_region_progMode_progM_hxRead(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_pumpON :
		{
			exseq_main_region_progMode_progM_pumpON(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_HX712Time :
		{
			exseq_main_region_progMode_progM_HX712Time(handle);
			break;
		}
		case Fsm_main_region_progMode_progM_hxExit :
		{
			exseq_main_region_progMode_progM_hxExit(handle);
			break;
		}
		default: break;
	}
}

/* Default exit sequence for region run */
static void exseq_main_region_runMode_run(Fsm* handle)
{
	/* Default exit sequence for region run */
	/* Handle exit of all possible states (of fsm.main_region.runMode.run) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Fsm_main_region_runMode_run_LDRandLED :
		{
			exseq_main_region_runMode_run_LDRandLED(handle);
			break;
		}
		case Fsm_main_region_runMode_run_HXRead :
		{
			exseq_main_region_runMode_run_HXRead(handle);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOn :
		{
			exseq_main_region_runMode_run_pumpRMOn(handle);
			break;
		}
		case Fsm_main_region_runMode_run_pumpRMOff :
		{
			exseq_main_region_runMode_run_pumpRMOff(handle);
			break;
		}
		case Fsm_main_region_runMode_run_lowPowerMode :
		{
			exseq_main_region_runMode_run_lowPowerMode(handle);
			break;
		}
		case Fsm_main_region_runMode_run_errorState :
		{
			exseq_main_region_runMode_run_errorState(handle);
			break;
		}
		default: break;
	}
}

/* Default react sequence for initial entry  */
static void react_main_region__entry_Default(Fsm* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_StateA_default(handle);
}

/* Default react sequence for initial entry  */
static void react_main_region_progMode_progM__entry_Default(Fsm* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_progMode_progM_hxRead_default(handle);
}

/* Default react sequence for shallow history entry HistProg */
static void react_main_region_progMode_progM_HistProg(Fsm* handle)
{
	/* Default react sequence for shallow history entry HistProg */
	/* Enter the region with shallow history */
	if (handle->historyVector[0] != Fsm_last_state)
	{
		shenseq_main_region_progMode_progM(handle);
	} else
	{
		enseq_main_region_progMode_progM_hxRead_default(handle);
	} 
}

/* Default react sequence for initial entry  */
static void react_main_region_runMode_run__entry_Default(Fsm* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_runMode_run_LDRandLED_default(handle);
}

/* Default react sequence for shallow history entry runH */
static void react_main_region_runMode_run_runH(Fsm* handle)
{
	/* Default react sequence for shallow history entry runH */
	/* Enter the region with shallow history */
	if (handle->historyVector[1] != Fsm_last_state)
	{
		shenseq_main_region_runMode_run(handle);
	} else
	{
		enseq_main_region_runMode_run_LDRandLED_default(handle);
	} 
}

/* The reactions of exit progExit. */
static void react_main_region_progMode_progM_progExit(Fsm* handle)
{
	/* The reactions of exit progExit. */
	effect_main_region_progMode_tr0(handle);
}

/* The reactions of exit RMExit. */
static void react_main_region_runMode_run_RMExit(Fsm* handle)
{
	/* The reactions of exit RMExit. */
	effect_main_region_runMode_tr0(handle);
}

static sc_boolean react(Fsm* handle, const sc_boolean try_transition) {
	/* State machine reactions. */
	return bool_false;
}

static sc_boolean main_region_StateA_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state StateA. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((react(handle, try_transition)) == (bool_false))
		{ 
			if ((handle->iface.prog_mode) == (1))
			{ 
				exseq_main_region_StateA(handle);
				handle->iface.prog_mode = 2;
				enseq_main_region_progMode_default(handle);
			}  else
			{
				if ((handle->iface.prog_mode) == (2))
				{ 
					exseq_main_region_StateA(handle);
					react_main_region_progMode_progM_HistProg(handle);
				}  else
				{
					if ((handle->iface.started) == (0))
					{ 
						exseq_main_region_StateA(handle);
						handle->iface.started = 1;
						enseq_main_region_runMode_default(handle);
					}  else
					{
						if (handle->timeEvents.fsm_main_region_StateA_tev0_raised == bool_true)
						{ 
							exseq_main_region_StateA(handle);
							react_main_region_runMode_run_runH(handle);
						}  else
						{
							if ((handle->iface.started) == (2))
							{ 
								exseq_main_region_StateA(handle);
								enseq_main_region__final__default(handle);
							}  else
							{
								did_transition = bool_false;
							}
						}
					}
				}
			}
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_progMode_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state progMode. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((react(handle, try_transition)) == (bool_false))
		{ 
			did_transition = bool_false;
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_progMode_progM_hxRead_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state hxRead. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_progMode_react(handle, try_transition)) == (bool_false))
		{ 
			exseq_main_region_progMode_progM_hxRead(handle);
			enseq_main_region_progMode_progM_pumpON_default(handle);
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_progMode_progM_pumpON_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state pumpON. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_progMode_react(handle, try_transition)) == (bool_false))
		{ 
			exseq_main_region_progMode_progM_pumpON(handle);
			handle->iface.sw1 = 1;
			enseq_main_region_progMode_progM_HX712Time_default(handle);
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_progMode_progM_HX712Time_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state HX712Time. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_progMode_react(handle, try_transition)) == (bool_false))
		{ 
			if ((handle->iface.sw1) == (1))
			{ 
				exseq_main_region_progMode_progM_HX712Time(handle);
				enseq_main_region_progMode_progM_HX712Time_default(handle);
			}  else
			{
				exseq_main_region_progMode_progM_HX712Time(handle);
				enseq_main_region_progMode_progM_hxExit_default(handle);
			}
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_progMode_progM_hxExit_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state hxExit. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_progMode_react(handle, try_transition)) == (bool_false))
		{ 
			exseq_main_region_progMode_progM_hxExit(handle);
			handle->iface.prog_mode = 0;
			handle->iface.sw1 = 0;
			react_main_region_progMode_progM_progExit(handle);
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_runMode_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state runMode. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((react(handle, try_transition)) == (bool_false))
		{ 
			did_transition = bool_false;
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_runMode_run_LDRandLED_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state LDRandLED. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_runMode_react(handle, try_transition)) == (bool_false))
		{ 
			if ((handle->iface.ldr) > (handle->iface.ldr_switch))
			{ 
				exseq_main_region_runMode_run_LDRandLED(handle);
				enseq_main_region_runMode_run_lowPowerMode_default(handle);
			}  else
			{
				if ((handle->iface.prog_mode) == (1))
				{ 
					exseq_main_region_runMode(handle);
					fsmIface_pumpOff(handle);
					handle->iface.pumping = 0;
					handle->iface.prog_mode = 2;
					handle->iface.sw1 = 1;
					enseq_main_region_progMode_default(handle);
				}  else
				{
					exseq_main_region_runMode_run_LDRandLED(handle);
					enseq_main_region_runMode_run_HXRead_default(handle);
				}
			}
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_runMode_run_HXRead_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state HXRead. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_runMode_react(handle, try_transition)) == (bool_false))
		{ 
			if ((handle->iface.mittelwert) < (handle->iface.eepromMin))
			{ 
				exseq_main_region_runMode_run_HXRead(handle);
				handle->iface.pumping = fsmIface_pumpOn(handle);
				enseq_main_region_runMode_run_pumpRMOn_default(handle);
			}  else
			{
				if ((handle->iface.pumping) == (1))
				{ 
					exseq_main_region_runMode_run_HXRead(handle);
					enseq_main_region_runMode_run_pumpRMOn_default(handle);
				}  else
				{
					exseq_main_region_runMode_run_HXRead(handle);
					enseq_main_region_runMode_run_LDRandLED_default(handle);
				}
			}
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_runMode_run_pumpRMOn_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state pumpRMOn. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_runMode_react(handle, try_transition)) == (bool_false))
		{ 
			if ((handle->iface.prog_mode) == (1))
			{ 
				exseq_main_region_runMode(handle);
				fsmIface_pumpOff(handle);
				handle->iface.pumping = 0;
				handle->iface.prog_mode = 2;
				handle->iface.sw1 = 1;
				enseq_main_region_progMode_default(handle);
			}  else
			{
				if ((((handle->iface.mittelwert) < (handle->iface.eepromMax)) && ((handle->iface.pumping) == (1))) && ((handle->iface.timeOut) < (handle->iface.eepromPmax)))
				{ 
					exseq_main_region_runMode_run_pumpRMOn(handle);
					handle->iface.mittelwert = fsmIface_hX712TimeRM(handle);
					enseq_main_region_runMode_run_pumpRMOn_default(handle);
				}  else
				{
					if ((handle->iface.mittelwert) >= (handle->iface.eepromMax))
					{ 
						exseq_main_region_runMode_run_pumpRMOn(handle);
						handle->iface.pumping = 0;
						enseq_main_region_runMode_run_pumpRMOff_default(handle);
					}  else
					{
						exseq_main_region_runMode_run_pumpRMOn(handle);
						enseq_main_region_runMode_run_errorState_default(handle);
					}
				}
			}
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_runMode_run_pumpRMOff_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state pumpRMOff. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_runMode_react(handle, try_transition)) == (bool_false))
		{ 
			exseq_main_region_runMode_run_pumpRMOff(handle);
			enseq_main_region_runMode_run_LDRandLED_default(handle);
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_runMode_run_lowPowerMode_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state lowPowerMode. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_runMode_react(handle, try_transition)) == (bool_false))
		{ 
			exseq_main_region_runMode_run_lowPowerMode(handle);
			react_main_region_runMode_run_RMExit(handle);
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region_runMode_run_errorState_react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state errorState. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((main_region_runMode_react(handle, try_transition)) == (bool_false))
		{ 
			did_transition = bool_false;
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}

static sc_boolean main_region__final__react(Fsm* handle, const sc_boolean try_transition) {
	/* The reactions of state null. */
	sc_boolean did_transition = try_transition;
	if (try_transition == bool_true)
	{ 
		if ((react(handle, try_transition)) == (bool_false))
		{ 
			did_transition = bool_false;
		} 
	} 
	if ((did_transition) == (bool_false))
	{ 
	} 
	return did_transition;
}


