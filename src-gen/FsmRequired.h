
#ifndef FSMREQUIRED_H_
#define FSMREQUIRED_H_

#include "../Inc/sc_types.h"
#include "Fsm.h"

#ifdef __cplusplus
extern "C"
{
#endif 

/*! \file This header defines prototypes for all functions that are required by the state machine implementation.

This is a state machine uses time events which require access to a timing service. Thus the function prototypes:
	- fsm_setTimer and
	- fsm_unsetTimer
are defined.

This state machine makes use of operations declared in the state machines interface or internal scopes. Thus the function prototypes:
	- fsmIface_getLdrValue
	- fsmIface_pumpOn
	- fsmIface_pumpOff
	- fsmIface_pumpError
	- fsmIface_hX712
	- fsmIface_hX712mw
	- fsmIface_hX712Time
	- fsmIface_hX712TimeRM
	- fsmIface_hXExit
	- fsmIface_lowPmode
are defined.

These functions will be called during a 'run to completion step' (runCycle) of the statechart. 
There are some constraints that have to be considered for the implementation of these functions:
	- never call the statechart API functions from within these functions.
	- make sure that the execution time is as short as possible.
 
*/
extern sc_integer fsmIface_getLdrValue(const Fsm* handle);
extern sc_integer fsmIface_pumpOn(const Fsm* handle);
extern sc_integer fsmIface_pumpOff(const Fsm* handle);
extern void fsmIface_pumpError(const Fsm* handle);
extern sc_integer fsmIface_hX712(const Fsm* handle);
extern sc_integer fsmIface_hX712mw(const Fsm* handle);
extern sc_integer fsmIface_hX712Time(const Fsm* handle);
extern sc_integer fsmIface_hX712TimeRM(const Fsm* handle);
extern void fsmIface_hXExit(const Fsm* handle, const sc_integer eepromMinS, const sc_integer eepromMaxS);
extern void fsmIface_lowPmode(const Fsm* handle, const sc_integer LpmOn);


/*!
 * This is a timed state machine that requires timer services
 */ 

/*! This function has to set up timers for the time events that are required by the state machine. */
/*! 
	This function will be called for each time event that is relevant for a state when a state will be entered.
	\param evid An unique identifier of the event.
	\time_ms The time in milliseconds
	\periodic Indicates the the time event must be raised periodically until the timer is unset 
*/
extern void fsm_setTimer(Fsm* handle, const sc_eventid evid, const sc_integer time_ms, const sc_boolean periodic);

/*! This function has to unset timers for the time events that are required by the state machine. */
/*! 
	This function will be called for each time event that is relevant for a state when a state will be left.
	\param evid An unique identifier of the event.
*/
extern void fsm_unsetTimer(Fsm* handle, const sc_eventid evid);



#ifdef __cplusplus
}
#endif 

#endif /* FSMREQUIRED_H_ */
