/*
 * fsm_timer.h
 *
 *  Created on: 06.01.2019
 *      Author: achim
 */

#ifndef FSM_TIMER_H_
#define FSM_TIMER_H_

#include "sc_types.h"

#ifdef __cplusplus
extern "C" {
#endif


/*! file/ Interface definition of a POSIX thread based timer service for YAKINDU SCT stet machines. */

struct sc_timer_service;

/* internal arguments of a timer pthread */
typedef struct {
	sc_integer time_ms;
	sc_boolean periodic;
	sc_integer elapsed_time_ms;
	sc_eventid pt_evid;
	void* handle;
	struct sc_timer_service *service;
} sc_timer_t;


/*! Function pointer type for state machines runCycle function. */
typedef void (*sc_raise_time_event_fp)(void *handle, sc_eventid evid);


typedef struct sc_timer_service {
	sc_integer timer_count;
	sc_timer_t *timers;
	sc_raise_time_event_fp raise_event;
} sc_timer_service_t;


/*! Initializes a timer service with a set of timers. */
extern void sc_timer_service_init(sc_timer_service_t *tservice,
		sc_timer_t *timers,
		sc_integer count,
		sc_raise_time_event_fp raise_event);

extern void sc_timer_service_proceed(sc_timer_service_t *timer_service, const sc_integer time_ms);



/*! Starts a timer with the specified parameters. */
extern void sc_timer_start (
		sc_timer_service_t *timer_service,
		void* handle,
		const sc_eventid evid,
		const sc_integer time_ms,
		const sc_boolean periodic);


/*! Cancels a timer for the specified time event. */
extern void sc_timer_cancel(sc_timer_service_t *timer_service, const sc_eventid evid);


#ifdef __cplusplus
}
#endif

#endif /* FSM_TIMER_H_ */
