/**
 * @file MainFsmMain.c
 *
 * @author FW Profile code generator version 5.22
 * @date Created on: Jan 27 2019 21:28:55
 */

/** MainFsm function definitions */
#include "MainFsm.h"

/** FW Profile function definitions */
#include "FwSmConstants.h"
#include "FwSmSCreate.h"
#include "FwSmConfig.h"
#include "FwSmCore.h"

#include <stdio.h>
#include <stdlib.h>

/* ----------------------------------------------------------------------------------------------------------------- */

/** Entry Action for the state S_saveEeprom_pumpOff. */
void f_save2Eeprom_pump_off(FwSmDesc_t smDesc)
{
	printf("  Entry Action for the state S_saveEeprom_pumpOff.\n");
}

/** Action on the transition from Initial State to CHOICE1. */
void A_readHX712(FwSmDesc_t smDesc)
{
	printf("  Action on the transition from Initial State to CHOICE1.\n");
}

/** Entry Action for the state S_START. */
void f_getTime(FwSmDesc_t smDesc)
{
	printf("  Entry Action for the state S_START.\n");
}

/** Entry Action for the state S_progLdrSwitchValue. */
void f_setNewLdrSwitchValue(FwSmDesc_t smDesc)
{
	printf("  Entry Action for the state S_progLdrSwitchValue.\n");
}

/** Entry Action for the state S_getLdrValue. */
void f_getLdrValue(FwSmDesc_t smDesc)
{
	printf("  Entry Action for the state S_getLdrValue.\n");
}

/** Guard on the transition from CHOICE3 to S_sleepMode. */
FwSmBool_t G_sleepMode(FwSmDesc_t smDesc)
{
	printf("  Guard on the transition from CHOICE3 to S_sleepMode.\n");
	return rand()>RAND_MAX/2 ? 1 : 0;
}

/* ----------------------------------------------------------------------------------------------------------------- */

int main(void)
{
	/** Define the state machine descriptor (SMD) */
	FwSmDesc_t smDesc = MainFsmCreate(NULL);

	/** Check that the SM is properly configured */
	if (FwSmCheckRec(smDesc) != smSuccess) {
		printf("The state machine MainFsm is NOT properly configured ... FAILURE\n");
		return EXIT_FAILURE;
	}

	printf("The state machine MainFsm is properly configured ... SUCCESS\n");

	/** Start the SM, send a few transition commands to it, and execute it */
	FwSmStart(smDesc);
	FwSmMakeTrans(smDesc, TCLK);
	FwSmMakeTrans(smDesc, TCLAK);

	return EXIT_SUCCESS;
}