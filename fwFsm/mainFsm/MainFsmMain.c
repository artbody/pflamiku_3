/**
 * @file MainFsmMain.c
 *
 * @author FW Profile code generator version 5.22
 * @date Created on: Jan 28 2019 15:51:0
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