#ifndef __DAP_CODE_C__
#define __DAP_CODE_C__

#include "DAP_config.h"
#include "../DAP.h"

void UserAppInit(CoreDescriptor_t *core);
void UserAppAbort(void);

__attribute__((section("USERINIT")))
const UserAppDescriptor_t UserAppDescriptor = {
	&UserAppInit,
	&DAP_ProcessCommand,
	&UserAppAbort
};

CoreDescriptor_t * pCoreDescriptor;

void UserAppInit(CoreDescriptor_t *core)
{
	pCoreDescriptor = core;
	DAP_Setup();
}

void UserAppAbort(void)
{
	DAP_TransferAbort = 1;
}

#include "..\DAP.c"
#include "..\SW_DP.c"
#include "..\JTAG_DP.c"

#endif
