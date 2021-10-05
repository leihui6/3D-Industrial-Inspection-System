#include "BackProcessExp.h"
#include "BackProcess.h"

BACKPROCESSCOM_EXPORT BackProcessCom * getBackProcessCom()
{
	BackProcessCom * t_BackProcessCom = new BackProcess();

	return t_BackProcessCom;
}

BACKPROCESSCOM_EXPORT const char * getVersion()
{
	return "This is a msvc2017 dll! version= 0.1";
}
