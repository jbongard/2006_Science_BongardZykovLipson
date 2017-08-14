// GA_Camera.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "GA_Camera.h"

#include "EE.h"
#include "simParams.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

CWinApp theApp;

using namespace std;

SIM_PARAMS *simParams;

extern char FILES_LOCATION[100];
extern char DATA_DIRECTORY[100];

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
	{
	
	}
	else
	{

		system("process -p GA_Camera.exe low");

		char command[100];

		sprintf(command,"del %s\\*.dat",FILES_LOCATION);
		system(command);

		simParams = new SIM_PARAMS(argc,argv);

		sprintf(command,"del %s\\*_%d_%d_*.dat",DATA_DIRECTORY,simParams->randSeed,simParams->GetRegimeIndex());
		system(command);

		simParams->OpenDataFiles();

		EE *ee = new EE;

		if ( simParams->performingBatch )
			ee->PerformBatch();
		else
			ee->PerformInference();

		//ee->CreateMotorCommandsForVictor();

		delete ee;
		ee = NULL;

		delete simParams;
		simParams = NULL;

		return 0;
	}

	return nRetCode;
}


