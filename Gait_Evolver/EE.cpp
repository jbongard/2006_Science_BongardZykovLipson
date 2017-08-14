

#include "stdafx.h"
#include "math.h"

#ifndef _EE_CPP
#define _EE_CPP

#include "EE.h"
#include "neuralNetwork.h"
#include "simParams.h"

extern int NUM_SENSORS;
extern int NUM_HIDDEN;
extern int NUM_MOTORS;
extern int EVAL_PERIOD;

extern char NN_OUT_FILENAME[100];
extern char TARGET_SOURCE[100];
extern char TARGET_DEST[100];
extern char SENSOR_FILENAME[100];
extern char EXP_FILENAME[100];
extern char EST_FILENAME[100];
extern char TARGET_DATA[100];
extern char TEMP2_FILENAME[100];
extern int  MOTOR_STEP_SIZE;
extern char FILES_LOCATION[200];
extern int  NUM_STICKS;
extern double MIN_RANGE_OF_MOTION;
extern double MAX_RANGE_OF_MOTION;
extern int    NUM_PHYSICAL_TRIALS;
extern char TEMP_FILENAME[100];
extern char BODY_OUT_FILENAME[100];
extern int	TOTAL_SENSORS;
extern int  EST_POPULATION_SIZE;
extern int  NUM_ANGLE_SENSORS;
extern double TEST_BANK_CUTOFF;

extern SIM_PARAMS *simParams;

EE::EE(void) {

	int t;

	dataFromTarget = new MATRIX * [simParams->totalCycles];
	dataForTarget  = new MATRIX * [simParams->totalCycles];
	for (t=0;t<simParams->totalCycles;t++) {
		dataForTarget[t]	= NULL;
		dataFromTarget[t]	= NULL;
	}

	if ( simParams->useTestBank ) {
		bankedMotorPrograms = new MATRIX * [simParams->totalCycles];
		bankedSensorData    = new MATRIX * [simParams->totalCycles];
		for (t=0;t<simParams->totalCycles;t++) {
			bankedMotorPrograms[t]	= NULL;
			bankedSensorData[t]		= NULL;
		}
	}
	else {
		bankedMotorPrograms = NULL;
		bankedSensorData    = NULL;
	}
	numBankedTests		= 0;
	testJustBanked		= false;

	models		   = new MATRIX * [EST_POPULATION_SIZE];
	for (t=0;t<EST_POPULATION_SIZE;t++)
		models[t]			= NULL;

	CreateDataFiles();
}

EE::~EE(void) {

	int t;

	for (t=0;t<simParams->totalCycles;t++) {

		if ( dataForTarget[t] ) {
			delete dataForTarget[t];
			dataForTarget[t] = NULL;
		}

		if ( dataFromTarget[t] ) {
			delete dataFromTarget[t];
			dataFromTarget[t] = NULL;
		}
	}

	delete[] dataForTarget;
	dataForTarget = NULL;

	delete[] dataFromTarget;
	dataFromTarget = NULL;

	if ( simParams->useTestBank ) {

		for (t=0;t<numBankedTests;t++) {
			delete bankedMotorPrograms[t];
			bankedMotorPrograms[t] = NULL;
			delete bankedSensorData[t];
			bankedSensorData[t] = NULL;
		}
		delete[] bankedMotorPrograms;
		bankedMotorPrograms = NULL;
		delete[] bankedSensorData;
		bankedSensorData = NULL;
	}

	for (t=0;t<EST_POPULATION_SIZE;t++) {

		if ( models[t] ) {
			delete models[t];
			models[t] = NULL;
		}
	}

	delete[] models;
	models = NULL;

	//WriteKillSignal();
}

void EE::CreateMotorCommandsForVictor(void) {

	char fileName[200];
	sprintf(fileName,"%s\\runThisPlease.dat",FILES_LOCATION);
	ofstream *outFile = new ofstream(fileName);

	for (int i=0;i<simParams->totalCycles;i++) {

		(*outFile) << "Move(1,";

		for (int j=0;j<NUM_STICKS-1;j++) {

			if ( (j%2)==0 )
				(*outFile) << 128;
			else
				(*outFile) << simParams->RandInt(30,150);

			if ( j<(NUM_STICKS-2) )
				(*outFile) << ",";
		}

		(*outFile) << ");\n";
	}

	outFile->close();
	delete outFile;
	outFile = NULL;
}

void EE::PerformBatch(void) {

	for (simParams->currentCycle=0;
	     simParams->currentCycle<simParams->totalCycles;
		 simParams->currentCycle++) {

		GenerateRandomMotorProgram();

		TestTarget();
	}

	simParams->currentCycle = simParams->totalCycles-1;

	PerformEstimationBatch();
}

void EE::PerformInference(void) {

	int readyBankedTest;

	GenerateRandomMotorProgram();

	while ( simParams->totalTargetEvals<simParams->totalCycles ) {

		TestTarget();

		PerformEstimation();

		if ( testJustBanked ) {
			DepositTest();
			IncreaseTestReliability();
		}
		else {
			simParams->currentCycle++;
			DecreaseTestReliability();
		}

		if ( simParams->currentCycle<simParams->totalCycles ) {
			
			readyBankedTest = BankedTestReadyNow();

			if ( readyBankedTest>=0 )
				WithdrawBankedTest(readyBankedTest);

			else {
				if ( simParams->performingIntelligentTesting )
					PerformExploration();
				else
					GenerateRandomMotorProgram();
			}
		}

		testJustBanked = false;
	}
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

int  EE::BankedTestReadyNow(void) {

	int readyTest = -1;

	if ( (simParams->useTestBank) && (numBankedTests>0) && (!testJustBanked) ) {

		WriteBankedTests();
		readyTest = ComputeBankedTestsReadinesses();
	}

	return( readyTest );
}

int EE::ComputeBankedTestsReadinesses(void) {

	int testReady = -1;

	simParams->WaitForFile(SENSOR_FILENAME);

	ifstream *inFile = new ifstream(SENSOR_FILENAME);

	double numTests;
	(*inFile) >> numTests;

	MATRIX *dataFromBankedTest;

	int currTest = 0;

	while ( (testReady==-1) && (currTest<numTests) ) {

		dataFromBankedTest = new MATRIX(inFile);

		double fitness =
		fabs( bankedSensorData[currTest]->GetTouch(0) - dataFromBankedTest->GetTouch(0) ) +
		fabs( bankedSensorData[currTest]->GetTouch(1) - dataFromBankedTest->GetTouch(1) ) +
		fabs( bankedSensorData[currTest]->GetTouch(2) - dataFromBankedTest->GetTouch(2) ) +
		fabs( bankedSensorData[currTest]->GetTouch(3) - dataFromBankedTest->GetTouch(3) ) +
		fabs( bankedSensorData[currTest]->GetLeftRightTiltReading()   - dataFromBankedTest->GetLeftRightTiltReading() ) +
		fabs( bankedSensorData[currTest]->GetForwardBackTiltReading() - dataFromBankedTest->GetForwardBackTiltReading() ) +
   10.0*fabs( bankedSensorData[currTest]->GetDistanceReading()        - dataFromBankedTest->GetDistanceReading() );

		delete dataFromBankedTest;

		if ( fitness < TEST_BANK_CUTOFF )
			testReady = currTest;

		currTest++;
	}

	inFile->close();
	delete inFile;
	inFile = NULL;

	simParams->FileDelete(SENSOR_FILENAME);

	return( testReady );
}

void EE::CopyTarget(void) {

	/*
	char commandString[200];

	sprintf(commandString,"copy %s %s",TARGET_SOURCE,TARGET_DEST);

	system(commandString);
	*/

	simParams->performingEstimation = true;
	GENOME *targetGenome = new GENOME(0);
	simParams->performingEstimation = false;

	ofstream *bodyFile = new ofstream(TEMP_FILENAME);

	targetGenome->WriteTargetBody(bodyFile);

	delete targetGenome;
	targetGenome = NULL;

	bodyFile->close();
	delete bodyFile;
	bodyFile = NULL;

	simParams->FileRename(TEMP_FILENAME,BODY_OUT_FILENAME);
}

void EE::CreateDataFiles(void) {

	char fileName[200];

	sprintf(fileName,"%s_%d_%d_fit.dat",EXP_FILENAME,simParams->randSeed,simParams->GetRegimeIndex());

	expFitFile = new ofstream(fileName);
	expFitFile->close();
	delete expFitFile;
	expFitFile = NULL;

	sprintf(fileName,"%s_%d_%d_fit.dat",EST_FILENAME,simParams->randSeed,simParams->GetRegimeIndex());

	estFitFile = new ofstream(fileName);
	estFitFile->close();
	delete estFitFile;
	estFitFile = NULL;
}

void EE::CreateEvolvedMotorCommands(int targettedTest) {

	double startCommand;
	double endCommand;
	double diff;
	double command;

	int selectedJoint1 = simParams->allTests->Get(targettedTest,0);
	int selectedJoint2 = simParams->allTests->Get(targettedTest,1);

	printf("[test %d]: %d %d\n",targettedTest,selectedJoint1,selectedJoint2);

	simParams->allTests->Set(targettedTest,2,1.0);

	dataForTarget[simParams->currentCycle] = new MATRIX(EVAL_PERIOD+1,NUM_STICKS-1);

	for (int currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++) {

		startCommand = 0.0;

		for (int i=0;i<EVAL_PERIOD;i=i+MOTOR_STEP_SIZE) {

			//endCommand = simParams->Scale(mc->Get(0,currentJoint),0.0,1.0,MIN_RANGE_OF_MOTION,MAX_RANGE_OF_MOTION);
			if ( (currentJoint==selectedJoint1) || (currentJoint==selectedJoint2) )
				endCommand = MIN_RANGE_OF_MOTION;
			else
				endCommand = MAX_RANGE_OF_MOTION;

			diff = (endCommand - startCommand) / (MOTOR_STEP_SIZE-1);

			for (int k=i;k<i+MOTOR_STEP_SIZE;k++) {

				command = startCommand + (k-i)*diff;
				dataForTarget[simParams->currentCycle]->Set(k,currentJoint,command);
			}

			startCommand = endCommand;
		}
		dataForTarget[simParams->currentCycle]->Set(EVAL_PERIOD,currentJoint,endCommand);
	}

	for (currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++)

		for (int i=MOTOR_STEP_SIZE;i<=EVAL_PERIOD;i++) {
		
			double mCommand = dataForTarget[simParams->currentCycle]->Get(MOTOR_STEP_SIZE-1,currentJoint);

			dataForTarget[simParams->currentCycle]->Set(i,currentJoint,mCommand);
		}

	//dataForTarget[simParams->currentCycle]->Round();
}

void EE::CreateRandomMotorCommands(void) {

	double startCommand;
	double endCommand;
	double diff;
	double command;

	int selectedJoint1;
	int selectedJoint2;

	/*
	switch ( simParams->currentCycle ) {
	case 0:
		selectedJoint1 = 0;
		selectedJoint2 = 2;
		break;
	case 1:
		selectedJoint1 = 0;
		selectedJoint2 = 4;
		break;
	case 2:
		selectedJoint1 = 1;
		selectedJoint2 = -1;
		break;
	case 3:
		selectedJoint1 = 1;
		selectedJoint2 = 5;
		break;
	case 4:
		selectedJoint1 = 2;
		selectedJoint2 = -1;
		break;
	case 5:
		selectedJoint1 = 2;
		selectedJoint2 = 6;
		break;
	case 6:
		selectedJoint1 = 3;
		selectedJoint2 = -1;
		break;
	case 7:
		selectedJoint1 = 3;
		selectedJoint2 = 7;
		break;
	case 8:
		selectedJoint1 = 0;
		selectedJoint2 = 1;
		break;
	case 9:
		selectedJoint1 = 1;
		selectedJoint2 = 2;
		break;
	case 10:
		selectedJoint1 = 2;
		selectedJoint2 = 3;
		break;
	case 11:
		selectedJoint1 = 0;
		selectedJoint2 = 3;
		break;

	}
	printf("[test %d:] %d %d\n",simParams->currentCycle,selectedJoint1,selectedJoint2);
	*/
	
	int chosenTest = simParams->RandInt(0,simParams->totalTests-1);

	while ( simParams->allTests->Get(chosenTest,2) == 1.0 )
		chosenTest = simParams->RandInt(0,simParams->totalTests-1);

	selectedJoint1 = int(simParams->allTests->Get(chosenTest,0));
	selectedJoint2 = int(simParams->allTests->Get(chosenTest,1));

	printf("[test %d:] %d %d\n",chosenTest,selectedJoint1,selectedJoint2);

	simParams->allTests->Set(chosenTest,2,1.0);

	dataForTarget[simParams->currentCycle] = new MATRIX(EVAL_PERIOD+1,NUM_STICKS-1);

	for (int currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++) {

		startCommand = 0.0;

		for (int i=0;i<EVAL_PERIOD;i=i+MOTOR_STEP_SIZE) {

			if ( (currentJoint==selectedJoint1) || (currentJoint==selectedJoint2) )
				//endCommand = simParams->Rand(MIN_RANGE_OF_MOTION,0.0);
				endCommand = MIN_RANGE_OF_MOTION;
			else
				endCommand = MAX_RANGE_OF_MOTION;
				//endCommand = simParams->Rand(MIN_RANGE_OF_MOTION,MAX_RANGE_OF_MOTION);

			diff = (endCommand - startCommand) / (MOTOR_STEP_SIZE-1);

			for (int k=i;k<i+MOTOR_STEP_SIZE;k++) {

				command = startCommand + (k-i)*diff;
				dataForTarget[simParams->currentCycle]->Set(k,currentJoint,command);
			}

			startCommand = endCommand;
		}
		dataForTarget[simParams->currentCycle]->Set(EVAL_PERIOD,currentJoint,endCommand);
	}

	for (currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++)

		for (int i=MOTOR_STEP_SIZE;i<=EVAL_PERIOD;i++) {
		
			double mCommand = dataForTarget[simParams->currentCycle]->Get(MOTOR_STEP_SIZE-1,currentJoint);

			dataForTarget[simParams->currentCycle]->Set(i,currentJoint,mCommand);
		}

	//dataForTarget[simParams->currentCycle]->Round();
}

void EE::DecreaseTestReliability(void) {

	simParams->reliabilityWeight = simParams->reliabilityWeight/1.5;

	if ( simParams->reliabilityWeight < 0.0 )
		simParams->reliabilityWeight = 0.0;
}

void EE::DepositTest(void) {

	bankedMotorPrograms[numBankedTests] = dataForTarget[simParams->currentCycle];

	dataForTarget[simParams->currentCycle] = NULL;

	bankedSensorData[numBankedTests] = dataFromTarget[simParams->currentCycle];

	dataFromTarget[simParams->currentCycle] = NULL;

	numBankedTests++;
}

void EE::GenerateRandomMotorProgram(void) {

	CreateRandomMotorCommands();
}

void EE::GetDummyTargetSensorValues(void) {

	/*
	char fileName[200];
	sprintf(fileName,"%s\\DATA1.TXT",TARGET_DATA,simParams->currentCycle+1);
	ifstream *inFile = new ifstream(fileName);

	MATRIX **trials = new MATRIX * [NUM_PHYSICAL_TRIALS];

	double inputVal;

	for (int i=0;i<NUM_PHYSICAL_TRIALS;i++) {

		trials[i] = new MATRIX((EVAL_PERIOD+1)*(NUM_STICKS-1),14);

		trials[i]->Load(inFile);

		trials[i]->Reduce(NUM_STICKS-1);

		(*inFile) >> inputVal;
	}

	inFile->close();
	delete inFile;
	inFile = NULL;

	dataFromTarget[simParams->currentCycle] = new MATRIX(EVAL_PERIOD+1,14);

	dataFromTarget[simParams->currentCycle]->AverageMatrices(trials,NUM_PHYSICAL_TRIALS);

	for (i=0;i<NUM_PHYSICAL_TRIALS;i++) {
		delete trials[i];
		trials[i] = NULL;
	}
	delete[] trials;
	trials = NULL;
	*/
}

void EE::GetSensorData(void) {

	simParams->WaitForFile(SENSOR_FILENAME);
	//while ( !simParams->FileExists(SENSOR_FILENAME) );

	ifstream *inFile = new ifstream(SENSOR_FILENAME);

	double temp;
	(*inFile) >> temp;

	dataFromTarget[simParams->currentCycle] = new MATRIX(inFile);

	inFile->close();
	delete inFile;
	inFile = NULL;

	simParams->FileDelete(SENSOR_FILENAME);
}

void EE::GetSensorDataFromScreen(void) {

	char answer[50];

	dataFromTarget[simParams->currentCycle] = new MATRIX(1,TOTAL_SENSORS,0.0);

	/*
	// -------------------Angle------------------------
	printf("Left lower leg angle (positive is up): \t\t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,4,atof(answer));
	
	printf("Left upper leg angle (positive is up): \t\t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,0,atof(answer));

	printf("Forward lower leg angle (positive is up): \t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,5,atof(answer));

	printf("Forward upper leg angle (positive is up): \t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,1,atof(answer));

	printf("Right lower leg angle (positive is up): \t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,6,atof(answer));

	printf("Right upper leg angle (positive is up): \t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,2,atof(answer));

	printf("Back lower leg angle (positive is up): \t\t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,7,atof(answer));

	printf("Back upper leg angle (positive is up): \t\t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,3,atof(answer));

	// -------------------Touch------------------------
	printf("Left foot touch (1==touching; 0==not touching): \t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,8,atof(answer));

	printf("Forward foot touch (1==touching; 0==not touching): \t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,9,atof(answer));

	printf("Right foot touch (1==touching; 0==not touching): \t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,10,atof(answer));

	printf("Back foot touch (1==touching; 0==not touching): \t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,11,atof(answer));

	// -------------------Orientation------------------------
	printf("Left/right tilt angle (left is negative): \t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,12,atof(answer));

	printf("Forward/back tilt angle (forward is negative): \t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,13,atof(answer));

	// -------------------Clearance------------------------
	printf("Belly clearance (in millimeters): \t\t\t"); scanf("%s",answer);
	dataFromTarget[simParams->currentCycle]->Set(0,14,atof(answer)/100.0);
	*/

	// Upper left
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,30.319);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-28.923);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30);
		dataFromTarget[simParams->currentCycle]->Set(0,1,28.698);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.407);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.457);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.351);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.197);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-7.0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,6/100.0);
	}

	// Upper forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.637);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.22);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.634);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-29.618);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.035);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.425);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.745);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.379);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,7);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5/100.0);
	}

	// Upper right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.347);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.719);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.242);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.723);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.847);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-30.741);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.7);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.921);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,6.8);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,4.5/100.0);
	}

	// Upper back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,30.279);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.797);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.103);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.137);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.587);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.157);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.242);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.773);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-7);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5/100.0);
	}

	// Lower left
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-31.338);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.367);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.372);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.317);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.684);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.589);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.661);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.479);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,30.021);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.695);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-28.935);
		dataFromTarget[simParams->currentCycle]->Set(0,1,28.747);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.532);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.861);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.548);
		dataFromTarget[simParams->currentCycle]->Set(0,3,31.854);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.858);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.863);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.174);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.324);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-27.64);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.915);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.699);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.073);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.745);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.345);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.572);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.257);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.901);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.518);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-31.477);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.425);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Upper left, upper forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.532);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-28.549);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.646);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-29.61);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.27);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.425);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.825);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.449);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-7);
		dataFromTarget[simParams->currentCycle]->Set(0,13,6);
		dataFromTarget[simParams->currentCycle]->Set(0,14,9.5/100.0);

	}

	// Upper left, upper right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.056);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-28.533);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.294);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.517);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.877);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-31.337);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.798);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.742);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-0.5);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-0.2);
		dataFromTarget[simParams->currentCycle]->Set(0,14,19.75/100.0);
	}

	// Upper left, upper back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.548);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-28.475);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.83);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.518);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.367);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.542);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.279);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.157);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-6.8);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-6.8);
		dataFromTarget[simParams->currentCycle]->Set(0,14,10.5/100.0);
	}

	// Upper left, lower left
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-30.725);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-28.676);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.728);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.343);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.241);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.368);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.049);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.466);
		dataFromTarget[simParams->currentCycle]->Set(0,8,1);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-25.5);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-0.3);
		dataFromTarget[simParams->currentCycle]->Set(0,14,21.5/100.0);
	}

	// Upper left, lower forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.888);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-29.179);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-28.975);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.033);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.339);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.974);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.601);
		dataFromTarget[simParams->currentCycle]->Set(0,3,31.922);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-7);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,6/100.0);
	}

	// Upper left, lower right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.677);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-28.401);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.042);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.971);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-27.766);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.896);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.971);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.269);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-7);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.75/100.0);
	}

	// Upper left, lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.851);
		dataFromTarget[simParams->currentCycle]->Set(0,0,-28.871);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.642);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.136);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.961);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.234);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-32.065);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.308);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-7);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.5/100.0);
	}

	// Upper forward, upper right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.124);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.702);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.454);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-29.226);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.635);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-31.139);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.813);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.708);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,6.7);
		dataFromTarget[simParams->currentCycle]->Set(0,13,6.9);
		dataFromTarget[simParams->currentCycle]->Set(0,14,9/100.0);
	}

	// Upper forward, upper back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.755);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.159);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.146);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-30.196);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.359);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.301);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.849);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.678);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,-0.7);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-0.1);
		dataFromTarget[simParams->currentCycle]->Set(0,14,19.75/100.0);
	}

	// Upper forward, lower left
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-30.747);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.36);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.523);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-29.659);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.156);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.543);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.698);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.562);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,7);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.5/100.0);
	}

	// Upper forward, lower forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.537);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.531);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-29.601);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-29.696);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.863);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.817);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.894);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.75);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,1);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,23);
		dataFromTarget[simParams->currentCycle]->Set(0,14,16.5/100.0);
	}

	// Upper forward, lower right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.8);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.132);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.187);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-29.726);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-28.034);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.236);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.932);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.633);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,6.7);
		dataFromTarget[simParams->currentCycle]->Set(0,14,6/100.0);
	}

	// Upper forward, lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.651);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.182);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.814);
		dataFromTarget[simParams->currentCycle]->Set(0,1,-29.756);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.24);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.261);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-31.527);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.267);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,7);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.75/100.0);
	}

	// Upper right, upper back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.511);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.121);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.085);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.513);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.056);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-31.877);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.967);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.589);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,6.8);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-6.9);
		dataFromTarget[simParams->currentCycle]->Set(0,14,9.5/100.0);
	}

	// Upper right, lower left
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-31.389);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.773);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.065);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.726);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.414);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-30.864);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.297);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.883);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,6.8);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.5/100.0);
	}

	// Upper right, lower forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,30.135);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.304);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-28.812);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.396);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.353);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-31.718);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.979);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.23);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,6.9);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.75/100.0);
	}

	// Upper right, lower right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.618);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.618);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.868);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.108);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-27.826);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-31.859);
		dataFromTarget[simParams->currentCycle]->Set(0,7,27.512);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.698);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,1);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,25.5);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,17.25/100.0);
	}

	// Upper right, lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.981);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.025);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.743);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.616);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.189);
		dataFromTarget[simParams->currentCycle]->Set(0,2,-31.873);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-31.783);
		dataFromTarget[simParams->currentCycle]->Set(0,3,31.564);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,7.5);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.75/100.0);
	}

	// Upper back, lower left
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-28.866);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.508);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.436);
		dataFromTarget[simParams->currentCycle]->Set(0,1,28.54);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.553);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.835);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.376);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.625);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-7);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.75/100.0);
	}

	// Upper back, lower forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.654);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.942);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-28.982);
		dataFromTarget[simParams->currentCycle]->Set(0,1,28.988);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.2);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.742);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.468);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.297);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-7.2);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.5/100.0);
	}

	// Upper back, lower right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.525);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.028);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.039);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.507);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-27.958);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.378);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.382);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.212);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-6.9);
		dataFromTarget[simParams->currentCycle]->Set(0,14,5.75/100.0);
	}

	// Upper back, lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,29.848);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.993);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.265);
		dataFromTarget[simParams->currentCycle]->Set(0,1,28.902);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.376);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.745);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-31.189);
		dataFromTarget[simParams->currentCycle]->Set(0,3,-28.654);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,1);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,-25.1);
		dataFromTarget[simParams->currentCycle]->Set(0,14,17.75/100.0);
	}

	// Lower left, lower forward
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-30.361);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.801);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-29.066);
		dataFromTarget[simParams->currentCycle]->Set(0,1,28.999);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.492);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.994);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.181);
		dataFromTarget[simParams->currentCycle]->Set(0,3,31.998);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower left, lower right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-30.7);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.794);
		dataFromTarget[simParams->currentCycle]->Set(0,5,30.445);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.365);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-27.904);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.761);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.284);
		dataFromTarget[simParams->currentCycle]->Set(0,3,31.953);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower left, lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,-30.715);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.349);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.632);
		dataFromTarget[simParams->currentCycle]->Set(0,1,30.222);
		dataFromTarget[simParams->currentCycle]->Set(0,6,30.933);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.448);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-31.533);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.412);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower forward, lower right
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)>0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,30.55);
		dataFromTarget[simParams->currentCycle]->Set(0,0,28.865);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-28.344);
		dataFromTarget[simParams->currentCycle]->Set(0,1,28.608);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-27.399);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.73);
		dataFromTarget[simParams->currentCycle]->Set(0,7,28.764);
		dataFromTarget[simParams->currentCycle]->Set(0,3,31.941);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower forward, lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,30.239);
		dataFromTarget[simParams->currentCycle]->Set(0,0,30.028);
		dataFromTarget[simParams->currentCycle]->Set(0,5,-28.973);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.781);
		dataFromTarget[simParams->currentCycle]->Set(0,6,31.497);
		dataFromTarget[simParams->currentCycle]->Set(0,2,27.718);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-31.232);
		dataFromTarget[simParams->currentCycle]->Set(0,3,32.091);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}

	// Lower right, lower back
	if ( (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5)>0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6)<0.0) &&
		 (dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7)<0.0) ) {

		dataFromTarget[simParams->currentCycle]->Set(0,4,30.221);
		dataFromTarget[simParams->currentCycle]->Set(0,0,29.74);
		dataFromTarget[simParams->currentCycle]->Set(0,5,29.822);
		dataFromTarget[simParams->currentCycle]->Set(0,1,29.222);
		dataFromTarget[simParams->currentCycle]->Set(0,6,-27.448);
		dataFromTarget[simParams->currentCycle]->Set(0,2,26.759);
		dataFromTarget[simParams->currentCycle]->Set(0,7,-31.097);
		dataFromTarget[simParams->currentCycle]->Set(0,3,31.792);
		dataFromTarget[simParams->currentCycle]->Set(0,8,0);
		dataFromTarget[simParams->currentCycle]->Set(0,9,0);
		dataFromTarget[simParams->currentCycle]->Set(0,10,0);
		dataFromTarget[simParams->currentCycle]->Set(0,11,0);
		dataFromTarget[simParams->currentCycle]->Set(0,12,0);
		dataFromTarget[simParams->currentCycle]->Set(0,13,0);
		dataFromTarget[simParams->currentCycle]->Set(0,14,0/100.0);
	}
}

void EE::GetTargetData(void) {

	dataForTarget[simParams->currentCycle]  = new MATRIX(EVAL_PERIOD,8);
	dataFromTarget[simParams->currentCycle] = new MATRIX(EVAL_PERIOD,14);

	double incomingValue;
	char dummy;

	char fileName[200];
	sprintf(fileName,"%s\\acc00%d1.txt",TARGET_DATA,simParams->currentCycle+1);
	ifstream *inFile = new ifstream(fileName);

	for (int i=0;i<EVAL_PERIOD;i++) {

		// Throw away time
		(*inFile) >> incomingValue;
		(*inFile) >> dummy;
		(*inFile) >> incomingValue;

		(*inFile) >> incomingValue;

	}
	inFile->close();
	delete inFile;
	inFile = NULL;
}

void EE::GetTargetSensorValues(void) {

	//GetDummyTargetSensorValues();
}

void EE::IncreaseTestReliability(void) {

	simParams->reliabilityWeight = simParams->reliabilityWeight*1.5;

	if ( simParams->reliabilityWeight > 1.0 )
		simParams->reliabilityWeight = 1.0;
}

void EE::LoadOldSensorData(void) {

	dataFromTarget[simParams->currentCycle] = new MATRIX(EVAL_PERIOD,TOTAL_SENSORS,0.0);

	switch (simParams->currentCycle) {
	case 0:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),23.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-39.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),-24.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),49.0);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	-11.5);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	+15.3);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.2225);
		break;

	case 1:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),-27.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-15.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),-9.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),-17.0);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	1.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	+10.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	0.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.2675);
		break;

	case 2:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),-32.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-7);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),+4.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),-34.0);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	1.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	+14.5);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	-7.5);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.2025);
		break;

	case 3:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),37.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-4.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),-42.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),-26.0);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	1.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	-20.7);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	-7.9);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.215);
		break;

	case 4:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),0.6);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),1.5);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),+40);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),-2.7);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	2.5);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.025);
		break;

	case 5:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),42.8);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-4.3);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),-1.9);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),14.32);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.0);
		break;

	case 9:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),-8.1);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-17.7);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),16.3);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),-18.2);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	1.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	2.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	0.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.0225);
		break;

	case 7:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),31.5);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-11.9);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),37.6);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),32.2);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	3.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.21);
		break;

	case 8:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),16.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),-19.4);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),-26.4);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),41.9);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	1.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	-10);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	6.4);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.135);
		break;

	case 6:
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(1),14.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(3),11.8);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(5),37.8);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,simParams->VictorToEEAJointIndex(7),-19.1);
		/*
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.319);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.247);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	0.399);
		*/
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,8,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,9,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,10,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,11,	1.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,12,	0.0);
		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,13,	-8.0);

		dataFromTarget[simParams->currentCycle]->Set(EVAL_PERIOD-1,14,	0.06);
		break;
	}
}

void EE::PerformEstimation(void) {

	simParams->performingEstimation = true;

	if ( simParams->currentCycle == 0 )
		ga = new GA;
	else
		ga = new GA(models);

	ga->Evolve(dataForTarget,dataFromTarget,NULL);

	if ( TestFailed() ) {

		if ( simParams->currentCycle==0 )

			StoreModelsForLater();

		else
			StoreBestModelForLater();

		testJustBanked = true;
	}
	else
		StoreModelsForLater();

	delete ga;
	ga = NULL;

	simParams->performingEstimation = false;
}

void EE::PerformEstimationBatch(void) {

	simParams->performingEstimation = true;

	ga = new GA;

	ga->Evolve(dataForTarget,dataFromTarget,NULL);

	models[simParams->currentCycle] = new MATRIX(ga->GetBestModel());

	delete ga;
	ga = NULL;

	simParams->performingEstimation = false;
}

void EE::PerformExploration(void) {

	ga = new GA;

	ga->Evolve(dataForTarget,dataFromTarget,models);

	/*
	int untriedTestFound = false;
	int currentGenome = 0;
	int consideringTest = ga->genomes[currentGenome]->ID;

	while ( !untriedTestFound ) {

		if ( simParams->allTests->Get(consideringTest,2) == 0.0 )
			untriedTestFound = true;
		else {
			currentGenome++;
			consideringTest = ga->genomes[currentGenome]->ID;
		}
	}

	CreateEvolvedMotorCommands(ga->genomes[currentGenome]->ID);
	*/

	delete ga;
	ga = NULL;
}

void EE::SendEvolvedController(void) {

	//dataForTarget[simParams->currentCycle]->Write(NN_OUT_FILENAME);
}

void EE::SendController(void) {

	ofstream *brainFile;

	brainFile = new ofstream(TEMP2_FILENAME);

	(*brainFile) << "1\n";

	dataForTarget[simParams->currentCycle]->Write(brainFile);
	
	brainFile->close();
	delete brainFile;
	brainFile = NULL;

	simParams->FileRename(TEMP2_FILENAME,NN_OUT_FILENAME);
}

void EE::SendControllerToScreen(void) {

	printf("FEED ME! [negative angle==downward; positive angle==upward]\n");

	printf("Left lower leg angle:\t\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,4));
	printf("Left upper leg angle:\t\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,0));

	printf("Forward lower leg angle:\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,5));
	printf("Forward upper leg angle:\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,1));

	printf("Right lower leg angle:\t\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,6));
	printf("Right upper leg angle:\t\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,2));
	
	printf("Back lower leg angle:\t\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,7));	
	printf("Back upper leg angle:\t\t%3.3f degrees.\n",dataForTarget[simParams->currentCycle]->Get(EVAL_PERIOD,3));
}

void EE::SendRandomController(void) {

	//dataForTarget[simParams->currentCycle] = new NEURAL_NETWORK(NUM_SENSORS,NUM_HIDDEN,NUM_MOTORS);

	//dataForTarget[simParams->currentCycle]->Write(NN_OUT_FILENAME);
}

void EE::StoreBestModelForLater(void) {

	if ( models[0] )
		delete models[0];

	models[0] = ga->genomes[0]->raw;
	ga->genomes[0]->raw = NULL;
}

void EE::StoreModelsForLater(void) {

	for (int i=0;i<EST_POPULATION_SIZE;i++) {

		if ( models[i] )
			delete models[i];

		models[i] = ga->genomes[i]->raw;
		ga->genomes[i]->raw = NULL;
	}
}

int  EE::TestFailed(void) {

	return( (simParams->useTestBank) && 
			(ga->genomes[0]->fitness>TEST_BANK_CUTOFF) &&
			(numBankedTests<simParams->totalCycles) );
}

void EE::TestTarget(void) {
	
	if ( simParams->usePhysicalMachine ) {

		SendControllerToScreen();
		GetSensorDataFromScreen();

	}
	else {
		CopyTarget();
		SendController();
		GetSensorData();
	}

	simParams->totalTargetEvals++;

	UpdateMotorProgram();

	ofstream *brainFile;
	char brainFileName[100];
	sprintf(brainFileName,"%s_%d_%d_brain%d.dat",EXP_FILENAME,simParams->randSeed,simParams->GetRegimeIndex(),simParams->currentCycle);
	brainFile = new ofstream(brainFileName);

	dataForTarget[simParams->currentCycle]->Write(brainFile);

	brainFile->close();
	delete brainFile;
	brainFile = NULL;
}

void EE::UpdateMotorProgram(void) {

	double startCommand;
	double endCommand;
	double diff;
	double command;

	for (int currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++) {

		startCommand = 0.0;

		for (int i=0;i<EVAL_PERIOD;i=i+MOTOR_STEP_SIZE) {

			endCommand = dataFromTarget[simParams->currentCycle]->Get(0,currentJoint);

			diff = (endCommand - startCommand) / (MOTOR_STEP_SIZE-1);

			for (int k=i;k<i+MOTOR_STEP_SIZE;k++) {

				command = startCommand + (k-i)*diff;
				dataForTarget[simParams->currentCycle]->Set(k,currentJoint,command);
			}

			startCommand = endCommand;
		}
		dataForTarget[simParams->currentCycle]->Set(EVAL_PERIOD,currentJoint,endCommand);
	}

	for (currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++)

		for (int i=MOTOR_STEP_SIZE;i<=EVAL_PERIOD;i++) {
		
			double mCommand = dataForTarget[simParams->currentCycle]->Get(MOTOR_STEP_SIZE-1,currentJoint);

			dataForTarget[simParams->currentCycle]->Set(i,currentJoint,mCommand);
		}
}

void EE::WithdrawBankedTest(int readyTest) {

	dataForTarget[simParams->currentCycle] =  bankedMotorPrograms[readyTest];
	bankedMotorPrograms[readyTest] = NULL;

	dataFromTarget[simParams->currentCycle] = bankedSensorData[readyTest];
	bankedSensorData[readyTest] = NULL;

	while ( readyTest < (numBankedTests-1) ) {

		bankedMotorPrograms[readyTest]   = bankedMotorPrograms[readyTest+1];
		bankedMotorPrograms[readyTest+1] = NULL;

		bankedSensorData[readyTest]    = bankedSensorData[readyTest+1];
		bankedSensorData[readyTest+1]    = NULL;

		readyTest++;
	}

	numBankedTests--;
}

void EE::WriteBankedTests(void) {

	ofstream *brainFile = new ofstream(TEMP2_FILENAME);
	(*brainFile) << numBankedTests << "\n";

	ofstream *bodyFile = new ofstream(TEMP_FILENAME);
	(*bodyFile) << numBankedTests << "\n";

	simParams->performingEstimation = true;
	GENOME *bestModel = new GENOME(models[0],1);
	simParams->performingEstimation = false;

	for (int b=0;b<numBankedTests;b++) {
		bestModel->WriteBody(bodyFile);
		bankedMotorPrograms[b]->Write(brainFile);
	}

	delete bestModel;
	bestModel = NULL;

	bodyFile->close();
	delete bodyFile;
	bodyFile = NULL;
	simParams->FileRename(TEMP_FILENAME,BODY_OUT_FILENAME);

	brainFile->close();
	delete brainFile;
	brainFile = NULL;
	simParams->FileRename(TEMP2_FILENAME,NN_OUT_FILENAME);
}

void EE::WriteKillSignal(void) {

	ofstream *bodyFile;
	ofstream *brainFile;

	bodyFile = new ofstream(TEMP_FILENAME);

	(*bodyFile) << "-1\n";
	
	bodyFile->close();
	delete bodyFile;
	bodyFile = NULL;

	simParams->FileRename(TEMP_FILENAME,BODY_OUT_FILENAME);

	brainFile = new ofstream(TEMP2_FILENAME);

	(*brainFile) << "-1\n";
	
	brainFile->close();
	delete brainFile;
	brainFile = NULL;

	simParams->FileRename(TEMP2_FILENAME,NN_OUT_FILENAME);
}

#endif