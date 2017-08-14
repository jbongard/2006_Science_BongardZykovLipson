/* ---------------------------------------------------
   FILE:     simParams.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 5, 2000
	FUNCTION: This class contains all miscellaneous
				 data and functions for this simulation.
 -------------------------------------------------- */

#include "stdafx.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#ifndef _SIM_PARAMS_CPP
#define _SIM_PARAMS_CPP

#include "simParams.h"

extern int		DEFAULT_DATASET_SIZE;
extern int		RANDOM_SEED;
extern int		DEFAULT_ALPHABET_SIZE;
extern int		DEFAULT_NUM_STATES;
extern int		DEFAULT_STRING_LENGTH;
extern double   CYLINDER_DIAMETER;
extern char		DATA_DIRECTORY[100];
extern char		EST_FILENAME[100];
extern int		MAX_TARGET_TRIALS;
extern int		NUM_ANGLE_SENSORS;
extern int		NUM_EST_GENERATIONS;
extern int		MODELS_FOR_DISAGREEMENT;

SIM_PARAMS::SIM_PARAMS(int argc, char **argv) {

	randSeed = RANDOM_SEED;
	loadFromFile = false;
	isEvaluator = false;
	currentCycle = 0;
	totalCycles = MAX_TARGET_TRIALS;
	performingIntelligentTesting = false;
	totalModelEvals = 0;
	totalTargetEvals = 0;
	performingBatch = false;
	perturbModel = false;
	usePareto = false;
	usePhysicalMachine = false;
	useTestBank = false;
	hideAttachments = false;
	reliabilityWeight = 0.5;
	noPrinting = false;
	numEstGenerations = NUM_EST_GENERATIONS;

	InitializeAllTests();

	ParseParameters(argc,argv);

	srand(randSeed);

	CreateTargetMatrix();
}

SIM_PARAMS::~SIM_PARAMS(void) {

	DestroyAllTests();

	WriteKillFile();
}

void  SIM_PARAMS::CloseDataFiles(void) {

	bodyPhylogenyFile->close();
	delete bodyPhylogenyFile;
	bodyPhylogenyFile = NULL;

	stickPhylogenyFile->close();
	delete stickPhylogenyFile;
	stickPhylogenyFile = NULL;
}

void  SIM_PARAMS::CreateTargetMatrix(void) {

	targetMatrix = new MATRIX(5,3,0);

	targetMatrix->Set(0,0,0);
	targetMatrix->Set(0,1,0.2);
	targetMatrix->Set(0,2,0);

	targetMatrix->Set(1,0,0);
	targetMatrix->Set(1,1,0.2);
	targetMatrix->Set(1,2,1.0);

	targetMatrix->Set(2,0,0);
	targetMatrix->Set(2,1,0.2);
	targetMatrix->Set(2,2,-1.0);

	targetMatrix->Set(3,0,-0.4);
	targetMatrix->Set(3,1,0.2);
	targetMatrix->Set(3,2,0.5);

	targetMatrix->Set(4,0,0.4);
	targetMatrix->Set(4,1,0.2);
	targetMatrix->Set(4,2,0.5);
}

double SIM_PARAMS::DefaultValueForJoint(int joint) {

	if ( joint<4 )
		return( 90.0 );
	else
		return( 128.0 );
}

void  SIM_PARAMS::DestroyAllTests(void) {

	delete allTests;
	allTests = NULL;
}

int   SIM_PARAMS::EEAToVictorJointIndex(int EEAIndex) {

	switch ( EEAIndex ) {
	case 0:
		return( 1 );
		break;
	case 1:
		return( 3 );
		break;
	case 2:
		return( 5 );
		break;
	case 3:
		return( 7 );
		break;
	case 4:
		return( 0 );
		break;
	case 5:
		return( 2 );
		break;
	case 6:
		return( 4 );
		break;
	case 7:
		return( 6 );
		break;
	}

	return( -1 );
}

void  SIM_PARAMS::FileDelete(char *fileName) {

	char command[100];

	sprintf(command,"del %s",fileName);

	system(command);
}

int   SIM_PARAMS::FileExists(char *fileName) {

	int exists;

	ifstream *inFile = new ifstream(fileName,ios::nocreate);

	if ( inFile->good() )
		exists = true;

	else
		exists = false;

	inFile->close();
	delete inFile;
	inFile = NULL;

	return( exists );
}

void SIM_PARAMS::FileRename(char *src, char *dest) {

	char command[200];

	sprintf(command,"rename %s %s",src,dest);

	system(command);
}

int SIM_PARAMS::FlipCoin(void) {

	return( Rand(0.0,1.0) < 0.5 );
}

ofstream *SIM_PARAMS::GetOutFile(char *fileName) {

	ofstream *outFile = new ofstream(fileName);

	return( outFile );
}

int SIM_PARAMS::GetRegimeIndex(void) {

	if ( performingBatch )
		return( 0 );

	else if ( !performingIntelligentTesting )
		return( 1 );

	else
		return( 2 );
}

void  SIM_PARAMS::InitializeAllTests(void) {

	int i,j;

	totalTests = 0;

	for (i=0;i<NUM_ANGLE_SENSORS;i++)
		totalTests++;

	for (i=0;i<NUM_ANGLE_SENSORS-1;i++)

		for (j=i+1;j<NUM_ANGLE_SENSORS;j++)

			totalTests++;

	allTests = new MATRIX(totalTests,3,0.0);

	totalTests = 0;

	for (i=0;i<NUM_ANGLE_SENSORS;i++) {
		allTests->Set(totalTests,0,i);
		allTests->Set(totalTests,1,-1);
		totalTests++;
	}

	for (i=0;i<NUM_ANGLE_SENSORS-1;i++)

		for (j=i+1;j<NUM_ANGLE_SENSORS;j++) {

			allTests->Set(totalTests,0,i);
			allTests->Set(totalTests,1,j);
			totalTests++;
		}
}

void  SIM_PARAMS::OpenDataFiles(void) {

	char fileName[100];

	sprintf(fileName,"%s_%d_%d_bodies.dat",EST_FILENAME,randSeed,GetRegimeIndex());

	bodyPhylogenyFile = new ofstream(fileName);

	bodyPhylogenyFile->close();
	delete bodyPhylogenyFile;
	bodyPhylogenyFile = NULL;

	for (int i=0;i<MODELS_FOR_DISAGREEMENT;i++) {

		sprintf(fileName,"%s_%d_%d_sticks%d.dat",EST_FILENAME,randSeed,GetRegimeIndex(),i);

		stickPhylogenyFile = new ofstream(fileName);

		stickPhylogenyFile->close();
		delete stickPhylogenyFile;
		stickPhylogenyFile = NULL;
	}
}

void  SIM_PARAMS::ParseParameters(int argc, char **argv) {

	int currParam;

	for(currParam=0;currParam<argc;currParam++) {

		if ( strcmp(argv[currParam],"-r") == 0 )
			randSeed = atoi(argv[currParam+1]);

		if ( strcmp(argv[currParam],"-c") == 0 )
			totalCycles = atoi(argv[currParam+1]);

		if ( strcmp(argv[currParam],"-g") == 0 )
			numEstGenerations = atoi(argv[currParam+1]);

		if ( strcmp(argv[currParam],"-l") == 0 )
			loadFromFile = true;

		if ( strcmp(argv[currParam],"-it") == 0 )
			performingIntelligentTesting = true;

		if ( strcmp(argv[currParam],"-b") == 0 )
			performingBatch = true;

		if ( strcmp(argv[currParam],"-tb") == 0 )
			useTestBank = true;

		if ( strcmp(argv[currParam],"-ha") == 0 )
			hideAttachments = true;

		if ( strcmp(argv[currParam],"-ph") == 0 )
			usePhysicalMachine = true;

		if ( strcmp(argv[currParam],"-null") == 0 )
			noPrinting = true;
	}
}

double SIM_PARAMS::Rand(double min, double max) {

	double zeroToOne = ((double)rand()) / RAND_MAX;
	double returnVal;

	returnVal = (zeroToOne * (max-min)) + min;
	return returnVal;
}

int SIM_PARAMS::RandInt(int min, int max) {

	if ( min == max )
		return( min );
	else
		return( (rand() % (max-min+1)) + min );
}

double SIM_PARAMS::Scale(double value, double min1, double max1,
								 double min2, double max2) {

	if ( min1 < 0 )
		value = value - min1;
	else
		value = value + min1;

	return( (value*(max2-min2)/(max1-min1)) + min2 );
}

double SIM_PARAMS::ServoToAngle(int servoVal, int currJoint) {

	double angle;

	if ( (currJoint==0) || (currJoint==2) || (currJoint==4) || (currJoint==6) ) {
		// Knee joints
		angle = Scale(double(servoVal),1.0,255.0,-95.0,95.0)*2.12;
	}
	else {
		// Hip joints
		angle = 0.0;

	}

	return( angle );
}

int SIM_PARAMS::VictorToEEAJointIndex(int victorIndex) {

	int eeaIndex;

	switch ( victorIndex ) {
	case 0:
		eeaIndex = 4;
		break;
	case 1:
		eeaIndex = 0;
		break;
	case 2:
		eeaIndex = 5;
		break;
	case 3:
		eeaIndex = 1;
		break;
	case 4:
		eeaIndex = 6;
		break;
	case 5:
		eeaIndex = 2;
		break;
	case 6:
		eeaIndex = 7;
		break;
	case 7:
		eeaIndex = 3;
		break;
	}

	return( eeaIndex );
}

void SIM_PARAMS::WaitForFile(char *fileName) {

	while ( !FileExists(fileName) );
}

void SIM_PARAMS::WriteBodyDifference(MATRIX *sticks, MATRIX *stickPos) {

	MATRIX *hisBody = new MATRIX(5,3,0);

	double lowestPoint = min(stickPos->MinValInColumn(2),stickPos->MinValInColumn(3));

	for (int i=0;i<4;i++) {

		hisBody->Set(i,0,(stickPos->Get(i,0)+stickPos->Get(i,1))/2.0);
		hisBody->Set(i,1,(stickPos->Get(i,2)+stickPos->Get(i,3))/2.0 - lowestPoint + CYLINDER_DIAMETER + 0.1);
		hisBody->Set(i,2,(stickPos->Get(i,4)+stickPos->Get(i,5))/2.0);
	}

	delete hisBody;
	hisBody = NULL;
}

void SIM_PARAMS::WriteKillFile(void) {

	char fileName[100];

	sprintf(fileName,"%s_%d_%d_end.dat",EST_FILENAME,randSeed,GetRegimeIndex());

	ofstream *outFile = new ofstream(fileName);

	outFile->close();
	delete outFile;
	outFile = NULL;
}

#endif