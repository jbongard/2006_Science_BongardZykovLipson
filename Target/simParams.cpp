/* ---------------------------------------------------
   FILE:     simParams.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 5, 2000
	FUNCTION: This class contains all miscellaneous
				 data and functions for this simulation.
 -------------------------------------------------- */

#include "stdafx.h"
#include "stdio.h"

#ifndef _SIM_PARAMS_CPP
#define _SIM_PARAMS_CPP

#include "simParams.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

extern int	OUTPUT_FOOTPRINTS;
extern int	OUTPUT_TRAJECTORY;
extern int	OUTPUT_DEBRIS;
extern int  CREATE_PLUME;
extern double DISP_DIST;
extern int  ADD_NOISE;
extern int	NOISE_TRIALS;
extern int  CURRENT_DEBRIS_TYPE;
extern int  DEFAULT_EVALUATION_PERIOD;

extern char BODY_FILENAME[100];
extern char BRAIN_FILENAME[100];

extern char TEMP_FILENAME[100];
extern char TEMP2_FILENAME[100];

extern char SENSOR_FILENAME[100];
extern char OBJECT_FILENAME[100];

SIM_PARAMS::SIM_PARAMS(int argc, char **argv) {

	rendererCreated = false;
	evalOver = false;
	functionAsReader = false;
	fitnessReported = false;
	showNetwork = false;
	displacementDistance = DISP_DIST;
	rendering = true;
	useTransparency = false;
	drawNetwork = false;
	evalPeriod = DEFAULT_EVALUATION_PERIOD;
	evaluating = false;
	internalTimer = 0;
	numSensors = 0;
	currentTest = 0;
	connectedObjects = NULL;
	agentExploding = false;
	testForExplosions = false;
	recordMovie = false;
	currFrame = 0;

	brainFile = NULL;
	bodyFile = NULL;

	randSeed = 0;
	noiseIteration = 0;
	debrisType = CURRENT_DEBRIS_TYPE;

	motorCommands = NULL;

	strcpy(envFileName,"env.dat");

	ParseParameters(argc,argv);

	OpenOutputFiles();

	srand(randSeed);
}

SIM_PARAMS::~SIM_PARAMS(void) {

	delete connectedObjects;
	connectedObjects = NULL;
}

double SIM_PARAMS::AngleBetween(double *v1, double *v2) {

	return( acos( (Dot(v1,v2)) / (MagnitudeOfVector(v1)*MagnitudeOfVector(v2)) ) );
}

void  SIM_PARAMS::CloseFiles(void) {

	bodyFile->close();
	delete bodyFile;
	bodyFile = NULL;

	brainFile->close();
	delete brainFile;
	brainFile = NULL;

	FileDelete(BODY_FILENAME);
	FileDelete(BRAIN_FILENAME);

	sensorFile->close();
	delete sensorFile;
	sensorFile = NULL;

	FileRename(TEMP_FILENAME,SENSOR_FILENAME);
}

void  SIM_PARAMS::CloseOutputFiles(void) {

	if ( torqueFile != NULL ) {
		torqueFile->close();
		delete torqueFile;
		torqueFile = NULL;
	}

	if ( hiddenLayerFile != NULL ) {
		hiddenLayerFile->close();
		delete hiddenLayerFile;
		hiddenLayerFile = NULL;
	}

	if ( footprintFile != NULL ) {
		footprintFile->close();
		delete footprintFile;
		footprintFile = NULL;
	}

	if ( plumeFile != NULL ) {
		plumeFile->close();
		delete plumeFile;
		plumeFile = NULL;
	}

	if ( trajectoryFile != NULL ) {
		trajectoryFile->close();
		delete trajectoryFile;
		trajectoryFile = NULL;
	}

	if ( debrisFile != NULL ) {
		debrisFile->close();
		delete debrisFile;
		debrisFile = NULL;
	}

	if ( (noiseIteration==(NOISE_TRIALS*7+1)) && (noiseFile != NULL) ) {
		noiseFile->close();
		delete noiseFile;
		noiseFile = NULL;
		exit(0);
	}
}

void   SIM_PARAMS::Cross(double *v1, double *v2, double *v3) {

	v3[0] = (v1[1]*v2[2] - v1[2]*v2[1]);
	v3[1] = (v1[0]*v2[2] - v1[2]*v2[0]); 
	v3[2] = (v1[0]*v2[1] - v1[1]*v2[0]);
}

void   SIM_PARAMS::DestroyMotorCommands(void) {

	int t;

	if ( motorCommands != NULL ) {

		for (t=0;t<numTests;t++) {
			delete motorCommands[t];
			motorCommands[t] = NULL;
		}

		delete[] motorCommands;
		motorCommands = NULL;
	}
}

double SIM_PARAMS::DistBetween(double *v1, double *v2) {

	double dist = sqrt(	(v1[0]-v2[0])*(v1[0]-v2[0]) +
								(v1[1]-v2[1])*(v1[1]-v2[1]) +
								(v1[2]-v2[2])*(v1[2]-v2[2]) );
	return(dist);
}

double SIM_PARAMS::Dot(double *v1, double *v2) {

	return( v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2] );
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

int    SIM_PARAMS::FlipCoin(void) {

	return( Rand(0.0,1.0) > 0.5 );
}

void SIM_PARAMS::GetMotorCommands(void) {

	int t;

	motorCommands = new MATRIX * [numTests];

	for (t=0;t<numTests;t++)

		motorCommands[t] = new MATRIX(brainFile);
}

double SIM_PARAMS::MagnitudeOfVector(double *vect) {

	return( sqrt( vect[0]*vect[0] + vect[1]*vect[1] + vect[2]*vect[2] ) );
}

void   SIM_PARAMS::Normalize(double *v) {

	double length = MagnitudeOfVector(v);
	v[0] /= length;
	v[1] /= length;
	v[2] /= length;
}

void   SIM_PARAMS::OpenFiles(void) {

	bodyFile = new ifstream(BODY_FILENAME);

	brainFile = new ifstream(BRAIN_FILENAME);

	(*bodyFile) >> agentsToEvaluate;

	(*brainFile) >> numTests;

	if ( agentsToEvaluate<0 )
		exit(0);

	sensorFile = new ofstream(TEMP_FILENAME);

	(*sensorFile) << agentsToEvaluate << "\n";
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

void SIM_PARAMS::SetDefaultRotation(double *R) {

	R[0] = 1.0;
	R[1] = 0.0;
	R[2] = 0.0;

	R[4] = 0.0;
	R[5] = 1.0;
	R[6] = 0.0;

	R[8] = 0.0;
	R[9] = 0.0;
	R[10] = 1.0;
}

void SIM_PARAMS::VectAdd(double *v1, double *v2, double *v3) {

	v3[0] = v1[0] + v2[0];
	v3[1] = v1[1] + v2[1];
	v3[2] = v1[2] + v2[2];
}

void SIM_PARAMS::VectMult(double *v, double amt) {

	v[0] = v[0]*amt;
	v[1] = v[1]*amt;
	v[2] = v[2]*amt;
}

void SIM_PARAMS::VectSub(double *v1, double *v2, double *v3) {

	v3[0] = v1[0] - v2[0];
	v3[1] = v1[1] - v2[1];
	v3[2] = v1[2] - v2[2];
}

void SIM_PARAMS::WriteToNoiseFile(double fit) {

	if ( noiseIteration == 0 ) {
		for (int i=0;i<NOISE_TRIALS;i++)
			(*noiseFile) << fit << ";";
		(*noiseFile) << "\n";
	}
	else {
		(*noiseFile) << fit << ";";
		if ( (noiseIteration%NOISE_TRIALS) == 0 )
			(*noiseFile) << "\n";
	}
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void SIM_PARAMS::OpenOutputFiles(void) {

	if ( OUTPUT_FOOTPRINTS ) {
		char fileName[30];
		strcpy(fileName,"data/footprints.dat");
		footprintFile = new ofstream(fileName);
	}
	else
		footprintFile = NULL;

	if ( CREATE_PLUME && functionAsReader ) {
		char fileName[30];
		strcpy(fileName,"data/plumeData.dat");
		plumeFile = new ofstream(fileName);
	}
	else
		plumeFile = NULL;
	
	if ( ADD_NOISE && functionAsReader ) {
		char fileName[30];
		sprintf(fileName,"data/run%d_noise.dat",randSeed);
		noiseFile = new ofstream(fileName);
	}
	else
		noiseFile = NULL;

	if ( OUTPUT_TRAJECTORY ) {
		char fileName[30];
		sprintf(fileName,"data/run%d_trajectory.dat",randSeed);
		trajectoryFile = new ofstream(fileName);
	}
	else
		trajectoryFile = NULL;

	if ( OUTPUT_DEBRIS && functionAsReader ) {
		char fileName[30];
		sprintf(fileName,"data/run%d_debris.dat",randSeed);
		debrisFile = new ofstream(fileName);
	}
	else
		debrisFile = NULL;

	torqueFile = NULL;
	hiddenLayerFile = NULL;
}

void  SIM_PARAMS::ParseParameters(int argc, char **argv) {

	int currParam;

	for(currParam=0;currParam<argc;currParam++) {

		if ( strcmp(argv[currParam],"-a") == 0 ) {
			functionAsReader = true;
			strcpy(genomeFileName,"data/");
			strcat(genomeFileName,argv[currParam+1]);
		}

		if ( strcmp(argv[currParam],"-null") == 0 )
			rendering = false;

		if ( strcmp(argv[currParam],"-r") == 0 )
			randSeed = atoi(argv[currParam+1]);

		if ( strcmp(argv[currParam],"-l") == 0 ) {
			char temp[50];

			strcpy(temp,argv[currParam+1]);
			sprintf(envFileName,"Creatures/%s",temp);
		}

		if ( strcmp(argv[currParam],"-d") == 0 )
			debrisType = atoi(argv[currParam+1]);

		if ( strcmp(argv[currParam],"-h") == 0 )
			PrintHelp();

		if ( strcmp(argv[currParam],"-?") == 0 )
			PrintHelp();

		if ( strcmp(argv[currParam],"-n") == 0 )
			showNetwork = true;
	}
}

void SIM_PARAMS::PrintHelp(void) {

	int i;
	
	for (i=0;i<40;i++)
		printf("\n");

	printf("-------------------------------------\n");
	printf("MorphEngine version 1.0\n");
	printf("-------------------------------------\n");
	printf("Author: Josh Bongard\n");
	printf("Release Date: April 3, 2001\n\n");
	printf("Options:\n");
	printf("-a [filename] \t Analyze an evolved creature stored in data\\[filename]\n");
	printf("-h -? \t\t This screen\n");
	printf("-l [filename] \t Use the creature specified in [filename]\n");
	printf("-n \t\t Show neural network behaviour\n");
	printf("-r [integer] \t Start evolution with the provided random seed.\n");
	printf("-null \t\t Perform evolution with the renderer turned off.\n");
	printf("-wireframe \t Visualize using wireframe.\n");
	printf("-------------------------------------\n");
	char ch = getchar();
	exit(0);
}

#endif