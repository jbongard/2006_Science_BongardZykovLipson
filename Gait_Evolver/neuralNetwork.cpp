/* ---------------------------------------------------
   FILE:     bodyPlan.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 2, 2000
	FUNCTION: This class contains all information for
				 a single physical segment of an organism
				 in the MathEngine environment.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _NEURAL_NETWORK_CPP
#define _NEURAL_NETWORK_CPP

#include "math.h"
#include "neuralNetwork.h"
#include "simParams.h"

extern SIM_PARAMS *simParams;
extern char		  TEMP_FILENAME[100];
extern double     MUTATION_BIAS;
extern int		  NUM_ANGLE_SENSORS;
extern double	  MIN_RANGE_OF_MOTION;
extern double	  MAX_RANGE_OF_MOTION;
extern int		  MOTOR_PROGRAM_CYCLES;

NEURAL_NETWORK::NEURAL_NETWORK(int numJoints) {

	motorCommands = new MATRIX(8,8,0.0);

	for (int i=0;i<8;i++)

		for (int j=0;j<8;j++)
			
			motorCommands->Set(i,j,simParams->Rand(0.0,1.0));
}

NEURAL_NETWORK::NEURAL_NETWORK(NEURAL_NETWORK *nn) {

	motorCommands = new MATRIX(nn->motorCommands);
}

NEURAL_NETWORK::~NEURAL_NETWORK(void) {

	delete motorCommands;
	motorCommands = NULL;
}

void NEURAL_NETWORK::Mutate(void) {

	int i = simParams->RandInt(0,3);
	int j = simParams->RandInt(0,7);

	// Change a value
	if ( simParams->FlipCoin() ) {

		if ( simParams->FlipCoin() ) {

			double bias = simParams->Rand(0.0,MUTATION_BIAS) - MUTATION_BIAS;

			if ( simParams->Rand(0.0,1.0) < 0.5 )
				motorCommands->Set(i,j,motorCommands->Get(i,j) * (1.0 - exp( bias )) );
			else
				motorCommands->Set(i,j,motorCommands->Get(i,j) * (1.0 + exp( bias )) );
		}
		else
			motorCommands->Set(i,j,simParams->Rand(0.0,1.0));

		if ( motorCommands->Get(i,j) < 0.0 )
			motorCommands->Set(i,j,0.0);
		
		else if ( motorCommands->Get(i,j) > 1.0 )
			motorCommands->Set(i,j,1.0);
	}
	// Copy a value
	else {
		if ( i==0 )
			motorCommands->Set(i+1,j,motorCommands->Get(i,j));

		else if ( i==(3) )
			motorCommands->Set(i-1,j,motorCommands->Get(i,j));

		else
			if ( simParams->FlipCoin() )
				motorCommands->Set(i+1,j,motorCommands->Get(i,j));
			else
				motorCommands->Set(i-1,j,motorCommands->Get(i,j));
	}
}

void NEURAL_NETWORK::Print(void) {

	motorCommands->Print();
}

void NEURAL_NETWORK::Write(char *fileName) {

	/*
	ofstream *outFile = new ofstream(TEMP_FILENAME);

	(*outFile) << "1\n";

	Write(outFile);

	outFile->close();
	delete outFile;
	outFile = NULL;

	char command[200];

	sprintf(command,"rename %s %s",TEMP_FILENAME,fileName);

	system(command);
	*/

}

void NEURAL_NETWORK::Write(ofstream *outFile) {

	for (int j=0;j<motorCommands->width;j++)
		
		(*outFile) << simParams->Scale(motorCommands->Get(0,j),0.0,1.0,MIN_RANGE_OF_MOTION,MAX_RANGE_OF_MOTION) << " ";

	(*outFile) << "\n";

	//motorCommands->Write(outFile);
}

#endif