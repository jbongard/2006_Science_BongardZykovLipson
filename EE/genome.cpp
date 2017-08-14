/* ---------------------------------------------------
   FILE:     genome.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 2, 2000
	FUNCTION: This class contains all information for
				 a single physical segment of an organism
				 in the MathEngine environment.
 -------------------------------------------------- */

#include "stdafx.h"
#include "math.h"

#ifndef _GENOME_CPP
#define _GENOME_CPP

#include "genome.h"
#include "matrix.h"
#include "simParams.h"

extern SIM_PARAMS *simParams;

extern int	NUM_STICKS;
extern int	RAW_WIDTH;

extern double MIN_LENGTH;
extern double MAX_LENGTH;
extern double CYLINDER_DIAMETER;

extern int EVAL_PERIOD;
extern int OBJECT_TO_MOVE;

extern char			FILES_LOCATION[100];
extern char         TEMP_FILENAME[100];
extern char			BODY_OUT_FILENAME[100];
extern char			NN_OUT_FILENAME[100];

extern char			SENSOR_FILENAME[100];

extern int NUM_TOUCH_SENSORS;
extern int NUM_ANGLE_SENSORS;

extern int HEADER_LENGTH;
extern int WINDOW_LENGTH;

extern int NUM_SENSORS;
extern int NUM_HIDDEN;
extern int NUM_MOTORS;

extern double Z_OFFSET;
extern double DANGLER_LENGTH;

extern double MAX_MASS_PER_PART;
extern double MUTATION_BIAS;

GENOME::GENOME(int myID) {

	ID = myID;
	
	fitness = 0.0;

	if ( simParams->performingEstimation )
		
		CreateEstimationGenome();

	else

		CreateExplorationGenome();

	parentsWorstFitness = 10000.0;
}

GENOME::GENOME(MATRIX *toSow, int myID) {

	ID = myID;
	
	fitness = 0.0;

	if ( simParams->performingEstimation )
		
		CreateEstimationGenome(toSow);

	else

		CreateExplorationGenome();

	parentsWorstFitness = 10000.0;
}

GENOME::GENOME(GENOME *parent) {

	ID = 0;
	fitness = 0.0;

	if ( simParams->performingEstimation )

		CreateEstimationGenome(parent);

	else
		CreateExplorationGenome(parent);

	parentsWorstFitness = parent->fitness;
}

GENOME::~GENOME(void) {

	if ( raw ) {
		delete raw;
		raw = NULL;
	}
	
	if ( nn ) {
		delete nn;
		nn = NULL;
	}
}

void GENOME::ComputeFitness(MATRIX *targetSensorData, ifstream *inFile) {

	MATRIX *mySensorData = new MATRIX(inFile);

	if ( mySensorData->Get(0,0) == 100.0 ) {

		//fitness = 100.0 + mySensorData->Get(0,1);
		fitness = fitness + 100.0 + mySensorData->Get(0,1);
	}
	else {

		double temp;

		temp =
		//mySensorData->GetTotalSpeed() +
		//fabs( targetSensorData->GetTouch(0) - mySensorData->GetTouch(0) ) +
		//fabs( targetSensorData->GetTouch(1) - mySensorData->GetTouch(1) ) +
		//fabs( targetSensorData->GetTouch(2) - mySensorData->GetTouch(2) ) +
		//fabs( targetSensorData->GetTouch(3) - mySensorData->GetTouch(3) ) +
		fabs( targetSensorData->GetLeftRightTiltReading() - mySensorData->GetLeftRightTiltReading() ) +
		fabs( targetSensorData->GetForwardBackTiltReading() - mySensorData->GetForwardBackTiltReading() ) +
		fabs( targetSensorData->GetDistanceReading() - mySensorData->GetDistanceReading() );

		//if ( temp > fitness )
		//	fitness = temp;
		fitness = fitness + temp;
	}

	delete mySensorData;
	mySensorData = NULL;
}

void GENOME::ConvertPartiallyToTarget(void) {

//	double eta = 0.01*pow(1.5,0.0);

	sticks->Set(0,2,1.4);
	sticks->Set(0,3,1.4);
	sticks->Set(0,4,0.5);
	sticks->Set(0,6,0);
	sticks->Set(0,7,0.5);

	// Upper legs

	if ( !simParams->hideAttachments ) {
		sticks->Set(1,0,0);
		sticks->Set(2,0,0);
		sticks->Set(3,0,0);
		sticks->Set(4,0,0);
	}

	//sticks->Set(1,1,0.125);
	//sticks->Set(2,1,0.375);
	//sticks->Set(3,1,0.625);
	//sticks->Set(4,1,0.875);
	//sticks->Set(1,1,simParams->Rand(0.125-eta,0.125+eta));
	//sticks->Set(2,1,simParams->Rand(0.375-eta,0.375+eta));
	//sticks->Set(3,1,simParams->Rand(0.625-eta,0.625+eta));
	//sticks->Set(4,1,simParams->Rand(0.875-eta,0.875+eta));

	sticks->Set(1,2,0.95); sticks->Set(1,3,CYLINDER_DIAMETER); sticks->Set(1,4,CYLINDER_DIAMETER);
	sticks->Set(2,2,0.95); sticks->Set(2,3,CYLINDER_DIAMETER); sticks->Set(2,4,CYLINDER_DIAMETER);
	sticks->Set(3,2,0.95); sticks->Set(3,3,CYLINDER_DIAMETER); sticks->Set(3,4,CYLINDER_DIAMETER);
	sticks->Set(4,2,0.95); sticks->Set(4,3,CYLINDER_DIAMETER); sticks->Set(4,4,CYLINDER_DIAMETER);

	sticks->Set(1,5,0.0);			sticks->Set(1,8,3.14159/2.0);
	sticks->Set(2,5,3.14159/2.0);	sticks->Set(2,8,0.0);
	sticks->Set(3,5,3.14159);		sticks->Set(3,8,0.0);
	sticks->Set(4,5,-3.14159/2.0);	sticks->Set(4,8,0.0);

	sticks->Set(1,6,0);
	sticks->Set(2,6,0);
	sticks->Set(3,6,0);
	sticks->Set(4,6,0);

	sticks->Set(1,7,0.5);
	sticks->Set(2,7,1.143);
	sticks->Set(3,7,0.143);
	sticks->Set(4,7,0.143);

	// Lower legs

	if ( !simParams->hideAttachments ) {
		sticks->Set(5,0,1);
		sticks->Set(6,0,2);
		sticks->Set(7,0,3);
		sticks->Set(8,0,4);
	}

//	sticks->Set(5,1,0.375);
//	sticks->Set(6,1,0.375);
//	sticks->Set(7,1,0.375);
//	sticks->Set(8,1,0.375);
//	sticks->Set(5,1,simParams->Rand(0.375-eta,0.375+eta));
//	sticks->Set(6,1,simParams->Rand(0.375-eta,0.375+eta));
//	sticks->Set(7,1,simParams->Rand(0.375-eta,0.375+eta));
//	sticks->Set(8,1,simParams->Rand(0.375-eta,0.375+eta));

	sticks->Set(5,2,0.97); sticks->Set(5,3,CYLINDER_DIAMETER); sticks->Set(5,4,CYLINDER_DIAMETER);
	sticks->Set(6,2,0.97); sticks->Set(6,3,CYLINDER_DIAMETER); sticks->Set(6,4,CYLINDER_DIAMETER);
	sticks->Set(7,2,0.97); sticks->Set(7,3,CYLINDER_DIAMETER); sticks->Set(7,4,CYLINDER_DIAMETER);
	sticks->Set(8,2,0.97); sticks->Set(8,3,CYLINDER_DIAMETER); sticks->Set(8,4,CYLINDER_DIAMETER);

	sticks->Set(5,5,0.0);			sticks->Set(5,8,3.14159/2.0);
	sticks->Set(6,5,3.14159/2.0);	sticks->Set(6,8,0.0);
	sticks->Set(7,5,3.14159);		sticks->Set(7,8,0.0);
	sticks->Set(8,5,-3.14159/2.0);	sticks->Set(8,8,0.0);

	sticks->Set(5,6,1);
	sticks->Set(6,6,1);
	sticks->Set(7,6,1);
	sticks->Set(8,6,1);

	sticks->Set(5,7,0.118);
	sticks->Set(6,7,0.118);
	sticks->Set(7,7,0.118);
	sticks->Set(8,7,0.118);
}

void GENOME::ConvertToTarget(void) {

	ID = -1;

	sticks->Set(0,2,1.4);
	sticks->Set(0,3,1.4);
	sticks->Set(0,4,0.5);
	sticks->Set(0,6,0);
	sticks->Set(0,7,0.5);

	// Upper legs

	sticks->Set(1,0,0);
	sticks->Set(2,0,0);
	sticks->Set(3,0,0);
	sticks->Set(4,0,0);

	sticks->Set(1,1,0.125);
	sticks->Set(2,1,0.1875);
	sticks->Set(3,1,0.2083);
	sticks->Set(4,1,0.2188);

	sticks->Set(1,2,0.95); sticks->Set(1,3,CYLINDER_DIAMETER); sticks->Set(1,4,CYLINDER_DIAMETER);
	sticks->Set(2,2,0.95); sticks->Set(2,3,CYLINDER_DIAMETER); sticks->Set(2,4,CYLINDER_DIAMETER);
	sticks->Set(3,2,0.95); sticks->Set(3,3,CYLINDER_DIAMETER); sticks->Set(3,4,CYLINDER_DIAMETER);
	sticks->Set(4,2,0.95); sticks->Set(4,3,CYLINDER_DIAMETER); sticks->Set(4,4,CYLINDER_DIAMETER);

	sticks->Set(1,5,0.0);			sticks->Set(1,8,3.14159/2.0);
	sticks->Set(2,5,3.14159/2.0);	sticks->Set(2,8,0.0);
	sticks->Set(3,5,3.14159);		sticks->Set(3,8,0.0);
	sticks->Set(4,5,-3.14159/2.0);	sticks->Set(4,8,0.0);

	sticks->Set(1,6,0);
	sticks->Set(2,6,0);
	sticks->Set(3,6,0);
	sticks->Set(4,6,0);
	
	sticks->Set(1,7,0.5);
	sticks->Set(2,7,1.143);
	sticks->Set(3,7,0.143);
	sticks->Set(4,7,0.143);

	// Lower legs

	sticks->Set(5,0,1);
	sticks->Set(6,0,2);
	sticks->Set(7,0,3);
	sticks->Set(8,0,4);

	sticks->Set(5,1,0.275);
	sticks->Set(6,1,0.3958);
	sticks->Set(7,1,0.4821);
	sticks->Set(8,1,0.5469);

	sticks->Set(5,2,0.97); sticks->Set(5,3,CYLINDER_DIAMETER); sticks->Set(5,4,CYLINDER_DIAMETER);
	sticks->Set(6,2,0.97); sticks->Set(6,3,CYLINDER_DIAMETER); sticks->Set(6,4,CYLINDER_DIAMETER);
	sticks->Set(7,2,0.97); sticks->Set(7,3,CYLINDER_DIAMETER); sticks->Set(7,4,CYLINDER_DIAMETER);
	sticks->Set(8,2,0.97); sticks->Set(8,3,CYLINDER_DIAMETER); sticks->Set(8,4,CYLINDER_DIAMETER);

	sticks->Set(5,5,0.0);			sticks->Set(5,8,3.14159/2.0);
	sticks->Set(6,5,3.14159/2.0);	sticks->Set(6,8,0.0);
	sticks->Set(7,5,3.14159);		sticks->Set(7,8,0.0);
	sticks->Set(8,5,-3.14159/2.0);	sticks->Set(8,8,0.0);

	sticks->Set(5,6,1);
	sticks->Set(6,6,1);
	sticks->Set(7,6,1);
	sticks->Set(8,6,1);

	sticks->Set(5,7,0.118);
	sticks->Set(6,7,0.118);
	sticks->Set(7,7,0.118);
	sticks->Set(8,7,0.118);
}

void GENOME::Cross(GENOME *sibling) {

	//raw->SwapValues(sibling->raw);

	raw->SwapRows(sibling->raw);
}

void GENOME::Evaluate_Estimation(void) {

	CreateSticks();

	CreateStickPositions();

	ofstream *outFile;
	outFile = new ofstream(TEMP_FILENAME);

	WriteBody(outFile);

	outFile->close();
	delete outFile;
	outFile = NULL;

	char command[200];
	sprintf(command,"rename %s %s",TEMP_FILENAME,BODY_OUT_FILENAME);
	system(command);

	DestroyStickPositions();

	DestroySticks();

	//while ( !simParams->FileExists(SENSOR_FILENAME) );
	simParams->WaitForFile(SENSOR_FILENAME);
}

void GENOME::Evaluate_Exploration(void) {

	nn->Write(NN_OUT_FILENAME);
}

NEURAL_NETWORK *GENOME::GetController(void) {

	return( nn );
}

MATRIX *GENOME::GetModel(void) {

	return( raw );
}

void GENOME::Mutate(void) {

	if ( simParams->performingEstimation )

		Mutate_Estimation();

	else
		Mutate_Exploration();
}

void GENOME::Print(void) {

	printf("[ID: %d]\t[F: %8.8f]\n",ID,fitness);
	
}

void GENOME::PrintSticks(void) {

	CreateSticks();

	sticks->Print();

	DestroySticks();
}

void GENOME::PrintTest(void) {

	nn->Print();
	
}

void GENOME::Replace(GENOME *loser, int nextAvailableID) {

	if ( simParams->performingEstimation ) {

		for (int i=0;i<raw->length;i++)

			for (int j=0;j<raw->width;j++)

				loser->raw->Set(i,j, raw->Get(i,j) );
	}
	else {

		delete loser->nn;
		loser->nn = new NEURAL_NETWORK(nn);
	}

	loser->fitness   = 0.0;

	loser->ID = nextAvailableID;
}

void GENOME::Save(ofstream *outFile) {

	if ( simParams->performingEstimation ) {

		//CreateSticks();

		//ConvertPartiallyToTarget();

		//CreateStickPositions();

		WriteBody(outFile);

		//DestroyStickPositions();

		//DestroySticks();

	}

	else
		nn->Write(outFile);
}

void GENOME::SaveSticks(ofstream *outFile) {

	CreateSticks();

	ConvertPartiallyToTarget();

	CreateStickPositions();

	(*outFile) << sticks->length << " " << 7 << "\n";
	for (int i=0;i<NUM_STICKS;i++) {
		(*outFile) << sticks->Get(i,0) << " ";
		(*outFile) << (stickPos->Get(i,0)+stickPos->Get(i,1))/2.0 << " ";
		(*outFile) << (stickPos->Get(i,4)+stickPos->Get(i,5))/2.0 << " ";

		(*outFile) << stickPos->Get(i,0) << " ";
		(*outFile) << stickPos->Get(i,1) << " ";
		(*outFile) << stickPos->Get(i,4) << " ";
		(*outFile) << stickPos->Get(i,5) << "\n";

	}

	DestroyStickPositions();

	DestroySticks();
}

int GENOME::Valid(void) {

	/*
	CreateSticks();

	ConvertPartiallyToTarget();

	int valid = true;

	int i=0;
	int j;

	while ( (i<sticks->length-2) && valid ) {

		j=i+1;

		while ( (j<sticks->length-1) && valid ) {

			if ( (sticks->Get(i,0)==sticks->Get(j,0)) && 
				 (sticks->Get(i,1)==sticks->Get(j,1)) )
				valid = false;

			j++;
		}
		i++;
	}

	DestroySticks();

	return( valid );
	*/
	return( true );
}

int  GENOME::WorseThanParent(void) {

	return( fitness >= parentsWorstFitness );
}

void GENOME::WriteBody(ofstream *outFile) {

	CreateSticks();

	ConvertPartiallyToTarget();

	if ( simParams->perturbModel )
		PerturbModel();

	CreateStickPositions();

	//simParams->WriteBodyDifference(sticks,stickPos);

	(*outFile) << sticks->length + NUM_TOUCH_SENSORS   << " ";
	(*outFile) << (sticks->length-1) + NUM_TOUCH_SENSORS << "\n\n";
	//(*outFile) << sticks->length + 2 + 2     << " ";
	//(*outFile) << 0 + 2 << "\n\n";
	
	//double lowestPoint = sticks->MaxValInColumnContingentOn(4,6)/2.0;
	double lowestPoint = 0.4;

	for (int currObj=0;currObj<sticks->length;currObj++)
		WriteObject(currObj,outFile,lowestPoint);

	//WriteDanglers(outFile,lowestPoint);

	for (int currJoint=1;currJoint<sticks->length;currJoint++)
		WriteJoint(currJoint,outFile,lowestPoint);

	//WriteDanglerJoints(outFile,lowestPoint);

	WriteSuffix(outFile);

	(*outFile) << "\n";

	DestroyStickPositions();

	DestroySticks();

}

void GENOME::WriteTargetBody(ofstream *outFile) {

	CreateSticks();

	ConvertToTarget();

	CreateStickPositions();

	//simParams->WriteBodyDifference(sticks,stickPos);

	(*outFile) << "1\n";

	(*outFile) << sticks->length + NUM_TOUCH_SENSORS   << " ";
	(*outFile) << (sticks->length-1) + NUM_TOUCH_SENSORS << "\n\n";
	//(*outFile) << sticks->length + 2 + 2     << " ";
	//(*outFile) << 0 + 2 << "\n\n";
	
	//double lowestPoint = sticks->MaxValInColumnContingentOn(4,6)/2.0;
	double lowestPoint = 0.4;

	for (int currObj=0;currObj<sticks->length;currObj++)
		WriteObject(currObj,outFile,lowestPoint);

	for (int currJoint=1;currJoint<sticks->length;currJoint++)
		WriteJoint(currJoint,outFile,lowestPoint);

	WriteSuffix(outFile);

	(*outFile) << "\n";

	DestroyStickPositions();

	DestroySticks();

}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

int  GENOME::AgentInvalid(MATRIX *mySensorData) {

	int invalid = mySensorData->length == 1;

	if ( !invalid )
		invalid = mySensorData->Get(mySensorData->length-1,14) < 0.0;

	return( invalid );
}

void GENOME::ComputeJointNormal(int currJoint, double *x, double *y, double *z) {

	int firstBody = currJoint;
	//int secondBody = int(sticks->Get(firstBody,0));

	int secondBody;
	double placement;
	GetParentAndPlacement(firstBody,&secondBody,&placement);

	double myX = (stickPos->Get(firstBody,0)+stickPos->Get(firstBody,1))/2.0;
	double myY = (stickPos->Get(firstBody,2)+stickPos->Get(firstBody,3))/2.0;
	double myZ = (stickPos->Get(firstBody,4)+stickPos->Get(firstBody,5))/2.0;

	double hisX = (stickPos->Get(secondBody,0)+stickPos->Get(secondBody,1))/2.0;
	double hisY = (stickPos->Get(secondBody,2)+stickPos->Get(secondBody,3))/2.0;
	double hisZ = (stickPos->Get(secondBody,4)+stickPos->Get(secondBody,5))/2.0;

	double vectBetweenX = myX - hisX;
	double vectBetweenY = myY - hisY;
	double vectBetweenZ = myZ - hisZ;

	double vertVectX = 0.0;
	double vertVectY = 1.0;
	double vertVectZ = 0.0;

	(*x) = -(vectBetweenY*vertVectZ - vectBetweenZ*vertVectY);
	(*y) = -(vectBetweenZ*vertVectX - vectBetweenX*vertVectZ);
	(*z) = -(vectBetweenX*vertVectY - vectBetweenY*vertVectX);
}

void GENOME::ComputeMyStickData(int currStick, double *myAttach, double backLength,
								double frontLength) {

	double totalThetas[2];

	totalThetas[0] = GetCurrentTheta(currStick);

	/*
	stickPos->Set(currStick,0 , myAttach[0] - (backLength)  * sin(totalThetas[0]+3.14159) * cos(totalThetas[1]));
	stickPos->Set(currStick,1 , myAttach[0] - (frontLength) * sin(totalThetas[0]) * cos(totalThetas[1]));
   
	stickPos->Set(currStick,2 , myAttach[1] - (backLength)  * sin(totalThetas[0]+3.14159) * sin(totalThetas[1]));
	stickPos->Set(currStick,3 , myAttach[1] - (frontLength) * sin(totalThetas[0]) * sin(totalThetas[1]));
    
	stickPos->Set(currStick,4 , myAttach[2] - (backLength)  * cos(totalThetas[0]+3.14159));
	stickPos->Set(currStick,5 , myAttach[2] - (frontLength) * cos(totalThetas[0]));
	*/

	/*
	stickPos->Set(currStick,0 , myAttach[0] - (backLength)  * sin(totalThetas[0]+3.14159));
	stickPos->Set(currStick,1 , myAttach[0] - (frontLength) * sin(totalThetas[0]));
   
	stickPos->Set(currStick,2 , myAttach[1] - (backLength)  * 0.0);
	stickPos->Set(currStick,3 , myAttach[1] - (frontLength) * 0.0);
    
	stickPos->Set(currStick,4 , myAttach[2] - (backLength)  * cos(totalThetas[0]+3.14159));
	stickPos->Set(currStick,5 , myAttach[2] - (frontLength) * cos(totalThetas[0]));
	*/

	stickPos->Set(currStick,0 , myAttach[0]);
	stickPos->Set(currStick,1 , myAttach[0] + (backLength+frontLength)*sin(totalThetas[0]));
   
	stickPos->Set(currStick,2 , myAttach[1]);
	stickPos->Set(currStick,3 , myAttach[1]);
    
	stickPos->Set(currStick,4 , myAttach[2]);
	stickPos->Set(currStick,5 , myAttach[2] + (backLength+frontLength)*cos(totalThetas[0]));

	stickPos->Set(currStick,6 , totalThetas[0]);
}

void GENOME::ComputeRollingMean(MATRIX *targetSensorData, MATRIX *mySensorData) {

	fitness = fitness + targetSensorData->RollingMean(mySensorData,HEADER_LENGTH,WINDOW_LENGTH);
}

void GENOME::ConformToTarget(void) {

	sticks->Set(0,3,1.0);
	sticks->Set(1,3,1.0);
	sticks->Set(2,3,1.0);

	sticks->Set(1,0,0);
	sticks->Set(1,1,1);
	sticks->Set(1,2,0);

	sticks->Set(2,0,0);
	sticks->Set(2,1,0);
	sticks->Set(2,2,0);
}

void GENOME::CopyBodyPart(void) {

	int i1 = simParams->RandInt(1,raw->length-1);

	int i2 = simParams->RandInt(1,raw->length-1);

	int numberOfCopies = simParams->RandInt(1,raw->length-2);

	while ( numberOfCopies > 0 ) {

		while ( i2 == i1 )
			i2 = simParams->RandInt(1,raw->length-1);

		//raw->Set(i2,2,raw->Get(i1,2));
		raw->Set(i2,3,raw->Get(i1,3));
		//raw->Set(i2,4,raw->Get(i1,4));
		//raw->Set(i2,7,raw->Get(i1,7));

		i2 = simParams->RandInt(1,raw->length-1);

		numberOfCopies--;
	}
}

void GENOME::CreateEstimationGenome(void) {

	raw = new MATRIX(NUM_STICKS,RAW_WIDTH);

	sticks = NULL;
	stickPos = NULL;
	attachPos = NULL;

	nn = NULL;
}

void GENOME::CreateEstimationGenome(MATRIX *toSow) {

	raw = new MATRIX(toSow);

	sticks = NULL;
	stickPos = NULL;
	attachPos = NULL;

	nn = NULL;
}

void GENOME::CreateEstimationGenome(GENOME *parent) {

	raw = new MATRIX(parent->raw);

	sticks = NULL;
	stickPos = NULL;
	attachPos = NULL;

	nn = NULL;
}

void GENOME::CreateExplorationGenome(void) {

	raw = NULL;

	sticks = NULL;
	stickPos = NULL;
	attachPos = NULL;

	nn = new NEURAL_NETWORK(NUM_STICKS-1);
}

void GENOME::CreateExplorationGenome(GENOME *parent) {

	raw = NULL;

	sticks = NULL;
	stickPos = NULL;
	attachPos = NULL;

	nn = new NEURAL_NETWORK(parent->nn);
}

void GENOME::CreateSticks(void) {

	int i,j;

	sticks = new MATRIX(raw);

	for (i=0;i<sticks->length;i++) {

		for (j=0;j<sticks->width;j++) {

			if ( j==0 ) {

				if ( i==0 )
					sticks->Set(i,j,-1.0);
				else if ( i==1 )
					sticks->Set(i,j,0.0);
				else
					sticks->Set(i,j, floor(sticks->Get(i,j)*i) );

				if ( sticks->Get(i,j)>=i )
					sticks->Set(i,j,i-1);
			}
			else if ( j==1 ) {
				
				//sticks->Set(i,j, sticks->Get(i,j)*2.0*3.14159);

				//sticks->Set(i,j, floor(sticks->Get(i,j)*4.0) );
			}
			else if ( (j>=2) && (j<=4) ) {

				sticks->Set(i,j, sticks->Get(i,j)*(MAX_LENGTH-MIN_LENGTH)+MIN_LENGTH );

				if ( (i>0) && (j==3) )
					sticks->Mult(i,j,0.1);

				if ( (i>0) && (j==4) )
					sticks->Set(i,j,sticks->Get(i,3));
			}
			else if ( (j==5) || (j==8) )
				sticks->Set(i,j, sticks->Get(i,j)*3.14159*2.0);
		}
	}

	//sticks->MultColumn(3,0.1);

	FindSolidSticks();

	//FindActuatedSticks();

	//ConformToTarget();
}

void GENOME::CreateStickPositions(void) {

	stickPos = new MATRIX(sticks->length,7,0.0);
	attachPos = new MATRIX(sticks->length,3,0.0);

	for (int s=0;s<sticks->length;s++)

		SetStickPos(s);
}

void GENOME::DestroyStickPositions(void) {

	delete attachPos;
	attachPos = NULL;

	delete stickPos;
	stickPos = NULL;
}

void GENOME::DestroySticks(void) {

	delete sticks;
	sticks = NULL;
}

void GENOME::FindActuatedSticks(void) {

	int i;

	MATRIX *maxVals = new MATRIX(1,NUM_ANGLE_SENSORS,0.0);

	sticks->GetMaxValsInColumn(7,1,sticks->length-1,maxVals);

	for (i=0;i<sticks->length;i++) {

		if ( maxVals->In( sticks->Get(i,7) ) )
			sticks->Set(i,7,1.0);
		else
			sticks->Set(i,7,0.0);
	}

	delete maxVals;
	maxVals = NULL;
}

void GENOME::FindSolidSticks(void) {

	int i;
	int solidSticksFound = 0;

	MATRIX *maxVals = new MATRIX(1,NUM_TOUCH_SENSORS,0.0);

	//sticks->GetMaxValsInColumn(6,maxVals);
	sticks->GetMaxValsInColumn(6,1,NUM_STICKS-1,maxVals);

	sticks->Set(0,6,0.0);

	i = 1;
	while ( (i<sticks->length) && (solidSticksFound<NUM_TOUCH_SENSORS) ) {
	//for (i=0;i<sticks->length;i++) {

		if ( maxVals->In( sticks->Get(i,6) ) ) {
			sticks->Set(i,6,1.0);
			solidSticksFound++;
		}
		else
			sticks->Set(i,6,0.0);

		i++;
	}

	while ( i<sticks->length ) {
		sticks->Set(i,6,0.0);
		i++;
	}

	delete maxVals;
	maxVals = NULL;
}

double GENOME::GetCurrentTheta(int currStick) {

	double currTheta = 0.0;

	int parent;
	double placement;

	GetParentAndPlacement(currStick,&parent,&placement);

	if ( currStick > 0 ) {

		currTheta = (placement*2.0*3.14159) - (3.14159/4.0) - (3.14159/2.0);
	}

	if ( currStick > 0 )
		return( currTheta + GetCurrentTheta( parent ) );
	else
		return( currTheta );
}

void GENOME::GetHisStickData(int currStick, int *stickToAttachTo, double *hisAttach,
						double *hisLength, double *hisWidth, double *hisPos, double *hisThetas) {

//	(*stickToAttachTo) = int(sticks->Get(currStick,0));
//	(*hisAttach) = sticks->Get(currStick,1);

	GetParentAndPlacement(currStick,&(*stickToAttachTo),&(*hisAttach));

	(*hisLength) = sticks->Get((*stickToAttachTo),2);
	(*hisWidth) = sticks->Get((*stickToAttachTo),3);

	hisPos[0] = (stickPos->Get((*stickToAttachTo),0)+stickPos->Get((*stickToAttachTo),1))/2.0;
	hisPos[1] = (stickPos->Get((*stickToAttachTo),2)+stickPos->Get((*stickToAttachTo),3))/2.0;
	hisPos[2] = (stickPos->Get((*stickToAttachTo),4)+stickPos->Get((*stickToAttachTo),5))/2.0;

	hisThetas[0] = stickPos->Get((*stickToAttachTo),6);
	//hisThetas[0] = 0.0;
}

void GENOME::GetMyAttachmentPoint(double *myAttach, int stickToAttachTo, double *hisPos,
								  double hisLength, double hisWidth, double hisAttach, double *hisThetas, int currStick) {

	int    parent;
	double myRotation;
	
	GetParentAndPlacement(currStick,&parent,&myRotation);

	double scalingFactor;
	double whereOnSide;

	if ( myRotation <=0.25 ) {

		scalingFactor = myRotation*4.0;
		whereOnSide = (scalingFactor*hisLength) - (hisLength/2.0);

		myAttach[0] = - (hisWidth/2.0);
		myAttach[1] = hisPos[1];
		myAttach[2] = + whereOnSide;
	}
	else if ( myRotation <= 0.5 ) {

		scalingFactor = myRotation*4.0 - 1.0;
		whereOnSide = (scalingFactor*hisWidth) - (hisWidth/2.0);

		myAttach[0] = + whereOnSide;
		myAttach[1] = hisPos[1];
		myAttach[2] = + (hisLength/2.0);
	}
	else if ( myRotation <= 0.75 ) {

		scalingFactor = myRotation*4.0 - 2.0;
		whereOnSide = (scalingFactor*hisLength) - (hisLength/2.0);

		myAttach[0] = + (hisWidth/2.0);
		myAttach[1] = hisPos[1];
		myAttach[2] = - whereOnSide;
	}
	else {
		scalingFactor = myRotation*4.0 - 3.0;
		whereOnSide = (scalingFactor*hisWidth) - (hisWidth/2.0);

		myAttach[0] = - whereOnSide;
		myAttach[1] = hisPos[1];
		myAttach[2] = - (hisLength/2.0);
	}

	double x = myAttach[0];
	double y = myAttach[2];

	myAttach[0] = cos(-hisThetas[0])*x - sin(-hisThetas[0])*y;
	myAttach[2] = sin(hisThetas[0])*x + cos(hisThetas[0])*y;

	myAttach[0] = myAttach[0] + hisPos[0];
	myAttach[2] = myAttach[2] + hisPos[2];
}

void GENOME::GetMyStickData(int currStick, double *myPos, double *myLength,
							double *backLength, double *frontLength) {

	//(*myPos) = sticks->Get(currStick,2);
	(*myPos) = 0.0;

    (*myLength) = sticks->Get(currStick,2);

    //myThetas[0] = sticks->Get(currStick,6);
    //myThetas[0] = 0.0;
	
    (*backLength)  = (*myLength) * (1.0-(*myPos));
    (*frontLength) = (*myLength) * (*myPos);
}

void GENOME::GetParentAndPlacement(int currStick, int *parent, double *placement) {

	double temp = sticks->Get(currStick,1)*(currStick-1 + 0.9999);

	(*parent) = int( floor(temp) );
	(*placement) = temp - (*parent);
}

void GENOME::Mutate_Estimation(void) {

	if ( simParams->FlipCoin() )
		PointMutation();
	else
		CopyBodyPart();
}

void GENOME::Mutate_Exploration(void) {

	nn->Mutate();
}

void GENOME::PerturbModel(void) {

	int parent;
	double placement;

	for (int i=1;i<NUM_STICKS;i++) {

		GetParentAndPlacement(i,&parent,&placement);

		placement = placement*simParams->Rand(0.9,1.1);

		if ( placement < 0.0 )
			placement = 1.0 + placement;

		else if ( placement > 1.0 )
			placement = placement - 1.0;

		SetParentAndPlacement(i,parent,placement);

		/*
		sticks->Mult(i,1,simParams->Rand(0.9,1.1));

		if ( sticks->Get(i,1) < 0.0 )
			sticks->Set(i,1,1.0);

		else if ( sticks->Get(i,1) > 1.0 )
			sticks->Set(i,1,0.0);
		*/
	}
}

void GENOME::PointMutation(void) {

//	int i = simParams->RandInt(0,raw->length-1);
//	int j = simParams->RandInt(0,raw->width-1);
	int i = simParams->RandInt(1,NUM_STICKS-1);

	int j = 1;

	/*
	int whichProperty = simParams->RandInt(0,1);

	if ( whichProperty == 0 )
		j=0;
	else
		j=1;

	if ( !simParams->hideAttachments )
		j=1;
	*/

	double mutAction = simParams->Rand(0.0,1.0);

	if ( simParams->Rand(0.0,1.0) < 0.33 ) {

		double bias = simParams->Rand(0.0,MUTATION_BIAS) - MUTATION_BIAS;

		if ( simParams->Rand(0.0,1.0) < 0.5 )
			raw->Set(i,j,raw->Get(i,j) * (1.0 - exp( bias )) );
		else
			raw->Set(i,j,raw->Get(i,j) * (1.0 + exp( bias )) );
	}
	else if ( simParams->Rand(0.0,1.0) < 0.67 ) {
		
		double bias = simParams->Rand(0.0,MUTATION_BIAS) - MUTATION_BIAS;

		int i2 = simParams->RandInt(1,8);

		while ( i2==i )
			i2 = simParams->RandInt(1,8);

		if ( simParams->Rand(0.0,1.0) < 0.5 ) {
			raw->Set(i,j,raw->Get(i,j) + exp( bias ) );
			raw->Set(i2,j,raw->Get(i,j) - exp( bias ) );
		}
		else {
			raw->Set(i,j,raw->Get(i,j) - exp( bias ) );
			raw->Set(i2,j,raw->Get(i,j) + exp( bias ) );
		}

		if ( raw->Get(i2,j) < 0.0 )
			raw->Set(i2,j,0.0);
	
		else if ( raw->Get(i2,j) > 1.0 )
			raw->Set(i2,j,1.0);

	}
	else
		raw->Set(i,j,simParams->Rand(0.0,1.0));

	if ( raw->Get(i,j) < 0.0 ) {
		if ( (j==1) || (j==5) || (j==8) )
			raw->Set(i,j,1.0);
		else
			raw->Set(i,j,0.0);
	}

	else if ( raw->Get(i,j) > 1.0 ) {
		if ( (j==1) || (j==5) || (j==8) )
			raw->Set(i,j,0.0);
		else
			raw->Set(i,j,1.0);
	}
}

void GENOME::SetFirstStickPos(void) {

	double r;
	double theta[2];

	r = sticks->Get(0,2);

    //theta[0] = sticks->Get(0,6);
	theta[0] = 0.0;

	double myPos[3];

	/*
	myPos[0] = r * sin(theta[0]) * cos(theta[1]);
    myPos[1] = r * sin(theta[0]) * sin(theta[1]);
    myPos[2] = r * cos(theta[0]);
	*/
	myPos[0] = r * sin(theta[0]);
    myPos[1] = r * 0.0;
    myPos[2] = r * cos(theta[0]);

	stickPos->Set(0,0, -myPos[0]/2.0);
	stickPos->Set(0,1, myPos[0]/2.0);

	stickPos->Set(0,2, -myPos[1]/2.0);
	stickPos->Set(0,3, myPos[1]/2.0);

	stickPos->Set(0,4, -myPos[2]/2.0);
	stickPos->Set(0,5, myPos[2]/2.0);

	stickPos->Set(0,6, theta[0]);
}

void GENOME::SetParentAndPlacement(int currStick, int parent, double placement) {

	double total = double(parent) + placement;

	sticks->Set(currStick,1,total/double(currStick));
}

void GENOME::SetStickPos(int currStick) {

	if ( currStick == 0 )
		SetFirstStickPos();
	else
		SetSubsequentStickPos(currStick);
}

void GENOME::SetSubsequentStickPos(int currStick) {

	int stickToAttachTo;
	double hisAttach;
	double hisLength;
	double hisWidth;
	double hisPos[3];
	double hisThetas[2];

	GetHisStickData(currStick,&stickToAttachTo,&hisAttach,&hisLength,&hisWidth,hisPos,hisThetas);

	double myAttach[3];

	GetMyAttachmentPoint(myAttach,stickToAttachTo,hisPos,hisLength,hisWidth,hisAttach,hisThetas,currStick);

	/*
	printf("%5.5f\t%5.5f\t%5.5f\n",myAttach[0],myAttach[1],myAttach[2]);
	printf("%3.3f\n",sticks->Get(currStick,1));
	char ch = getchar();
	*/

	attachPos->Set(currStick,0,myAttach[0]);
	attachPos->Set(currStick,1,myAttach[1]);
	attachPos->Set(currStick,2,myAttach[2]);

	double myPos;
	double myLength;
	double backLength;
	double frontLength;

	GetMyStickData(currStick,&myPos,&myLength,&backLength,&frontLength);

	ComputeMyStickData(currStick,myAttach,backLength,frontLength);
}

void GENOME::WriteDanglers(ofstream *outFile, double lowestPoint) {

	(*outFile) << "( Dangler1\n";
		
		(*outFile) << "-position ";
		(*outFile) << (stickPos->Get(0,0)+stickPos->Get(0,1))/2.0 << " ";
		(*outFile) << (stickPos->Get(0,2)+stickPos->Get(0,3))/2.0 + lowestPoint - DANGLER_LENGTH/2.0 << " ";
		(*outFile) << (stickPos->Get(0,4)+stickPos->Get(0,5))/2.0 << "\n";

		(*outFile) << "-rotation 0 90 0\n";
		(*outFile) << "-mass 0.001\n";
		(*outFile) << "-shape -cylinder 0.01 " << DANGLER_LENGTH << "\n";
		(*outFile) << "-colour 0 0.2 0\n";

	(*outFile) << ")\n\n";

	(*outFile) << "( Dangler2\n";

		(*outFile) << "-position ";
		(*outFile) << (stickPos->Get(0,0)+stickPos->Get(0,1))/2.0 << " ";
		(*outFile) << (stickPos->Get(0,2)+stickPos->Get(0,3))/2.0 + lowestPoint - DANGLER_LENGTH/2.0 << " ";
		(*outFile) << (stickPos->Get(0,4)+stickPos->Get(0,5))/2.0 << "\n";

		(*outFile) << "-rotation 0 90 0\n";
		(*outFile) << "-mass 0.001\n";
		(*outFile) << "-shape -cylinder 0.01 " << DANGLER_LENGTH << "\n";
		(*outFile) << "-colour 0 1 0\n";

	(*outFile) << ")\n\n";

}

void GENOME::WriteDanglerJoints(ofstream *outFile, double lowestPoint) {

	(*outFile) << "( 0_Dangler1\n";

		(*outFile) << "-connect 0 Dangler1\n";

		(*outFile) << "-jointType Hinge\n";
		(*outFile) << "-jointPosition ";
		(*outFile) << (stickPos->Get(0,0)+stickPos->Get(0,1))/2.0 << " ";
		(*outFile) << (stickPos->Get(0,2)+stickPos->Get(0,3))/2.0 + lowestPoint << " ";
		(*outFile) << (stickPos->Get(0,4)+stickPos->Get(0,5))/2.0 << "\n";

		(*outFile) << "-jointNormal 0 0 1\n";

	(*outFile) << ")\n\n";

	(*outFile) << "( 0_Dangler2\n";

		(*outFile) << "-connect 0 Dangler2\n";

		(*outFile) << "-jointType Hinge\n";
		(*outFile) << "-jointPosition ";
		(*outFile) << (stickPos->Get(0,0)+stickPos->Get(0,1))/2.0 << " ";
		(*outFile) << (stickPos->Get(0,2)+stickPos->Get(0,3))/2.0 + lowestPoint << " ";
		(*outFile) << (stickPos->Get(0,4)+stickPos->Get(0,5))/2.0 << "\n";

		(*outFile) << "-jointNormal 1 0 0\n";

	(*outFile) << ")\n\n";
}

void GENOME::WriteJoint(int j, ofstream *outFile, double lowestPoint) {

	int parent;
	double placement;

	GetParentAndPlacement(j,&parent,&placement);

	(*outFile) << "( " << parent << "-" << j << "\n";

	(*outFile) << "-connect ";

		(*outFile) << parent << " ";
		(*outFile) << j << "\n";

	(*outFile) << "-jointPosition ";

		(*outFile) << attachPos->Get(j,0) << " ";
		(*outFile) << attachPos->Get(j,1) + lowestPoint << " ";
		(*outFile) << attachPos->Get(j,2) << "\n";

	(*outFile) << "-jointType Hinge\n";

	(*outFile) << "-jointNormal ";

	double x, y, z;
	ComputeJointNormal(j,&x,&y,&z);

	/*
	double theta	= sticks->Get(j,5);
	double phi		= sticks->Get(j,8);

	double x		= sin(theta)*cos(phi);
	double y		= sin(theta)*sin(phi);
	double z		= cos(theta);

	printf("%d %3.3f %3.3f %3.3f\n",j,x,y,z);
	char ch = getchar();
	*/
	(*outFile) << x << " " << y << " " << z << "\n";

	(*outFile) << "-jointLimits -90 90\n";
	(*outFile) << "-addMotor\n";
	(*outFile) << "-motorForce 40\n";
	(*outFile) << "-motorSpeed 8\n";

	//if ( sticks->Get(j,8) )
		(*outFile) << "-addSensor\n";

	(*outFile) << ")\n\n";

	if ( sticks->Get(j,6) ) {

		/*
		//-----------------Foot-------------------------
		(*outFile) << "( " << j << "-" << j << "_tip\n";

		(*outFile) << "-connect ";
		(*outFile) << j << " " << j << "_tip\n";

		(*outFile) << "-jointType Fixed\n";

		(*outFile) << ")\n\n";
		*/

		//-----------------Shank-------------------------
		(*outFile) << "( " << j << "-" << j << "_shank\n";

		(*outFile) << "-connect ";
		(*outFile) << j << " " << j << "_shank\n";

		(*outFile) << "-jointType Fixed\n";

		(*outFile) << ")\n\n";
	}
}

void GENOME::WriteObject(int o, ofstream *outFile, double lowestPoint) {

	(*outFile) << "( " << o << "\n";

	(*outFile) << "-position ";

		(*outFile) << (stickPos->Get(o,0)+stickPos->Get(o,1))/2.0 << " ";

		if ( o==0 )
			(*outFile) << (stickPos->Get(o,2)+stickPos->Get(o,3))/2.0 + (0.9/2.0) << " ";
		else
			(*outFile) << (stickPos->Get(o,2)+stickPos->Get(o,3))/2.0 + lowestPoint << " ";

		(*outFile) << (stickPos->Get(o,4)+stickPos->Get(o,5))/2.0 << "\n";

	(*outFile) << "-rotation ";
	
	if ( o==0 ) {

		(*outFile) << "0 -1.5708 0\n";

	}
	else {

		(*outFile) << stickPos->Get(o,1) - stickPos->Get(o,0) << " ";
		(*outFile) << stickPos->Get(o,3) - stickPos->Get(o,2) << " ";
		(*outFile) << stickPos->Get(o,5) - stickPos->Get(o,4) << "\n";
	}

	if ( o==0 ) {
		(*outFile) << "-shape -rectangle ";
		//(*outFile) << sticks->Get(o,3) << " " << sticks->Get(o,2) << " " << sticks->Get(o,4) << "\n";
		(*outFile) << 0.82 << " " << 0.82 << " " << 0.9 << "\n";
	}
	else {
		(*outFile) << "-shape -cylinder ";
		(*outFile) << sticks->Get(o,3) << " " << sticks->Get(o,2) << "\n";
	}

	(*outFile) << "-mass " << sticks->Get(o,7)*MAX_MASS_PER_PART << "\n";
	//(*outFile) << "-mass 0.4\n";

	if ( o==0 )
		(*outFile) << "-colour 0.0 1.0 0.0\n";
	else
		if ( o==1 )
			(*outFile) << "-colour 1.0 0.0 0.0\n";
		else if ( o==2 )
			(*outFile) << "-colour 0.0 0.0 1.0\n";
		else if ( o==3 )
			(*outFile) << "-colour 1.0 0.0 1.0\n";
		else if ( o==4 )
			(*outFile) << "-colour 1.0 1.0 0.0\n";
		else if ( o==5 )
			(*outFile) << "-colour 0.5 0.0 0.0\n";
		else if ( o==6 )
			(*outFile) << "-colour 0.0 0.0 0.5\n";
		else if ( o==7 )
			(*outFile) << "-colour 0.5 0.0 0.5\n";
		else if ( o==8 )
			(*outFile) << "-colour 0.5 0.5 0.0\n";

	/*
	if ( o==0 ) {
		(*outFile) << "-colour 1.0 0.0 0.0\n";
		(*outFile) << "-floorContact\n";
		(*outFile) << "-addTouchSensor\n";
	}
	*/

	(*outFile) << "-floorContact\n";

	(*outFile) << ")\n\n";

	if ( sticks->Get(o,6) ) {

		/*
		//-------------------Foot-----------------------
		(*outFile) << "( " << o << "_tip \n";

		(*outFile) << "-position ";
			(*outFile) << stickPos->Get(o,1) << " ";
			(*outFile) << stickPos->Get(o,3) + lowestPoint << " ";
			(*outFile) << stickPos->Get(o,5) << "\n";

		(*outFile) << "-shape -sphere ";
			//(*outFile) << 0.1 << "\n";
			(*outFile) << 1.1*CYLINDER_DIAMETER << "\n";

		(*outFile) << "-mass 0.023\n";
		//(*outFile) << "-mass 0.4\n";

		if ( o==5 )
			(*outFile) << "-colour 0.5 0.0 0.0\n";
		else if ( o==6 )
			(*outFile) << "-colour 0.0 0.0 0.5\n";
		else if ( o==7 )
			(*outFile) << "-colour 0.5 0.0 0.5\n";
		else if ( o==8 )
			(*outFile) << "-colour 0.5 0.5 0.0\n";

		(*outFile) << "-floorContact\n";
		(*outFile) << "-addTouchSensor\n";

		(*outFile) << ")\n\n";
		*/

		//-------------------Shank-----------------------
		(*outFile) << "( " << o << "_shank \n";

		(*outFile) << "-position ";
			(*outFile) << stickPos->Get(o,0) << " ";
			(*outFile) << stickPos->Get(o,2) + lowestPoint << " ";
			(*outFile) << stickPos->Get(o,4) << "\n";

		(*outFile) << "-rotation ";
			(*outFile) << stickPos->Get(o,1) - stickPos->Get(o,0) << " ";
			(*outFile) << stickPos->Get(o,3) - stickPos->Get(o,2) << " ";
			(*outFile) << stickPos->Get(o,5) - stickPos->Get(o,4) << "\n";

		(*outFile) << "-shape -rectangle ";
			//(*outFile) << 0.1 << "\n";
			(*outFile) << 0.37 << " ";
			(*outFile) << 0.30 << " ";
			(*outFile) << 0.40 << "\n";

		(*outFile) << "-mass 0.023\n";
		//(*outFile) << "-mass 0.4\n";

		if ( o==5 )
			(*outFile) << "-colour 0.5 0.0 0.0\n";
		else if ( o==6 )
			(*outFile) << "-colour 0.0 0.0 0.5\n";
		else if ( o==7 )
			(*outFile) << "-colour 0.5 0.0 0.5\n";
		else if ( o==8 )
			(*outFile) << "-colour 0.5 0.5 0.0\n";

		(*outFile) << "-floorContact\n";

		(*outFile) << ")\n\n";
	}
}

void GENOME::WriteSuffix(ofstream *outFile) {

	(*outFile) << "(\n";

	(*outFile) << "-evaluationPeriod " << EVAL_PERIOD << "\n";
	
//	if ( simParams->performingEstimation )
		(*outFile) << "-testForExplosions\n";

	//(*outFile) << "-recordObjectPosition " << OBJECT_TO_MOVE << "\n";

	(*outFile) << ")\n";
}

#endif