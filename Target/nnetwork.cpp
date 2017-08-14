// ---------------------------------------------------
//    FILE:     nnetwork.cpp
//		AUTHOR:	 Josh Bongard
//		DATE:     March 26, 2001
//		FUNCTION: This class defines a neural network to
//              control a user-designed agent.
// --------------------------------------------------- 

#include "stdafx.h"
#include "stdio.h"

#include "nnetwork.h"
#include "simParams.h"

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

extern int  TOUCH_SENSORS;
extern int  PROP_SENSORS;

extern int  BIAS_NODES;

extern SIM_PARAMS  *simParams;

extern int NOISE_TRIALS;
extern int ADD_NOISE;
extern double SENSOR_NEURON_RADIUS;
extern double HIDDEN_NEURON_RADIUS;
extern double MOTOR_NEURON_RADIUS;
extern double NN_WIDTH;

#ifdef dDOUBLE
#define dsDrawLine dsDrawLineD
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

NEURAL_NETWORK::NEURAL_NETWORK(void) {

	numInput = 0;
	numHidden = 0;
	numOutput = 0;

	inputValues = NULL;
	hiddenValues = NULL;
	outputValues = NULL;

	nextHiddenValues = NULL;
	nextOutputValues = NULL;

	ih = NULL;
	hh = NULL;
	ho = NULL;
}

NEURAL_NETWORK::~NEURAL_NETWORK(void) {
	
	delete inputValues;
	inputValues = NULL;

	delete hiddenValues;
	hiddenValues = NULL;

	delete outputValues;
	outputValues = NULL;

	delete nextHiddenValues;
	nextHiddenValues = NULL;

	delete nextOutputValues;
	nextOutputValues = NULL;

	delete ih;
	ih = NULL;

	delete hh;
	hh = NULL;

	delete ho;
	ho = NULL;
}

int NEURAL_NETWORK::AddMotor(void) {

	numOutput++;

	return(numOutput-1);
}

int NEURAL_NETWORK::AddSensor(void) {

	numInput++;

	return(numInput-1);
}

/*
void NEURAL_NETWORK::ClearValues(void) {

	int i;

	for (i=0;i<numInput;i++)
		inputValues[i] = 0.0;

	for (i=0;i<numHidden;i++)
		hiddenValues[i] = 0.0;

	for (i=0;i<numOutput;i++)
		outputValues[i] = 0.0;
}

void NEURAL_NETWORK::DrawNeurons(double *com) {

	DrawSensorNeurons(com);

	DrawHiddenNeurons(com);

	DrawMotorNeurons(com);
}
*/
double NEURAL_NETWORK::GetMotorCommand(int neuronIndex) {

	return( outputValues->Get(0,neuronIndex) );
}

double NEURAL_NETWORK::GetSensorValue(int neuronIndex) {

	return( inputValues->Get(0,neuronIndex) );
}


void NEURAL_NETWORK::Init(ifstream *brainFile) {

	ih = new MATRIX(brainFile);
	hh = new MATRIX(brainFile);
	ho = new MATRIX(brainFile);

	numHidden = ih->width;

	inputValues  = new MATRIX(1,numInput,0.0);
	hiddenValues = new MATRIX(1,numHidden,0.0);
	outputValues = new MATRIX(1,numOutput,0.0);

	nextHiddenValues = new MATRIX(1,numHidden,0.0);
	nextOutputValues = new MATRIX(1,numOutput,0.0);
}

/*
void NEURAL_NETWORK::LabelSynapses(int genomeLength,
											  const double *params, int numMorphParams) {

	int k = 0;
	int skippedValues = 0;

	int i,j;
	int synapticWeightSetSize;

	if ( numMorphParams>0 ) {
		synapticWeightSetSize = (genomeLength-numMorphParams) / numMorphParams;
		skippedValues++;
		k++;
	}
	else
		synapticWeightSetSize = (genomeLength-numMorphParams);

	for (i=0;i<numInput;i++)
		for (j=0;j<numHidden;j++) {
			weightsIH[(i*numHidden)+j] = params[k++];
			if ( (numMorphParams>1) && (skippedValues < numMorphParams) && 
				((k%synapticWeightSetSize) == 0) ) {
				skippedValues++;
				k++;
			}
		}

	for (i=0;i<numHidden;i++)
		for (j=0;j<numHidden;j++) {

			weightsHH[(i*numHidden)+j] = params[k++];
			
			if ( (numMorphParams>1) && (skippedValues < numMorphParams) && 
				((k%synapticWeightSetSize) == 0) ) {
				skippedValues++;
				k++;
			}
		}

	for (i=0;i<numHidden;i++)
		for (j=0;j<numOutput;j++) {
			weightsHO[(i*numOutput)+j] = params[k++];
			if ( (numMorphParams>1) && (skippedValues < numMorphParams) && 
				((k%synapticWeightSetSize) == 0) ) {
				skippedValues++;
				k++;
			}
		}

	for (i=0;i<BIAS_NODES;i++)
		for (j=0;j<numHidden;j++) {
			weightsBH[(i*numHidden)+j] = params[k++];
			if ( (numMorphParams>1) && (skippedValues < numMorphParams) && 
				((k%synapticWeightSetSize) == 0) ) {
				skippedValues++;
				k++;
			}
		}

	for (i=0;i<BIAS_NODES;i++)
		for (j=0;j<numOutput;j++) {
			weightsBO[(i*numOutput)+j] = params[k++];
			if ( (numMorphParams>1) && (skippedValues < numMorphParams) && 
				((k%synapticWeightSetSize) == 0) ) {
				skippedValues++;
				k++;
			}
		}
}

void NEURAL_NETWORK::PerturbSynapses(void) {

	int i, j;

	for (i=0;i<numInput;i++)
		for (j=0;j<numHidden;j++) {

			if ( weightsIH[(i*numHidden)+j] < 0.0 )
				weightsIH[(i*numHidden)+j] = weightsIH[(i*numHidden)+j] + 0.01;

			else
				weightsIH[(i*numHidden)+j] = weightsIH[(i*numHidden)+j] - 0.01;

		}
}
*/
void NEURAL_NETWORK::Print(void) {

	PrintValues();
	//PrintWeights();
}
/*
void NEURAL_NETWORK::RecordHiddenValues(ofstream *outFile) {

	int h;

	for (h=0;h<numHidden;h++)
		if ( h == (numHidden-1) )
			(*outFile) << hiddenValues[h] << "\n";
		else
			(*outFile) << hiddenValues[h] << ";";
}
*/
void NEURAL_NETWORK::UpdateSensorValue(double value, int neuronIndex) {

	if ( neuronIndex >= 0 )

		inputValues->Set(0,neuronIndex,value);
}

void NEURAL_NETWORK::Update(void){

	UpdateHiddenValues();

	UpdateOutputValues();
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

/*
void NEURAL_NETWORK::DrawSensorNeurons(double *com) {

	double R[12];
	simParams->SetDefaultRotation(R);

	double myPos[3];
	double neuronValue;
	double spacing = NN_WIDTH / (numInput-1);
	double flushLeft = -NN_WIDTH/2.0;

	for (int i=0;i<numInput;i++) {
	
		myPos[0] = com[0];
		myPos[1] = com[1] + flushLeft;
		myPos[2] = com[2] + 2.0;

		neuronValue = inputValues[i]*0.5 + 0.5;

		dsSetColor(neuronValue,neuronValue,neuronValue);

		dsDrawSphere(myPos,R,SENSOR_NEURON_RADIUS);

		flushLeft = flushLeft + spacing;
	}

}

void NEURAL_NETWORK::DrawHiddenNeurons(double *com) {

	double R[12];
	simParams->SetDefaultRotation(R);

	double myPos[3];
	double neuronValue;
	double spacing = NN_WIDTH / (numHidden-1);
	double flushLeft = -NN_WIDTH/2.0;

	for (int h=0;h<numHidden;h++) {
	
		myPos[0] = com[0];
		myPos[1] = com[1] + flushLeft;
		myPos[2] = com[2] + 1.5;

		neuronValue = hiddenValues[h]*0.5 + 0.5;

		dsSetColor(neuronValue,neuronValue,neuronValue);

		dsDrawSphere(myPos,R,HIDDEN_NEURON_RADIUS);

		flushLeft = flushLeft + spacing;
	}
}

void NEURAL_NETWORK::DrawMotorNeurons(double *com) {

	double R[12];
	simParams->SetDefaultRotation(R);

	double myPos[3];
	double neuronValue;
	double spacing = NN_WIDTH / (numOutput-1);
	double flushLeft = -NN_WIDTH/2.0;

	for (int o=0;o<numOutput;o++) {
	
		myPos[0] = com[0];
		myPos[1] = com[1] + flushLeft;
		myPos[2] = com[2] + 1.0;

		neuronValue = outputValues[o]*0.5 + 0.5;

		dsSetColor(neuronValue,neuronValue,neuronValue);

		dsDrawSphere(myPos,R,MOTOR_NEURON_RADIUS);

		flushLeft = flushLeft + spacing;
	}
}
*/
void NEURAL_NETWORK::PrintValues(void) {

	inputValues->Print();
	hiddenValues->Print();
	outputValues->Print();

}

void NEURAL_NETWORK::PrintWeights(void) {

	ih->Print();
	hh->Print();
	ho->Print();
}

void NEURAL_NETWORK::UpdateHiddenValues(void) {

	int i, j;
	double thresholdedValue;

	for (i=0;i<numHidden;i++) {

		nextHiddenValues->Set(0,i,0.0);

		for (j=0;j<numInput;j++)
			nextHiddenValues->Set(0,i,nextHiddenValues->Get(0,i) +
											(inputValues->Get(0,j)*ih->Get(j,i)));

		for (j=0;j<numHidden;j++)
			nextHiddenValues->Set(0,i,nextHiddenValues->Get(0,i) +
											(hiddenValues->Get(0,j)*hh->Get(j,i)));

		thresholdedValue = (2.0)/(1.0 + ( exp(-nextHiddenValues->Get(0,i)) ) ) - 1.0;

		hiddenValues->Set(0,i,thresholdedValue);
	}
}

void NEURAL_NETWORK::UpdateOutputValues(void) {

	int i, j;
	double thresholdedValue;

	for (i=0;i<numOutput;i++) {

		nextOutputValues->Set(0,i,0.0);

		for (j=0;j<numHidden;j++)
			nextOutputValues->Set(0,i,nextOutputValues->Get(0,i) +
											(hiddenValues->Get(0,j)*ho->Get(j,i)));


		thresholdedValue = (2.0)/(1.0 + ( exp(-nextOutputValues->Get(0,i)) ) ) - 1.0;

		outputValues->Set(0,i,thresholdedValue);
	}
}
