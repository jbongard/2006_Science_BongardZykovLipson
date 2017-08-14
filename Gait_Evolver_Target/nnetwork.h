/* ---------------------------------------------------
   FILE:      nnetwork.h
	AUTHOR:   Chandana Paul
	DATE:     November 27, 2000
	FUNCTION: This class contains the neural network
			  which controls the biped agent. 	
 -------------------------------------------------- */

#include <iostream.h>
#include "fstream.h"
#include "math.h"
#include "matrix.h"

#ifndef _NEURAL_NETWORK_H
#define _NEURAL_NETWORK_H

class NEURAL_NETWORK {

public:
	int   numInput;
	int   numHidden;
	int   numOutput;

private:
	MATRIX *inputValues;
	MATRIX *hiddenValues;
	MATRIX *outputValues;

	MATRIX *nextHiddenValues;
	MATRIX *nextOutputValues;

	MATRIX *ih;
	MATRIX *hh;
	MATRIX *ho;

public:
	NEURAL_NETWORK(void);
	~NEURAL_NETWORK(void);
	int   AddMotor(void);
	int   AddSensor(void);
	//void  ClearValues(void);
	//void  DrawNeurons(double *com);
	double GetMotorCommand(int neuronIndex);
	double GetSensorValue(int neuronIndex);
	void  Init(ifstream *brainFile);
	//void  LabelSynapses(int genomeLength, const double *params, int numMorphParams);
	//void  PerturbSynapses(void);
	void    Print(void);
	//void  RecordHiddenValues(ofstream *outFile);
	void  UpdateSensorValue(double value, int neuronIndex);
	void  Update(void);

private:
	//void DrawSensorNeurons(double *com);
	//void DrawHiddenNeurons(double *com);
	//void DrawMotorNeurons(double *com);
	void PrintValues(void);
	void PrintWeights(void);
	void UpdateHiddenValues(void);
	void UpdateOutputValues(void);
};

#endif