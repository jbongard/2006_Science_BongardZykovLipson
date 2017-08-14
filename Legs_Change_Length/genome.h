/* ---------------------------------------------------
   FILE:     bodyPlan.h
	AUTHOR:   Josh Bongard
	DATE:     October 2, 2000
	FUNCTION: This class contains all information for
				 a single physical segment of an organism
				 in the MathEngine environment.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _GENOME_H
#define _GENOME_H

#include "matrix.h"
#include "neuralNetwork.h"

class GENOME {

public:
	int	ID;
	double fitness;
	double secondaryFitness;
	int dominated;
	MATRIX *raw;
	NEURAL_NETWORK *nn;

private:
	MATRIX *sticks;
	MATRIX *attachPos;
	MATRIX *stickPos;
	double parentsWorstFitness;

public:
	GENOME(int myID);
	GENOME(MATRIX *toSow, int myID);
	GENOME(GENOME *parent);
	~GENOME(void);
	void ComputeFitness(void);
	void ComputeFitness(MATRIX *targetSensorData,ifstream *inFile);
	void ConvertPartiallyToTarget(void);
	void ConvertToTarget(void);
	void Cross(GENOME *sibling);
	void Evaluate_Estimation(void);
	void Evaluate_Exploration(void);
	NEURAL_NETWORK *GetController(void);
	MATRIX *GetModel(void);
	void Mutate(void);
	void Print(void);
	void PrintTest(void);
	void PrintSticks(void);
	void Replace(GENOME *loser, int nextAvailableID);
	void Save(ofstream *outFile);
	void SaveGenome(ofstream *outFile);
	void SaveSticks(ofstream *outFile);
	int  Valid(void);
	int  WorseThanParent(void);
	void WriteBody(ofstream *outFile);
	void WriteTargetBody(ofstream *outFile);

private:
	int  AgentInvalid(MATRIX *mySensorData);
	void ComputeJointNormal(int currJoint, double *x, double *y, double *z);
	void ComputeMyStickData(int currStick, double *myAttach, double backLength,
							double frontLength);
	void ComputeRollingMean(MATRIX *targetSensorData, MATRIX *mySensorData);
	void ConformToTarget(void);
	void CopyBodyPart(void);
	void CreateEstimationGenome(void);
	void CreateEstimationGenome(MATRIX *toSow);
	void CreateEstimationGenome(GENOME *parent);
	void CreateExplorationGenome(void);
	void CreateExplorationGenome(GENOME *parent);
	void CreateSticks(void);
	void CreateStickPositions(void);
	void DestroyStickPositions(void);
	void DestroySticks(void);
	void FindActuatedSticks(void);
	void FindSolidSticks(void);
	double GetCurrentTheta(int currStick);
	void GetHisStickData(int currStick, int *stickToAttachTo, double *hisAttach,
						 double *hisLength, double *hisWidth, double *hisPos, double *hisThetas);
	void GetMyAttachmentPoint(double *myAttach, int stickToAttachTo, double *hisPos,
							  double hisLength, double hisWidth, double hisAttach, double *hisThetas, int currStick);
	void GetMyStickData(int currStick, double *myPos, double *myLength,
						double *backLength, double *frontLength);
	void GetParentAndPlacement(int currStick, int *parent, double *placement);
	void Mutate_Estimation(void);
	void Mutate_Exploration(void);
	void PerturbModel(void);
	void PointMutation(void);
	void SetFirstStickPos(void);
	void SetParentAndPlacement(int currStick, int parent, double placement);
	void SetStickPos(int currStick);
	void SetSubsequentStickPos(int currStick);
	void WriteDanglers(ofstream *outFile, double lowestPoint);
	void WriteDanglerJoints(ofstream *outFile, double lowestPoint);
	void WriteJoint(int j, ofstream *outFile, double lowestPoint);
	void WriteObject(int o, ofstream *outFile, double lowestPoint);
	void WriteSuffix(ofstream *outFile);
};

#endif