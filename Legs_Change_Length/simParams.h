/* ---------------------------------------------------
   FILE:     simParams.h
	AUTHOR:   Josh Bongard
	DATE:     October 5, 2000
	FUNCTION: This class contains all miscellaneous
				 data and functions for this simulation.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"

#ifndef _SIM_PARAMS_H
#define _SIM_PARAMS_H

#include "matrix.h"

class SIM_PARAMS {

public:
	int		 randSeed;
	int		 loadFromFile;
	int		 isEvaluator;
	int		 performingEstimation;
	int		 performingIntelligentTesting;
	int		 performingBatch;
	int		 currentCycle;
	int		 totalCycles;
	ofstream *bodyPhylogenyFile;
	ofstream *stickPhylogenyFile;
	ofstream *genomeFile;
	MATRIX   *targetMatrix;
	int		 totalModelEvals;
	int		 totalTargetEvals;
	int		 perturbModel;
	int		 usePareto;
	int		 usePhysicalMachine;
	int		 useTestBank;
	int		 hideAttachments;
	double	 reliabilityWeight;
	MATRIX	 *allTests;
	int		 totalTests;
	int		 noPrinting;
	int		 numEstGenerations;

public:
	SIM_PARAMS(int argc, char **argv);
	~SIM_PARAMS(void);
	void   CloseDataFiles(void);
	void   CreateTargetMatrix(void);
	double DefaultValueForJoint(int joint);
	void   DestroyAllTests(void);
	int    EEAToVictorJointIndex(int EEAIndex);
	void   FileDelete(char *fileName);
	int    FileExists(char *fileName);
	void   FileRename(char *src, char *dest);
	int    FlipCoin(void);
	ofstream *GetOutFile(char *fileName);
	int    GetRegimeIndex(void);
	void   InitializeAllTests(void);
	void   OpenDataFiles(void);
	void   ParseParameters(int argc, char **argv);
	double Rand(double min, double max);
	int    RandInt(int min, int max);
	double Scale(double value, double min1, double max1,
		         double min2, double max2);
	double ServoToAngle(int servoVal, int currJoint);
	int    VictorToEEAJointIndex(int victorIndex);
	void   WaitForFile(char *fileName);
	void   WriteBodyDifference(MATRIX *sticks, MATRIX *stickPos);
	void   WriteKillFile(void);
};

#endif