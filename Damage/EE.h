
#include "stdafx.h"

#ifndef _EE_H
#define _EE_H

#include "ga.h"
#include "matrix.h"
#include "neuralNetwork.h"

class EE {

private:
	MATRIX			**dataFromTarget;
	MATRIX			**dataForTarget;
	MATRIX			**models;
	GA				*ga;

private:
	ofstream		*estFitFile;
	ofstream		*expFitFile;
	MATRIX			**bankedMotorPrograms;
	MATRIX			**bankedSensorData;
	int				numBankedTests;
	int				testJustBanked;

public:

	EE(void);
	~EE(void);
	void CreateMotorCommandsForVictor(void);
	void PerformBatch(void);
	void PerformInference(void);

private:
	int  BankedTestReadyNow(void);
	int  ComputeBankedTestsReadinesses(void);
	void CopyTarget(void);
	void CreateDataFiles(void);
	void CreateEvolvedMotorCommands(int selectedTest);
	void CreateRandomMotorCommands(void);
	void DecreaseTestReliability(void);
	void DepositTest(void);
	void GenerateRandomMotorProgram(void);
	void GetDummyTargetSensorValues(void);
	void GetSensorData(void);
	void GetSensorDataFromScreen(void);
	void GetTargetData(void);
	void GetTargetSensorValues(void);
	void IncreaseTestReliability(void);
	void LoadOldSensorData(void);
	void PerformEstimation(void);
	void PerformEstimationBatch(void);
	void PerformExploration(void);
	void SendController(void);
	void SendControllerToScreen(void);
	void SendRandomController(void);
	void SendEvolvedController(void);
	void StoreBestModelForLater(void);
	void StoreModelsForLater(void);
	int  TestFailed(void);
	void TestTarget(void);
	void UpdateMotorProgram(void);
	void WithdrawBankedTest(int readyTest);
	void WriteBankedTests(void);
	void WriteKillSignal(void);
};

#endif
