/* ---------------------------------------------------
   FILE:     growGA.h
	AUTHOR:   Josh Bongard
	DATE:     October 20, 2000
	FUNCTION: This class contains all information for
			  a population of variable-length genotypes

 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _GA_H
#define _GA_H

#include "fstream.h"
#include "genome.h"
#include "neuralNetwork.h"

class GA {

public:
	int      popSize;
	int      currGeneration;
	int		 totalGenerations;
	GENOME   **genomes;
	GENOME	 **parents;
	GENOME	 **children;

private:
	double    *bestFits;
	double    *avgFits;
	int		   nextAvailableID;
	ofstream  *fitFile;
	ofstream  *genomeFile;
	char	  fitFileName[100];
	char	  genomeFileName[100];
	int		  stagnationCounter;

public:
	GA(void);
	GA(MATRIX **bestSoFar);
	~GA(void);
	void	Evolve(MATRIX **dataForTarget, MATRIX **dataFromTarget, MATRIX **models);
	NEURAL_NETWORK *GetBestController(void);
	MATRIX *GetBestModel(void);
	void    PrintGenomes(GENOME **g);

private:
	void    AntiSort(GENOME **g);
	void	ComputeFitnesses(int currentTest, int numToEvaluate, MATRIX **dataFromTarget);
	void	ComputeTestFitnesses(void);
	void    ComputeTestFitnesses(MATRIX *variances);
	void    ComputeTestMeans(int numOfResults, MATRIX **testResults, MATRIX *means);
	void	ComputeTestReliabilities(int numOfResults, MATRIX **testResults);
	void    ComputeTestVariances(int numOfResults, MATRIX **testResults, MATRIX *means, MATRIX *variances);
	void    CopyBestModel(void);
	void    CreateDCOffspring(void);
	void    CreateOffspring(void);
	void	CreateParetoOffspring(void);
	void    CreateTournamentOffspring(void);
	void	DeleteGenomes(void);
	void    DeleteFitnessDuplicates(GENOME **g, int numG);
	void	EvolveForOneGeneration(MATRIX **dataForTarget, MATRIX **dataFromTarget, MATRIX **models);
	void	EvolveForOneGeneration_Estimation(MATRIX **dataForTarget, MATRIX **dataFromTarget);
	void	EvolveForOneGeneration_Exploration(MATRIX **dataForTarget, MATRIX **dataFromTarget, MATRIX **models);
	void    FindDominatedGenomes(GENOME **g, int numG);
	int		GetNumToEvaluate(void);
	void	InitGA(void);
	void	InitGenomes(void);
	void	InitGenomes(MATRIX **bestSoFar);
	void	LoadState(void);
	void	CloseDataFiles(void);
	void	Fill(void);
	void	OpenDataFiles(int append);
	GENOME *PerformTournament(GENOME **g);
	void	PrintSummary(void);
	void    ReplaceParents(void);
	void    ReplaceParentsUsingDC(void);
	void    ReplaceParentsUsingPareto(void);
	void	SaveState(void);
	void    Shuffle(GENOME **g);
	void    Sort(void);
	void    SwitchGenomes(GENOME **g, int firstGenome, int secondGenome);
	void	WriteAll(void);
	void    WriteGenomesForEvaluation(int currentTest, int numToEvaluate, MATRIX **dataForTarget);
	void    WriteBest(void);
	void	WriteEstReports(void);
	void	WriteExpReports(void);
	void    WriteModelForEvaluation(int currentModel, ofstream *bodyFile);
	void	WriteModelsForEvaluation(MATRIX **models);
	MATRIX *WriteMotorProgramForEvaluation(int currentTest);
	void	WriteMotorProgramForOutput(int currentTest);
	MATRIX *WriteMotorProgramForEvaluation(int selectedJoint1, int selectedJoint2);
	void	WriteMotorProgramsForEvaluation(void);
};

#endif