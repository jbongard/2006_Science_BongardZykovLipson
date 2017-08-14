/* ---------------------------------------------------
   FILE:     growGA.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 20, 2000
	FUNCTION: This class contains all information for
				 a population of variable-length genotypes
 -------------------------------------------------- */

#include "stdafx.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#ifndef _GA_CPP
#define _GA_CPP

#include "constants.h"
#include "genome.h"
#include "ga.h"
#include "simParams.h"

extern int			EST_POPULATION_SIZE;
extern int			EXP_POPULATION_SIZE;
extern int			STAGNATION_LIMIT;
extern int			NUM_EXP_GENERATIONS;

extern SIM_PARAMS	*simParams;

extern char			EST_FILENAME[100];
extern char			EXP_FILENAME[100];
extern char			DATA_DIRECTORY[100];

extern char			TEMP_FILENAME[100];
extern char			TEMP2_FILENAME[100];
extern int			TOURNAMENT_SIZE;

GA::GA(void) {

	InitGA();

	InitGenomes();
}

GA::GA(MATRIX **bestSoFar) {

	InitGA();

	InitGenomes(bestSoFar);
}

GA::~GA(void) {

	DeleteGenomes();
}

void GA::Evolve(MATRIX **dataForTarget, MATRIX **dataFromTarget, MATRIX **models) {

	if ( simParams->performingEstimation ) {

		while ( (currGeneration<totalGenerations) ) { 
//			    (stagnationCounter<STAGNATION_LIMIT) ) {

			stagnationCounter++;

			EvolveForOneGeneration(dataForTarget,dataFromTarget,models);

			currGeneration++;
		}
	}
	else {
		while ( currGeneration < totalGenerations ) {

			EvolveForOneGeneration(dataForTarget,dataFromTarget,models);

			currGeneration++;
		}
	}
}

NEURAL_NETWORK *GA::GetBestController(void) {

	return( genomes[0]->GetController() );
}

MATRIX *GA::GetBestModel(void) {

	return( genomes[0]->GetModel() );
}

void GA::PrintGenomes(GENOME **g) {

	int tempGenome;

	for (tempGenome=0;tempGenome<popSize;tempGenome++)

		g[tempGenome]->Print();
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void GA::AntiSort(GENOME **g) {

   int bound = popSize-1;
   int t;
   int j;
   double firstFitness, secondFitness;

   GENOME *firstGenome, *secondGenome;

   do {
       t = 0;
       for(j = 0; j < bound; ++j) {

			  firstGenome = g[j];
			  secondGenome = g[j+1];

			  firstFitness = firstGenome->fitness;
			  secondFitness = secondGenome->fitness;

           if( firstFitness < secondFitness ) {
					SwitchGenomes(g,j,j+1);
					t = j;
           }
       }
       bound = t;
   } while (t != 0);
}

void GA::CloseDataFiles(void) {

	fitFile->close();
	delete fitFile;
	fitFile = NULL;

	if ( currGeneration == (totalGenerations-1) ) {

		genomeFile->close();
		delete genomeFile;
		genomeFile = NULL;
	}
}

void GA::ComputeFitnesses(int currentTest, int numToEvaluate, MATRIX **dataFromTarget) {

	ifstream *sensorFile = NULL;

	if ( numToEvaluate > 0 ) {

		sensorFile = new ifstream(SENSOR_FILENAME);

		double temp;
		(*sensorFile) >> temp;
	}

	for (int p=0;p<popSize;p++) {

		if ( genomes[p]->Valid() && (!genomes[p]->WorseThanParent()) )
		//if ( genomes[p]->Valid() )
			genomes[p]->ComputeFitness(dataFromTarget[currentTest],sensorFile);
		else
			genomes[p]->fitness = 10000.0;
	}

	if ( numToEvaluate > 0 ) {

		sensorFile->close();
		delete sensorFile;
		sensorFile = NULL;
		simParams->FileDelete(SENSOR_FILENAME);
	}
}


void GA::ComputeTestFitnesses(void) {

	ifstream *sensorFile = new ifstream(SENSOR_FILENAME);

	int numOfResults;

	(*sensorFile) >> numOfResults;

	MATRIX **testResults = new MATRIX*[numOfResults];

	for (int i=0;i<numOfResults;i++) {

		testResults[i] = new MATRIX(sensorFile);
	}

	sensorFile->close();
	delete sensorFile;
	sensorFile = NULL;
	simParams->FileDelete(SENSOR_FILENAME);

	/*
	MATRIX *means = new MATRIX(popSize,testResults[0]->width,0.0);
	MATRIX *variances = new MATRIX(popSize,testResults[0]->width,0.0);

	ComputeTestMeans(numOfResults,testResults,means);

	ComputeTestVariances(numOfResults,testResults,means,variances);

	ComputeTestFitnesses(variances);

	delete means;
	means = NULL;

	delete variances;
	variances = NULL;
	*/
	
	ComputeTestReliabilities(numOfResults,testResults);

	for (i=0;i<numOfResults;i++) {

		delete testResults[i];
		testResults[i] = NULL;
	}

	delete[] testResults;
	testResults = NULL;
}

void GA::ComputeTestFitnesses(MATRIX *variances) {

	for (int i=0;i<popSize;i++)

		genomes[i]->fitness = -( variances->SumOfRow(i,8,14) );
}

void GA::ComputeTestMeans(int numOfResults, MATRIX **testResults, MATRIX *means) {

	int testIndex = 0;

	int i,j,k;

	for (k=0;k<MODELS_FOR_DISAGREEMENT;k++) {

		for (i=0;i<popSize;i++) {

			for (j=0;j<testResults[0]->width;j++) {

				means->Add(i,j,testResults[testIndex]->Get(0,j));
			}
			testIndex++;
		}
	}

	for (i=0;i<popSize;i++)
		for (j=0;j<testResults[0]->width;j++)
			means->Div(i,j,double(MODELS_FOR_DISAGREEMENT));
}

void GA::ComputeTestReliabilities(int numOfResults, MATRIX **testResults) {
	
	int i,j,currentTest;
	double totalDiffs;

	MATRIX *means     = new MATRIX(popSize,testResults[0]->width,0.0);
	MATRIX *variances = new MATRIX(popSize,testResults[0]->width,0.0);

	for (currentTest=0;currentTest<popSize;currentTest++) {
	
		totalDiffs = 0.0;

		for (i=currentTest;i<(popSize*2*MODELS_FOR_DISAGREEMENT);i=i+(popSize*2)) {
			
			for (j=0;j<testResults[0]->width;j++) {

				totalDiffs = totalDiffs + fabs(testResults[i]->Get(0,j) - 
											   testResults[i+popSize]->Get(0,j));

				means->Add(currentTest,j,testResults[i]->Get(0,j));
			}
		}

		genomes[currentTest]->fitness = totalDiffs;
	}

	means->Div(double(MODELS_FOR_DISAGREEMENT));

	for (currentTest=0;currentTest<popSize;currentTest++)

		for (i=currentTest;i<(popSize*2*MODELS_FOR_DISAGREEMENT);i=i+(popSize*2))
			
			for (j=0;j<testResults[0]->width;j++)

				variances->Add(currentTest,j,pow(means->Get(currentTest,j)-testResults[i]->Get(0,j),2.0));

	variances->Div(double(MODELS_FOR_DISAGREEMENT));
	variances->Sqrt();

	for (currentTest=0;currentTest<popSize;currentTest++) {
		
		genomes[currentTest]->secondaryFitness = 0.0;

		for (j=0;j<testResults[0]->width;j++)
			genomes[currentTest]->secondaryFitness = genomes[currentTest]->secondaryFitness + variances->Get(currentTest,j);

		genomes[currentTest]->fitness = (simParams->reliabilityWeight)*genomes[currentTest]->fitness - 
										(1.0-simParams->reliabilityWeight)*genomes[currentTest]->secondaryFitness;

		genomes[currentTest]->ID = currentTest;
	}

	delete means;
	means = NULL;

	delete variances;
	variances = NULL;
}

void GA::ComputeTestVariances(int numOfResults, MATRIX **testResults, MATRIX *means, MATRIX *variances) {

	int testIndex = 0;

	int i,j,k;

	for (k=0;k<MODELS_FOR_DISAGREEMENT;k++) {

		for (i=0;i<popSize;i++) {

			for (j=0;j<testResults[0]->width;j++) {

				variances->Add(i,j,pow(means->Get(i,j)-testResults[testIndex]->Get(0,j),2.0));
			}
			testIndex++;
		}
	}

	for (i=0;i<popSize;i++)
		for (j=0;j<testResults[0]->width;j++) {
			variances->Div(i,j,double(MODELS_FOR_DISAGREEMENT));
			variances->Sqrt(i,j);
		}
}

void GA::CopyBestModel(void) {

	char command[200];

	sprintf(command,"copy %s_%d_body%d.dat %s",EST_FILENAME,simParams->randSeed,simParams->currentCycle,TARGET_DEST);

	system(command);
}

void GA::CreateDCOffspring(void) {

//	Shuffle(parents);

	children = new GENOME * [popSize];

	int tempGenome;
	int numMuts;

	for (tempGenome=0;tempGenome<popSize;tempGenome++) {
		
		children[tempGenome] = new GENOME(parents[tempGenome]);

		if ( simParams->performingEstimation )
			numMuts = simParams->RandInt(1,4);
		else
			numMuts = 1;

		for (int m=0;m<numMuts;m++)
			children[tempGenome]->Mutate();
	}

	/*
	if ( simParams->performingEstimation ) {

		for (tempGenome=0;tempGenome<popSize;tempGenome=tempGenome+2)

			if ( simParams->Rand(0.0,1.0) < CROSSOVER_PROBABILITY )

				children[tempGenome]->Cross(children[tempGenome+1]);
	}
	*/
}

void GA::CreateOffspring(void) {

	//if ( USE_DETERMINISTIC_CROWDING ) {

		//if ( simParams->usePareto )
		//	CreateParetoOffspring();
		//else
			CreateDCOffspring();
	//}
	//else {
	//	if ( simParams->usePareto )
	//		CreateParetoOffspring();
	//	else
	//		CreateTournamentOffspring();
	//}
}

void GA::CreateParetoOffspring(void) {

	FindDominatedGenomes(parents,popSize);

	children = new GENOME * [popSize];

	int nonDominatedGenome;

	int tempGenome;

	for (tempGenome=0;tempGenome<popSize;tempGenome++) {

		if ( parents[tempGenome]->dominated ) {

			nonDominatedGenome = simParams->RandInt(0,popSize-1);

			while ( parents[nonDominatedGenome]->dominated )
				nonDominatedGenome = simParams->RandInt(0,popSize-1);

			children[tempGenome] = new GENOME(parents[nonDominatedGenome]);
		}
		else {

			children[tempGenome] = new GENOME(parents[tempGenome]);
		}

		children[tempGenome]->Mutate();
	}
}

void GA::CreateTournamentOffspring(void) {

	children = new GENOME * [popSize];

	int tempGenome;
	int numMuts;

	GENOME *tourneyWinner;

	for (tempGenome=0;tempGenome<popSize;tempGenome++) {

		tourneyWinner = PerformTournament(parents);

		children[tempGenome] = new GENOME(tourneyWinner);

		if ( simParams->performingEstimation )
			numMuts = simParams->RandInt(1,4);
		else
			numMuts = 1;

		for (int m=0;m<numMuts;m++)
			children[tempGenome]->Mutate();

	}

	for (tempGenome=0;tempGenome<popSize;tempGenome=tempGenome+2)

		if ( simParams->Rand(0.0,1.0) < CROSSOVER_PROBABILITY )

			children[tempGenome]->Cross(children[tempGenome+1]);
}

void GA::DeleteGenomes(void) {

	int tempGenome;

	for (tempGenome=0;tempGenome<popSize;tempGenome++) {

		delete genomes[tempGenome];
		genomes[tempGenome] = NULL;
	}

	delete[] genomes;
	genomes = NULL;
}

void GA::DeleteFitnessDuplicates(GENOME **g, int numG) {

	for (int g1=0;g1<(numG-1);g1++)

		for (int g2=g1+1;g2<numG;g2++)

			if ( g[g1] && 
				 g[g2] && 
				 (g[g1]->fitness          == g[g2]->fitness) && 
				 (g[g1]->secondaryFitness == g[g2]->secondaryFitness) ) {

				delete g[g2];
				g[g2] = NULL;
			}

}

void GA::EvolveForOneGeneration(MATRIX **dataForTarget, MATRIX **dataFromTarget, MATRIX **models) {

	if ( simParams->performingEstimation )

		EvolveForOneGeneration_Estimation(dataForTarget,dataFromTarget);
	else {
		simParams->usePareto = true;
		EvolveForOneGeneration_Exploration(dataForTarget,dataFromTarget,models);
		simParams->usePareto = false;
	}
}

void GA::EvolveForOneGeneration_Estimation(MATRIX **dataForTarget, MATRIX **dataFromTarget) {

	if ( currGeneration == 0 ) {

		genomes = parents;
		parents = NULL;
	}
	else {
		genomes = children;
		children = NULL;
	}

	for (int i=simParams->currentCycle;i>=0;i--) {

		int numToEvaluate = GetNumToEvaluate();

		if ( numToEvaluate > 0 ) {

			WriteGenomesForEvaluation(i,numToEvaluate,dataForTarget);

			simParams->WaitForFile(SENSOR_FILENAME);
		}

		ComputeFitnesses(i,numToEvaluate,dataFromTarget);
	}

	if ( currGeneration == 0 ) {
		parents = genomes;
		genomes = NULL;
	}
	else {
		children = genomes;
		genomes = NULL;
		ReplaceParents();
	}

	genomes = parents;
	parents = NULL;

	Sort();

	if ( !simParams->noPrinting )
		PrintSummary();

	WriteEstReports();

	if ( (stagnationCounter < STAGNATION_LIMIT) && (currGeneration < (totalGenerations-1)) ) {

		// Get ready to spawn.
		parents = genomes;
		genomes = NULL;

		CreateOffspring();
	}
}

void GA::EvolveForOneGeneration_Exploration(MATRIX **dataForTarget, MATRIX **dataFromTarget, MATRIX **models) {

	if ( currGeneration == 0 ) {

		genomes = parents;
		parents = NULL;
	}
	else {
		genomes = children;
		children = NULL;
	}

	WriteMotorProgramsForEvaluation();

	WriteModelsForEvaluation(models);

	simParams->WaitForFile(SENSOR_FILENAME);

	ComputeTestFitnesses();

	Sort();

	PrintSummary();

	WriteExpReports();
}

void GA::InitGA(void) {

	if ( simParams->loadFromFile )
		OpenDataFiles(true);
	else
		OpenDataFiles(false);

	CloseDataFiles();

	nextAvailableID = 0;

	if ( simParams->performingEstimation ) {
		popSize = EST_POPULATION_SIZE;
		totalGenerations = simParams->numEstGenerations;
	}
	else {
		popSize = EXP_POPULATION_SIZE;
		totalGenerations = NUM_EXP_GENERATIONS;
	}

	currGeneration = 0;

	stagnationCounter = 0;

	parents = NULL;
	genomes = NULL;
	children = NULL;
}

void GA::InitGenomes(void) {

	genomes = new GENOME * [popSize];

	int tempGenome;

	if ( simParams->loadFromFile ) {

		LoadState();
	}
	else {

		for (tempGenome=0;tempGenome<popSize;tempGenome++)
			genomes[tempGenome] = new GENOME(nextAvailableID++);
	}

	parents = genomes;
	genomes = NULL;
}

void GA::InitGenomes(MATRIX **bestSoFar) {

	genomes = new GENOME * [popSize];

	int tempGenome;

	if ( simParams->loadFromFile ) {

		LoadState();
	}
	else {

		for (tempGenome=0;tempGenome<popSize;tempGenome++)

			genomes[tempGenome] = new GENOME(bestSoFar[tempGenome],nextAvailableID++);

	}

	parents = genomes;
	genomes = NULL;
}

void GA::LoadState(void) {

	/*
	ifstream *inFile;
	char fileName[50];

	sprintf(fileName,"run%d_currentState.dat",simParams->randSeed);

	inFile = new ifstream(fileName);

	(*inFile) >> currGeneration;

	for (int g=0;g<popSize;g++)
		genomes[g] = new GENOME(nextAvailableID++,inFile);

	inFile->close();
	delete inFile;
	inFile = NULL;
	*/
}

void GA::Fill(void) {

	/*
	int numberOfReplacements = int(popSize * SELECTION_PRESSURE);
	int winner;
	int loser;
	int temp;

	while ( numberOfReplacements > 0 ) {

		winner = simParams->RandInt(0,popSize-1);
		loser  = simParams->RandInt(0,popSize-1);

		while ( winner == loser ) {
			winner = simParams->RandInt(0,popSize-1);
			loser  = simParams->RandInt(0,popSize-1);
		}

		if ( winner > loser ) {

			temp = loser;
			loser = winner;
			winner = temp;
		}

		if ( genomes[winner]->evaluated && genomes[loser]->evaluated ) {

			genomes[winner]->Replace(genomes[loser],nextAvailableID++);

			genomes[loser]->Mutate();

			numberOfReplacements--;
		}
	}
	*/
}

void GA::FindDominatedGenomes(GENOME **g, int numG) {

	int currGenome;
	int otherGenome;

	for (currGenome=0;currGenome<numG;currGenome++) {

		if ( g[currGenome] ) {

			otherGenome = 0;

			g[currGenome]->dominated = false;

			while ( (otherGenome<numG) && (!g[currGenome]->dominated) ) {

				if ( g[otherGenome] && (currGenome!=otherGenome) )
					g[currGenome]->dominated =	(g[currGenome]->fitness			 > g[otherGenome]->fitness) &&
												(g[currGenome]->secondaryFitness > g[otherGenome]->secondaryFitness);

				otherGenome++;
			}
		}
	}
}

int GA::GetNumToEvaluate(void) {

	int total = 0;

	for (int p=0;p<popSize;p++)

		if ( genomes[p]->Valid() && (!genomes[p]->WorseThanParent()) )
		//if ( !genomes[p]->WorseThanParent() )
			total++;

	return( total );
	
	//return( popSize );
}

void GA::OpenDataFiles(int append) {

	if ( simParams->performingEstimation )

		sprintf(fitFileName,"%s_%d_%d_fit.dat",EST_FILENAME,simParams->randSeed,simParams->GetRegimeIndex());

	else
		sprintf(fitFileName,"%s_%d_%d_fit.dat",EXP_FILENAME,simParams->randSeed,simParams->GetRegimeIndex());

	fitFile = new ofstream(fitFileName,ios::app);

	if ( currGeneration == (totalGenerations-1) ) {

		if ( simParams->performingEstimation )

			sprintf(genomeFileName,"%s_%d_%d_body%d.dat",EST_FILENAME,simParams->randSeed,simParams->GetRegimeIndex(),simParams->currentCycle);

		else
			sprintf(genomeFileName,"%s_%d_%d_brain%d.dat",EXP_FILENAME,simParams->randSeed,simParams->GetRegimeIndex(),simParams->currentCycle);

		genomeFile = new ofstream(genomeFileName);
	}
}

GENOME *GA::PerformTournament(GENOME **g) {

	int bestGenomeIndex = 10000;
	int contenderIndex;

	for (int i=0;i<TOURNAMENT_SIZE;i++) {

		contenderIndex = simParams->RandInt(0,popSize-1);

		if ( contenderIndex < bestGenomeIndex )
			bestGenomeIndex = contenderIndex;
	}

	return( g[bestGenomeIndex] );
}

void GA::PrintSummary(void) {

	printf("[C: %d]\tGen %d of %d:\t",simParams->currentCycle,currGeneration,totalGenerations);

	int tempGenome;

	for (tempGenome=0;tempGenome<popSize;tempGenome++) {

		printf("%3.3f ",genomes[tempGenome]->fitness);
	}

	printf("\n");

	//char ch = getchar();
}

void GA::ReplaceParents(void) {

//	if ( !USE_DETERMINISTIC_CROWDING )
//		AntiSort(children);

	//if ( simParams->usePareto )
	//	ReplaceParentsUsingPareto();
	//else
		ReplaceParentsUsingDC();
}

void GA::ReplaceParentsUsingDC(void) {

	double probOfReplacement;

	for (int tempGenome=0;tempGenome<popSize;tempGenome++) {

		if ( children[tempGenome]->fitness < parents[tempGenome]->fitness )
			probOfReplacement = 1.0;
		else
			//probOfReplacement = 1.0 - (double(currGeneration)/double(totalGenerations));
			probOfReplacement = 0.0;

		if ( simParams->Rand(0.0,1.0) < probOfReplacement ) {
			delete parents[tempGenome];
			parents[tempGenome] = children[tempGenome];
			stagnationCounter = 0;
		}
		else
			delete children[tempGenome];
					
		children[tempGenome] = NULL;
	}

	delete[] children;
	children = NULL;
}

void GA::ReplaceParentsUsingPareto(void) {

	GENOME **allGenomes = new GENOME*[popSize*2];

	int g;
	int genomeToMove;

	for (g=0;g<popSize;g++) {
 		allGenomes[g] = children[g];
		children[g] = NULL;
	}

	for (g=popSize;g<popSize*2;g++) {
		allGenomes[g] = parents[g-popSize];
		parents[g-popSize] = NULL;
	}

	DeleteFitnessDuplicates(allGenomes,popSize*2);

	FindDominatedGenomes(allGenomes,popSize*2);

	for (g=0;g<popSize;g++) {

		genomeToMove = simParams->RandInt(0,popSize*2-1);

		while ( (!allGenomes[genomeToMove]) || allGenomes[genomeToMove]->dominated )
			genomeToMove = simParams->RandInt(0,popSize*2-1);

		parents[g] = new GENOME(allGenomes[genomeToMove]);
		parents[g]->fitness = allGenomes[genomeToMove]->fitness;
		parents[g]->secondaryFitness = allGenomes[genomeToMove]->secondaryFitness;

	}

	for (g=0;g<popSize*2;g++) {
		if ( allGenomes[g] ) {
			delete allGenomes[g];
			allGenomes[g] = NULL;
		}
	}

	delete[] allGenomes;

	delete[] children;
	children = NULL;
}

void GA::SaveState(void) {

	int g;

	ofstream *outFile;
	char fileName[50];

	sprintf(fileName,"run%d_currentState.dat",simParams->randSeed);

	outFile = new ofstream(fileName);

	(*outFile) << currGeneration << "\n";

	for (g=0; g<popSize; g++)
		genomes[g]->Save(outFile);

	outFile->close();
	delete outFile;
	outFile = NULL;
}

void GA::Shuffle(GENOME **g) {

	MATRIX *perm = new MATRIX(1,popSize,0.0);
	perm->CreatePermutation(0,popSize-1);

	GENOME **temp = new GENOME * [popSize];

	for (int i=0;i<popSize;i++) {
		temp[i] = g[i];
		g[i] = NULL;
	}

	for (i=0;i<popSize;i++) {
		g[int(perm->Get(0,i))] = temp[i];
		temp[i] = NULL;
	}

	delete[] temp;
	temp = NULL;

	delete perm;
	perm = NULL;
}

void GA::Sort(void) {

   int bound = popSize-1;
   int t;
   int j;
   double firstFitness, secondFitness;

   GENOME *firstGenome, *secondGenome;

   do {
       t = 0;
       for(j = 0; j < bound; ++j) {

			  firstGenome = genomes[j];
			  secondGenome = genomes[j+1];

			  firstFitness = firstGenome->fitness;
			  secondFitness = secondGenome->fitness;

           if( firstFitness > secondFitness ) {
					SwitchGenomes(genomes,j,j+1);
					t = j;
           }
       }
       bound = t;
   } while (t != 0);
}

void GA::SwitchGenomes(GENOME **g, int firstGenome, int secondGenome) {

	GENOME *tempGenome;

	tempGenome = g[secondGenome];
	g[secondGenome] = g[firstGenome];
	g[firstGenome] = tempGenome;
}

void GA::WriteAll(void) {

	int tempGenome;

	(*fitFile) << simParams->currentCycle << "\t";
	(*fitFile) << currGeneration << "\t";
	(*fitFile) << simParams->totalModelEvals << "\t";

	for (tempGenome=0;tempGenome<popSize;tempGenome++)

		(*fitFile) << genomes[tempGenome]->fitness << "\t";

	(*fitFile) << "\n";
}

void GA::WriteBest(void) {

	(*genomeFile) << popSize << "\n";

	if ( simParams->performingEstimation ) {

		for (int i=0;i<popSize;i++)
			genomes[i]->Save(genomeFile);
	}
	else {

		MATRIX *motorProgram;

		int selectedJoint1 = simParams->allTests->Get(genomes[0]->ID,0);
		int selectedJoint2 = simParams->allTests->Get(genomes[0]->ID,1);

		motorProgram = WriteMotorProgramForEvaluation(selectedJoint1,selectedJoint2);

		motorProgram->Write(genomeFile);

		delete motorProgram;
		motorProgram = NULL;
	}
}

void GA::WriteEstReports(void) {

	OpenDataFiles(true);

	WriteAll();

	// Create body phylogeny file
	char fileName[100];
	sprintf(fileName,"%s_%d_%d_bodies.dat",EST_FILENAME,simParams->randSeed,simParams->GetRegimeIndex());
	simParams->bodyPhylogenyFile = new ofstream(fileName,ios::app);

	genomes[0]->Save(simParams->bodyPhylogenyFile);

	simParams->bodyPhylogenyFile->close();
	delete simParams->bodyPhylogenyFile;
	simParams->bodyPhylogenyFile = NULL;

	// Create stick phylogeny files
	for (int i=0;i<MODELS_FOR_DISAGREEMENT;i++) {

		sprintf(fileName,"%s_%d_%d_sticks%d.dat",EST_FILENAME,simParams->randSeed,simParams->GetRegimeIndex(),i);
		simParams->stickPhylogenyFile = new ofstream(fileName,ios::app);

		genomes[i]->SaveSticks(simParams->stickPhylogenyFile);

		simParams->stickPhylogenyFile->close();
		delete simParams->stickPhylogenyFile;
		simParams->stickPhylogenyFile = NULL;
	}

	if ( currGeneration == (totalGenerations-1) )
		WriteBest();

	CloseDataFiles();
}

void GA::WriteExpReports(void) {

	OpenDataFiles(true);

	WriteAll();

	WriteBest();

	CloseDataFiles();
}

void GA::WriteGenomesForEvaluation(int currentTest, int numToEvaluate, MATRIX **dataForTarget) {

	ofstream *bodyFile;
	ofstream *brainFile;

	bodyFile = new ofstream(TEMP_FILENAME);
	brainFile = new ofstream(TEMP2_FILENAME);

	(*bodyFile) << numToEvaluate << "\n";
	(*brainFile) << 1 << "\n";

	for (int p=0;p<popSize;p++)

		if ( genomes[p]->Valid() && (!genomes[p]->WorseThanParent()) ) {
			genomes[p]->WriteBody(bodyFile);
			simParams->totalModelEvals++;
		}

	dataForTarget[currentTest]->Write(brainFile);

	bodyFile->close();
	delete bodyFile;
	bodyFile = NULL;
	
	brainFile->close();
	delete brainFile;
	brainFile = NULL;

	simParams->FileRename(TEMP_FILENAME,BODY_OUT_FILENAME);
	simParams->FileRename(TEMP2_FILENAME,NN_OUT_FILENAME);
}

void GA::WriteModelForEvaluation(MATRIX *currentModel, ofstream *bodyFile) {

	simParams->performingEstimation = true;

	GENOME *currentM = new GENOME(currentModel,0);

	simParams->performingEstimation = false;

	currentM->WriteBody(bodyFile);

	delete currentM;
	currentM = NULL;
}

void GA::WriteModelsForEvaluation(MATRIX **models) {

	int i,j;

	ofstream *bodyFile;

	bodyFile = new ofstream(TEMP_FILENAME);
	(*bodyFile) << popSize*MODELS_FOR_DISAGREEMENT*2 << "\n";

	for (j=0;j<MODELS_FOR_DISAGREEMENT;j++) {

		for (i=0;i<popSize;i++) {

			WriteModelForEvaluation(models[j],bodyFile);
			simParams->totalModelEvals++;
		}

		simParams->perturbModel = true;

		for (i=0;i<popSize;i++) {

			WriteModelForEvaluation(models[j],bodyFile);
			simParams->totalModelEvals++;
		}

		simParams->perturbModel = false;
	}

	bodyFile->close();
	delete bodyFile;
	bodyFile = NULL;

	simParams->FileRename(TEMP_FILENAME,BODY_OUT_FILENAME);
}

MATRIX *GA::WriteMotorProgramForEvaluation(int currentTest) {

	double startCommand;
	double endCommand;
	double diff;
	double command;

	MATRIX *motorProgram = new MATRIX(EVAL_PERIOD+1,NUM_STICKS-1);

	int selectedJoint1 = simParams->allTests->Get(currentTest,0);
	int selectedJoint2 = simParams->allTests->Get(currentTest,1);

	for (int currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++) {

		startCommand = 0.0;

		for (int i=0;i<EVAL_PERIOD;i=i+MOTOR_STEP_SIZE) {

			if ( (currentJoint==selectedJoint1) || (currentJoint==selectedJoint2) )
				endCommand = MIN_RANGE_OF_MOTION;
			else
				endCommand = MAX_RANGE_OF_MOTION;

			diff = (endCommand - startCommand) / (MOTOR_STEP_SIZE-1);

			for (int k=i;k<i+MOTOR_STEP_SIZE;k++) {

				command = startCommand + (k-i)*diff;
				motorProgram->Set(k,currentJoint,command);
			}

			startCommand = endCommand;
		}
		motorProgram->Set(EVAL_PERIOD,currentJoint,endCommand);
	}

	for (currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++)

		for (int i=MOTOR_STEP_SIZE;i<=EVAL_PERIOD;i++) {
		
			double mCommand = motorProgram->Get(MOTOR_STEP_SIZE-1,currentJoint);

			motorProgram->Set(i,currentJoint,mCommand);
		}

	return( motorProgram );
}

MATRIX *GA::WriteMotorProgramForEvaluation(int selectedJoint1, int selectedJoint2) {

	double startCommand;
	double endCommand;
	double diff;
	double command;

	MATRIX *motorProgram = new MATRIX(EVAL_PERIOD+1,NUM_STICKS-1);

	for (int currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++) {

		startCommand = 0.0;

		for (int i=0;i<EVAL_PERIOD;i=i+MOTOR_STEP_SIZE) {

			if ( (currentJoint==selectedJoint1) || (currentJoint==selectedJoint2) )
				endCommand = MIN_RANGE_OF_MOTION;
			else
				endCommand = MAX_RANGE_OF_MOTION;

			diff = (endCommand - startCommand) / (MOTOR_STEP_SIZE-1);

			for (int k=i;k<i+MOTOR_STEP_SIZE;k++) {

				command = startCommand + (k-i)*diff;
				motorProgram->Set(k,currentJoint,command);
			}

			startCommand = endCommand;
		}
		motorProgram->Set(EVAL_PERIOD,currentJoint,endCommand);
	}

	for (currentJoint=0;currentJoint<NUM_STICKS-1;currentJoint++)

		for (int i=MOTOR_STEP_SIZE;i<=EVAL_PERIOD;i++) {
		
			double mCommand = motorProgram->Get(MOTOR_STEP_SIZE-1,currentJoint);

			motorProgram->Set(i,currentJoint,mCommand);
		}

	return( motorProgram );
}

void GA::WriteMotorProgramsForEvaluation(void) {

//	popSize = simParams->totalTests-simParams->totalTargetEvals;
	popSize = simParams->totalTests;

	ofstream *brainFile = new ofstream(TEMP2_FILENAME);
	(*brainFile) << popSize << "\n";

	MATRIX *motorProgram;

	for (int i=0;i<simParams->totalTests;i++) {

		//if ( simParams->allTests->Get(i,2)==0.0 ) {

			motorProgram = WriteMotorProgramForEvaluation(i);

			motorProgram->Write(brainFile);

			delete motorProgram;
			motorProgram = NULL;
		//}
	}

	brainFile->close();
	delete brainFile;
	brainFile = NULL;

	simParams->FileRename(TEMP2_FILENAME,NN_OUT_FILENAME);
}

#endif