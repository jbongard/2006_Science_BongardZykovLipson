/* ---------------------------------------------------
   FILE:     environment.h
	AUTHOR:   Josh Bongard
	DATE:     February 14, 2001
	FUNCTION: This class contains all information for
				 a set of objects in a MathEngine world.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _ENVIRONMENT_H
#define _ENVIRONMENT_H

#include "drawstuff/drawstuff.h"

#include "agent.h"
#include "debris.h"
#include "plume.h"
#include "fstream.h"

class ENVIRONMENT {

public:
	int							evalPeriod;
	int							numEvaluations;
	int							addTime;
	int							multTime;
	int							numOfAgents;
	int							numOfDebrisFields;
	AGENT						**agents;
	DEBRIS						**debrisFields;
	dWorldID					world;
	dSpaceID					space;
	dJointGroupID				contactgroup;
	PLUME						*plume;
	int							useReactiveController;
	dGeomID						floor;

private:
	ifstream					*envFile;
	int							defaultObjectMaterial;
	int							terminationCriteria;
	ME_OBJECT					*objectToMove;
	ME_OBJECT					*addObjectToMove;
	ME_OBJECT					*add2ObjectToMove;
	ME_OBJECT					*multObjectToMove;
	ME_OBJECT					*mult2ObjectToMove;
	ME_OBJECT					*objectNotToDrop;
	double						verticalThreshold;
	int							populationSize;
	int							generations;
	double						mutationRate;
	ME_OBJECT					*lightSource;
	int							hiddenNodes;
	double						cameraDistance;
	int							numMorphParams;
	int							genomeLength;
	ofstream					*raytracerFile;
	double						restitution;
	double						softness;

public:
	ENVIRONMENT(void);
	~ENVIRONMENT(void);
	int			AgentAvailable(void);
	void		ChangeDebrisField(void);
	void		ChangePlumePtSource(int evalIndex);
	void		CreateAgent(void);
	void		CreatePlume(void);
	void		DestroyAgent(void);
	void		DestroyPlume(void);
	void		Draw(void);
	int			EvaluationFinished(void);
	double		GetFitness(void);
	void		GetObjectPosition(char *objName, double **pos);
	void		HandleCollisions(void);
	void		IncreasePerturbation(double amt);
	void        MoveAgents(void);
	void		MoveDebris(void);
	void		PerturbSynapses(void);
	void		Push(double x, double y, double z);
	void		RefreshTouchPoints(void);
	void		Reset(void);
	void		ResetTouchSensors(void);
	void		SaveToRaytracerFile(void);
	void		ToggleFootprintDrawing(void);
	void		ToggleTrajectoryDrawing(void);

private:
	int  AgentExploding(void);
	int  AgentTipping(void);
	void AttemptAttachment(void);
	void CloseRaytracerFile(void);
	int  CollisionBetweenAgents(void);
	int  CountTotalJoints(void);
	int  CountTotalObjects(void);
	int  CountTotalPossibleJoints(void);
	void CreateDebris(void);
	void CreateFloor(void);
	void CreateLightSource(int currEval);
	void CreateODEStuff(void);
	void DestroyDebris(void);
	void DestroyFloor(void);
	void DestroyLightSource(void);
	void DestroyODEStuff(void);
	void DetermineFitnessCriteria(AGENT *currAgent);
	ME_OBJECT* FindObject(char *name, AGENT *currAgent);
	double GetDistToLight(void);
	double GetDistToPtSource(void);
	double GetForwardDist(void);
	double GetMaxHeight(void);
	double GetNumOfExternalJoints(void);
	void OpenRaytracerFile(void);
	void ReadFromFile(void);
	void RepositionLightSource(int currEval);
	double SumOfDists(void);
};

#endif