/* ---------------------------------------------------
   FILE:     agent.h
	AUTHOR:   Josh Bongard
	DATE:     February 19, 2002
	FUNCTION: This class contains all information for
				 a single agent.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _AGENT_H
#define _AGENT_H

#include "nnetwork.h"
#include "meobject.h"
#include "mejoint.h"
#include "plume.h"
#include "trajectory.h"

class AGENT {

public:
	int							ID;
	int							numObjects;
	int							numJoints;
	NEURAL_NETWORK				*network;
	int							numMorphParams;
	int							numOfExternalJoints;
	TRAJECTORY					*trajectory;
	MATRIX						*sensorValues;
	ME_OBJECT					**objects;
	ME_JOINT					**joints;
	double						totalMass;
	double						totalVolume;

private:
	int							hiddenNodes;
	double						maxLightSensorReading;
	double						displacement[2];
	ME_JOINT					**externalJoints;
	int							numOfPossibleExternalJoints;
	ME_OBJECT					*targetObject;
	int							timeRemainingUntilNextMotorCommand;

public:
	AGENT(int IDNum, dWorldID world, dSpaceID space);
	AGENT(int IDNum, AGENT *templateAgent);
	~AGENT(void);
	int			AttachedTo(AGENT *otherAgent);
	void		AttachTo(AGENT *otherAgent, dWorldID world);
	int			ContainsLightSensor(void);
	void		CreateJoints(dWorldID world);
	void		CreateObjects(int genomeLength, double *genes, ME_OBJECT *lightSource,
							 dWorldID world, dSpaceID space);
	void		CreateODEStuff(dWorldID world, dSpaceID space);
	void		DestroyODEStuff(dSpaceID space);
	double		DistTo(int objIndex, AGENT *otherAgent);
	void		Draw(void);
	int			Exploding(void);
	void		GetCentreOfMass(double *com);
	int			GetID(void);
	ME_JOINT	*GetJoint(int jointIndex);
	ME_JOINT	*GetJoint(char *jointName);
	ME_OBJECT	*GetObject(int objIndex);
	double		GetMaxHeight(void);
	double      GetMinHeight(void);
	char		*GetName(int objIndex);
	int			GetNumOfJoints(void);
	int			GetNumOfObjects(void);
	int			GetNumOfPossibleJoints(void);
	int			GetNumOfWeights(void);
	double		GetWidth(void);
	double		GetLength(void);
	void		HandleCollisions(int internalTimer);
	void		IncreasePerturbation(double amt);
	void		InitNetwork(int hiddenN, int useReactiveController);
	void		LabelSynapses(int genomeLength, double *genes);
	void		Move(int numOfAgents,ME_OBJECT *lightSource, PLUME *plume, dWorldID world, dGeomID floor);
	void		PerturbSynapses(void);
	void		Push(double x, double y, double z);
	void		RemoveExtraneousObjects(dSpaceID space);
	void		Reset(void);
	void		ResetTouchSensors(void);
	void		SaveToRaytracerFile(ofstream *outFile);
	void		SaveCentreOfMass(double *com);
	void		SpecifyPhenotype(int genomeLength, double *genes);
	void		SwitchOnPressureSensors(dWorldID world);
	void		ToggleFootprintDrawing(void);
	void		ToggleTrajectoryDrawing(void);

private:
	void ActuateJoints(void);
	void BreakExternalConnectionsIfNecessary(void);
	int  CountPossibleExternalJoints(void);
	void CreateBody(dWorldID world, dSpaceID space);
	void CreateBrain(void);
	void DetachJoints(void);
	void DetermineMaxLightSensorReading(ME_OBJECT *lightSource);
	void DrawJointNormals(void);
	void DrawNeuralNetwork(void);
	void FindRelatedJoints(void);
	void IntroduceNoise(void);
	void ReadInDataParameters(void);
	void ReadInJoint(int currJoint);
	void ReadInObject(int currObject);
	void RecordObjectData(void);
	void RecordSensorData(void);
	void RecordCentreOfMass(double *com);
	void TakeAgentSensorReadings(int numOfAgents);
	void TakeAngleSensorReadings(void);
	void TakeAttachmentSensorReadings(void);
	void TakeChemoSensorReadings(PLUME *plume);
	void TakeDistanceSensorReading(dGeomID floor);
	void TakeLightSensorReadings(ME_OBJECT *lightSource);
	void TakeLowestHeightSensorReading(void);
	void TakeObj0HeightSensorReading(void);
	void TakeOrientationSensorReadings(void);
	void TakePressureSensorReadings(dGeomID floor);
	void TakeSensorReadings(int numOfAgents, ME_OBJECT *lightSource, PLUME *plume, dGeomID floor);
	void TakeTouchSensorReadings(void);
	void TakeVelocitySensorReadings(void);
	void UpdateSpringConnections(void);

};

#endif