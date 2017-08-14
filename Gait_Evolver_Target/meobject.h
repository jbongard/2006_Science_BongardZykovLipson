/* ---------------------------------------------------
   FILE:     meObject.h
	AUTHOR:   Josh Bongard
	DATE:     February 15, 2001
	FUNCTION: This class contains all information for
				 a generic mathEngine object.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"

#include "agent.h"
#include "trajectory.h"

#include <ode/ode.h>

#ifndef _ME_OBJECT_H
#define _ME_OBJECT_H

class ME_OBJECT {

public:
	int ID;
	class AGENT		*containingAgent;
	char			name[50];
	char			textureName[50];
	int				coverWithTexture;
	double			position[3];
	double			rotation[3];
	double			size[3];
	double			tempSize[3];
	double			originalPosition[3];
	double			originalSize[3];
	dBodyID			body;
	double			weight;
	double			tempMass;
	int				touchSensorID;
	int				lightSensorID;
	int				chemoSensorID;
	int				agentSensorID;
	int				attachmentSensorID;
	int				detacherID;
	int				morphParamIndices[5];
	int				numOfMorphParamsINeed;
	int				isAttacher;
	int				drawTouchdowns;
	int				isAttached;
	ME_OBJECT		*objICanPassThrough;
	char			objICanPassThroughName[50];
	double			minDistForLocking;
	double			minForceForUnlocking;
	int				contactWithFloor;
	int				objectType;
	ME_OBJECT		*startOfAggregate;
	int				fixedToWorld;
	NEURAL_NETWORK	*nn;
	dGeomID			distanceSensor;
	dGeomID			geom;
	double			volume;

private:
	dGeomID			encapsulatedGeom;
	dMass			mass;
	float 			colour[4];
	double			minMass;
	double			minHeight, maxHeight;
	ME_OBJECT		*massLockedWith;
	char			massLockedWithName[50];
	ME_OBJECT		*heightLockedWith;
	char			heightLockedWithName[50];
	double			minXPosition, maxXPosition;
	double			minYPosition, maxYPosition;
	double			minZPosition, maxZPosition;
	ME_OBJECT		*sourceObject, *targetObject;
	TRAJECTORY		*touchdowns;
	double			amountOfPerturbation;

public:
	ME_OBJECT(dWorldID world, dSpaceID space);
	ME_OBJECT(int oType, double *s, double *p);
	ME_OBJECT(AGENT *a, ME_OBJECT *templateObj, double disp[2], class NEURAL_NETWORK *network);
	ME_OBJECT(AGENT *a, int numOfObjects, ifstream *bodyFile,
			    int inputID, class NEURAL_NETWORK *network, ME_OBJECT **objs);
	~ME_OBJECT(void);
	int   BeyondRange(void);
	int   BreakageDesired(NEURAL_NETWORK *network);
	int   CloseEnough(ME_OBJECT *otherObj);
	int   CloseEnoughToPlume(double currHeight, double plumeHeight);
	int	  CountMorphParams(int morphParamsSoFar);
	void  CreateODEStuff(dWorldID world, dSpaceID space);
	void  Darken(double amt);
	void  DestroyODEStuff(dSpaceID space);
	void  DisableContacts(void);
	void  Display(void);
	void  Draw(void);
	void  DrawSensorNeurons(double maxHeight, double offset);
	int   EvolvableHeight(void);
	int   EvolvableMass(void);
	ME_OBJECT* FindObject(ME_OBJECT **objs, int numObjs, char *name);
	void  FindOtherObjects(int numOfObjs, ME_OBJECT **objs);
	double GetDistance(ME_OBJECT *otherObject);
	double GetForward(void);
	double GetHeight(void);
	double GetLeft(void);
	void  GetPosAndRot(dReal *act_pos, dReal *actual_R);
	void  IncreasePerturbation(double amt);
	void  IntroduceNoise(void);
	void  InvertColour(void);
	int   IsBelow(double verticalPos);
	void  IsInside(ME_OBJECT *otherObj, double *dist);
	int	  IsPartOfEnvironment(void);
	void  Lighten(void);
	int   PartOfDebris(void);
	void  Push(double x, double y, double z);
	void  RecordTouchdown(double *pos);
	void  Reposition(double x, double y, double z);
	int   RequiresGeneticSpecification(void);
	void  Reset(void);
	void  Reset(dWorldID world, dSpaceID space);
	void  SaveToRaytracerFile(ofstream *outFile);
	void  SetColour(float r, float g, float b);
	void  SetEvolvableParams(int numMorphParams, int genomeLength, double *genes);
	void  SetParamsBasedOnOtherObjects(void);
	void  SetOrientationSensors(void);
	void  SetPressureSensor(void);
	void  SwitchToGeomTransform(ME_OBJECT *otherObj, dSpaceID space);
	void  ToggleFootprintDrawing(void);
	void  TouchingGround(dVector3 contactPosition);

private:
	void AdjustHeight(double morphParam);
	void AdjustMass(double morphParam);
	void AdjustRotation(void);
	void AdjustX(double morphParam);
	void AdjustY(double morphParam);
	void AdjustZ(double morphParam);
	void Draw(dGeomID g, const dReal *pos, const dReal *R);
	void InvertRotation(dReal *rot, double *theta);
	void LockHeightTo(void);
	void LockMassTo(void);
	int  NumberEntered(char *name);
	void ReadIn(ifstream *envFile, NEURAL_NETWORK *network, ME_OBJECT **objs);
	void RotToAxisAndAngle(dReal *rot, dReal *axis, dReal *theta);
	void SetColour(void);
	void SetDefaultProperties(void);
	void SetLightSourceProperties(void);
};

#endif