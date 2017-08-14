/* ---------------------------------------------------
   FILE:     debris.cpp
	AUTHOR:   Josh Bongard
	DATE:     May 10, 2003
	FUNCTION: This class contains all information for
			  a single collection of debris.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _DEBRIS_CPP
#define _DEBRIS_CPP

#include "debris.h"

#include "simParams.h"
extern SIM_PARAMS *simParams;

extern int   DEBRIS_NUM_OF_RUBBLE;
extern int   DEBRIS_LOOSE_RUBBLE;
extern int	 DEBRIS_FIXED_RUBBLE;
extern int   DEBRIS_EMBEDDED_RUBBLE;
extern int	 DEBRIS_UNEVEN_FLOOR;
extern int   DEBRIS_RAMP;

extern double DEBRIS_RUBBLE_RADIUS;
extern double DEBRIS_UNEVEN_FLOOR_HEIGHT;

extern int	 SPHERE;
extern int	 CYLINDER;
extern int	 RECTANGLE;

extern int	 OUTPUT_DEBRIS;
extern int	 DEBRIS_OBJECT_ID;

DEBRIS::DEBRIS(int debrisType, double agentWidth, double agentLength) {
	
	type = debrisType;
	
	joints = NULL;
	numJoints = 0;

	if ( type == DEBRIS_LOOSE_RUBBLE )
		CreateLooseRubble();

	if ( type == DEBRIS_FIXED_RUBBLE )
		CreateFixedRubble();

	if ( type == DEBRIS_EMBEDDED_RUBBLE )
		CreateEmbeddedRubble();

	if ( type == DEBRIS_UNEVEN_FLOOR )
		CreateUnevenFloor(agentWidth,agentLength);

	if ( type == DEBRIS_RAMP )
		CreateRamp();

	if ( OUTPUT_DEBRIS )
		RecordDebris();
}

DEBRIS::~DEBRIS(void) {

	for (int i=0;i<numObjects;i++) {
		delete objects[i];
		objects[i] = NULL;
	}

	delete[] objects;
	objects = NULL;

	/*
	if ( joints != NULL ) {

		for (int i=0;i<numJoints;i++) {
			delete joints[i];
			joints[i] = NULL;
		}
		delete[] joints;
		joints = NULL;
	}
	*/
}

void DEBRIS::CreateODEStuff(dWorldID world, dSpaceID space) {

	for (int i=0;i<numObjects;i++) {
		objects[i]->CreateODEStuff(world,space);
		joints[i]->AttachToWorld(world,objects[i]);
	}

	/*
	if ( (type == DEBRIS_FIXED_RUBBLE) ||
		 (type == DEBRIS_EMBEDDED_RUBBLE) ||
		 (type == DEBRIS_RAMP) ||
		 (type == DEBRIS_UNEVEN_FLOOR) )

		for (int i=0;i<numJoints;i++)
			joints[i]->AttachToWorld(world,objects[i]);
	*/
}

void DEBRIS::DestroyODEStuff(dSpaceID space) {

	for (int i=0;i<numObjects;i++)
		objects[i]->DestroyODEStuff(space);

	/*
	if ( joints != NULL )
		for (int i=0;i<numJoints;i++)
			joints[i]->DestroyODEStuff();
	*/
}

void DEBRIS::Draw(void) {

	for (int i=0;i<numObjects;i++)
		objects[i]->Draw();
}

void DEBRIS::Move(dWorldID world, dSpaceID space, double *agentPos, double agentWidth, double agentLength) {

	double myCOM[3];

	GetCentreOfMass(myCOM);

	/*
	if ( simParams->internalTimer%10 == 0 )
		if ( simParams->RandInt(0,1) )
			RollRight(world,space);
		else
			RollLeft(world,space);

	for (int i=0;i<numObjects;i++) {

		dBodySetPosition(objects[i]->body,
						objects[i]->position[0],
						objects[i]->position[1],
						objects[i]->position[2]);
	}
	*/

	// ( (agentPos[0]+(agentLength/2.0)) < myCOM[0] ) {

		// FORWARD
		//RollForward(world,space);
	/*
		printf("too far forward\n");
	}
	else if ( (agentPos[0]-(agentLength/2.0)) > myCOM[0] ) {

		// BACK		
		printf("too far back\n");
	}
	else if ( (agentPos[1]+(agentWidth/2.0)) < myCOM[1] ) {

		// LEFT
		RollLeft(world,space);
	}
	else if ( (agentPos[1]-(agentWidth/2.0)) > myCOM[1] ) {

		// RIGHT
		RollRight(world,space);
	}
	*/
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void DEBRIS::CreateEmbeddedRubble(void) {

	CreateFixedRubble();

	for (int i=0;i<numObjects;i++)
		objects[i]->position[2] = 0.0;
}

void DEBRIS::CreateLooseRubble(void) {

	numObjects = DEBRIS_NUM_OF_RUBBLE;

	objects = new ME_OBJECT * [numObjects];

	for (int i=0;i<numObjects;i++) {
		double rubbleRadius = simParams->Rand(DEBRIS_RUBBLE_RADIUS/2.0,DEBRIS_RUBBLE_RADIUS);
		
		int rubbleType = SPHERE;
		
		double pos[3];
		double size[3];

		pos[0] = simParams->Rand(-20.0,0.0);
		pos[1] = simParams->Rand(-5.0,5.0);
		
		if ( rubbleType == SPHERE )
			pos[2] = rubbleRadius;
		else
			pos[2] = rubbleRadius/2.0;

		size[0] = rubbleRadius;
		size[1] = rubbleRadius;
		size[2] = rubbleRadius;

		objects[i] = new ME_OBJECT(rubbleType,size,pos);
	}
}

void DEBRIS::CreateFixedRubble(void) {

	CreateLooseRubble();

	numJoints = numObjects;
	joints = new ME_JOINT * [numJoints];

	for (int i=0;i<numJoints;i++)
		joints[i]  = new ME_JOINT;
}

void DEBRIS::CreateRamp(void) {

	CreateFixedRubble();

	double currDistance = -10.0;
	double currHeight = 0.05;

	for (int i=0;i<numObjects;i++) {

		objects[i]->objectType = RECTANGLE;
		objects[i]->size[2] = 0.05;
		objects[i]->size[0] = 20.0;
		objects[i]->size[1] = 20.0;

		objects[i]->tempSize[0] = objects[i]->size[0];
		objects[i]->tempSize[1] = objects[i]->size[1];
		objects[i]->tempSize[2] = objects[i]->size[2];

		objects[i]->position[0] = currDistance;
		objects[i]->position[1] = 0.0;
		objects[i]->position[2] = currHeight;

		currHeight = currHeight + 0.05;
		currDistance = currDistance - simParams->Rand(0.1,1.0);
	}
}

void DEBRIS::CreateUnevenFloor(double agentWidth, double agentLength) {

	agentWidth = agentWidth * 3.0;
	agentLength = agentLength * 3.0;

	numObjects = DEBRIS_NUM_OF_RUBBLE;

	objects = new ME_OBJECT * [numObjects];

	numJoints = numObjects;
	joints = new ME_JOINT * [numJoints];

	double position[3];
	double size[3];
	int rubbleType = RECTANGLE;

	int blocksPerRow = int( sqrt(numObjects) );

	size[0] = agentWidth/double(blocksPerRow) - 0.1;
	size[1] = agentLength/double(blocksPerRow) - 0.1;

	position[1] = -agentLength/2.0 + (size[0]/2.0);

	int k=0;

	for (int i=0;i<blocksPerRow;i++) {

		position[0] = -agentWidth/2.0 + (size[2]/2.0);

		for (int j=0;j<blocksPerRow;j++) {

			size[2] = simParams->Rand(0.1,DEBRIS_UNEVEN_FLOOR_HEIGHT*10.0);
			position[2] = size[2]/2.0;

			objects[k] = new ME_OBJECT(rubbleType,size,position);
			objects[k]->ID == DEBRIS_OBJECT_ID;

			joints[k] = new ME_JOINT;
	
			k++;

			position[0] = position[0] + agentWidth/double(blocksPerRow);
		}

		position[1] = position[1] + agentLength/double(blocksPerRow);
	}
}

void DEBRIS::GetCentreOfMass(double *com) {

	com[0] = 0.0;
	com[1] = 0.0;
	com[2] = 0.0;

	for (int i=0;i<numObjects;i++) {

		com[0] = com[0] + objects[i]->position[0];
		com[1] = com[1] + objects[i]->position[1];
		com[2] = com[2] + objects[i]->position[2];
	}
	
	com[0] = com[0] / numObjects;
	com[1] = com[1] / numObjects;
	com[2] = com[2] / numObjects;
}

void DEBRIS::RecordDebris(void) {

	if ( simParams->debrisFile )

		for (int i=0;i<numObjects;i++) {
			(*simParams->debrisFile) << objects[i]->position[0] << ";";
			(*simParams->debrisFile) << objects[i]->position[1] << ";";
			(*simParams->debrisFile) << objects[i]->position[2] << ";";
			(*simParams->debrisFile) << objects[i]->size[0] << "\n";
		}
}

void DEBRIS::RollLeft(dWorldID world, dSpaceID space) {

	int blocksPerRow = int( sqrt(numObjects) );
	ME_OBJECT *tempObject;

	int i,j;

	for (i=0;i<blocksPerRow;i++) {

		tempObject = objects[(numObjects-1)-i];

		for (j=(numObjects-1-i);j>=blocksPerRow;j=j-blocksPerRow) {

			objects[j] = objects[j-blocksPerRow];
		}

		tempObject->position[1] = tempObject->position[1] - (tempObject->size[1]*blocksPerRow);

		dBodySetPosition(tempObject->body,
						 tempObject->position[0],
						 tempObject->position[1],
						 tempObject->position[2]);

		objects[(blocksPerRow-1)-i] = tempObject;
	}
}

void DEBRIS::RollRight(dWorldID world, dSpaceID space) {

	int blocksPerRow = int( sqrt(numObjects) );
	ME_OBJECT *tempObject;

	int i,j;

	for (i=0;i<blocksPerRow;i++) {

		tempObject = objects[i];

		for (j=i;j<=(numObjects-1)-blocksPerRow;j=j+blocksPerRow) {

			objects[j] = objects[j+blocksPerRow];
		}

		tempObject->position[1] = tempObject->position[1] + (tempObject->size[1]*blocksPerRow);

		dBodySetPosition(tempObject->body,
						 tempObject->position[0],
						 tempObject->position[1],
						 tempObject->position[2]);

		objects[(numObjects-1)-(blocksPerRow-1)+i] = tempObject;
	}
}

#endif