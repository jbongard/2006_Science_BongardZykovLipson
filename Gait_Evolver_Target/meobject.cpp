/* ---------------------------------------------------
   FILE:     meObject.cpp
	AUTHOR:   Josh Bongard
	DATE:     February 15, 2001
	FUNCTION: This class contains all information for
				 a generic mathEngine object.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _ME_OBJECT_CPP
#define _ME_OBJECT_CPP

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "constants.h"
#include "meobject.h"
#include "nnetwork.h"
#include "simParams.h"

extern double		PI;
extern double		LIGHT_SOURCE_DISTANCE;
extern double		LIGHT_SOURCE_SIZE;
extern SIM_PARAMS	*simParams;
extern double		PLUME_DEPTH;
extern double		BELLY_SENSOR_DISTANCE;

#ifdef dDOUBLE
#define dsDrawLine dsDrawLineD
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

ME_OBJECT::ME_OBJECT(dWorldID world, dSpaceID space) {

	ID = 0;
	containingAgent = NULL;

	touchSensorID = -1;
	lightSensorID = -1;
	chemoSensorID = -1;
	agentSensorID = -1;
	attachmentSensorID = -1;
	detacherID = -1;
	nn = NULL;
	startOfAggregate = this;
	numOfMorphParamsINeed = 0;
	sourceObject = NULL;
	targetObject = NULL;
	amountOfPerturbation = 0.0;
	body = 0;

	isAttacher = false;
	isAttached = false;

	fixedToWorld = false;

	minDistForLocking = 100.0;
	minForceForUnlocking = 1000.0;

	minMass = -100.0;
	minHeight = -100.0;
	maxHeight = -100.0;

	drawTouchdowns = false;
	touchdowns = NULL;

	objICanPassThrough = NULL;
	massLockedWith = NULL;
	heightLockedWith = NULL;
	strcpy(objICanPassThroughName,"");
	strcpy(massLockedWithName,"");
	strcpy(heightLockedWithName,"");

	coverWithTexture = false;

	SetDefaultProperties();
	SetLightSourceProperties();
	CreateODEStuff(world,space);
}

ME_OBJECT::ME_OBJECT(int oType, double *s, double *p) {

	ID = -1;

	objectType = oType;
	
	size[0] = s[0];
	size[1] = s[1];
	size[2] = s[2];

	tempSize[0] = s[0];
	tempSize[1] = s[1];
	tempSize[2] = s[2];

	numOfMorphParamsINeed = 0;
	sourceObject = NULL;
	targetObject = NULL;

	position[0] = p[0];
	position[1] = p[1];
	position[2] = p[2];
	body = 0;

	rotation[0] = 0.0;
	rotation[1] = 0.0;
	rotation[2] = 0.0;

	//weight = size[0];
	weight = 1000.0;

	drawTouchdowns = false;
	touchdowns = NULL;

	contactWithFloor = true;

	fixedToWorld = false;
	amountOfPerturbation = 0.0;
}

ME_OBJECT::ME_OBJECT(AGENT *a, ME_OBJECT *templateObj, double disp[2], NEURAL_NETWORK *network) {

	containingAgent = a;
	ID = templateObj->ID;
	startOfAggregate = this;

	sprintf(name,"%s_%d",templateObj->name,a->ID);
	//strcpy(name,templateObj->name);

	nn = network;

	fixedToWorld = false;

	if ( templateObj->touchSensorID > -1 )
		touchSensorID = network->AddSensor();

	if ( templateObj->lightSensorID > -1 )
		lightSensorID = network->AddSensor();

	if ( templateObj->chemoSensorID > -1 )
		chemoSensorID = network->AddSensor();

	if ( templateObj->agentSensorID > -1 )
		agentSensorID = network->AddSensor();

	if ( templateObj->attachmentSensorID > -1 )
		attachmentSensorID = network->AddSensor();

	if ( templateObj->detacherID > -1 )
		detacherID = network->AddMotor();

	weight = templateObj->weight;

	minMass = templateObj->minMass;
	minHeight = templateObj->minHeight;
	maxHeight = templateObj->maxHeight;
	amountOfPerturbation = templateObj->amountOfPerturbation;
	body = 0;

	drawTouchdowns = templateObj->drawTouchdowns;
	touchdowns = NULL;

	numOfMorphParamsINeed = templateObj->numOfMorphParamsINeed;

	sourceObject = NULL;
	targetObject = NULL;

	minXPosition = templateObj->minXPosition;
	minYPosition = templateObj->minYPosition;
	minZPosition = templateObj->minZPosition;

	maxXPosition = templateObj->maxXPosition;
	maxYPosition = templateObj->maxYPosition;
	maxZPosition = templateObj->maxZPosition;

	isAttacher = templateObj->isAttacher;
	isAttached = templateObj->isAttached;

	minDistForLocking = templateObj->minDistForLocking;
	minForceForUnlocking = templateObj->minForceForUnlocking;

	massLockedWith = NULL;
	heightLockedWith = NULL;
	objICanPassThrough = NULL;
	strcpy(massLockedWithName,templateObj->massLockedWithName);
	strcpy(heightLockedWithName,templateObj->heightLockedWithName);
	strcpy(objICanPassThroughName,templateObj->objICanPassThroughName);

	position[0] = templateObj->position[0] + disp[0];
	position[1] = templateObj->position[1] + disp[1];
	position[2] = templateObj->position[2] + 0.1;

	rotation[0] = templateObj->rotation[0];
	rotation[1] = templateObj->rotation[1];
	rotation[2] = templateObj->rotation[2];

	objectType = templateObj->objectType;
	
	size[0] = templateObj->size[0];
	size[1] = templateObj->size[1];
	size[2] = templateObj->size[2];

	mass = templateObj->mass;

	colour[0] = templateObj->colour[0];
	colour[1] = templateObj->colour[1];
	colour[2] = templateObj->colour[2];

	contactWithFloor = templateObj->contactWithFloor;

	coverWithTexture = templateObj->coverWithTexture;
	strcpy(textureName,templateObj->textureName);
}

ME_OBJECT::ME_OBJECT(AGENT *a, int numOfObjects, ifstream *bodyFile, int inputID,
							NEURAL_NETWORK *network, ME_OBJECT **objs) {

	ID = inputID;
	containingAgent = a;

	touchSensorID = -1;
	lightSensorID = -1;
	chemoSensorID = -1;
	agentSensorID = -1;
	attachmentSensorID = -1;
	detacherID = -1;
	nn = network;
	startOfAggregate = this;
	numOfMorphParamsINeed = 0;
	sourceObject = NULL;
	targetObject = NULL;
	amountOfPerturbation = 0.0;
	body = 0;

	fixedToWorld = false;

	minMass = -100.0;
	minHeight = -100.0;
	maxHeight = -100.0;

	isAttacher = false;
	isAttached = false;

	drawTouchdowns = false;
	touchdowns = NULL;

	minDistForLocking = 100.0;
	minForceForUnlocking = 1000.0;

	objICanPassThrough = NULL;
	massLockedWith = NULL;
	heightLockedWith = NULL;
	strcpy(objICanPassThroughName,"");
	strcpy(massLockedWithName,"");
	strcpy(heightLockedWithName,"");

	coverWithTexture = false;

	SetDefaultProperties();
	ReadIn(bodyFile,network,objs);
}

ME_OBJECT::~ME_OBJECT(void) {

	containingAgent = NULL;
	
	if ( sourceObject != NULL )
		sourceObject = NULL;

	if ( targetObject != NULL )
		targetObject = NULL;

	if ( objICanPassThrough != NULL )
		objICanPassThrough = NULL;

	if ( massLockedWith != NULL )
		massLockedWith = NULL;

	if ( heightLockedWith != NULL )
		heightLockedWith = NULL;

	if ( touchdowns ) {
		delete touchdowns;
		touchdowns = NULL;
	}
}

int ME_OBJECT::BeyondRange(void) {

	dReal pos[3], orientation[12];

	GetPosAndRot(pos,orientation);

	double dist = sqrt( pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2] );

	return( dist < 0 );
}

int ME_OBJECT::BreakageDesired(NEURAL_NETWORK *network) {

	/*
	if ( detacherID < 0 )
		return( false );
	else {
		
		if ( network->GetMotorCommand( detacherID ) > 0.0 )
			return( true );
		else
			return( false );
	}
	*/
	return( false );
}

int  ME_OBJECT::CloseEnough(ME_OBJECT *otherObj) {

	/*
	double currDist;
	MeVector3 myPos, hisPos;

	MdtBodyGetPosition(body,myPos);
	MdtBodyGetPosition(otherObj->body,hisPos);

	currDist = sqrt(	(myPos[0]-hisPos[0])*(myPos[0]-hisPos[0]) +
							(myPos[1]-hisPos[1])*(myPos[1]-hisPos[1]) +
							(myPos[2]-hisPos[2])*(myPos[2]-hisPos[2]) );

	return( currDist <= minDistForLocking );
	*/
	return( false );
}

int  ME_OBJECT::CloseEnoughToPlume(double currHeight, double plumeHeight) {

	return( (currHeight>0.0) &&
		    (currHeight<2.0*plumeHeight) );
}

int ME_OBJECT::CountMorphParams(int morphParamsSoFar) {

	int numMorphParams = 0;

	for (int m=0;m<5;m++)
		if ( morphParamIndices[m] == -1 ) {
			morphParamIndices[m] = morphParamsSoFar + numMorphParams + 1;
			numMorphParams++;
		}
		else
			morphParamIndices[m] = 0;

	return( numMorphParams );
}

void ME_OBJECT::CreateODEStuff(dWorldID world, dSpaceID space) {

	encapsulatedGeom = 0;

	body = dBodyCreate(world);

	dBodySetPosition(body,position[0],position[1],position[2]);

	dBodySetData(body,(void*)this);
	
	if ( minMass == -100.0 )

		if ( massLockedWith != NULL )
			LockMassTo();
		else {

			tempMass = weight;

			// Add noise to mass distribution
			if ( (simParams->noiseIteration >  (NOISE_TRIALS*3)) && 
				 (simParams->noiseIteration <= (NOISE_TRIALS*4)) )
				weight = weight*simParams->Rand(0.5,2.0);

			tempSize[0] = size[0];
			tempSize[1] = size[1];
			tempSize[2] = size[2];
		}

	/*
	if ( (rotation[0]!=0.0) || (rotation[1]!=0.0) || (rotation[2]!=0.0) ) {

		dMatrix3 R;
		dRFromZAxis(R,rotation[0],rotation[1],rotation[2]);
		dBodySetRotation(body,R);
	}
	*/

	if ( ID >= 0 ) {
		dMatrix3 R;
		dRFromZAxis(R,-rotation[2],rotation[0],rotation[1]);
		dBodySetRotation(body,R);
	}

	if ( objectType == SPHERE ) {
		dMassSetSphere(&mass,DENSITY*weight,tempSize[0]);
		geom = dCreateSphere(space,tempSize[0]);
	}

	if ( objectType == CYLINDER ) {
		dMassSetCappedCylinder(&mass,DENSITY*weight,3,tempSize[0],tempSize[1]);
		geom = dCreateCCylinder(space,tempSize[0],tempSize[1]);
	}

	if ( objectType == RECTANGLE ) {
		dMassSetBox(&mass,DENSITY*weight,tempSize[0],tempSize[1],tempSize[2]);
		geom = dCreateBox(space,tempSize[0],tempSize[1],tempSize[2]);
	}

	weight = tempMass;
	
	dGeomSetData(geom,(void*)this);
	dGeomSetBody(geom,body);
    dBodySetMass(body,&mass);

	if ( ID == 0 ) {

		distanceSensor = dCreateRay(space,BELLY_SENSOR_DISTANCE);
		dGeomSetBody(distanceSensor,body);
		dGeomSetData(distanceSensor,(void*)this);
	}
	else
		distanceSensor = NULL;
}

void ME_OBJECT::Darken(double amt) {

	if ( colour[0] - amt >= 0.0 )
		colour[0] = colour[0] - amt;

	if ( colour[1] - amt >= 0.0 )
		colour[1] = colour[1] - amt;

	if ( colour[2] - amt >= 0.0 )
		colour[2] = colour[2] - amt;
}

void ME_OBJECT::DestroyODEStuff(dSpaceID space) {

	if ( body != 0 )
		dBodyDestroy(body);
	
	if ( geom != 0 ) {
		dSpaceRemove(space,geom);
		dGeomDestroy(geom);
	}

	if ( encapsulatedGeom != 0 ) {
		dGeomDestroy(encapsulatedGeom);
		startOfAggregate = this;
	}

	if ( distanceSensor != 0 ) {
		dSpaceRemove(space,distanceSensor);
		dGeomDestroy(distanceSensor);
	}
}

void ME_OBJECT::DisableContacts(void) {

	/*
	if ( objICanPassThrough != NULL )
		McdSpaceDisablePair(bodyCM,objICanPassThrough->bodyCM);
	*/
}

void ME_OBJECT::Display(void) {

	printf("Object %d: %s\n",ID,name);
	printf("\t Position [%2.2f,%2.2f,%2.2f]\n",
		position[0],position[1],position[2]);
	printf("\t Rotation [%2.2f,%2.2f,%2.2f]\n",
		rotation[0],rotation[1],rotation[2]);
	switch ( objectType ) {
	case 0:
		printf("\t Rectangular solid [w=%2.2f m, h=%2.2f m, l=%2.2f m]\n",
			size[0],size[1],size[2]);
		break;
	case 1:
		printf("\t Cylinder [h=%2.2f m, r=%2.2f m]\n",
			size[0],size[1]);
		break;
	case 2:
		printf("\t Sphere [r=%2.2f m]\n",size[0]);
		break;
	}
	printf("\t Weight: %2.2f\n",weight);
	printf("\t Colour [%2.2f,%2.2f,%2.2f]\n",
		colour[0],colour[1],colour[2]);

	printf("\t Floor contact: [");
	if ( contactWithFloor )
		printf("y");
	else
		printf("n");
	printf("]\n");
}

void  ME_OBJECT::Draw(void) {

	Draw(geom,0,0);
}

void  ME_OBJECT::DrawSensorNeurons(double maxHeight, double heightOffset) {

	/*
	if ( (touchSensorID > -1) || (chemoSensorID > -1) ) {

		dReal pos[3], neuronPos[3];
		dReal orientation[12];
		double sensorValue;
		
		if ( touchSensorID > -1 )
			sensorValue = nn->GetSensorValue(touchSensorID);
		else
			sensorValue = nn->GetSensorValue(chemoSensorID);

		GetPosAndRot(pos,orientation);

		neuronPos[0] = pos[0];
		neuronPos[1] = pos[1];
		neuronPos[2] = pos[2];

		dsSetColor(sensorValue,sensorValue,sensorValue);
		dsDrawSphere(neuronPos,orientation,SENSOR_NEURON_RADIUS);
	}
	*/
}

int   ME_OBJECT::EvolvableHeight(void) {

	return( minHeight != -100.0 ); 
}

int   ME_OBJECT::EvolvableMass(void) {

	return( minMass != -100.0 ); 
}

ME_OBJECT* ME_OBJECT::FindObject(ME_OBJECT **objs, int numObjs, char *name) {

	int objFound = false;
	ME_OBJECT *foundObject;
	int currObj = 0;

	while ( (!objFound) && (currObj<numObjs) ) {

		if ( strcmp(name,objs[currObj]->name) == 0 ) {
			objFound = true;
			foundObject = objs[currObj];
		}

		currObj++;
	}

	if ( objFound )
		return foundObject;
	else
		return NULL;
}

void ME_OBJECT::FindOtherObjects(int numOfObjs, ME_OBJECT **objs) {

	if ( strcmp(massLockedWithName,"") != 0 ) {

		massLockedWith = FindObject(objs,numOfObjs,massLockedWithName);

		if ( massLockedWith == NULL ) {
			printf("Object %s not found in object %s.\n",
			massLockedWithName,name);
			char ch = getchar();
			exit(0);
		}
	}

	if ( strcmp(heightLockedWithName,"") != 0 ) {

		heightLockedWith = FindObject(objs,numOfObjs,heightLockedWithName);

		if ( heightLockedWith == NULL ) {
			printf("Object %s not found in object %s.\n",
			heightLockedWithName,name);
			char ch = getchar();
			exit(0);
		}
	}

	if ( strcmp(objICanPassThroughName,"") != 0 ) {

		objICanPassThrough = FindObject(objs,numOfObjs,objICanPassThroughName);

		if ( objICanPassThrough == NULL ) {
			printf("Object %s not found in object %s.\n",
			objICanPassThroughName,name);
			char ch = getchar();
			exit(0);
		}
	}
}

double ME_OBJECT::GetDistance(ME_OBJECT *otherObject) {

	const dReal *pos, *pos2;
	double dist;

	pos = dBodyGetPosition(body);
	pos2 = dBodyGetPosition(otherObject->body);

	dist = sqrt(	pow(pos[0]-pos2[0],2.0) + 
						pow(pos[1]-pos2[1],2.0) + 
						pow(pos[2]-pos2[2],2.0) );
	return( dist );
}

double ME_OBJECT::GetForward(void) {

	dReal pos[3];
	dReal orientation[12];

	GetPosAndRot(pos,orientation);

	return( pos[0] );
}

double ME_OBJECT::GetHeight(void) {

	dReal pos[3];
	dReal orientation[12];

	if ( body ) 
		GetPosAndRot(pos,orientation);
	else
		pos[2] = position[2];

	return( pos[2] );
}

double ME_OBJECT::GetLeft(void) {

	dReal pos[3];
	dReal orientation[12];

	GetPosAndRot(pos,orientation);

	return( pos[1] );
}

void ME_OBJECT::GetPosAndRot(dReal *act_pos, dReal *actual_R) {

	if ( encapsulatedGeom != 0 ) {

		const dReal *R = dGeomGetRotation(geom);

		const dReal *pos = dGeomGetPosition(geom);

		dGeomID g2 = dGeomTransformGetGeom(geom);

		const dReal *pos2 = dGeomGetPosition(g2);
		const dReal *R2 = dGeomGetRotation(g2);

		dVector3 actual_pos;
		dMatrix3 newR;

		dMULTIPLY0_331(actual_pos,R,pos2);
		act_pos[0] = actual_pos[0] + pos[0];
		act_pos[1] = actual_pos[1] + pos[1];
		act_pos[2] = actual_pos[2] + pos[2];

		dMULTIPLY0_333(newR,R,R2);

		for (int i=0;i<12;i++)
			actual_R[i] = newR[i];
	}

	else {

		const dReal *pos = dBodyGetPosition(body);
		
		act_pos[0] = pos[0];
		act_pos[1] = pos[1];
		act_pos[2] = pos[2];

		const dReal *rot = dBodyGetRotation(body);

		for (int i=0;i<12;i++)
			actual_R[i] = rot[i];
	}
}

void ME_OBJECT::IncreasePerturbation(double amt) {

	if ( amountOfPerturbation + amt <= 1.0 ) {
		Darken(amt);
		amountOfPerturbation = amountOfPerturbation + amt;
	}
}

void ME_OBJECT::IntroduceNoise(void) {

	if ( amountOfPerturbation > 0.0 )
		Push(	simParams->Rand(-1.0,1.0)*amountOfPerturbation,
	 			simParams->Rand(-1.0,1.0)*amountOfPerturbation,
				simParams->Rand(-1.0,1.0)*amountOfPerturbation);
}

void ME_OBJECT::InvertColour(void) {

	/*
	if ( rc->m_options.m_renderType != kRNull ) {

		double newColour[4];
		double tempColour[4];

		RGraphicGetColor(bodyG,tempColour);
		newColour[0] = 1.0 - tempColour[0];
		newColour[1] = 1.0 - tempColour[1];
		newColour[2] = 1.0 - tempColour[2];

		RGraphicSetColor(bodyG,newColour);
	}
	*/
}

int ME_OBJECT::IsBelow(double verticalPos) {

	const dReal  *pos = dBodyGetPosition(startOfAggregate->body);

	return( pos[2] < verticalPos );
}

void ME_OBJECT::IsInside(ME_OBJECT *otherObj, double *dist) {

	int isInside;

	isInside =             (fabs(originalPosition[0]-otherObj->originalPosition[0]) < (otherObj->originalSize[0]/2.0));
	isInside = isInside && (fabs(originalPosition[1]-otherObj->originalPosition[1]) < (otherObj->originalSize[1]/2.0));
	isInside = isInside && (fabs(originalPosition[2]-otherObj->originalPosition[2]) < (otherObj->originalSize[2]/2.0));

	if ( isInside )
		(*dist) = (*dist) + 1;
}

int	 ME_OBJECT::IsPartOfEnvironment(void) {

	return( ID == -1 );
}

void ME_OBJECT::Lighten(void) {

	if ( colour[0] < 1.0 )
		colour[0] = colour[0] + 0.1;

	if ( colour[1] < 1.0 )
		colour[1] = colour[1] + 0.1;

	if ( colour[2] < 1.0 )
		colour[2] = colour[2] + 0.1;
}

int  ME_OBJECT::PartOfDebris(void) {

	return( ID < 0 );
}

void ME_OBJECT::Push(double x, double y, double z) {

	dReal pos[3];
	dReal offset[3];

	dReal orientation[12];

	GetPosAndRot(pos,orientation);

	offset[0] = pos[0] - x;
	offset[1] = pos[1] - y;
	offset[2] = pos[2] - z;

	dsSetColor(1.0,0.0,0.0);
	dsDrawLineD(pos,offset);

	dBodyAddForce(startOfAggregate->body, x*weight*5.0, y*weight*5.0, z*weight*5.0);
}

void ME_OBJECT::RecordTouchdown(double *pos) {

	if ( touchdowns )
		touchdowns->Store(simParams->internalTimer,pos[0],pos[1],0.0);
}

void ME_OBJECT::Reposition(double x, double y, double z) {

	dBodySetPosition(body,x,y,z);
}

int ME_OBJECT::RequiresGeneticSpecification(void) {

	int requires = false;
	int currentMorphIndex = 0;

	while ( (currentMorphIndex<5) && (!requires) ) {

		if ( morphParamIndices[currentMorphIndex] > 0 )
			requires = true;

		currentMorphIndex++;
	}

	return( requires );
}

void ME_OBJECT::Reset(void) {

	if ( touchdowns )
		touchdowns->Reset();
}

void ME_OBJECT::Reset(dWorldID world, dSpaceID space) {

	DestroyODEStuff(space);
	startOfAggregate = this;

	CreateODEStuff(world,space);
}

void ME_OBJECT::SaveToRaytracerFile(ofstream *outFile) {

	dReal pos[3];
	dReal orientation[12];
	dReal axis[3];
	dReal theta;

	GetPosAndRot(pos,orientation);
	RotToAxisAndAngle(orientation,axis,&theta);

	for (int i=0;i<12;i++)
		if ( (orientation[i] < 0.001) &&
			 (orientation[i] > -0.001) )
			orientation[i] = 0.0;

	char tempStr[50];

	switch ( objectType ) {
		case 0: // Rectangle
			(*outFile) << "RECTANGLE { ";

			(*outFile) << (size[0]*100.0) << " " << (size[2]*100.0) << " ";

			sprintf(tempStr,"%3.3f %3.3f %3.3f ",
				(double)pos[0]*100.0,(double)pos[1]*100.0,(double)pos[2]*100.0);
			(*outFile) << tempStr;

			(*outFile) << colour[0] << " " << colour[1] << " " << colour[2] << " ";

			sprintf(tempStr,"%3.3f %3.3f %3.3f %3.3f }\n",
				(double)axis[0],(double)axis[1],(double)axis[2],theta);

			(*outFile) << tempStr;

			break;
		case 1: // Cylinder
			(*outFile) << "CYLINDER { ";
			(*outFile) << (size[0]*100.0) << " " << (size[1]*100.0) << " ";

			sprintf(tempStr,"%3.3f %3.3f %3.3f ",
				(double)pos[0]*100.0,(double)pos[1]*100.0,(double)pos[2]*100.0);
			(*outFile) << tempStr;
			(*outFile) << colour[0] << " " << colour[1] << " " << colour[2] << " ";

			sprintf(tempStr,"%3.3f %3.3f %3.3f %3.3f }\n",
				(double)axis[0],(double)axis[1],(double)axis[2],theta);

			(*outFile) << tempStr;
			break;
		case 2: // Sphere
			(*outFile) << "SPHERE { ";
			(*outFile) << (size[0]*100.0) << " ";

			sprintf(tempStr,"%3.3f %3.3f %3.3f ",
				(double)pos[0]*100.0,(double)pos[1]*100.0,(double)pos[2]*100.0);
			(*outFile) << tempStr;
			(*outFile) << colour[0] << " " << colour[1] << " " << colour[2] << " }\n";
			break;
	}
}

void ME_OBJECT::SetColour(float r, float g, float b) {

	colour[0] = r;
	colour[1] = g;
	colour[2] = b;
}

void ME_OBJECT::SetEvolvableParams(int numMorphParams, int genomeLength, double *genes) {

	if ( RequiresGeneticSpecification() ) {

		int geneIndex;
		double geneValue;
		int currentMorphParam = 0;

		for (int m=0;m<5;m++) {

			if ( morphParamIndices[m] > 0 ) {

				geneIndex =
				(morphParamIndices[m]-1) * 
					( (genomeLength-numMorphParams) / numMorphParams );

				geneValue = genes[geneIndex];

				switch ( m ) {
				case 0:
					AdjustMass(geneValue);
					break;
				case 1:
					AdjustHeight(geneValue);
					break;
				case 2:
					AdjustX(geneValue);
					break;
				case 3:
					AdjustY(geneValue);
					break;
				case 4:
					AdjustZ(geneValue);
					break;
				}

				currentMorphParam++;
			}
		}
	}
}

void ME_OBJECT::SetParamsBasedOnOtherObjects(void) {

	if ( sourceObject ) {

		double x0, y0, z0, x1, y1, z1;

		x0 = sourceObject->position[1];
		y0 = sourceObject->position[2];
		z0 = sourceObject->position[0];

		x1 = targetObject->position[1];
		y1 = targetObject->position[2];
		z1 = targetObject->position[0];

		double length = sqrt( pow(( x0 - x1 ),2.0) + 
			                  pow(( y0 - y1 ),2.0) +
							  pow(( z0 - z1 ),2.0) );

		size[1] = length*originalSize[1];

		position[1] = ( (0.5+originalSize[1]/2.0)*x0 + (0.5-originalSize[1]/2.0)*x1 );
		position[2] = ( (0.5+originalSize[1]/2.0)*y0 + (0.5-originalSize[1]/2.0)*y1 );
		position[0] = ( (0.5+originalSize[1]/2.0)*z0 + (0.5-originalSize[1]/2.0)*z1 );

		if ( x1-x0==0.0 )
			if ( x0 > minXPosition )
				x0 = x0 - 0.0001;
			else
				x0 = x0 + 0.0001;

		if ( y1-y0==0.0 )
			if ( y0 > minYPosition )
				y0 = y0 - 0.0001;
			else
				y0 = y0 + 0.0001;

		if ( z1-z0==0.0 )
			if ( z0 > minZPosition )
				z0 = z0 - 0.0001;
			else
				z0 = z0 + 0.0001;

		rotation[0] = x1-x0;
		rotation[1] = y1-y0;
		rotation[2] = z0-z1;
	}
}

void ME_OBJECT::SetOrientationSensors(void) {

	const dReal *rot = dBodyGetRotation(body);

	double restoringRot[12];

	restoringRot[0] = 0.0;  restoringRot[1] = 1.0; restoringRot[2] = 0.0;
	restoringRot[4] = -1.0; restoringRot[5] = 0.0; restoringRot[6] = 0.0;
	restoringRot[8] = 0.0;  restoringRot[9] = 0.0; restoringRot[10] = 1.0;

	double normalizedRot[12];

	normalizedRot[0] = (rot[0]*restoringRot[0])+(rot[1]*restoringRot[4])+(rot[2]*restoringRot[8]);
	normalizedRot[1] = (rot[0]*restoringRot[1])+(rot[1]*restoringRot[5])+(rot[2]*restoringRot[9]);
	normalizedRot[2] = (rot[0]*restoringRot[2])+(rot[1]*restoringRot[6])+(rot[2]*restoringRot[10]);

	normalizedRot[4] = (rot[4]*restoringRot[0])+(rot[5]*restoringRot[4])+(rot[6]*restoringRot[8]);
	normalizedRot[5] = (rot[4]*restoringRot[1])+(rot[5]*restoringRot[5])+(rot[6]*restoringRot[9]);
	normalizedRot[6] = (rot[4]*restoringRot[2])+(rot[5]*restoringRot[6])+(rot[6]*restoringRot[10]);

	normalizedRot[8] = (rot[8]*restoringRot[0])+(rot[9]*restoringRot[4])+(rot[10]*restoringRot[8]);
	normalizedRot[9] = (rot[8]*restoringRot[1])+(rot[9]*restoringRot[5])+(rot[10]*restoringRot[9]);
	normalizedRot[10] = (rot[8]*restoringRot[2])+(rot[9]*restoringRot[6])+(rot[10]*restoringRot[10]);

	double thetaLeftRight;
	double thetaForwardBack;

	thetaLeftRight = atan(normalizedRot[6]/normalizedRot[5]);

	thetaLeftRight = thetaLeftRight*180.0/3.14159;

	thetaForwardBack = atan(normalizedRot[2]/normalizedRot[0]);

	thetaForwardBack = thetaForwardBack*180.0/3.14159;

	containingAgent->sensorValues->Set(simParams->internalTimer,12,thetaLeftRight);
	containingAgent->sensorValues->Set(simParams->internalTimer,13,thetaForwardBack);
}

void ME_OBJECT::SetPressureSensor(void) {

	//containingAgent->sensorValues->Set(simParams->internalTimer,containingAgent->numJoints+touchSensorID,penetration);
	//"Pressure" sensor

	//containingAgent->sensorValues->Set(simParams->internalTimer,containingAgent->numJoints+touchSensorID,1.0);
	//Touch sensor

	containingAgent->sensorValues->Set(simParams->internalTimer,8+touchSensorID,1.0);
}

void ME_OBJECT::SwitchToGeomTransform(ME_OBJECT *otherObj, dSpaceID space) {

	startOfAggregate = otherObj;
	dReal myPos[3], otherPos[3], myRot[12], otherRot[12];
	dReal offset[3];
	dMass myMass, otherMass;

	GetPosAndRot(myPos,myRot);
	otherObj->GetPosAndRot(otherPos,otherRot);

	offset[0] = myPos[0] - otherPos[0];
	offset[1] = myPos[1] - otherPos[1];
	offset[2] = myPos[2] - otherPos[2];

	dGeomDestroy(geom);
	dBodyDestroy(body);
	body = 0;

	geom = dCreateGeomTransform(space);
	dGeomSetData(geom,(void*)this);

	dGeomTransformSetCleanup(geom,0);

	if ( objectType == SPHERE ) {
		encapsulatedGeom = dCreateSphere(0,tempSize[0]);
		dMassSetSphere(&myMass,DENSITY,tempSize[0]);
	}

	if ( objectType == CYLINDER ) {
		encapsulatedGeom = dCreateCCylinder(0,tempSize[0],tempSize[1]);
		dMassSetCappedCylinder(&myMass,DENSITY,3,tempSize[0],tempSize[1]);
	}

	if ( objectType == RECTANGLE ) {
		encapsulatedGeom = dCreateBox(0,tempSize[0],tempSize[1],tempSize[2]);
		dMassSetBox(&myMass,DENSITY,tempSize[0],tempSize[1],tempSize[2]);
	}
	
	dMassTranslate(&myMass,offset[0],offset[1],offset[2]);

	dBodyGetMass(otherObj->body,&otherMass);
	dMassAdd(&otherMass,&myMass);
	dMassTranslate(&otherMass,-otherMass.c[0],-otherMass.c[1],-otherMass.c[2]);
	
	dBodySetMass(otherObj->body,&otherMass);

	dGeomTransformSetGeom(geom,encapsulatedGeom);

	dGeomSetPosition(encapsulatedGeom,offset[0],offset[1],offset[2]);

	dGeomSetRotation(encapsulatedGeom,myRot);
	dGeomSetBody(geom,startOfAggregate->body);
}

void ME_OBJECT::ToggleFootprintDrawing(void) {

	if ( contactWithFloor ) {

		if ( touchdowns ) {
			delete touchdowns;
			touchdowns = NULL;
		}
		else
			touchdowns = new TRAJECTORY(simParams->evalPeriod);
	}
}

void ME_OBJECT::TouchingGround(dVector3 contactPosition) {

	simParams->touchPoints->Set(ID,0,contactPosition[0]);
	simParams->touchPoints->Set(ID,1,contactPosition[1]);
	simParams->touchPoints->Set(ID,2,contactPosition[2]);
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void ME_OBJECT::AdjustHeight(double morphParam) {

	position[1] = simParams->Scale(morphParam,-1.0,1.0,minHeight,maxHeight);
}

void ME_OBJECT::AdjustMass(double morphParam) {

	double massChange = simParams->Scale(morphParam,-1.0,1.0,minMass,1.0);

	tempMass = weight*massChange;

	if ( objectType == 0 ) {
		tempSize[0] = size[0]*massChange;
		tempSize[1] = size[1]*massChange;
		tempSize[2] = size[2]*massChange;
	}

	if ( objectType == 1 ) {
		tempSize[0] = size[0]*sqrt(massChange);
		tempSize[1] = size[1];
	}

	if ( objectType == 2 ) {
		tempSize[0] = size[0]*pow(massChange,1.0/3.0);
	}
}

void ME_OBJECT::AdjustRotation(void) {

	rotation[0] = rotation[0]*(PI/180.0);
	rotation[1] = rotation[1]*(PI/180.0);
	rotation[2] = rotation[2]*(PI/180.0);
}

void ME_OBJECT::AdjustX(double morphParam) {

	position[1] = originalPosition[1] + simParams->Scale(morphParam,-1.0,1.0,minXPosition,maxXPosition);
}

void ME_OBJECT::AdjustY(double morphParam) {

	position[2] = originalPosition[2] + simParams->Scale(morphParam,-1.0,1.0,minYPosition,maxYPosition);
}

void ME_OBJECT::AdjustZ(double morphParam) {

	position[0] = originalPosition[0] + simParams->Scale(morphParam,-1.0,1.0,minZPosition,maxZPosition);
}

void ME_OBJECT::Draw(dGeomID g, const dReal *pos, const dReal *R) {

	if ( touchdowns )
		touchdowns->Draw(colour[0],colour[1],colour[2]);

	SetColour();

	if (!g)
		return;

	if (!pos)
		pos = dGeomGetPosition(g);

	if (!R)
		R = dGeomGetRotation(g);

	int type = dGeomGetClass(g);

	if (type == dBoxClass) {
		dVector3 sides;
		dGeomBoxGetLengths(g,sides);
		
		dsDrawBox(pos,R,sides);
	}
	else if (type == dSphereClass) {
		float radius = (float)dGeomSphereGetRadius(g);
		dsDrawSphere(pos,R,radius);
	}
	else if (type == dCCylinderClass) {

		double radius,length;
		dGeomCCylinderGetParams(g,&radius,&length);

		dsDrawCappedCylinder(pos,R,(float)length,(const float)radius);
		
	}
	else if (type == dGeomTransformClass) {
	
		dGeomID g2 = dGeomTransformGetGeom(g);

		dReal actual_pos[3];
		dReal actual_R[12];

		GetPosAndRot(actual_pos,actual_R);

		Draw(g2,actual_pos,actual_R);
	}
}

void ME_OBJECT::InvertRotation(dReal *rot, double *theta) {

	dQuaternion q;
	dRtoQ(rot,q);

	(*theta) = 2.0*acos(q[0]);
	double x = q[1] / sin((*theta)/2.0);
	double y = q[2] / sin((*theta)/2.0);
	double z = q[3] / sin((*theta)/2.0);

	(*theta) = -(*theta);

	dQuaternion q2;
	q2[0] = cos((*theta)/2.0);
	q2[1] = x*sin((*theta)/2.0);
	q2[2] = y*sin((*theta)/2.0);
	q2[3] = z*sin((*theta)/2.0);

	dQtoR(q2,rot);
}

void ME_OBJECT::LockHeightTo(void) {

	position[1] = heightLockedWith->position[1];
}

void ME_OBJECT::LockMassTo(void) {

	tempMass = massLockedWith->tempMass;
	tempSize[0] = massLockedWith->tempSize[0];
	tempSize[1] = massLockedWith->tempSize[1];
	tempSize[2] = massLockedWith->tempSize[2];
}

int ME_OBJECT::NumberEntered(char *name) {

	return( ((name[0]>=48)&&(name[0]<=57)) || (name[0]==45) );
}

void ME_OBJECT::ReadIn(ifstream *envFile, NEURAL_NETWORK *network,
							  ME_OBJECT **objs) {

	contactWithFloor = false;

	char readIn[50];

	(*envFile) >> readIn;
	
	if ( strcmp(readIn,"(") != 0 ) {
		printf("Read in error: expecting opening bracket.\n");
		char ch = getchar();
		exit(0);
	}

	(*envFile) >> name;

	(*envFile) >> readIn;

	while ( strcmp(readIn,")") != 0 ) {

		if ( strcmp(readIn,"-position") == 0 ) {

			(*envFile) >> readIn;

			if ( NumberEntered(readIn) ) {

				originalPosition[1] = atof(readIn);
				position[1] = originalPosition[1]; // Left / right
				(*envFile) >> readIn;
				originalPosition[2] = atof(readIn);
				position[2] = originalPosition[2]; // Up / down
				(*envFile) >> readIn;
				originalPosition[0] = -atof(readIn);
				position[0] = originalPosition[0]; // In / out
			}
			else {

				sourceObject = FindObject(objs,ID,readIn);
				(*envFile) >> readIn;
				targetObject = FindObject(objs,ID,readIn);
			}
		}
		else
		if ( strcmp(readIn,"-rotation") == 0 ) {

			(*envFile) >> readIn;
			rotation[0] = atof(readIn);
			(*envFile) >> readIn;
			rotation[1] = atof(readIn);
			(*envFile) >> readIn;
			rotation[2] = atof(readIn);
			AdjustRotation();
		}
		else
		if ( strcmp(readIn,"-shape") == 0 ) {
			(*envFile) >> readIn;
			if ( strcmp(readIn,"-rectangle") == 0 ) {
				objectType = RECTANGLE;
				(*envFile) >> readIn;
				size[0] = atof(readIn);
				originalSize[0] = size[0];
				(*envFile) >> readIn;
				size[1] = atof(readIn);
				originalSize[1] = size[1];
				(*envFile) >> readIn;
				size[2] = atof(readIn);
				originalSize[2] = size[2];

				volume = size[0]*size[1]*size[2];
			}
			if ( strcmp(readIn,"-cylinder") == 0 ) {
				objectType = CYLINDER;
				(*envFile) >> readIn;
				size[0] = atof(readIn);
				originalSize[0] = size[0];
				(*envFile) >> readIn;
				size[1] = atof(readIn);
				originalSize[1] = size[1];

				volume = (4/3*3.14159*size[0]*size[0]*size[0]) + (3.14159*size[0]*size[0]*size[1]);

			}
			if ( strcmp(readIn,"-sphere") == 0 ) {
				objectType = SPHERE;
				(*envFile) >> readIn;
				size[0] = atof(readIn);
				originalSize[0] = size[0];
				volume = 4.0/3.0 * 3.14159 * size[0]*size[0]*size[0];
			}
			containingAgent->totalVolume = containingAgent->totalVolume + volume;
		}
		else
		if ( strcmp(readIn,"-mass") == 0 ) {
			(*envFile) >> readIn;
			weight = atof(readIn);

			containingAgent->totalMass = containingAgent->totalMass + weight;

		}
		else
		if ( strcmp(readIn,"-lockMassWith") == 0 )
			(*envFile) >> massLockedWithName;
		else
		if ( strcmp(readIn,"-lockHeightWith") == 0 )
			(*envFile) >> heightLockedWithName;
		else
		if ( strcmp(readIn,"-disableContactWith") == 0 )
			(*envFile) >> objICanPassThroughName;
		else
		if ( strcmp(readIn,"-evolve") == 0 ) {

			numOfMorphParamsINeed++;

			(*envFile) >> readIn;
			if ( strcmp(readIn,"-mass") == 0 ) {
				(*envFile) >> readIn;
				minMass = atof(readIn);
				morphParamIndices[0] = -1;
			}
			if ( strcmp(readIn,"-height") == 0 ) {
				(*envFile) >> readIn;
				minHeight = atof(readIn);
				(*envFile) >> readIn;
				maxHeight = atof(readIn);
				morphParamIndices[1] = -1;
			}
			if ( strcmp(readIn,"-xPosition") == 0 ) {
				(*envFile) >> readIn;
				minXPosition = atof(readIn);
				(*envFile) >> readIn;
				maxXPosition = atof(readIn);
				morphParamIndices[2] = -1;

			}
			if ( strcmp(readIn,"-yPosition") == 0 ) {
				(*envFile) >> readIn;
				minYPosition = atof(readIn);
				(*envFile) >> readIn;
				maxYPosition = atof(readIn);
				morphParamIndices[3] = -1;
			}
			if ( strcmp(readIn,"-zPosition") == 0 ) {
				(*envFile) >> readIn;
				minZPosition = atof(readIn);
				(*envFile) >> readIn;
				maxZPosition = atof(readIn);
				morphParamIndices[4] = -1;
			}	

		}
		else
		if ( strcmp(readIn,"-colour") == 0 ) {
			(*envFile) >> readIn;
			colour[0] = (float)atof(readIn);
			(*envFile) >> readIn;
			colour[1] = (float)atof(readIn);
			(*envFile) >> readIn;
			colour[2] = (float)atof(readIn);
			colour[3] = 0.0;
		}
		else
		if ( strcmp(readIn,"-texture") == 0 ) {
			coverWithTexture = true;
			(*envFile) >> textureName;
		}
		else
		if ( strcmp(readIn,"-floorContact") == 0 )
			contactWithFloor = true;
		else
		if ( strcmp(readIn,"-attacher") == 0 ) {
			isAttacher = true;
			(*envFile) >> readIn;
			minDistForLocking = atof(readIn);
			(*envFile) >> readIn;
			minForceForUnlocking = atof(readIn);
		}
		else
		if ( strcmp(readIn,"-addTouchSensor") == 0 ) {

		}
		else {
			printf("ERROR: command %s not recognized in object %s.\n",
				readIn,name);
			printf("Valid commands:\n");
			printf("\t -position [x y z] \n");
			printf("\t -rotation [x y z] \n");
			printf("\t -shape [-sphere r] [-cylinder r h] [-rectangle w h d] \n");
			printf("\t -mass [kilograms] \n");
			printf("\t -evolve [-mass [min max]] [-height [min max]] \n");
			printf("\t -lockHeightWith [name] \n");
			printf("\t -lockMassWith [name] \n");
			printf("\t -colour [r g b] \n");
			printf("\t -texture [fileName] \n");
			printf("\t -floorContact \n");
			printf("\t -attacher \n");
			printf("\t -addTouchSensor \n");
			printf("\t -addChemoSensor \n");
			printf("\t -addLightSensor \n");
			printf("\t -addAgentSensor \n");
			printf("\t -addAttachmentSensor \n");
			printf("\t -addDetacher \n");
			char ch = getchar();
			exit(0);
		}
		(*envFile) >> readIn;
	}
	if ( objectType == SPHERE ) {
		rotation[0] = 0.0;
		rotation[1] = 0.0;
		rotation[2] = 0.0;
	}
}

/*
void ME_OBJECT::ReadIn(ifstream *envFile, NEURAL_NETWORK *network,
							  ME_OBJECT **objs) {

	contactWithFloor = false;

	char readIn[50];

	(*envFile) >> readIn;
	
	if ( strcmp(readIn,"(") != 0 ) {
		printf("Read in error: expecting opening bracket.\n");
		char ch = getchar();
		exit(0);
	}

	(*envFile) >> name;
	
	(*envFile) >> readIn;

	while ( strcmp(readIn,")") != 0 ) {

		if ( strcmp(readIn,"-position") == 0 ) {

			(*envFile) >> readIn;

			if ( NumberEntered(readIn) ) {

				originalPosition[1] = atof(readIn);
				position[1] = originalPosition[1];
				(*envFile) >> readIn;
				originalPosition[2] = atof(readIn);
				position[2] = originalPosition[2];
				(*envFile) >> readIn;
				originalPosition[0] = -atof(readIn);
				position[0] = originalPosition[0];
			}
			else {

				sourceObject = FindObject(objs,ID,readIn);
				(*envFile) >> readIn;
				targetObject = FindObject(objs,ID,readIn);
			}
		}
		else
		if ( strcmp(readIn,"-rotation") == 0 ) {

			(*envFile) >> readIn;
			rotation[1] = atof(readIn);
			(*envFile) >> readIn;
			rotation[2] = atof(readIn);
			(*envFile) >> readIn;
			rotation[0] = atof(readIn);
			AdjustRotation();
		}
		else
		if ( strcmp(readIn,"-shape") == 0 ) {
			(*envFile) >> readIn;
			if ( strcmp(readIn,"-rectangle") == 0 ) {
				objectType = RECTANGLE;
				(*envFile) >> readIn;
				size[0] = atof(readIn);
				originalSize[0] = size[0];
				(*envFile) >> readIn;
				size[1] = atof(readIn);
				originalSize[1] = size[1];
				(*envFile) >> readIn;
				size[2] = atof(readIn);
				originalSize[2] = size[2];
			}
			if ( strcmp(readIn,"-cylinder") == 0 ) {
				objectType = CYLINDER;
				(*envFile) >> readIn;
				size[0] = atof(readIn);
				originalSize[0] = size[0];
				(*envFile) >> readIn;
				size[1] = atof(readIn);
				originalSize[1] = size[1];
			}
			if ( strcmp(readIn,"-sphere") == 0 ) {
				objectType = SPHERE;
				(*envFile) >> readIn;
				size[0] = atof(readIn);
				originalSize[0] = size[0];
			}
		}
		else
		if ( strcmp(readIn,"-mass") == 0 ) {
			(*envFile) >> readIn;
			weight = atof(readIn);
		}
		else
		if ( strcmp(readIn,"-lockMassWith") == 0 )
			(*envFile) >> massLockedWithName;
		else
		if ( strcmp(readIn,"-lockHeightWith") == 0 )
			(*envFile) >> heightLockedWithName;
		else
		if ( strcmp(readIn,"-disableContactWith") == 0 )
			(*envFile) >> objICanPassThroughName;
		else
		if ( strcmp(readIn,"-evolve") == 0 ) {

			numOfMorphParamsINeed++;

			(*envFile) >> readIn;
			if ( strcmp(readIn,"-mass") == 0 ) {
				(*envFile) >> readIn;
				minMass = atof(readIn);
				morphParamIndices[0] = -1;
			}
			if ( strcmp(readIn,"-height") == 0 ) {
				(*envFile) >> readIn;
				minHeight = atof(readIn);
				(*envFile) >> readIn;
				maxHeight = atof(readIn);
				morphParamIndices[1] = -1;
			}
			if ( strcmp(readIn,"-xPosition") == 0 ) {
				(*envFile) >> readIn;
				minXPosition = atof(readIn);
				(*envFile) >> readIn;
				maxXPosition = atof(readIn);
				morphParamIndices[2] = -1;

			}
			if ( strcmp(readIn,"-yPosition") == 0 ) {
				(*envFile) >> readIn;
				minYPosition = atof(readIn);
				(*envFile) >> readIn;
				maxYPosition = atof(readIn);
				morphParamIndices[3] = -1;
			}
			if ( strcmp(readIn,"-zPosition") == 0 ) {
				(*envFile) >> readIn;
				minZPosition = atof(readIn);
				(*envFile) >> readIn;
				maxZPosition = atof(readIn);
				morphParamIndices[4] = -1;
			}	

		}
		else
		if ( strcmp(readIn,"-colour") == 0 ) {
			(*envFile) >> readIn;
			colour[0] = (float)atof(readIn);
			(*envFile) >> readIn;
			colour[1] = (float)atof(readIn);
			(*envFile) >> readIn;
			colour[2] = (float)atof(readIn);
			colour[3] = 0.0;
		}
		else
		if ( strcmp(readIn,"-texture") == 0 ) {
			coverWithTexture = true;
			(*envFile) >> textureName;
		}
		else
		if ( strcmp(readIn,"-texture") == 0 ) {
			coverWithTexture = true;
			(*envFile) >> textureName;
		}
		else
		if ( strcmp(readIn,"-floorContact") == 0 )
			contactWithFloor = true;
		else
		if ( strcmp(readIn,"-attacher") == 0 ) {
			isAttacher = true;
			(*envFile) >> readIn;
			minDistForLocking = atof(readIn);
			(*envFile) >> readIn;
			minForceForUnlocking = atof(readIn);
		}
		else
		if ( strcmp(readIn,"-addTouchSensor") == 0 )
			touchSensorID = simParams->numSensors++;
		else {
			printf("ERROR: command %s not recognized in object %s.\n",
				readIn,name);
			printf("Valid commands:\n");
			printf("\t -position [x y z] \n");
			printf("\t -rotation [x y z] \n");
			printf("\t -shape [-sphere r] [-cylinder r h] [-rectangle w h d] \n");
			printf("\t -mass [kilograms] \n");
			printf("\t -evolve [-mass [min max]] [-height [min max]] \n");
			printf("\t -lockHeightWith [name] \n");
			printf("\t -lockMassWith [name] \n");
			printf("\t -colour [r g b] \n");
			printf("\t -texture [fileName] \n");
			printf("\t -floorContact \n");
			printf("\t -attacher \n");
			printf("\t -addTouchSensor \n");
			printf("\t -addChemoSensor \n");
			printf("\t -addLightSensor \n");
			printf("\t -addAgentSensor \n");
			printf("\t -addAttachmentSensor \n");
			printf("\t -addDetacher \n");
			char ch = getchar();
			exit(0);
		}
		(*envFile) >> readIn;
	}
	if ( objectType == SPHERE ) {
		rotation[0] = 0.0;
		rotation[1] = 0.0;
		rotation[2] = 0.0;
	}
}
*/

void ME_OBJECT::RotToAxisAndAngle(dReal *rot, dReal *axis, dReal *theta) {

	dQuaternion q;
	dRtoQ(rot,q);

	if ( (q[0] == 1.0) && (fabs(q[1])<0.0001) ) {
		(*theta) = 0.0;
		axis[0] = 0.0;
		axis[1] = 0.0;
		axis[2] = 1.0;
	}
	else {
		(*theta) = 2*acos(q[0]);
		axis[0] = q[1] / sin((*theta)/2.0);
		axis[1] = q[2] / sin((*theta)/2.0);
		axis[2] = q[3] / sin((*theta)/2.0);
	}
}

void ME_OBJECT::SetColour(void) {

	if ( IsPartOfEnvironment() )
		dsSetColor(0.5,0.5,0.5);
	else {

		if ( simParams->useTransparency )
			dsSetColorAlpha(colour[0],colour[1],colour[2],0.5);
		else
			dsSetColor(colour[0],colour[1],colour[2]);
	}
}

void ME_OBJECT::SetDefaultProperties(void) {

	position[0] = 0.0;
	position[1] = 0.0;
	position[2] = 0.0;

	rotation[0] = 0.0;
	rotation[1] = 0.0;
	rotation[2] = 0.0;

	colour[0] = 1.0;
	colour[1] = 1.0;
	colour[2] = 1.0;

	objectType = 0;

	size[0] = 1.0;
	size[1] = 1.0;
	size[2] = 1.0;

	weight = 1.0;

	contactWithFloor = true;
}

void ME_OBJECT::SetLightSourceProperties(void) {

	size[0] = LIGHT_SOURCE_SIZE;
	size[1] = LIGHT_SOURCE_SIZE;
	size[2] = LIGHT_SOURCE_SIZE;

	position[1] = LIGHT_SOURCE_SIZE/2.0;
	position[2] = LIGHT_SOURCE_DISTANCE;
}

#endif