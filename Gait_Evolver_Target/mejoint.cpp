/* ---------------------------------------------------
   FILE:     meJoint.cpp
	AUTHOR:   Josh Bongard
	DATE:     March 23, 2001
	FUNCTION: This class contains all information for
				 a generic mathEngine joint.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _ME_JOINT_CPP
#define _ME_JOINT_CPP

#include <drawstuff/drawstuff.h>

#include "mejoint.h"
#include "nnetwork.h"
#include "simParams.h"

extern double		PI;
extern SIM_PARAMS	*simParams;
extern double		DEF_MOTOR_FORCE;
extern double       DEF_MOTOR_SPEED;
extern int		    SPHERE;
extern int			JOINT_HINGE;
extern int			JOINT_BALL;
extern int			JOINT_FIXED;
extern int			JOINT_SLIDER;
extern int			JOINT_WORLD;
extern int			JOINT_SPRING;
extern double		MOTOR_NEURON_RADIUS;
extern double		SENSOR_NEURON_RADIUS;

#ifdef dDOUBLE
#define dsDrawLine dsDrawLineD
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

ME_JOINT::ME_JOINT(void) {

	ID = 0;
	agentID = 0;
	containingAgent = NULL;

	firstBody = NULL;
	secondBody = NULL;
	objectContainingJoint = NULL;

	limitsSpecified = false;
	sensorNeuronID = -1;
	motorNeuronID = -1;
	lowerLimit = -PI/2;
	upperLimit = PI/2;
	coneLimitAngle = PI/2;
	restitution = 0.0;
	stiffness = -1.0;
	motorForce = DEF_MOTOR_FORCE;
	motorSpeed = DEF_MOTOR_SPEED;
	isWheel = false;
	isBinaryMotor = false;
	strcpy(relatedJointName,"");
	relatedJoint = NULL;
	forceControlled = false;

	jointPosition[0] = 0;
	jointPosition[1] = 0;
	jointPosition[2] = 0;

	jointNormal[0] = 1;
	jointNormal[1] = 0;
	jointNormal[2] = 0;

	universalAxis1[0] = 1;
	universalAxis1[1] = 0;
	universalAxis1[2] = 0;

	universalAxis2[0] = 0;
	universalAxis2[1] = 0;
	universalAxis2[2] = 1;

	recordTorque = false;
}

ME_JOINT::ME_JOINT(int myID, int aID, ME_OBJECT *firstObj, ME_OBJECT *secondObj) {

	ID = myID;
	agentID = aID;

	limitsSpecified = false;
	sensorNeuronID = -1;
	motorNeuronID = -1;
	lowerLimit = -PI/2;
	upperLimit = PI/2;
	coneLimitAngle = PI/2;
	restitution = 0.0;
	stiffness = -1.0;
	motorForce = DEF_MOTOR_FORCE;
	motorSpeed = DEF_MOTOR_SPEED;
	isWheel = false;
	isBinaryMotor = false;
	strcpy(relatedJointName,"");
	relatedJoint = NULL;
	forceControlled = false;

	jointType = JOINT_FIXED;

	firstBody = firstObj;
	secondBody = secondObj;

	objectContainingJoint = NULL;

	containingAgent = firstBody->containingAgent;

	const double *myPos;
	myPos = dBodyGetPosition(firstObj->body);
	
	jointPosition[0] = myPos[0];
	jointPosition[1] = myPos[1];
	jointPosition[2] = myPos[2];

	jointNormal[0] = 1;
	jointNormal[1] = 0;
	jointNormal[2] = 0;

	universalAxis1[0] = 1;
	universalAxis1[1] = 0;
	universalAxis1[2] = 0;

	universalAxis2[0] = 0;
	universalAxis2[1] = 0;
	universalAxis2[2] = 1;

	recordTorque = false;
}

ME_JOINT::ME_JOINT(int aID, ME_OBJECT **objs, int numObjs, double *disp,
						 ME_JOINT *templateJoint, NEURAL_NETWORK *network) {

	ID = templateJoint->ID;
	agentID = aID;

	strcpy(name,templateJoint->name);

	limitsSpecified = templateJoint->limitsSpecified;

	if ( templateJoint->sensorNeuronID > -1 )
		sensorNeuronID = network->AddSensor();

	if ( templateJoint->motorNeuronID > -1 )
		motorNeuronID = network->AddMotor();

	lowerLimit = templateJoint->lowerLimit;
	upperLimit = templateJoint->upperLimit;
	coneLimitAngle = templateJoint->coneLimitAngle;
	restitution = templateJoint->restitution;
	stiffness = templateJoint->stiffness;
	motorForce = templateJoint->motorForce;
	motorSpeed = templateJoint->motorSpeed;
	isWheel = templateJoint->isWheel;
	isBinaryMotor = templateJoint->isBinaryMotor;
	forceControlled = templateJoint->forceControlled;

	strcpy(relatedJointName,templateJoint->relatedJointName);
	relatedJoint = NULL;

	jointType = templateJoint->jointType;

	char objToFind[50];

	sprintf(objToFind,"%s_%d",templateJoint->firstBody->name,aID);
	firstBody = FindObject(objs,numObjs,objToFind);

	sprintf(objToFind,"%s_%d",templateJoint->secondBody->name,aID);
	secondBody = FindObject(objs,numObjs,objToFind);

	objectContainingJoint = NULL;

	containingAgent = firstBody->containingAgent;

	jointPosition[0] = templateJoint->jointPosition[0] + disp[0];
	jointPosition[1] = templateJoint->jointPosition[1] + disp[1];
	jointPosition[2] = templateJoint->jointPosition[2] + 0.1;

	jointNormal[0] = templateJoint->jointNormal[0];
	jointNormal[1] = templateJoint->jointNormal[1];
	jointNormal[2] = templateJoint->jointNormal[2];

	universalAxis1[0] = templateJoint->universalAxis1[0];
	universalAxis1[1] = templateJoint->universalAxis1[1];
	universalAxis1[2] = templateJoint->universalAxis1[2];

	universalAxis2[0] = templateJoint->universalAxis2[0];
	universalAxis2[1] = templateJoint->universalAxis2[1];
	universalAxis2[2] = templateJoint->universalAxis2[2];

	recordTorque = templateJoint->recordTorque;
}

ME_JOINT::ME_JOINT(int aID, ME_OBJECT **objs, int numObjs,
							ifstream *envFile, int inputID, NEURAL_NETWORK *network) {

	ID = inputID;
	agentID = aID;

	limitsSpecified = false;
	sensorNeuronID = -1;
	motorNeuronID = -1;
	lowerLimit = -PI/2;
	upperLimit = PI/2;
	coneLimitAngle = PI/2;
	restitution = 0.0;
	stiffness = -1.0;
	motorForce = DEF_MOTOR_FORCE;
	motorSpeed = DEF_MOTOR_SPEED;
	isWheel = false;
	isBinaryMotor = false;
	strcpy(relatedJointName,"");
	relatedJoint = NULL;
	forceControlled = false;

	firstBody = NULL;
	secondBody = NULL;
	objectContainingJoint = NULL;

	jointPosition[0] = 0;
	jointPosition[1] = 0;
	jointPosition[2] = 0;

	jointNormal[0] = 1;
	jointNormal[1] = 0;
	jointNormal[2] = 0;

	universalAxis1[0] = 1;
	universalAxis1[1] = 0;
	universalAxis1[2] = 0;

	universalAxis2[0] = 0;
	universalAxis2[1] = 0;
	universalAxis2[2] = 1;

	recordTorque = false;

	ReadIn(envFile,objs,numObjs,network);
}

ME_JOINT::~ME_JOINT(void) {

	if ( firstBody != NULL )
		firstBody = NULL;

	if ( secondBody != NULL )
		secondBody = NULL;

	if ( objectContainingJoint != NULL )
		objectContainingJoint = NULL;

	if ( containingAgent != NULL )
		containingAgent = NULL;

	if ( relatedJoint != NULL )
		relatedJoint = NULL;
}

void ME_JOINT::Actuate(double motorPosition) {

	if ( (recordTorque) && 
		  (simParams->functionAsReader) && 
		  (simParams->torqueFile != NULL) )
		RecordTorque();

	if ( forceControlled )
		ActuateByForce(motorPosition);
	else
		if ( isWheel )
			ActuateByVelocity(motorPosition);
		else
			ActuateByPosition(motorPosition);
}

void ME_JOINT::Attach(dWorldID world) {

	firstBody->InvertColour();
	firstBody->isAttached = true;

//	secondBody->InvertColour();
	secondBody->isAttached = true;

	//McdSpaceDisablePair(firstBody->bodyCM,secondBody->bodyCM);

	CreateODEStuff(world);
}

void ME_JOINT::AttachToWorld(dWorldID world, ME_OBJECT *o) {

	o->fixedToWorld = true;

	firstBody = o;
	secondBody = NULL;

	//jointType = JOINT_FIXED;
	joint = dJointCreateFixed(world,0);
	dJointAttach(joint,o->body,0);
	dJointSetFixed(joint);
}

int  ME_JOINT::AttachesTo(int firstAgentID, int secondAgentID) {

	return( (firstBody->containingAgent->GetID()==firstAgentID) &&
		     (secondBody->containingAgent->GetID()==secondAgentID) );
}

void ME_JOINT::Break(void) {

	firstBody->InvertColour();
	firstBody->isAttached = false;

//	secondBody->InvertColour();
	secondBody->isAttached = false;

	//McdSpaceEnablePair(firstBody->bodyCM,secondBody->bodyCM);

	DestroyODEStuff();
}

int ME_JOINT::BreakageDesired(NEURAL_NETWORK *network) {

	return( firstBody->BreakageDesired(network) ||
		     secondBody->BreakageDesired(network) );
}

ME_OBJECT *ME_JOINT::ComplementaryObj(ME_OBJECT *obj) {

	if ( obj==firstBody )
		return( secondBody );
	else
		return( firstBody );
}

void ME_JOINT::CreateODEStuff(dWorldID world) {

	dReal objPos1[3], objRot1[12], objPos2[3], objRot2[12];

	firstBody->GetPosAndRot(objPos1,objRot1);

	if ( secondBody && (jointType!=JOINT_FIXED) ) {

		secondBody->GetPosAndRot(objPos2,objRot2);

		originalRelativeDistance = sqrt( pow(objPos1[0]-objPos2[0],2.0) +
													pow(objPos1[1]-objPos2[1],2.0) +
													pow(objPos1[2]-objPos2[2],2.0) );

	}

	if ( jointType == JOINT_HINGE ) {

		joint = dJointCreateHinge(world,0);

		if ( !isWheel ) {
			dJointSetHingeParam(joint,dParamLoStop,lowerLimit);
			dJointSetHingeParam(joint,dParamHiStop,upperLimit);
		}

		//double h = 0.01;
		//dJointSetHingeParam(joint,dParamStopERP,(h*stiffness)	/ (h*stiffness+restitution) );
		//dJointSetHingeParam(joint,dParamStopCFM,(1)				/ (h*stiffness+restitution) );

		dJointAttach(joint,firstBody->startOfAggregate->body,secondBody->startOfAggregate->body);
		dJointSetHingeAnchor(joint,jointPosition[0],jointPosition[1],jointPosition[2]);
		dJointSetHingeAxis(joint,jointNormal[0],jointNormal[1],jointNormal[2]);
	}

	if ( jointType == JOINT_BALL ) {

		joint = dJointCreateBall(world,0);
		dJointAttach(joint,firstBody->startOfAggregate->body,secondBody->startOfAggregate->body);
		dJointSetBallAnchor(joint,jointPosition[0],jointPosition[1],jointPosition[2]);
	}

	if ( jointType == JOINT_FIXED ) {

		joint = dJointCreateFixed(world,0);
		dJointAttach(joint,firstBody->body,secondBody->body);
		dJointSetFixed(joint);

		/*
		if ( ((firstBody->body) && (secondBody->body)) ||
			 ((firstBody->body) && (!secondBody->body) && (secondBody->startOfAggregate!=firstBody))  ||
			 ((secondBody->body) && (!firstBody->body) && (firstBody->startOfAggregate!=secondBody)) ) {

			joint = dJointCreateHinge(world,0);
			dJointSetHingeParam(joint,dParamLoStop,-0.01);
			dJointSetHingeParam(joint,dParamHiStop,0.01);

			dJointAttach(joint,firstBody->startOfAggregate->body,secondBody->startOfAggregate->body);
			dJointSetHingeAnchor(joint,jointPosition[0],jointPosition[1],jointPosition[2]);
			dJointSetHingeAxis(joint,1.0,0.0,0.0);
			//dJointSetHingeAxis(joint,0.0,1.0,0.0);
		}
		else
			joint = NULL;
		*/
	}

	if ( jointType == JOINT_SLIDER ) {

		//joint = dJointCreateFixed(world,0);
		//dJointAttach(joint,firstBody->body,secondBody->body);
		//dJointSetFixed(joint);

		joint = dJointCreateSlider(world,0);

		dJointSetSliderParam(joint,dParamLoStop,lowerLimit);
		dJointSetSliderParam(joint,dParamHiStop,upperLimit);
		
		double h = 0.01;
		dJointSetSliderParam(joint,dParamStopERP,(h*stiffness)	/ (h*stiffness+restitution) );
		dJointSetSliderParam(joint,dParamStopCFM,(1)			/ (h*stiffness+restitution) );

		dJointAttach(joint,firstBody->startOfAggregate->body,secondBody->startOfAggregate->body);

		jointNormal[0] = firstBody->position[0] - secondBody->position[0];
		jointNormal[1] = firstBody->position[1] - secondBody->position[1];
		jointNormal[2] = firstBody->position[2] - secondBody->position[2];

		dJointSetSliderAxis(joint,jointNormal[0],jointNormal[1],jointNormal[2]);
	}

	if ( jointType == JOINT_WORLD )
		AttachToWorld(world,firstBody);

	if ( jointType == JOINT_SPRING ) {
		joint = NULL;
		
		dReal objPos1[3], objRot1[12], objPos2[3], objRot2[12];

		firstBody->GetPosAndRot(objPos1,objRot1);
		secondBody->GetPosAndRot(objPos2,objRot2);
		originalRelativeDistance = simParams->DistBetween(objPos1,objPos2);
	}
}

void ME_JOINT::DestroyODEStuff(void) {

	if ( joint != NULL ) {
		dJointDestroy(joint);
		joint = NULL;
	}
}

void ME_JOINT::Display(void) {

	printf("Joint %d: %s\n",ID,name);

	printf("\t Connects %s to %s using a",firstBody->name,secondBody->name);

	if ( jointType == JOINT_HINGE )
		printf(" hinge ");
	if ( jointType == JOINT_BALL )
		printf(" ball and socket ");
	if ( jointType == JOINT_FIXED )
		printf(" fixed ");
	if ( jointType == JOINT_SLIDER )
		printf(" prismatic ");
	if ( jointType == JOINT_WORLD )
		printf(" world ");
	if ( jointType == JOINT_SPRING )
		printf(" spring ");

	printf("joint.\n");

	printf("\t Position [%2.2f,%2.2f,%2.2f]\n",
		jointPosition[0],jointPosition[1],jointPosition[2]);

	if ( jointType == JOINT_HINGE )
		printf("\t Normal   [%2.2f,%2.2f,%2.2f]\n",
			jointNormal[0],jointNormal[1],jointNormal[2]);

	if ( sensorNeuronID > -1 )
		printf("Contains a proprioceptive sensor.");
}

void ME_JOINT::DrawNormals(void) {

	if ( (joint) && (jointType == JOINT_SLIDER) ) {

		double res1[3];

		dJointGetSliderAxis(joint,res1);

		const dReal *firstPos = dBodyGetPosition(firstBody->startOfAggregate->body);
		const dReal *secondPos = dBodyGetPosition(secondBody->startOfAggregate->body);

		res1[0] = firstPos[0] + res1[0];
		res1[1] = firstPos[1] + res1[1];
		res1[2] = firstPos[2] + res1[2];

		dsSetColor(1.0,0.0,0.0);
		dsDrawLine(firstPos,res1);
		dsSetColor(0.0,0.0,1.0);
		dsDrawLine(firstPos,secondPos);
	}
}

void ME_JOINT::Draw(void) {

	if ( jointType == JOINT_HINGE ) {

		dReal jointPos[3];
		dJointGetHingeAnchor(joint,jointPos);

		double R[12];
		simParams->SetDefaultRotation(R);
		
		dsSetColor(0.0,1.0,0.0);
		dsDrawSphere(jointPos,R,0.1);

	}

	if ( jointType == JOINT_SPRING ) {

		dReal objPos1[3], objRot1[12], objPos2[3], objRot2[12];

		firstBody->GetPosAndRot(objPos1,objRot1);
		secondBody->GetPosAndRot(objPos2,objRot2);

		dsDrawLineD(objPos1,objPos2);
	}
}

void ME_JOINT::DrawMotorNeurons(double maxHeight, double heightOffset) {

	/*
	if ( motorNeuronID > -1 ) {

		dReal pos[3], neuronPos[3];
		
		dReal tempPos[3], orientation[12];

		ME_OBJECT *tempObject;

		if ( objectContainingJoint )
			tempObject = objectContainingJoint;
		else
			tempObject = firstBody;

		tempObject->GetPosAndRot(tempPos,orientation);
		
		double motorValue = fabs(tempObject->nn->GetMotorCommand(motorNeuronID));

			if ( tempObject->objectType == SPHERE )
				tempObject->GetPosAndRot(pos,orientation);
			else
				dBodyGetRelPointPos (tempObject->body,0.0,0.0,tempObject->size[1]/2.0,pos);

		neuronPos[0] = pos[0];
		neuronPos[1] = pos[1];
		neuronPos[2] = maxHeight + heightOffset;

		dsSetColor(motorValue,motorValue,motorValue);
		dsDrawSphere(neuronPos,orientation,MOTOR_NEURON_RADIUS);
		dsSetColor(0.0,0.0,0.0);
		dsDrawLineD(pos,neuronPos);

		tempObject = NULL;
	}
	*/
}

void ME_JOINT::DrawSensorNeurons(double maxHeight, double heightOffset) {

	/*
	if ( sensorNeuronID > -1 ) {

		dReal pos[3], neuronPos[3];
		
		dReal tempPos[3], orientation[12];

		ME_OBJECT *tempObject;

		if ( objectContainingJoint )
			tempObject = objectContainingJoint;
		else
			tempObject = firstBody;

		tempObject->GetPosAndRot(tempPos,orientation);
		
		double sensorVal = fabs(tempObject->nn->GetSensorValue(sensorNeuronID));

		if ( tempObject->position[1] < 0.0 ) { // On left side of body

			if ( tempObject->objectType == SPHERE )
				tempObject->GetPosAndRot(pos,orientation);
			else
				dBodyGetRelPointPos (tempObject->body,0.0,0.0,tempObject->size[1]/2.0,pos);
		}
		else {
			if ( tempObject->objectType == SPHERE )
				tempObject->GetPosAndRot(pos,orientation);
			else
				dBodyGetRelPointPos(tempObject->body,0.0,0.0,-tempObject->size[1]/2.0,pos);
		}

		neuronPos[0] = pos[0];
		neuronPos[1] = pos[1];
		//neuronPos[2] = maxHeight + heightOffset;
		neuronPos[2] = pos[2];

		dsSetColor(sensorVal,sensorVal,sensorVal);
		dsDrawSphere(neuronPos,orientation,SENSOR_NEURON_RADIUS);
		//dsSetColor(0.0,0.0,0.0);
		//dsDrawLineD(pos,neuronPos);
		
		tempObject = NULL;
	}
	*/
}

void ME_JOINT::FindPressureSensor(void) {

	if ( jointType == JOINT_SLIDER ) {

		sensorNeuronID = simParams->numSensors++;
	}
}

ME_OBJECT *ME_JOINT::GeomTransformPossible(void) {

	if ( (jointType == JOINT_FIXED) && (firstBody->body) && (secondBody->body ) ) {

		if ( firstBody->objectType == 2 ) // SPHERE
			return( secondBody );
		else
			if ( secondBody->objectType == 2 ) // SPHERE
				return( firstBody );
			else
				return( NULL );
	}
	else
		return( NULL );
}

double ME_JOINT::GetAngle(void) {

	if ( jointType == JOINT_HINGE ) {
		double angle = dJointGetHingeAngle(joint);
		//angle = simParams->Scale(angle,lowerLimit,upperLimit,-1.0,1.0);
		return( angle );
	}
	else
		return( 0.0 );
}

ME_JOINT *ME_JOINT::GetRelatedJoint(void) {

	return( relatedJoint );
}

char *ME_JOINT::GetRelatedJointName(void) {

	return( relatedJointName );
}

int  ME_JOINT::IsBreaking(void) {

	/*
	MeVector3 firstBodyPos, secondBodyPos, separation, force, diff;
	double cosOfAngle;

	MdtBodyGetPosition(firstBody->body,firstBodyPos);
	MdtBodyGetPosition(secondBody->body,secondBodyPos);

	separation[0] = secondBodyPos[0] - firstBodyPos[0];
	separation[1] = secondBodyPos[1] - firstBodyPos[1];
	separation[2] = secondBodyPos[2] - firstBodyPos[2];

	MdtHingeGetForce(hinge,0,force);

	cosOfAngle = simParams->Dot(separation,force) /
		(simParams->MagnitudeOfVector(separation) * simParams->MagnitudeOfVector(force));

	return( (cosOfAngle>0.9) && (simParams->MagnitudeOfVector(force)>firstBody->minForceForUnlocking) );
	*/
	return(false);
}

int  ME_JOINT::IsMyName(char *jointName) {

	return( strcmp(name,jointName) == 0 );
}

void ME_JOINT::RecordPressure(void) {

	if ( jointType == JOINT_SLIDER ) {

		double dist = firstBody->GetDistance(secondBody);
		containingAgent->sensorValues->Set(simParams->internalTimer,sensorNeuronID,dist);
	}
}

int  ME_JOINT::RelatedToOtherJoint(void) {

	return( strcmp(relatedJointName,"") != 0 );
}

void ME_JOINT::Reset(dWorldID world) {

	DestroyODEStuff();
	CreateODEStuff(world);
}

void ME_JOINT::ResetPosition(void) {
	
	if ( objectContainingJoint ) {

		jointPosition[0] = objectContainingJoint->position[0];
		jointPosition[1] = objectContainingJoint->position[1];
		jointPosition[2] = objectContainingJoint->position[2];
	}
}

int  ME_JOINT::Separated(void) {

	extern double MAX_JOINT_SEPARATION;
	extern int JOINT_HINGE;

	if ( jointType == JOINT_HINGE )
		return ( dJointGetHingeAngleRate(joint) > MAX_JOINT_SEPARATION );
	else
		return( false );
}

void ME_JOINT::SetOrientationSensor(void) {

	double angle = dJointGetHingeAngle(joint);

	containingAgent->sensorValues->Set(simParams->internalTimer,ID,-2.0*angle*180.0/(2*3.14159));

	//printf("%d %3.3f\n",ID,-2.0*angle*180.0/(2*3.14159));
}

void ME_JOINT::SetRelatedJoint(ME_JOINT *otherJoint) {

	relatedJoint = otherJoint;
	motorForce = relatedJoint->motorForce;
	motorSpeed = relatedJoint->motorSpeed;
}

void ME_JOINT::SwitchOnPressureSensor(dWorldID world) {

	if ( jointType == JOINT_SLIDER ) {

		dJointDestroy(joint);
		joint = NULL;

		joint = dJointCreateSlider(world,0);

		dJointSetSliderParam(joint,dParamLoStop,lowerLimit);
		dJointSetSliderParam(joint,dParamHiStop,upperLimit);
		
		double h = 0.01;
		dJointSetSliderParam(joint,dParamStopERP,(h*stiffness)	/ (h*stiffness+restitution) );
		dJointSetSliderParam(joint,dParamStopCFM,(1)			/ (h*stiffness+restitution) );

		dJointAttach(joint,firstBody->startOfAggregate->body,secondBody->startOfAggregate->body);

		jointNormal[0] = firstBody->position[0] - secondBody->position[0];
		jointNormal[1] = firstBody->position[1] - secondBody->position[1];
		jointNormal[2] = firstBody->position[2] - secondBody->position[2];

		dJointSetSliderAxis(joint,jointNormal[0],jointNormal[1],jointNormal[2]);
	}

}

void ME_JOINT::UpdateSpring(void) {

	if ( jointType == JOINT_SPRING ) {

		dReal objPos1[3], objRot1[12], objPos2[3], objRot2[12];
		
		firstBody->GetPosAndRot(objPos1,objRot1);
		secondBody->GetPosAndRot(objPos2,objRot2);

		dQuaternion q;
		q[0] = 1.0;
		q[1] = 0.0;
		q[2] = 0.0;
		q[3] = 0.0;

		dBodySetQuaternion(firstBody->startOfAggregate->body,q);
		dBodySetQuaternion(secondBody->startOfAggregate->body,q);

		double distBetween = simParams->DistBetween(objPos1,objPos2);
		double connectingVector[3];
		simParams->VectSub(objPos1,objPos2,connectingVector);

		if ( distBetween < originalRelativeDistance ) {

			dBodyAddForce(firstBody->startOfAggregate->body,
				connectingVector[0]*fabs(distBetween-originalRelativeDistance),
				connectingVector[1]*fabs(distBetween-originalRelativeDistance),
				connectingVector[2]*fabs(distBetween-originalRelativeDistance));

			dBodyAddForce(secondBody->startOfAggregate->body,
				-connectingVector[0]*fabs(distBetween-originalRelativeDistance),
				-connectingVector[1]*fabs(distBetween-originalRelativeDistance),
				-connectingVector[2]*fabs(distBetween-originalRelativeDistance));
		}
		else {
			dBodyAddForce(firstBody->startOfAggregate->body,
				-connectingVector[0]*fabs(distBetween-originalRelativeDistance),
				-connectingVector[1]*fabs(distBetween-originalRelativeDistance),
				-connectingVector[2]*fabs(distBetween-originalRelativeDistance));

			dBodyAddForce(secondBody->startOfAggregate->body,
				connectingVector[0]*fabs(distBetween-originalRelativeDistance),
				connectingVector[1]*fabs(distBetween-originalRelativeDistance),
				connectingVector[2]*fabs(distBetween-originalRelativeDistance));
		}
	}
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void ME_JOINT::ActuateByForce(double forceToApply) {

	int firstPump = false;
	int secondPump = false;

	if ( forceToApply < -0.5 ) {}
	else
		if ( forceToApply < 0.0 )
			firstPump = true;
		else
			if ( forceToApply < 0.5 )
				secondPump = true;
			else {
				firstPump = true;
				secondPump = true;
			}

	if ( firstPump )
		firstBody->SetColour(1.0,1.0,1.0);
	else
		firstBody->SetColour(0.0,0.0,0.0);

	if ( secondPump )
		secondBody->SetColour(1.0,1.0,1.0);
	else
		secondBody->SetColour(0.0,0.0,0.0);

	if ( firstPump || secondPump ) {

		const dReal *firstPos = dBodyGetPosition(firstBody->startOfAggregate->body);
		const dReal *secondPos = dBodyGetPosition(secondBody->startOfAggregate->body);

		double forceVect[3];
		forceVect[0] = firstPos[0] - secondPos[0];
		forceVect[1] = firstPos[1] - secondPos[1];
		forceVect[2] = firstPos[2] - secondPos[2];

		simParams->Normalize(forceVect);

		if ( firstPump )
			if ( !secondPump )
				simParams->VectMult(forceVect,motorForce);
			else
				simParams->VectMult(forceVect,motorForce - compressionForce);
		else
			simParams->VectMult(forceVect,-compressionForce);

		dBodyAddForce(firstBody->startOfAggregate->body,forceVect[0],forceVect[1],forceVect[2]);
		dBodyAddForce(secondBody->startOfAggregate->body,-forceVect[0],-forceVect[1],-forceVect[2]);
	}
}

void ME_JOINT::ActuateByPosition(double motorPosition) {

	if ( jointType == JOINT_HINGE )
		ActuateHinge(motorPosition);
	
	else if ( jointType == JOINT_SLIDER )
		ActuateSlider(motorPosition);
}

void ME_JOINT::ActuateHinge(double motorPosition) {

	dJointSetHingeParam(joint,dParamFMax,motorForce*1000000.0);

	//double desiredJointAngle = simParams->Scale(motorPosition,0.0,255.0,lowerLimit,upperLimit);
	double desiredJointAngle = motorPosition*3.14159/180.0;

	double actualJointAngle =  GetAngle();

	double error = desiredJointAngle - actualJointAngle;

	if ( motorPosition == 0.0 )
		error = 0.0;

	//dJointSetHingeParam(joint,dParamVel,2.0*error);
	dJointSetHingeParam(joint,dParamVel,5.0*error);
}

void ME_JOINT::ActuateSlider(double motorPosition) {

	if ( joint ) {

		//motorPosition = simParams->Rand(-1.0,1.0);

		dJointSetSliderParam(joint,dParamFMax,motorForce/2.0);

		double desiredJointAngle =
			simParams->Scale(motorPosition,-1.0,1.0,lowerLimit,upperLimit);

		double error = desiredJointAngle - dJointGetSliderPosition(joint);
		//error = 0.0;

		dJointSetSliderParam(joint,dParamVel,motorSpeed*error);
	}
}

void ME_JOINT::ActuateByVelocity(double motorVel) {

	/*
	double desiredVelocity = 
		simParams->Scale(motorVel,-1.0,1.0,lowerLimit,upperLimit);

	MdtLimitSetLimitedForceMotor(limit, desiredVelocity,motorForce);
	*/
}

ME_OBJECT* ME_JOINT::FindObject(ME_OBJECT **objs, int numObjs, char *name) {

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

int ME_JOINT::NumberEntered(char *name) {

	return( ((name[0]>=48)&&(name[0]<=57)) || (name[0]==45) );
}

void ME_JOINT::ReadIn(ifstream *envFile, ME_OBJECT **objs, int numObjs,
								NEURAL_NETWORK *network) {

	char readIn[50];
	char name1[50], name2[50];

	(*envFile) >> readIn;
	
	if ( strcmp(readIn,"(") != 0 ) {
		printf("Read in error: expecting opening bracket.\n");
		char ch = getchar();
		exit(0);
	}

	(*envFile) >> name;
	
	(*envFile) >> readIn;

	while ( strcmp(readIn,")") != 0 ) {

		if ( strcmp(readIn,"-connect") == 0 ) {

			(*envFile) >> name1;
			(*envFile) >> name2;

			firstBody = FindObject(objs,numObjs,name1);
			secondBody = FindObject(objs,numObjs,name2);
			containingAgent = firstBody->containingAgent;

			simParams->connectedObjects->Set(firstBody->ID,secondBody->ID,1.0);

			if ( firstBody == NULL ) {
				printf("Error: could not locate object %s.\n",name1);
				char ch = getchar();
				exit(0);
			}

			if ( secondBody == NULL ) {
				printf("Error: could not locate object %s.\n",name2);
				char ch = getchar();
				exit(0);
			}
		}
		else
		if ( strcmp(readIn,"-jointType") == 0 ) {

			(*envFile) >> readIn;

			if ( strcmp(readIn,"Hinge") == 0 )
				jointType = JOINT_HINGE;

			if ( strcmp(readIn,"BallAndSocket") == 0 )
				jointType = JOINT_BALL;

			if ( strcmp(readIn, "Fixed") == 0 ) {
				simParams->numSensors++;
				jointType = JOINT_FIXED;
			}

			if ( strcmp(readIn, "Wheel") == 0 ) {
				jointType = JOINT_HINGE;
				isWheel = true;
			}
			if ( strcmp(readIn, "Prismatic") == 0 )
				jointType = JOINT_SLIDER;

			if ( strcmp(readIn, "ToWorld") == 0 )
				jointType = JOINT_WORLD;

			if ( strcmp(readIn, "Spring") == 0 )
				jointType = JOINT_SPRING;
		}
		else
		if ( strcmp(readIn,"-jointPosition") == 0 ) {

			(*envFile) >> name1;

			if ( NumberEntered(name1) ) {
				jointPosition[1] = atof(name1);
				(*envFile) >> name1;
				jointPosition[2] = atof(name1);
				(*envFile) >> name1;
				jointPosition[0] = -atof(name1);
			}
			else {
				objectContainingJoint = FindObject(objs,numObjs,name1);

				if ( objectContainingJoint == NULL ) {
					printf("Error: could not locate object %s.\n",name1);
					char ch = getchar();
					exit(0);
				}
				else {
					jointPosition[0] = (double)objectContainingJoint->position[0];
					jointPosition[1] = (double)objectContainingJoint->position[1];
					jointPosition[2] = (double)objectContainingJoint->position[2];
				}
			}
		}
		else
		if ( strcmp(readIn,"-jointNormal") == 0 ) {

			(*envFile) >> readIn;
			jointNormal[1] = -(double)atof(readIn);
			(*envFile) >> readIn;
			jointNormal[2] = (double)atof(readIn);
			(*envFile) >> readIn;
			jointNormal[0] = (double)atof(readIn);
		}
		else
		if ( strcmp(readIn,"-jointLimits") == 0 ) {

			limitsSpecified = true;
			(*envFile) >> readIn;

			if ( jointType == JOINT_HINGE ) {

				if ( !isWheel ) {
					lowerLimit = double(atof(readIn)*(PI/180.0));
					(*envFile) >> readIn;
					upperLimit = double(atof(readIn)*(PI/180.0));
				}
				else {
					lowerLimit = double(atof(readIn));
					(*envFile) >> readIn;
					upperLimit = double(atof(readIn));
				}
			}

			if ( jointType == JOINT_SLIDER ) {
				lowerLimit = double(atof(readIn));
				(*envFile) >> readIn;
				upperLimit = double(atof(readIn));
			}

			if ( lowerLimit > upperLimit ) {
				double temp = lowerLimit;
				lowerLimit = upperLimit;
				upperLimit = temp;
			}

			if ( jointType == JOINT_BALL )
				coneLimitAngle = double(atof(readIn)*(PI/180.0));
		}
		else
		if ( strcmp(readIn,"-springConstants") == 0 ) {

			(*envFile) >> readIn;
			restitution = atof(readIn);

			(*envFile) >> readIn;
			stiffness = atof(readIn);
		}
		else
		if ( strcmp(readIn,"-motorForce") == 0 ) {

			(*envFile) >> readIn;
			motorForce = atoi(readIn);
		}
		else
		if ( strcmp(readIn,"-compressionForce") == 0 ) {

			forceControlled = true;
			(*envFile) >> readIn;
			compressionForce = atoi(readIn);
		}
		else
		if ( strcmp(readIn,"-motorSpeed") == 0 ) {

			(*envFile) >> readIn;
			motorSpeed = atoi(readIn);
		}
		else
		if ( strcmp(readIn,"-addSensor") == 0 ) {
			simParams->numSensors++;
			sensorNeuronID = 1.0;
		}
		else
		if ( strcmp(readIn,"-addMotor") == 0 )
			motorNeuronID = 1.0;
		else
		if ( strcmp(readIn,"-motorizedBy") == 0 )
			(*envFile) >> relatedJointName;
		else
		if ( strcmp(readIn,"-binaryMotor") == 0 )
			isBinaryMotor = true;
		else
		if ( strcmp(readIn,"-uniAxis1") == 0 ) {
			(*envFile) >> readIn;
			universalAxis1[1] = (double)atof(readIn);
			(*envFile) >> readIn;
			universalAxis1[2] = (double)atof(readIn);
			(*envFile) >> readIn;
			universalAxis1[0] = (double)atof(readIn);
		}
		else
		if ( strcmp(readIn,"-uniAxis2") == 0 ) {
			(*envFile) >> readIn;
			universalAxis2[1] = (double)atof(readIn);
			(*envFile) >> readIn;
			universalAxis2[2] = (double)atof(readIn);
			(*envFile) >> readIn;
			universalAxis2[0] = (double)atof(readIn);
		}
		else
		if ( strcmp(readIn,"-recordTorque") == 0 ) {
			recordTorque = true;
			if ( (simParams->torqueFile == NULL) && (simParams->functionAsReader) ) {
				char tempStr[50];
				strcpy(tempStr,"data/torqueFile.dat");
				simParams->torqueFile = new ofstream(tempStr);
			}
		}
		else {
			printf("ERROR: command %s not recognized in joint %s.\n",
				readIn,name);
			printf("Valid commands:\n");
			printf("\t -connect [object1 object2] \n");
			printf("\t -jointType [Hinge BallAndSocket Fixed Wheel Prismatic Universal] \n");
			printf("\t -jointPosition [object | x y z] \n");
			printf("\t -jointNormal [x y z] \n");
			printf("\t (for Hinge): -jointLimits [minAngle maxAngle] \n");
			printf("\t (for BallAndSocket): -jointLimits [angle] \n");
			printf("\t (for Wheel): -jointLimits [minVelocity maxVelocity] \n");
			printf("\t (for Prismatic): -jointLimits [minExtension maxExtension] \n");
			printf("\t -springConstants [restitution stiffness] \n");
			printf("\t -motorForce [value] \n");
			printf("\t -motorSpeed [value] \n");
			printf("\t -binaryMotor \n");
			printf("\t -addSensor \n");
			printf("\t -addMotor \n");
			printf("\t -motorizedBy [jointName]\n");
			printf("\t -recordTorque \n");
			char ch = getchar();
			exit(0);
		}
		(*envFile) >> readIn;
	}

	double temp[3];

	temp[0] = jointNormal[0];
	temp[1] = jointNormal[1];
	temp[2] = jointNormal[2];
	simParams->Normalize(temp);
	jointNormal[0] = temp[0];
	jointNormal[1] = temp[1];
	jointNormal[2] = temp[2];

	temp[0] = universalAxis1[0];
	temp[1] = universalAxis1[1];
	temp[2] = universalAxis1[2];
	simParams->Normalize(temp);
	universalAxis1[0] = temp[0];
	universalAxis1[1] = temp[1];
	universalAxis1[2] = temp[2];
	
	temp[0] = universalAxis2[0];
	temp[1] = universalAxis2[1];
	temp[2] = universalAxis2[2];
	simParams->Normalize(temp);
	universalAxis2[0] = temp[0];
	universalAxis2[1] = temp[1];
	universalAxis2[2] = temp[2];
}

void ME_JOINT::RecordTorque(void) {

	/*
	MeVector3 torque;

	if ( jointType == JOINT_HINGE )
		MdtHingeGetTorque(hinge,0,torque);
	else
		if ( jointType == JOINT_SLIDER )
			MdtPrismaticGetTorque(prismatic,0,torque);

	(*simParams->torqueFile) << simParams->internalTimer << ";";
	(*simParams->torqueFile) << containingAgent->ID << ";";
	(*simParams->torqueFile) << ID << ";";
	(*simParams->torqueFile) << torque[0] << ";";
	(*simParams->torqueFile) << torque[1] << ";";
	(*simParams->torqueFile) << torque[2] << "\n";
	*/
}

#endif