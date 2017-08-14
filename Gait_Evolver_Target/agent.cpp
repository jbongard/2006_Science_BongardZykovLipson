/* ---------------------------------------------------
   FILE:     agent.cpp
	AUTHOR:   Josh Bongard
	DATE:     February 19, 2002
	FUNCTION: This class contains all information for
				 a single agent.
 -------------------------------------------------- */

#include "stdafx.h"

#include <drawstuff/drawstuff.h>

#ifndef _AGENT_CPP
#define _AGENT_CPP

#include "agent.h"
#include "simParams.h"

extern int OUTPUT_FOOTPRINTS;
extern int OUTPUT_TRAJECTORY;
extern int CREATE_PLUME;
extern double GROUND_LENGTH;
extern int RANDOM_DISPERSAL;
extern double DISP_PTS[16];
extern int			JOINT_SPRING;
extern int	JOINT_HINGE;
extern int  CYLINDER;
extern int  SPHERE;

extern char BODY_FILENAME[50];
extern char BRAIN_FILENAME[50];
extern char OBJECT_FILENAME[50];
extern char SENSOR_FILENAME[50];

extern SIM_PARAMS *simParams;

AGENT::AGENT(int IDNum, dWorldID world, dSpaceID space) {

	ID = IDNum;
	timeRemainingUntilNextMotorCommand = 10;

	trajectory = NULL;
	targetObject = NULL;
	totalMass = 0.0;
	totalVolume = 0.0;

	CreateBody(world,space);

	ReadInDataParameters();

	sensorValues = new MATRIX(simParams->evalPeriod,19,0.0); 
	// 8 angle, 4 touch, 2 orientation, 1 lowest height, 1 sensed distance from floor
}

AGENT::AGENT(int IDNum, AGENT *templateAgent) {

	ID = IDNum;
	numObjects = templateAgent->numObjects;
	numJoints = templateAgent->numJoints;
	hiddenNodes = templateAgent->hiddenNodes;
	numMorphParams = templateAgent->numMorphParams;
	trajectory = NULL;

	if ( RANDOM_DISPERSAL ) {
		displacement[0] = simParams->Rand((-GROUND_LENGTH/2.0)*simParams->displacementDistance,
													 (GROUND_LENGTH/2.0)*simParams->displacementDistance);
		displacement[1] = simParams->Rand((-GROUND_LENGTH/2.0)*simParams->displacementDistance,
													 (GROUND_LENGTH/2.0)*simParams->displacementDistance);
	}
	else {
		displacement[0] = DISP_PTS[(ID-1)*2] * (GROUND_LENGTH/2.0)*simParams->displacementDistance;
		displacement[1] = DISP_PTS[(ID-1)*2+1] * (GROUND_LENGTH/2.0)*simParams->displacementDistance;
	}

	objects = new ME_OBJECT * [numObjects];
	joints  = new ME_JOINT  * [numJoints];
	network = new NEURAL_NETWORK;

	int currObj;
	for (currObj=0;currObj<numObjects;currObj++)
		objects[currObj] = new ME_OBJECT(this,templateAgent->objects[currObj],displacement,network);

	for (currObj=0;currObj<numObjects;currObj++)
		objects[currObj]->FindOtherObjects(numObjects,objects);

	int currJoint;
	for (currJoint=0;currJoint<numJoints;currJoint++)
		joints[currJoint] = new ME_JOINT(ID,objects,numObjects,displacement,
												 templateAgent->joints[currJoint],network);

	FindRelatedJoints();

	//network->Init(hiddenNodes,templateAgent->network->isReactive);
}

AGENT::~AGENT(void) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		delete objects[currObject];

	int currJoint;

	for (currJoint=0;currJoint<numJoints;currJoint++)
		delete joints[currJoint];

	if ( trajectory ) {

		delete trajectory;
		trajectory = NULL;
	}

	targetObject = NULL;

	//sensorValues->NormalizeToMass();

	//sensorValues->WriteRows(simParams->internalTimer,simParams->sensorFile);

	double meanDiff;

	for (int j=1;j<15;j++) {

		meanDiff = simParams->totalChanges->NonZeroMeanOfColumn(j);
		sensorValues->Set(simParams->internalTimer-1,j,meanDiff);
	}

	simParams->totalChanges->Zero();

	//double totalHeight = sensorValues->SumOfColumn(17);
	//sensorValues->Set(simParams->internalTimer-1,17,totalHeight/double(simParams->evalPeriod));

	double maxHeight = sensorValues->MaxOfColumn(17);
	sensorValues->Set(simParams->internalTimer-1,17,maxHeight);

	sensorValues->WriteRow(simParams->internalTimer-1,simParams->sensorFile);

	/*
	if ( objectData ) {

		objectData->Write(simParams->movementFile);
	}

	if ( sensorData ) {

		sensorData->Write(simParams->sensorFile);
	}
	*/

	delete[] objects;
	objects = NULL;

	delete[] joints;
	joints = NULL;

	delete sensorValues;
	sensorValues = NULL;
}

int  AGENT::AttachedTo(AGENT *otherAgent) {

	if ( numOfExternalJoints==0 )
		if ( otherAgent->numOfExternalJoints==0 )
			return( false );
		else
			return( otherAgent->AttachedTo(this) );
	else {
		int currExtJoint = 0;
		int extJointToOtherAgent = false;

		while ( (currExtJoint<numOfExternalJoints) && (!extJointToOtherAgent) ) {

			if ( externalJoints[currExtJoint]->AttachesTo(ID,otherAgent->ID) )
				extJointToOtherAgent = true;
			currExtJoint++;
		}
		return( extJointToOtherAgent );
	}
}

void AGENT::AttachTo(AGENT *otherAgent, dWorldID world) {

	/*
	ME_OBJECT *firstObj, *secondObj;

	firstObj = (ME_OBJECT *)McdModelGetUserData(result.pair->model1);
	secondObj = (ME_OBJECT *)McdModelGetUserData(result.pair->model2);

	externalJoints[numOfExternalJoints] = new ME_JOINT(numOfExternalJoints,ID,firstObj,secondObj);
	externalJoints[numOfExternalJoints]->Attach(world);
	numOfExternalJoints++;
	*/
}

int  AGENT::ContainsLightSensor(void) {

	int containsLightSensor = false;
	int objIndex = 0;

	while ( (objIndex < numObjects) && (!containsLightSensor) ) {
		if ( objects[objIndex]->lightSensorID > -1 )
			containsLightSensor = true;
		objIndex++;
	}

	return( containsLightSensor );

}

void AGENT::CreateJoints(dWorldID world) {

	int currJoint;

	for (currJoint=0;currJoint<numJoints;currJoint++)
		joints[currJoint]->CreateODEStuff(world);
}

void AGENT::CreateObjects(int genomeLength, double *genes, ME_OBJECT *lightSource,
								  dWorldID world, dSpaceID space) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)

		objects[currObject]->CreateODEStuff(world,space);

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->DisableContacts();

	if ( lightSource != NULL )
		DetermineMaxLightSensorReading(lightSource);
}

void AGENT::CreateODEStuff(dWorldID world, dSpaceID space) {

	int currObject, currJoint;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->CreateODEStuff(world,space);

	RemoveExtraneousObjects(space);

	for (currJoint=0;currJoint<numJoints;currJoint++)
		joints[currJoint]->CreateODEStuff(world);
}

void AGENT::DestroyODEStuff(dSpaceID space) {

	int currObject;
	int currJoint;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->DestroyODEStuff(space);

	for (currJoint=0;currJoint<numJoints;currJoint++)
		joints[currJoint]->DestroyODEStuff();

	//if ( network != NULL )
	//	network->ClearValues();
}

double AGENT::DistTo(int objIndex, AGENT *otherAgent) {

	double myPos[3], myRot[12], otherPos[3], otherRot[12];
	ME_OBJECT *otherObj;

	objects[objIndex]->GetPosAndRot(myPos,myRot);

	otherObj = otherAgent->GetObject(objIndex);
	otherObj->GetPosAndRot(otherPos,otherRot);

	return( simParams->DistBetween(myPos,otherPos) );
}

void AGENT::Draw(void) {

	int currObject, currJoint;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->Draw();

	//for (currJoint=0;currJoint<numJoints;currJoint++)
	//	joints[currJoint]->Draw();

	if ( trajectory )
		trajectory->Draw(1.0,1.0,1.0);

	if ( simParams->drawNetwork )
		DrawNeuralNetwork();
}

int  AGENT::Exploding(void) {

	/*
	int i = 0;
	int exploding = false;

	//while ( (i<numObjects) && (!exploding) ) {
		//exploding = joints[i]->Separated();
		exploding = objects[0]->BeyondRange();
	//	i++;
	//}
	*/

	int currObject = 1;
	double dist = 0.0;

	while ( currObject<numObjects ) {

		objects[currObject]->IsInside(objects[0],&dist);

		currObject++;
	}

	if ( dist > 0.0 ) {
		sensorValues->SetAllTo(100.0);
		sensorValues->Set(0,1,dist);
	}

	return( dist > 0.0 );

	/*
	if ( simParams->agentExploding > 0 ) {

		sensorValues->SetAllTo(100.0);
		sensorValues->Set(0,1,simParams->agentExploding);
		return( true );
	}
	else
		return( false );
	*/
}

void AGENT::GetCentreOfMass(double *com) {

	int currObject;
	double pos[3], rot[12];

	com[0] = 0.0;
	com[1] = 0.0;
	com[2] = 0.0;

	for (currObject=0;currObject<numObjects;currObject++) {

		objects[currObject]->GetPosAndRot(pos,rot);

		com[0] = com[0] + pos[0];
		com[1] = com[1] + pos[1];
		com[2] = com[2] + pos[2];
	}

	com[0] = com[0] / (numObjects);
	com[1] = com[1] / (numObjects);
	com[2] = com[2] / (numObjects);
}

int AGENT::GetID(void) {

	return(ID);
}

ME_JOINT *AGENT::GetJoint(int jointIndex) {

	return( joints[jointIndex] );
}

ME_JOINT *AGENT::GetJoint(char *jointName) {

	int jointFound = false;
	int jointIndex = 0;

	while ( (jointIndex<numJoints) && (!jointFound) ) {

		jointFound = joints[jointIndex]->IsMyName(jointName);
		if ( !jointFound )
			jointIndex++;
	}

	if ( jointFound == false )
		return( NULL );
	else
		return( joints[jointIndex] );
}

double AGENT::GetMaxHeight(void) {

	double maxHeight = -100.0;

	int currObject;
	double currObjHeight;

	for (currObject=0;currObject<numObjects;currObject++) {

		currObjHeight = objects[currObject]->GetHeight();

		if ( currObjHeight > maxHeight )
			maxHeight = currObjHeight;
	}

	return( maxHeight );
}

double AGENT::GetMinHeight(void) {

	double minHeight = 100.0;

	int currObject;
	double currObjHeight;

	for (currObject=0;currObject<numObjects;currObject++) {

		if ( objects[currObject]->objectType != CYLINDER ) {

			currObjHeight = objects[currObject]->GetHeight();

			if ( currObjHeight < minHeight )
				minHeight = currObjHeight;
		}
	}

	return( minHeight );
}

char *AGENT::GetName(int objIndex) {

	return( objects[objIndex]->name );
}

int  AGENT::GetNumOfJoints(void) {

	return( numJoints );
}

int  AGENT::GetNumOfObjects(void) {

	return( numObjects );
}

int  AGENT::GetNumOfPossibleJoints(void) {

	return( numJoints + numOfPossibleExternalJoints );
}

int  AGENT::GetNumOfWeights(void) {

	//return( network->numWeights );

	return( 0 );
}

ME_OBJECT *AGENT::GetObject(int objIndex) {

	return( objects[objIndex] );
}

double AGENT::GetWidth(void) {

	double minWidth = 100.0;
	double maxWidth = -100.0;

	double pos[3];
	double rot[12];

	for (int i=0;i<numObjects;i++) {

		objects[i]->GetPosAndRot(pos,rot);

		if ( pos[0] < minWidth )
			minWidth = pos[0];

		if ( pos[0] > maxWidth )
			maxWidth = pos[0];
	}

	return( maxWidth - minWidth );
}

double AGENT::GetLength(void) {

	double minLength = 100.0;
	double maxLength = -100.0;

	double pos[3];
	double rot[12];

	for (int i=0;i<numObjects;i++) {

		objects[i]->GetPosAndRot(pos,rot);

		if ( pos[1] < minLength )
			minLength = pos[1];

		if ( pos[1] > maxLength )
			maxLength = pos[1];
	}

	return( maxLength - minLength );
}

void AGENT::HandleCollisions(int internalTimer) {

	/*
	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		
		network->UpdateSensorValue(1.0,objects[currObject]->touchSensorID );
	*/
}

void AGENT::IncreasePerturbation(double amt) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		
		objects[currObject]->IncreasePerturbation(amt);
}

void AGENT::InitNetwork(int hiddenN, int useReactiveController) {

	/*
	hiddenNodes = hiddenN;
	network->Init(hiddenNodes,useReactiveController);
	*/
}

void AGENT::Move(int numOfAgents, ME_OBJECT *lightSource, PLUME *plume, dWorldID world, dGeomID floor) {

	TakeSensorReadings(numOfAgents,lightSource,plume,floor);

	//if ( simParams->internalTimer == 400 )
	//	SwitchOnPressureSensors(world);

	ActuateJoints();

	//timeRemainingUntilNextMotorCommand--;
	
	//if ( timeRemainingUntilNextMotorCommand == 0 ) {
	//	timeRemainingUntilNextMotorCommand = 1;
		simParams->internalTimer++;
	//}

	//if ( sensorData )
	//	RecordSensorData();

	//if ( objectData )
	//	RecordObjectData();

	/*
	double com[3];
	GetCentreOfMass(com);

	if ( trajectory )
		SaveCentreOfMass(com);

	if ( simParams->trajectoryFile )
		RecordCentreOfMass(com);

	if ( numOfExternalJoints > 0 )
		BreakExternalConnectionsIfNecessary();

	UpdateSpringConnections();

	DetachJoints();

	IntroduceNoise();
	*/
}

void AGENT::PerturbSynapses(void) {

	//network->PerturbSynapses();
}

void AGENT::Push(double x, double y, double z) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->Push(x,y,z);
}

void AGENT::RemoveExtraneousObjects(dSpaceID space) {

	/*
	int j;
	ME_OBJECT *targettedObj;

	for (j=0;j<numJoints;j++) {
		targettedObj = joints[j]->GeomTransformPossible();
		if ( targettedObj != NULL )
			targettedObj->SwitchToGeomTransform(joints[j]->ComplementaryObj(targettedObj),space);
	}
	*/
}

void AGENT::Reset(void) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->Reset();
}

void AGENT::ResetTouchSensors(void) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)

		if ( objects[currObject]->touchSensorID > -1 )
			network->UpdateSensorValue(-1.0,objects[currObject]->touchSensorID);
}

void AGENT::SaveToRaytracerFile(ofstream *outFile) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->SaveToRaytracerFile(outFile);
}

void AGENT::SpecifyPhenotype(int genomeLength, double *genes) {

	//network->LabelSynapses(genomeLength,genes,numMorphParams);
	//network->ClearValues();

	int currObject, currJoint;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->SetEvolvableParams(numMorphParams,genomeLength,genes);
	
	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->SetParamsBasedOnOtherObjects();

	for (currJoint=0;currJoint<numJoints;currJoint++)
		joints[currJoint]->ResetPosition();

}

void AGENT::SwitchOnPressureSensors(dWorldID world) {

	for (int currJoint=0;currJoint<numJoints;currJoint++)
		joints[currJoint]->SwitchOnPressureSensor(world);
}

void AGENT::ToggleFootprintDrawing(void) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		objects[currObject]->ToggleFootprintDrawing();
}

void AGENT::ToggleTrajectoryDrawing(void) {

	if ( !trajectory )
		trajectory = new TRAJECTORY(simParams->evalPeriod);
	else {
		delete trajectory;
		trajectory = NULL;
	}
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void AGENT::ActuateJoints(void) {

	int currJoint;
	double motorValue;

	int currMotorizedJoint = 0;

	for (currJoint=0;currJoint<numJoints;currJoint++)
		if ( joints[currJoint]->motorNeuronID > -1 ) {

			//motorValue =
			//	network->GetMotorCommand( joints[currJoint]->motorNeuronID );

			motorValue = simParams->motorCommands[simParams->currentTest]->Get(simParams->internalTimer,currMotorizedJoint++);
			//motorValue = simParams->motorCommands[simParams->currentTest]->Get(0,currJoint);

			joints[currJoint]->Actuate( motorValue );
		}
}

void AGENT::BreakExternalConnectionsIfNecessary(void) {

	int currJoint = 0;
	int jointsToProcess;

	while ( currJoint<numOfExternalJoints ) {

		if ( externalJoints[currJoint]->IsBreaking() ) {
			
			externalJoints[currJoint]->Break();
			delete externalJoints[currJoint];
			externalJoints[currJoint] = NULL;

			for (jointsToProcess=currJoint+1;jointsToProcess<numOfExternalJoints;jointsToProcess++)
				externalJoints[jointsToProcess-1] = externalJoints[jointsToProcess];
			numOfExternalJoints--;
		}
		else
			currJoint++;
	}
}

int  AGENT::CountPossibleExternalJoints(void) {

	int possibleJoints = 0;
	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)

		if ( objects[currObject]->isAttacher )
			possibleJoints++;

	return( possibleJoints );
}

void AGENT::CreateBody(dWorldID world, dSpaceID space) {

	(*simParams->bodyFile) >> numObjects;
	(*simParams->bodyFile) >> numJoints;

	if ( !simParams->connectedObjects )
		simParams->connectedObjects = new MATRIX(numObjects,numObjects,0.0);
	else
		simParams->connectedObjects->SetAllTo(0.0);

	objects = new ME_OBJECT * [numObjects];
	joints  = new ME_JOINT  * [numJoints];

	for (int o=0;o<numObjects;o++)
		ReadInObject(o);

	for (int j=0;j<numJoints;j++)
		ReadInJoint(j);

	for (j=0;j<numJoints;j++)
		joints[j]->FindPressureSensor();

	CreateODEStuff(world,space);
}

void AGENT::CreateBrain(void) {

	network->Init(simParams->brainFile);
}

void AGENT::DetachJoints(void) {

	int currJoint = 0;
	int jointsToProcess;

	while ( currJoint<numOfExternalJoints ) {

		if ( externalJoints[currJoint]->BreakageDesired(network) ) {
			
			externalJoints[currJoint]->Break();
			delete externalJoints[currJoint];
			externalJoints[currJoint] = NULL;

			for (jointsToProcess=currJoint+1;jointsToProcess<numOfExternalJoints;jointsToProcess++)
				externalJoints[jointsToProcess-1] = externalJoints[jointsToProcess];
			numOfExternalJoints--;
		}
		else
			currJoint++;
	}
}

void AGENT::DetermineMaxLightSensorReading(ME_OBJECT *lightSource) {

	int currObject;
	double maxDistance = -1.0;
	double tempDist;

	for (currObject=0;currObject<numObjects;currObject++) {
	
		tempDist = objects[currObject]->GetDistance(lightSource);

		if ( tempDist > maxDistance )
			maxDistance = tempDist;
	}

	maxLightSensorReading = maxDistance;
}

void AGENT::DrawJointNormals(void) {

	int currJoint;

	for (currJoint=0;currJoint<numJoints;currJoint++)
		joints[currJoint]->DrawNormals();
}

void AGENT::DrawNeuralNetwork(void) {

	/*
	if ( simParams->drawNetwork == 1 ) {
		double com[3];
		GetCentreOfMass(com);
		network->DrawNeurons(com);
	}
	else {

		double maxHeight = GetMaxHeight();

		int currObject;
		for (currObject=0;currObject<numObjects;currObject++)
			objects[currObject]->DrawSensorNeurons(maxHeight,3.0);

		int currJoint;
		for (currJoint=0;currJoint<numJoints;currJoint++) {
			joints[currJoint]->DrawSensorNeurons(maxHeight,2.0);
			joints[currJoint]->DrawMotorNeurons(maxHeight,1.0);
		}
	}
	*/
}

void AGENT::FindRelatedJoints(void) {

	int currJoint;
	ME_JOINT *relatedJoint;

	for (currJoint=0;currJoint<numJoints;currJoint++) {

		if ( joints[currJoint]->RelatedToOtherJoint() ) {
			relatedJoint = GetJoint(joints[currJoint]->GetRelatedJointName());
			joints[currJoint]->SetRelatedJoint(relatedJoint);
		}
	}
}

void AGENT::IntroduceNoise(void) {

	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)

		objects[currObject]->IntroduceNoise();
}

void AGENT::ReadInDataParameters(void) {

	char readIn[50];

	(*simParams->bodyFile) >> readIn;
	
	if ( strcmp(readIn,"(") != 0 ) {
		printf("Read in error: expecting opening bracket.\n");
		char ch = getchar();
		exit(0);
	}

	(*simParams->bodyFile) >> readIn;

	while ( strcmp(readIn,")") != 0 ) {

		if ( strcmp(readIn,"-evaluationPeriod") == 0 ) {

			(*simParams->bodyFile) >> readIn;
			simParams->evalPeriod = atoi(readIn);
		}
		else
		if ( strcmp(readIn,"-testForExplosions") == 0 ) {
			simParams->testForExplosions = true;
		}

		/*
		else if ( strcmp(readIn,"-recordSensors") == 0 ) {

			//sensorData = new MATRIX(simParams->evalPeriod,network->numInput,0.0);
			//sensorData = new MATRIX(simParams->evalPeriod,numObjects*4,0.0);
			sensorData = new MATRIX(simParams->evalPeriod,numObjects*1,0.0);
		}
		else if ( strcmp(readIn,"-recordObjectPosition") == 0 ) {

			(*simParams->bodyFile) >> readIn;
			targetObject = objects[0]->FindObject(objects,numObjects,readIn);
			objectData = new MATRIX(simParams->evalPeriod,3,0.0);
		}
		*/
		(*simParams->bodyFile) >> readIn;
	}
}

void AGENT::ReadInJoint(int currJoint) {

	joints[currJoint] = new ME_JOINT(ID,objects,numObjects,simParams->bodyFile,currJoint,network);

	//FindRelatedJoints();
}

void AGENT::ReadInObject(int currObject) {

	objects[currObject] = new ME_OBJECT(this,numObjects,simParams->bodyFile,currObject,network,objects);

	/*
	for (currObject=0;currObject<numObjects;currObject++) {

		numMorphParams = numMorphParams + objects[currObject]->CountMorphParams(numMorphParams);
		objects[currObject]->FindOtherObjects(numObjects,objects);
	}
	*/
}

void AGENT::RecordObjectData(void) {

	/*
	double pos[3], rot[12];

	targetObject->GetPosAndRot(pos,rot);

	objectData->Set(simParams->internalTimer,0,pos[0]);
	objectData->Set(simParams->internalTimer,1,pos[1]);
	objectData->Set(simParams->internalTimer,2,pos[2]);
	*/
}

void AGENT::RecordSensorData(void) {

	/*
	for (int s=0;s<network->numInput;s++) {

		sensorData->Set(simParams->internalTimer,s,network->GetSensorValue(s));
	}
	*/

	/*
	double pos[3], rot[12];
	double pos2[3], rot2[12];
	double pos3[3], rot3[12];

	double distFromOrigin;

	int k = 0;
	int otherObject, otherObject2;

	for (int o=0;o<numObjects;o++) {

		objects[o]->GetPosAndRot(pos,rot);

		distFromOrigin = sqrt( pow(pos[0]-1.0,2.0) + pow(pos[1]-1.0,2.0) + pow(pos[2]-0.0,2.0) );
		sensorData->Set(simParams->internalTimer,k,distFromOrigin);

		/*
		distFromOrigin = sqrt( pow(pos[0]+1.0,2.0) + pow(pos[1]-1.0,2.0) + pow(pos[2]-0.0,2.0) );
		sensorData->Set(simParams->internalTimer,k+1,distFromOrigin);

		distFromOrigin = sqrt( pow(pos[0]-1.0,2.0) + pow(pos[1]+1.0,2.0) + pow(pos[2]-0.0,2.0) );
		sensorData->Set(simParams->internalTimer,k+2,distFromOrigin);

		distFromOrigin = sqrt( pow(pos[0]+1.0,2.0) + pow(pos[1]+1.0,2.0) + pow(pos[2]-0.0,2.0) );
		sensorData->Set(simParams->internalTimer,k+3,distFromOrigin);
		*/
		//k = k + 1;

		/*
		switch ( o ) {
		case 0:
			otherObject = 1;
			otherObject2 = 2;
			break;
		case 1:
			otherObject = 0;
			otherObject2 = 2;
			break;
		case 2:
			otherObject = 0;
			otherObject2 = 1;
			break;
		case 4:
			otherObject = 1;
			otherObject2 = 0;
			break;
		case 5:
			otherObject = 1;
			otherObject2 = 0;
			break;
		}

		objects[o]->GetPosAndRot(pos,rot);
		objects[otherObject]->GetPosAndRot(pos2,rot2);
		objects[otherObject2]->GetPosAndRot(pos3,rot3);

		distFromOrigin = sqrt( pow(pos[0]-pos2[0],2.0) + pow(pos[1]-pos2[1],2.0) + pow(pos[2]-pos2[2],2.0) );
		sensorData->Set(simParams->internalTimer,k,distFromOrigin);

		distFromOrigin = sqrt( pow(pos[0]-pos3[0],2.0) + pow(pos[1]-pos3[1],2.0) + pow(pos[2]-pos3[2],2.0) );
		sensorData->Set(simParams->internalTimer,k+1,distFromOrigin);

		k = k + 1;
		*/
	//}
}

void AGENT::RecordCentreOfMass(double *com) {

	(*simParams->trajectoryFile) << com[0] << ";";
	(*simParams->trajectoryFile) << com[1] << ";";
	(*simParams->trajectoryFile) << com[2] << "\n";
}

void AGENT::SaveCentreOfMass(double *com) {

	trajectory->x[simParams->internalTimer-1] = com[0];
	trajectory->y[simParams->internalTimer-1] = com[1];
	trajectory->z[simParams->internalTimer-1] = com[2];
}

void AGENT::TakeAgentSensorReadings(int numOfAgents) {

	/*
	int currObject;
	double sensorValue;

	for (currObject=0;currObject<numObjects;currObject++)
		if ( objects[currObject]->agentSensorID > -1 ) {
		
			if ( numOfAgents > 1 )
				sensorValue = simParams->Scale(double(ID),0.0,double(numOfAgents)-1.0,-1.0,1.0);
			else
				sensorValue = 0.0;

			network->UpdateSensorValue( sensorValue,
												 objects[currObject]->agentSensorID );
		}
	*/
}

void AGENT::TakeAngleSensorReadings(void) {

	int currJoint;
	int currHingeJoint = 0;

	for (currJoint=0;currJoint<numJoints;currJoint++) 
		if ( (joints[currJoint]->sensorNeuronID > -1) &&
			 (joints[currJoint]->jointType==JOINT_HINGE) ) {

			double normalizedAngle = joints[currJoint]->GetAngle()*180.0/3.14159;

			//double normalizedAngle = simParams->Scale(joints[currJoint]->GetAngle(),
			//										  joints[currJoint]->lowerLimit,
			//										  joints[currJoint]->upperLimit,1.0,255.0);

			sensorValues->Set(simParams->internalTimer,currHingeJoint,normalizedAngle);

			currHingeJoint++;

			//network->UpdateSensorValue( joints[currJoint]->GetAngle(),
			//									 joints[currJoint]->sensorNeuronID );
		}
}

void AGENT::TakeAttachmentSensorReadings(void) {

	/*
	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)
		if ( objects[currObject]->attachmentSensorID > -1 )
			if ( objects[currObject]->isAttached )
				network->UpdateSensorValue( 1.0,
					objects[currObject]->attachmentSensorID );
			else
				network->UpdateSensorValue( -1.0,
					objects[currObject]->attachmentSensorID );
	*/
}

void AGENT::TakeChemoSensorReadings(PLUME *plume) {

	/*
	int currObject;
	double chemConc;

	for (currObject=0;currObject<numObjects;currObject++)

		if ( objects[currObject]->chemoSensorID > -1 ) {

			const dReal *currPos;
			currPos = dBodyGetPosition(objects[currObject]->body);
			
			//if ( objects[currObject]->CloseEnoughToPlume(currPos[2],plume->height) ) {
			if ( true ) {
				//if ( currObject == 22 )
				//	chemConc = 0.5;
				//else
					chemConc = plume->GetConcAt(currPos[1],currPos[0]);
			}
			else
				chemConc = 0.0;

			chemConc = (chemConc*2.0) - 1.0;

			network->UpdateSensorValue( chemConc,objects[currObject]->chemoSensorID );
		}
	*/
}

void AGENT::TakeDistanceSensorReading(dGeomID floor) {

	dContact contact;

	if ( dCollide(objects[0]->distanceSensor,floor,0,&contact.geom,sizeof(dContactGeom)) ) {

		double dist = contact.geom.depth - (objects[0]->size[2]/2.0);

		if ( dist < 0 )
			dist = 0;

		sensorValues->Set(simParams->internalTimer,14,dist);

		/*
		double origin[3], end[3];
		double rot[12];

		printf("%5.5f\n",contact.geom.depth);

		objects[0]->GetPosAndRot(origin,rot);

		end[0] = contact.geom.pos[0];
		end[1] = contact.geom.pos[1];
		end[2] = contact.geom.pos[2];

		dsDrawLineD(origin,end);
		*/
	}
}

void AGENT::TakeForwardSensorReading(void) {

	double obj0Forward = objects[0]->GetForward();
//	double obj0Left = objects[0]->GetLeft();
//	double objOHeight = objects[0]->GetHeight();

//	printf("[Left: %3.3f]\t[Up: %3.3f]\t[Forward: %3.3f]\n",obj0Left,objOHeight,obj0Forward);

	sensorValues->Set(simParams->internalTimer,16,obj0Forward);
}

void AGENT::TakeLeftSensorReading(void) {

	double obj0Left = objects[0]->GetLeft();

	//printf("%3.3f\n",lowestHeight);

	sensorValues->Set(simParams->internalTimer,15,obj0Left);
}

void AGENT::TakeLightSensorReadings(ME_OBJECT *lightSource) {

	/*
	int currObject;
	double lightDist;

	for (currObject=0;currObject<numObjects;currObject++)
		if ( objects[currObject]->lightSensorID > -1 ) {

			lightDist = objects[currObject]->GetDistance(lightSource);

			if ( lightDist > maxLightSensorReading )
				lightDist = -1;
			else
				lightDist = ((-2 * lightDist) / maxLightSensorReading) + 1;

			network->UpdateSensorValue( lightDist,
												 objects[currObject]->lightSensorID );
		}
	*/

}

void AGENT::TakeLowestHeightSensorReading(void) {

	double lowestHeight = GetMinHeight();

	//printf("%3.3f\n",lowestHeight);

	sensorValues->Set(simParams->internalTimer,15,lowestHeight);
}

void AGENT::TakeObj0HeightSensorReading(void) {

	double obj0Height = objects[0]->GetHeight();

	sensorValues->Set(simParams->internalTimer,17,obj0Height);
}

void AGENT::TakeOrientationSensorReadings(void) {

	objects[0]->SetOrientationSensors();
}

void AGENT::TakePressureSensorReadings(dGeomID floor) {

	/*
	int currJoint;

	for (currJoint=0;currJoint<numJoints;currJoint++)

		joints[currJoint]->RecordPressure();

	*/

	ME_OBJECT *currObject;
	int currFoot = 0;
	dContact contact;
	double totalDepth = 0.0;
	double totalDepthNormalized;
	double indivWeight;

	for (int currObj=0;currObj<numObjects;currObj++) {

		currObject = objects[currObj];
		
		if ( (currObject->objectType == SPHERE) || (currObject->ID==0) ) {

			if (dCollide(currObject->geom,floor,0,&contact.geom,sizeof(dContactGeom))) {

				totalDepth = totalDepth + contact.geom.depth;
			}
		}
	}

	totalDepthNormalized = totalDepth/totalVolume;

	currObject = NULL;

	/*
	for (currObj=0;currObj<numObjects;currObj++) {

		currObject = objects[currObj];
//		if ( currObject->objectType == SPHERE ) {

		if ( currObject->weight==0.023 ) {

			if (dCollide(currObject->geom,floor,0,&contact.geom,sizeof(dContactGeom))) {

				indivWeight = ((contact.geom.depth/totalVolume)*totalMass) / totalDepthNormalized;

				sensorValues->Set(simParams->internalTimer,8+currFoot,indivWeight);
				currFoot++;
			}
		}
	}
	*/
}

void AGENT::TakeSensorReadings(int numOfAgents, ME_OBJECT *lightSource, PLUME *plume, dGeomID floor) {

	TakeAngleSensorReadings();

	TakePressureSensorReadings(floor);

	TakeOrientationSensorReadings();

	TakeDistanceSensorReading(floor);

	//TakeVelocitySensorReadings();

	TakeLeftSensorReading();

	TakeForwardSensorReading();

	TakeObj0HeightSensorReading();

	//TakeAgentSensorReadings(numOfAgents);

	//TakeTouchSensorReadings();

	//TakeLowestHeightSensorReading();

	//TakeLightSensorReadings(lightSource);
	
	//if ( plume )
	//	TakeChemoSensorReadings(plume);
}

void AGENT::TakeTouchSensorReadings(void) {

	/*
	int currObject;

	for (currObject=0;currObject<numObjects;currObject++)

		if ( objects[currObject]->touchSensorID > -1 )
			if ( objects[currObject]->isAttached )
				network->UpdateSensorValue( 1.0, objects[currObject]->touchSensorID );
	*/
}

void AGENT::TakeVelocitySensorReadings(void) {

	const dReal *vel =  dBodyGetLinearVel(objects[0]->body);

	double linearSpeed = sqrt( vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);

	vel = dBodyGetAngularVel(objects[0]->body);

	double angularSpeed = sqrt( vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);

	sensorValues->Set(simParams->internalTimer,15,linearSpeed+angularSpeed);
}

void AGENT::UpdateSpringConnections(void) {

	int currJoint;

	for (currJoint=0;currJoint<numJoints;currJoint++)
		if ( joints[currJoint]->jointType == JOINT_SPRING )
			joints[currJoint]->UpdateSpring();
}

#endif