/* ---------------------------------------------------
   FILE:     environment.cpp
	AUTHOR:   Josh Bongard
	DATE:     February 14, 2001
	FUNCTION: This class contains all information for
				 a set of objects in a MathEngine world.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _ENVIRONMENT_CPP
#define _ENVIRONMENT_CPP

#include "environment.h"
#include "simParams.h"

extern double			white[4];
//extern MeMatrix4		groundTransform;
extern double			GROUND_LENGTH;
extern int				DEFAULT_EVALUATION_PERIOD;
extern double			LIGHT_SOURCE_SIZE;
extern double			LIGHT_SOURCE_DISTANCE;
extern double			PI;
extern double			DEF_CAMERA_DISTANCE;
extern SIM_PARAMS		*simParams;
extern double			AVG_NUM_OF_MUTS;
extern int				OUTPUT_RAYTRACER_FILE;
extern int				OUTPUT_FOOTPRINTS;
extern int				CREATE_PLUME;
extern double			DEFAULT_SOFTNESS;
extern double			DEFAULT_RESTITUTION;
extern int				DEBRIS_NUM_OF_DEBRIS_FIELDS;
extern int				DEBRIS_NONE;
extern int				DEBRIS_EMBEDDED_RUBBLE;
extern int				DEBRIS_LOOSE_RUBBLE;
extern int				NOISE_TRIALS;
extern int				DEBRIS_LOOSE_RUBBLE;
extern int				DEBRIS_RAMP;
extern int				DEBRIS_UNEVEN_FLOOR;

extern char				BODY_FILENAME[50];
extern char				BRAIN_FILENAME[50];

ENVIRONMENT::ENVIRONMENT(void) {

	numOfAgents = 1;
	objectNotToDrop = NULL;

	/*
	objectToMove = NULL;
	addObjectToMove = NULL;
	add2ObjectToMove = NULL;
	multObjectToMove = NULL;
	mult2ObjectToMove = NULL;
	lightSource = NULL;
	numObjects = 0;
	numJoints = 0;

	softness = DEFAULT_SOFTNESS;
	restitution = DEFAULT_RESTITUTION;

	evalPeriod = DEFAULT_EVALUATION_PERIOD;
	numEvaluations = 1;
	populationSize = -1;
	generations = -1;
	mutationRate = AVG_NUM_OF_MUTS;
	hiddenNodes = -1;
	testForExplosions = false;
	addTime = false;
	multTime = false;
	plume = NULL;
	useReactiveController = false;

	numMorphParams = 0;

	cameraDistance = DEF_CAMERA_DISTANCE;

	ReadFromFile();

	AGENT *tempAgent = new AGENT(0,envFile,numObjects,numJoints);

	DetermineFitnessCriteria(tempAgent);

	agents = new AGENT * [numOfAgents];
	agents[0] = tempAgent;
	tempAgent = NULL;
	*/

	CreateODEStuff();

	/*
	if ( simParams->debrisType != DEBRIS_NONE ) {
		numOfDebrisFields = DEBRIS_NUM_OF_DEBRIS_FIELDS;
		CreateDebris();
	}

	agents[0]->InitNetwork(hiddenNodes,useReactiveController);

	int currAgent;
	for (currAgent=1;currAgent<numOfAgents;currAgent++)
		agents[currAgent] = new AGENT(currAgent,agents[0]);

	if ( agents[0]->ContainsLightSensor() )
		CreateLightSource(0);

	if ( lightSource != NULL )
		RepositionLightSource(0);

	genomeLength = agents[0]->GetNumOfWeights()+agents[0]->numMorphParams;

	ga->InitGenomes(genomeLength,populationSize,
		generations,mutationRate,numEvaluations,agents[0]->numMorphParams);

	for (currAgent=0;currAgent<numOfAgents;currAgent++)
		agents[currAgent]->SpecifyPhenotype(genomeLength,ga->currGenome->genes);

	for (currAgent=0;currAgent<numOfAgents;currAgent++) {
		agents[currAgent]->CreateObjects(genomeLength, ga->currGenome->genes, lightSource, world, space);
		agents[currAgent]->RemoveExtraneousObjects(space);
		agents[currAgent]->CreateJoints(world);
	}
	*/
}

ENVIRONMENT::~ENVIRONMENT(void) {

	if ( OUTPUT_RAYTRACER_FILE ) {
		raytracerFile->close();
		delete raytracerFile;
	}

	int currAgent;

	for (currAgent=0;currAgent<numOfAgents;currAgent++) {
		delete agents[currAgent];
		agents[currAgent] = NULL;
	}

	delete[] agents;
	agents = NULL;

	envFile->close();

	if ( lightSource != NULL )
		DestroyLightSource();

	if ( plume ) {
		delete plume;
		plume = NULL;
	}

	if ( simParams->debrisType != DEBRIS_NONE )
		DestroyDebris();
	
	DestroyODEStuff();
}

int ENVIRONMENT::AgentAvailable(void) {

	int available = simParams->FileExists(BODY_FILENAME) && 
		            simParams->FileExists(BRAIN_FILENAME);

	return( available );
}

void ENVIRONMENT::ChangePlumePtSource(int evalIndex) {

	plume->ChangePtSource(evalIndex);
}

void ENVIRONMENT::CreateAgent(void) {

	//if ( simParams->rendering )
		printf("Creating agent %d.\n",simParams->agentsToEvaluate);

	AGENT *tempAgent = new AGENT(0,world,space);

	//DetermineFitnessCriteria(tempAgent);

	agents = new AGENT * [numOfAgents];
	agents[0] = tempAgent;
	tempAgent = NULL;

	simParams->internalTimer = 0;
}

void ENVIRONMENT::CreatePlume(void) {

	if ( !plume )
		plume = new PLUME(agents[0]->GetMaxHeight());
}

void ENVIRONMENT::DestroyAgent(void) {

	int currAgent;

	for (currAgent=0;currAgent<numOfAgents;currAgent++) {
		agents[currAgent]->DestroyODEStuff(space);
		delete agents[currAgent];
		agents[currAgent] = NULL;
	}

	delete[] agents;
	agents = NULL;
}

void ENVIRONMENT::DestroyPlume(void) {

	if ( plume ) {
		delete plume;
		plume = NULL;
	}
}

void ENVIRONMENT::Draw(void) {

	for (int d=0;d<numOfDebrisFields;d++)
		debrisFields[d]->Draw();

	for (int a=0;a<numOfAgents;a++)
		agents[a]->Draw();

	//if ( plume )
	//	plume->Draw();
}

int ENVIRONMENT::EvaluationFinished(void) {

	//if ( (simParams->internalTimer == simParams->evalPeriod) || AgentExploding() || AgentTipping() )
	if ( simParams->internalTimer == simParams->evalPeriod )

		return( true );
	else

		if ( objectNotToDrop != NULL )
			if ( objectNotToDrop->IsBelow(verticalThreshold) )
				return( true );
			else
				return( false );
		else
			return( false );
}

double ENVIRONMENT::GetFitness(void) {

	if ( AgentExploding() )
		return( 0.0 );
	else
		if ( plume )
			return( GetDistToPtSource() );
		else
		if ( lightSource == NULL )
			if ( addTime )
				return( GetForwardDist() + double(simParams->internalTimer) );
			else
				if ( multTime )
					return( GetForwardDist() * double(simParams->internalTimer) );
				else
					return( GetForwardDist() );
		else
			return( GetDistToLight() );
}

void ENVIRONMENT::GetObjectPosition(char *objName, double **pos) {

	(*pos)[0] = 0.0;
	(*pos)[1] = 0.0;
	(*pos)[2] = 0.0;
}

void ENVIRONMENT::HandleCollisions(void) {

	int currAgent;

	if ( CollisionBetweenAgents() )
		AttemptAttachment();

	for (currAgent=0; currAgent<numOfAgents;currAgent++)
		agents[currAgent]->HandleCollisions(simParams->internalTimer);
}

void ENVIRONMENT::IncreasePerturbation(double amt) {

	for (int a=0;a<numOfAgents;a++)
		agents[a]->IncreasePerturbation(amt);
}

void ENVIRONMENT::MoveAgents(void) {
	
	for (int a=0;a<numOfAgents;a++)
		agents[a]->Move(numOfAgents,lightSource,plume,world,floor);

	//simParams->internalTimer++;
}

void ENVIRONMENT::MoveDebris(void) {

	if ( simParams->debrisType == DEBRIS_UNEVEN_FLOOR ) {
		
		double com[3];
		agents[0]->GetCentreOfMass(com);

		debrisFields[0]->Move(world,space,com,agents[0]->GetWidth(),agents[0]->GetLength());
	}
}

void ENVIRONMENT::PerturbSynapses(void) {

	for (int a=0;a<numOfAgents;a++)
		agents[a]->PerturbSynapses();
}

void ENVIRONMENT::Push(double x, double y, double z) {

	for (int a=0;a<numOfAgents;a++)
		agents[a]->Push(x,y,z);
}

void ENVIRONMENT::RefreshTouchPoints(void) {

	double diff;

	for (int i=1;i<15;i++) {

		if ( (simParams->previousTouchPoints->Get(i,0)!=0.0) &&
			 (simParams->previousTouchPoints->Get(i,1)!=0.0) &&
			 (simParams->touchPoints->Get(i,0)!=0.0) &&
			 (simParams->touchPoints->Get(i,1)!=0.0) ) {

			diff = sqrt(pow((simParams->previousTouchPoints->Get(i,0) -
						     simParams->touchPoints->Get(i,0)),2.0) + 
						pow((simParams->previousTouchPoints->Get(i,1) - 
						     simParams->touchPoints->Get(i,1)),2.0) );

			simParams->totalChanges->Set(simParams->internalTimer,i,diff);
		}
	}

	simParams->previousTouchPoints->CopyInto(simParams->touchPoints);

	simParams->touchPoints->Zero();
}

void ENVIRONMENT::Reset(void) {

	simParams->internalTimer = 0;

	simParams->numSensors = 0;
	
	simParams->currentTest++;

	simParams->agentExploding = false;

	simParams->testForExplosions = false;

	if ( simParams->currentTest == simParams->numTests )
		simParams->currentTest = 0;

	/*
	int currAgent;

	for (currAgent=0;currAgent<numOfAgents;currAgent++) {
		agents[currAgent]->DestroyODEStuff(space);
		agents[currAgent]->Reset();
	}

	if ( simParams->debrisType != DEBRIS_NONE )
		DestroyDebris();

	DestroyODEStuff();

	if ( plume )
		plume->Reset();

	CreateODEStuff();
	*/

	/*
	for (currAgent=0;currAgent<numOfAgents;currAgent++)
		agents[currAgent]->SpecifyPhenotype(genomeLength,ga->currGenome->genes);
	*/

	/*
	for (currAgent=0;currAgent<numOfAgents;currAgent++)
		agents[currAgent]->CreateODEStuff(world,space);

	if ( simParams->debrisType != DEBRIS_NONE )
		CreateDebris();
	*/
}

void ENVIRONMENT::ResetTouchSensors(void) {

	int currAgent;
	for (currAgent=0;currAgent<numOfAgents;currAgent++)
		agents[currAgent]->ResetTouchSensors();
}

void ENVIRONMENT::SaveToRaytracerFile(void) {

	if ( simParams->internalTimer < evalPeriod ) {
		OpenRaytracerFile();
		agents[0]->SaveToRaytracerFile(raytracerFile);
		CloseRaytracerFile();
	}
}

void ENVIRONMENT::ToggleFootprintDrawing(void) {

	int currAgent;
	for (currAgent=0;currAgent<numOfAgents;currAgent++)
		agents[currAgent]->ToggleFootprintDrawing();
}

void ENVIRONMENT::ToggleTrajectoryDrawing(void) {

	int currAgent;
	for (currAgent=0;currAgent<numOfAgents;currAgent++)
		agents[currAgent]->ToggleTrajectoryDrawing();
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

int  ENVIRONMENT::AgentExploding(void) {

	if ( !simParams->testForExplosions )
		return( false );
	else
		return( agents[0]->Exploding() );
}

int ENVIRONMENT::AgentTipping(void) {

	double leftRightTip				= fabs(agents[0]->sensorValues->Get(simParams->internalTimer-1,12));
	double leftRightOvertipping		= leftRightTip > 60.0;

	double forwardBackTip			= fabs(agents[0]->sensorValues->Get(simParams->internalTimer-1,13));
	double forwardBackOvertipping	= forwardBackTip > 60.0;

	if ( leftRightOvertipping || forwardBackOvertipping )
		agents[0]->sensorValues->Set(simParams->internalTimer-1,18,simParams->evalPeriod-simParams->internalTimer);

	return( leftRightOvertipping || forwardBackOvertipping );
}

void ENVIRONMENT::AttemptAttachment(void) {

	/*
	ME_OBJECT *ptr1, *ptr2;
	double colour[4];
	
	ptr1 = (ME_OBJECT *)McdModelGetUserData(result.pair->model1);
	ptr2 = (ME_OBJECT *)McdModelGetUserData(result.pair->model2);

	if ( (!ptr1->containingAgent->AttachedTo(ptr2->containingAgent)) &&
		  (ptr1->CloseEnough(ptr2)) &&
		  (!ptr1->BreakageDesired(ptr1->containingAgent->network)) &&
		  (!ptr2->BreakageDesired(ptr2->containingAgent->network)) ) {

		ptr1->containingAgent->AttachTo(ptr2->containingAgent,world,result);
	}
	*/
}

void ENVIRONMENT::ChangeDebrisField(void) {

	/*
	if ( simParams->debrisType == DEBRIS_NONE ) {
		simParams->debrisType = DEBRIS_LOOSE_RUBBLE;
		numOfDebrisFields = 1;
	}
	else if ( simParams->debrisType == DEBRIS_LOOSE_RUBBLE ) {
		DestroyDebris();
		simParams->debrisType = DEBRIS_EMBEDDED_RUBBLE;
		numOfDebrisFields = 1;
	}
	else if ( simParams->debrisType == DEBRIS_EMBEDDED_RUBBLE ) {
		DestroyDebris();
		simParams->debrisType = DEBRIS_RAMP;
		numOfDebrisFields = 1;
	}
	else if ( simParams->debrisType == DEBRIS_RAMP ) {
		DestroyDebris();
		simParams->debrisType = DEBRIS_UNEVEN_FLOOR;
		numOfDebrisFields = 1;
	}
	else {
		DestroyDebris();
		simParams->debrisType = DEBRIS_NONE;
		numOfDebrisFields = 0;
	}
	*/

	if ( simParams->debrisType == DEBRIS_NONE ) {
		simParams->debrisType = DEBRIS_EMBEDDED_RUBBLE;
		numOfDebrisFields = 1;
	}
	else {
		DestroyDebris();
		simParams->debrisType = DEBRIS_NONE;
		numOfDebrisFields = 0;
	}

	if ( numOfDebrisFields )
		CreateDebris();
}

void ENVIRONMENT::CloseRaytracerFile(void) {

	raytracerFile->close();
}

int  ENVIRONMENT::CollisionBetweenAgents(void) {

	/*
	ME_OBJECT *ptr1, *ptr2;
	
	ptr1 = (ME_OBJECT *)McdModelGetUserData(result.pair->model1);
	ptr2 = (ME_OBJECT *)McdModelGetUserData(result.pair->model2);

	if ( (ptr1==NULL) || (ptr2==NULL) || (!ptr1->isAttacher) || (!ptr2->isAttacher) )
		return( false );
	else
		return( ptr1->containingAgent->ID != ptr2->containingAgent->ID );
	*/
	return( false );
}

int  ENVIRONMENT::CountTotalJoints(void) {

	return(agents[0]->GetNumOfJoints()*numOfAgents);
}

int  ENVIRONMENT::CountTotalObjects(void) {

	return(agents[0]->GetNumOfObjects()*numOfAgents);
}

int  ENVIRONMENT::CountTotalPossibleJoints(void) {

	return(agents[0]->GetNumOfPossibleJoints()*numOfAgents);
}

void ENVIRONMENT::CreateDebris(void) {

	debrisFields = new DEBRIS * [numOfDebrisFields];

	debrisFields[0] = new DEBRIS(simParams->debrisType, agents[0]->GetWidth(),agents[0]->GetLength());
	debrisFields[0]->CreateODEStuff(world,space);
}

void ENVIRONMENT::CreateFloor(void) {

	floor = dCreatePlane(space,0,0,1,0);
}

void ENVIRONMENT::CreateLightSource(int currEval) {

	lightSource = new ME_OBJECT(world,space);

	RepositionLightSource(currEval);
}

void ENVIRONMENT::CreateODEStuff(void) {

	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(world,0,0,-9.8);
	//dWorldSetGravity(world,0,0,0.0);

	dWorldSetCFM(world,1e-5);

	CreateFloor();
}

void ENVIRONMENT::DestroyDebris(void) {

	for (int i=0;i<numOfDebrisFields;i++) {
		debrisFields[i]->DestroyODEStuff(space);
		delete debrisFields[i];
		debrisFields[i] = NULL;
	}

	delete[] debrisFields;
	debrisFields = NULL;
}

void ENVIRONMENT::DestroyFloor(void) {

	dSpaceRemove(space,floor);
	dGeomDestroy(floor);
}

void ENVIRONMENT::DestroyLightSource(void) {

	/*
	lightSource->DestroyMathEngineStuff();
	delete lightSource;
	lightSource = NULL;
	*/
}

void ENVIRONMENT::DestroyODEStuff(void) {

	DestroyFloor();

	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);
}

void ENVIRONMENT::DetermineFitnessCriteria(AGENT *currAgent) {

	char readIn[50];

	(*envFile) >> readIn;

	if ( strcmp(readIn,"(") != 0 )
		terminationCriteria = false;

	else {

		(*envFile) >> readIn;

		while ( strcmp(readIn,")") != 0 ) {

			if ( strcmp(readIn,"-copies") == 0 ) {
				(*envFile) >> readIn;
				numOfAgents = atoi(readIn);
			}
			else
			if ( strcmp(readIn,"-displacementDistance") == 0 ) {
				(*envFile) >> readIn;
				simParams->displacementDistance = atof(readIn);
			}
			else
			if ( strcmp(readIn,"-evaluations") == 0 ) {
				(*envFile) >> readIn;
				numEvaluations = atoi(readIn);
			}
			else
			if ( strcmp(readIn,"-createPlume") == 0 ) {
				if ( !plume )
					plume = new PLUME(currAgent->GetMaxHeight());
			}
			else
			if ( strcmp(readIn,"-restitution") == 0 ) {
				(*envFile) >> readIn;
				restitution = atof(readIn);
			}
			else
			if ( strcmp(readIn,"-softness") == 0 ) {
				(*envFile) >> readIn;
				softness = atof(readIn);
			}
			else
			if ( strcmp(readIn,"-evaluationPeriod") == 0 ) {
				(*envFile) >> readIn;
				simParams->evalPeriod = atoi(readIn);
			}
			else
			if ( strcmp(readIn,"-populationSize") == 0 ) {
				(*envFile) >> readIn;
				populationSize = atoi(readIn);
			}
			else
			if ( strcmp(readIn,"-generations") == 0 ) {
				(*envFile) >> readIn;
				generations = atoi(readIn);
			}
			else
			if ( strcmp(readIn,"-mutationRate") == 0 ) {
				(*envFile) >> readIn;
				mutationRate = atof(readIn);
			}
			else
			if ( strcmp(readIn,"-objectToMove") == 0 ) {
				(*envFile) >> readIn;
				objectToMove = FindObject(readIn,currAgent);
				objectToMove->SetColour(1.0,0.0,0.0);
			}
			else
			if ( strcmp(readIn,"-addForwardDist") == 0 ) {
				(*envFile) >> readIn;
				if ( addObjectToMove == NULL )
					addObjectToMove = FindObject(readIn,currAgent);
				else
					add2ObjectToMove = FindObject(readIn,currAgent);
			}
			else
			if ( strcmp(readIn,"-multForwardDist") == 0 ) {
				(*envFile) >> readIn;
				if ( multObjectToMove == NULL )
					multObjectToMove = FindObject(readIn,currAgent);
				else
					mult2ObjectToMove = FindObject(readIn,currAgent);
			}
			else
			if ( strcmp(readIn,"-addTime") == 0 ) {
				addTime = true;
			}
			else
			if ( strcmp(readIn,"-multTime") == 0 ) {
				multTime = true;
			}
			else
			if ( strcmp(readIn,"-terminateIf") == 0 ) {
				(*envFile) >> readIn;
				objectNotToDrop = FindObject(readIn,currAgent);
				(*envFile) >> readIn;
				verticalThreshold = atof(readIn);
			}
			else
			if ( strcmp(readIn,"-cameraDistance") == 0 ) {
				(*envFile) >> readIn;
				cameraDistance = atof(readIn);
			}
			else
			if ( strcmp(readIn,"-hiddenNodes") == 0 ) {
				(*envFile) >> readIn;
				hiddenNodes = atoi(readIn);
			}
			else
			if ( strcmp(readIn,"-testForExplosions") == 0 ) {
				simParams->testForExplosions = true;
			}
			else
			if ( strcmp(readIn,"-recordHiddenLayerValues") == 0 ) {
				char fileName[30];
				strcpy(fileName,"data/hiddenLayers.dat");
				simParams->hiddenLayerFile = new ofstream(fileName);
			}
			else
			if ( strcmp(readIn,"-useReactiveController") == 0 ) {
				useReactiveController = true;
			}
			else {
				printf("ERROR: command %s not recognized in fitness description.\n",
					readIn);
				printf("Valid commands:\n");
				printf("\t -cameraDistance [integer]\n");
				printf("\t -evaluations [integer]\n");
				printf("\t -evaluationPeriod [integer]\n");
				printf("\t -populationSize [integer]\n");
				printf("\t -generations [integer]\n");
				printf("\t -mutationRate [integer]\n");
				printf("\t -maximizeTime\n");
				printf("\t -objectToMove [object name]\n");
				printf("\t -addForwardDist [object name]\n");
				printf("\t -multForwardDist [object name]\n");
				printf("\t -terminateIf [object name] [height]\n");
				printf("\t -hiddenNodes [integer]\n");
				printf("\t -testForExplosions \n");
				printf("\t -recordHiddenLayerValues \n");
				printf("\t -useReactiveController \n");
				char ch = getchar();
				exit(0);
			}

			(*envFile) >> readIn;

		}
	}
}

ME_OBJECT* ENVIRONMENT::FindObject(char *name, AGENT *currAgent) {

	/*
	int objFound = false;
	ME_OBJECT *foundObject;
	int currObj = 0;

	while ( (!objFound) && (currObj<numObjects) ) {

		if ( strcmp(name,currAgent->GetName(currObj)) == 0 ) {
			objFound = true;
			foundObject = currAgent->GetObject(currObj);
		}

		currObj++;
	}

	if ( objFound )
		return foundObject;
	else
		return NULL;
	*/

	return( NULL );
}

double ENVIRONMENT::GetDistToLight(void) {

	return( -objectToMove->GetDistance(lightSource) );
}

double ENVIRONMENT::GetDistToPtSource(void) {

	double pos[3], orientation[12];
	objectToMove->GetPosAndRot(pos,orientation);
	
	return( -plume->DistToPtSource(pos[1],pos[0]) );
}

double ENVIRONMENT::GetForwardDist(void) {

	const dReal *pos, *pos2, *pos3;
	double dist, dist2, dist3;

	pos = dBodyGetPosition(objectToMove->startOfAggregate->body);
	dist = -pos[0];

	if ( addObjectToMove != NULL ) {
		pos2 = dBodyGetPosition(addObjectToMove->startOfAggregate->body);
		dist2 = -pos2[0];

		if ( add2ObjectToMove == NULL )
			return( dist + dist2 );
		
		else {
			pos3 = dBodyGetPosition(add2ObjectToMove->startOfAggregate->body);
			dist3 = -pos3[0];
			return( dist + dist2 + dist3 );
		}
	}
	else
		if ( multObjectToMove != NULL ) {
			pos2 = dBodyGetPosition(multObjectToMove->startOfAggregate->body);
			dist2 = -pos2[0];
			if ( mult2ObjectToMove == NULL )
				return( dist * dist2 );
			
			else {
				pos3 = dBodyGetPosition(mult2ObjectToMove->startOfAggregate->body);
				dist3 = -pos3[0];
				return( dist * dist2 * dist3 );
			}
		}
		else
			return( dist );
}

double ENVIRONMENT::GetMaxHeight(void) {

	double currMaxHeight = -1000.0;
	double pos[3], rot[12];
	int currAgent;
	ME_OBJECT *currObj;

	for (currAgent=0;currAgent<numOfAgents;currAgent++) {

		currObj = agents[currAgent]->GetObject(0);
		currObj->GetPosAndRot(pos,rot);

		if ( pos[1] > currMaxHeight )
			currMaxHeight = pos[1];
	}

	return( currMaxHeight );
}

double ENVIRONMENT::GetNumOfExternalJoints(void) {

	int extJoints = 0;
	int currAgent;

	for (currAgent=0;currAgent<numOfAgents;currAgent++)
		extJoints = extJoints + agents[currAgent]->numOfExternalJoints;

	return( double(extJoints) );
}

void ENVIRONMENT::OpenRaytracerFile(void) {

	char fileName[50];

	sprintf(fileName,"data/frame%d.dat",simParams->internalTimer);
	raytracerFile = new ofstream(fileName);
}

void ENVIRONMENT::RepositionLightSource(int currEval) {

	if ( numEvaluations > 1 ) {

		double x, y, z;
		double angleIncrement = PI/double(numEvaluations-1);

		x = -LIGHT_SOURCE_DISTANCE * cos( currEval*angleIncrement );
		y = LIGHT_SOURCE_SIZE / 2.0;
		z = LIGHT_SOURCE_DISTANCE * sin( currEval*angleIncrement );

		lightSource->Reposition(x,y,z);
	}
}

void ENVIRONMENT::ReadFromFile(void) {

	/*
	envFile = new ifstream(simParams->envFileName);
	(*envFile) >> numObjects;
	(*envFile) >> numJoints;

	if ( (numObjects==0) && (numJoints==0) ) {
		printf("Script file empty.\n");
		char ch = getchar();
		exit(0);
	}
	*/
}

double ENVIRONMENT::SumOfDists(void) {

	double totalDists = 0.0;
	int currAgent, otherAgent;

	for (currAgent=0;currAgent<(numOfAgents-1);currAgent++)
		for (otherAgent=currAgent+1;otherAgent<numOfAgents;otherAgent++)
			totalDists = totalDists + agents[currAgent]->DistTo(0,agents[otherAgent]);

	return(totalDists);
}

#endif