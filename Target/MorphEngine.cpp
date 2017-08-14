// MorphEngine.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "stdlib.h"

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

//#include <glut.h>

#include <time.h>
//#include <resource.h>
//#include <unistd.h>

#include "environment.h"
#include "simParams.h"

SIM_PARAMS  *simParams;
ENVIRONMENT *env;
dsFunctions fn;

extern int OUTPUT_RAYTRACER_FILE;
extern int ADD_NOISE;
extern int SPHERE;
extern double STEP_SIZE;

char locase (char c)
{
  if (c >= 'A' && c <= 'Z')
	  return c - ('a'-'A');
  else
	  return c;
}

static void start(void) {

}

void CaptureFrame(int num)
{

	/*
	int width = 352*2;
	int height = 288*2;

	fprintf(stderr,"capturing frame %04d\n",num);

   char s[100];
   sprintf (s,"frame/%d.ppm",num);
   FILE *f = fopen (s,"wb");
   fprintf (f,"P6\n%d %d\n255\n",width,height);

   void *buf = malloc( width * height * 3 );
   glReadPixels( 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buf );

   for (int y=(height - 1); y>=0; y--) {
     for (int x=0; x<width; x++) {
       unsigned char *pixel = ((unsigned char *)buf)+((y*width+ x)*3);
       unsigned char b[3];
       b[0] = *pixel;
       b[1] = *(pixel+1);
       b[2] = *(pixel+2);
       fwrite(b,3,1,f);
     }
    }
   free(buf);
   fclose(f);
   */
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {

	dBodyID b1,b2;
	dContact contact;

	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);

	if ( (!b1) && (!b2) )
		return;

	if (b1 && b2 && dAreConnected(b1,b2))
		return;
	
	ME_OBJECT *obj1 = (ME_OBJECT *)dGeomGetData(o1);
	ME_OBJECT *obj2 = (ME_OBJECT *)dGeomGetData(o2);

	if ( obj1 && obj2 ) {

		if ( obj1->ID > obj2->ID ) {
			ME_OBJECT *temp = obj2;
			obj2 = obj1;
			obj1 = temp;
			temp = NULL;
		}
	}

	contact.surface.mode = 0;
	//contact.surface.mu = 10.0;
	//contact.surface.mu2 = 10.0;
	//contact.surface.mu = dInfinity;
	//contact.surface.mu2 = dInfinity;
	contact.surface.mu = 1000;
	contact.surface.mu2 = 1000;

	double gkp=300000; // ground spring coefficient
	double gdp=10000;  // ground damper coefficient
	double dt = STEP_SIZE;  //  sampling time

	contact.surface.soft_erp = dt*gkp/( dt * gkp * gdp);
	contact.surface.soft_cfm = 1/( dt * gkp + gdp);

	//contact.surface.mode = dContactBounce;
	//contact.surface.bounce = 0;

	if ( ((!obj1) && (obj2->contactWithFloor) && (!obj2->fixedToWorld) ) ||
		 ((!obj2) && (obj1->contactWithFloor) && (!obj1->fixedToWorld) ) ||
		 ((obj1) &&
		  (obj2) &&
		  (!(obj1->IsPartOfEnvironment() && obj2->IsPartOfEnvironment())) && 
		  (obj1->contactWithFloor) && (obj2->contactWithFloor)) ) {

		if (dCollide(o1,o2,0,&contact.geom,sizeof(dContactGeom))) {

			if ( obj1 && obj2 && (simParams->connectedObjects->Get(obj1->ID,obj2->ID)==0) )
				simParams->agentExploding++;

			if ( !( ((o1==env->floor) && (o2==env->agents[0]->objects[0]->distanceSensor)) || 
				    ((o2==env->floor) && (o1==env->agents[0]->objects[0]->distanceSensor)) ) &&
			 	((o1==env->floor) || (o2==env->floor)) ) {

				dJointID c = dJointCreateContact(env->world,env->contactgroup,&contact);
				dJointAttach(c,b1,b2);
			}
			/*
			if ( (obj1!=NULL) && (!obj1->IsPartOfEnvironment()) && (obj1->contactWithFloor) ) {
				obj1->SetTouchSensor(contact.geom.depth);
				obj1->RecordTouchdown(contact.geom.pos);
				dJointID c = dJointCreateContact(env->world,env->contactgroup,&contact);
				dJointAttach(c,b1,b2);
			}
			if ( (obj2!=NULL) && (!obj2->IsPartOfEnvironment()) && (obj2->contactWithFloor) ) {
				obj2->SetTouchSensor(contact.geom.depth);
				obj2->RecordTouchdown(contact.geom.pos);
				dJointID c = dJointCreateContact(env->world,env->contactgroup,&contact);
				dJointAttach(c,b1,b2);
			}
			*/

			//dJointID c = dJointCreateContact(env->world,env->contactgroup,&contact);
			//dJointAttach(c,b1,b2);
		}
	}
}

void SetViewpoint(void) {

		float xyz[3];
		float hpr[3];

		/*
		dsGetViewpoint(xyz,hpr);

		printf("%3.3f %3.3f %3.3f %3.3f %3.3f %3.3f\n",
			xyz[0],xyz[1],xyz[2],hpr[0],hpr[1],hpr[2]);
		*/

		xyz[0] = 1.31;
		xyz[1] = 0.996;
		xyz[2] = 1.46;

		hpr[0] = -132.5;
		hpr[1] = -48.0;
		hpr[2] = 0.0;

		dsSetViewpoint(xyz,hpr);
}

void Tick(int pause, int shouldDraw) {

	if ( simParams->evaluating ) {

		if ( shouldDraw ) {
			env->Draw();
			//SetViewpoint();

			if ( (simParams->recordMovie) && (simParams->internalTimer%10==0) )
				CaptureFrame(simParams->currFrame++);
		}

		if ( !pause ) {

			dSpaceCollide(env->space,0,&nearCallback);

//			dWorldStep(env->world,0.001);
			dWorldStep(env->world,STEP_SIZE);

			dJointGroupEmpty(env->contactgroup);

			env->MoveAgents();

			if ( env->EvaluationFinished() ) {

				env->DestroyAgent();

				env->Reset();

				simParams->agentsToEvaluate--;

				if ( simParams->agentsToEvaluate == 0 ) {

					simParams->CloseFiles();

					simParams->evaluating = false;

					//exit(0);
				}
				else
					env->CreateAgent();
			}
		}
	}

	else {

		if ( env->AgentAvailable() ) {

			simParams->DestroyMotorCommands();

			simParams->OpenFiles();

			simParams->GetMotorCommands();

			env->CreateAgent();

			simParams->evaluating = true;
		}
	}
}


static void simLoop(int pause) {

	Tick(pause,true);
}

static void command(int cmd) {

	cmd = locase(cmd);

	switch ( cmd ) {

	case '8':
		env->Push(0.0,0.0,1.0);
		break;

	case '2':
		env->Push(0.0,0.0,-1.0);
		break;

	case '9':
		env->Push(-1.0,0.0,0.0);
		break;

	case '1':
		env->Push(1.0,0.0,0.0);
		break;

	case '4':
		env->Push(0.0,-1.0,0.0);
		break;

	case '6':
		env->Push(0.0,1.0,0.0);
		break;

	case 'd':
		env->ChangeDebrisField();
		break;

	case 'f':
		env->ToggleFootprintDrawing();
		break;

	case 'm':
		printf("Recording movie.\n");
		simParams->recordMovie = true;
		break;

	case 'n':
		simParams->drawNetwork++;
		if ( simParams->drawNetwork == 3 )
			simParams->drawNetwork = 0;
		break;

	case 'c':
		if ( env->plume )
			env->DestroyPlume();
		else
			env->CreatePlume();
		break;

	case 'p':
		env->IncreasePerturbation(0.1);
		break;

	case 'r':
		env->ToggleTrajectoryDrawing();
		break;

	case 's':
		env->PerturbSynapses();
		break;

	case 't':
		simParams->useTransparency = !simParams->useTransparency;
		break;

	case 'x':
		exit(0);
		break;
	}
}

void InitializeODE(int argc, char **argv) {

	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "textures";
}

int main(int argc, char **argv)
{

	system("process -p MorphEngine.exe low");

	simParams = new SIM_PARAMS(argc, argv);

	if ( simParams->rendering )
		InitializeODE(argc,argv);

	env = new ENVIRONMENT;

	if ( simParams->rendering ) {
		dsSimulationLoop(argc,argv,352,288,&fn);
		//dsSimulationLoop(argc,argv,352*2,288*2,&fn);
	}
	else {

		while ( 1 ) {

			Tick(false,false);
		}
	}

	delete env;
	env = NULL;

	delete simParams;
	simParams = NULL;

	return 0;
}