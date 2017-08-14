/* ---------------------------------------------------
   FILE:     constants.h
	AUTHOR:   Josh Bongard
	DATE:     March 21, 2001
	FUNCTION: This class contains all of the constants
			    for this project.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _CONSTANTS_H
#define _CONSTANTS_H

double white[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

double	GROUND_LENGTH = 30.0;

double PI		= 3.14159;

int   RAND_SEED = 1;

double STEP_SIZE = 0.01;

// ---------------------------------------------------
//			File exchange specific constants
// ---------------------------------------------------

char	BODY_FILENAME[100] = "..\\..\\Files\\Body.dat";

char	BRAIN_FILENAME[100] = "..\\..\\Files\\Brain.dat";

char	SENSOR_FILENAME[100] = "Sensors.dat";

char	OBJECT_FILENAME[100] = "Object.dat";

char	TEMP_FILENAME[100] = "..\\..\\Files\\temp.dat";
char	TEMP2_FILENAME[100] = "..\\..\\Files\\temp2.dat";

// ---------------------------------------------------
//			External environment specific constants
// ---------------------------------------------------

double LIGHT_SOURCE_DISTANCE = 2.0;
double LIGHT_SOURCE_SIZE     = 0.3;
double DEF_CAMERA_DISTANCE	 = 9.0;

double DEFAULT_RESTITUTION	 = 0.5;
double DEFAULT_SOFTNESS		 = 0.0005;

int    DEBRIS_NUM_OF_DEBRIS_FIELDS = 1;

int	   DEBRIS_NONE = 0;
int    DEBRIS_LOOSE_RUBBLE = 1;
int	   DEBRIS_FIXED_RUBBLE = 2;
int	   DEBRIS_EMBEDDED_RUBBLE = 3;
int    DEBRIS_UNEVEN_FLOOR = 4;
int    DEBRIS_RAMP = 5;

int	   DEBRIS_NUM_OF_RUBBLE = 8*8;
double DEBRIS_RUBBLE_RADIUS = 0.2;
double DEBRIS_UNEVEN_FLOOR_HEIGHT = 0.03;

int	   CURRENT_DEBRIS_TYPE = DEBRIS_NONE;

// ---------------------------------------------------
//			        Multiple agent constants
// ---------------------------------------------------

int RANDOM_DISPERSAL = true;
double DISP_DIST = 1.0;
double DISP_PTS[2*24] = {-1.0,	 1.0,
						 0.0,	 1.0,
						 1.0,	 1.0,
						-1.0,	 0.0,
						 1.0,	 0.0,
						-1.0,	-1.0,
						 0.0,	-1.0,
						 1.0,	-1.0,
						-2.0,	 2.0,
						-1.0,	 2.0,
						 0.0,	 2.0,
						 1.0,	 2.0,
						 2.0,	 2.0,
						-2.0,	 1.0,
						 2.0,	 1.0,
						-2.0,	 0.0,
						 2.0,	 0.0,
						-2.0,	-1.0,
						 2.0,	-1.0,
						-2.0,	-2.0,
						-1.0,	-2.0,
						 0.0,	-2.0,
						 1.0,	-2.0,
						 2.0,	-2.0};

// ---------------------------------------------------
//			        Plume specific constants
// ---------------------------------------------------

int MOVING_PLUME		  = false;
int TURBULENT_PLUME		  = false;
int PLUME_EVALUATIONS	  = 4;

int PLUME_RESOLUTION		  = 30;
double PLUME_AREA			  = 1.0;

double	PLUME_PT_SRC_X[4]		  = { 0.0, 0.33, 0.67, 1.0};
double PLUME_PT_SRC_Y[4]		  = { 1.0, 1.0, 1.0, 1.0};

// ---------------------------------------------------
//			  Genetic algorithm specific constants
// ---------------------------------------------------

int FUNCTION_AS_READER	  = false;
int OUTPUT_RAYTRACER_FILE = false;
int OUTPUT_FOOTPRINTS	  = false;
int CREATE_PLUME		  = false;
int ADD_NOISE			  = false;
int	OUTPUT_TRAJECTORY	  = false;
int OUTPUT_DEBRIS		  = false;

int	NOISE_TRIALS		  = 30;

int POPULATION_SIZE       = 100;
int NUM_GENERATIONS		  = 100;

int FLOATING_PT_PRECISION = 2;

int DEFAULT_EVALUATION_PERIOD = 100;

double CULL_FRACTION		  = 0.5;
double CROSS_FRACTION		  = 0.25;

int   UNEVEN_CROSSOVER	  = false;

double AVG_NUM_OF_MUTS     = 2.0;

double	MUTATION_INCREMENT   = 0.01;

int   TOURNEY_SIZE			= 3;

// ---------------------------------------------------
//			    Neural network specific constants
// ---------------------------------------------------

int BIAS_NODES = 1;

double SENSOR_NEURON_RADIUS = 0.25;
double HIDDEN_NEURON_RADIUS = 0.15;
double MOTOR_NEURON_RADIUS = 0.1;

double NN_WIDTH = 3.0;

// ---------------------------------------------------
//			        Object specific constants
// ---------------------------------------------------

int RECTANGLE	= 0;
int CYLINDER	= 1;
int SPHERE		= 2;

//#define DENSITY 50.0
#define DENSITY 1.0

double TRAJECTORY_POINT_RADIUS = 0.1;

double BELLY_SENSOR_DISTANCE = 10.0;

// ---------------------------------------------------
//			        Joint specific constants
// ---------------------------------------------------

double DEF_MOTOR_FORCE = 50.0;
double DEF_MOTOR_SPEED = 10.0;

//double MAX_JOINT_SEPARATION = 3.0;
double MAX_JOINT_SEPARATION = 100.0;

int    JOINT_HINGE = 0;
int	   JOINT_BALL = 1;
int	   JOINT_FIXED = 2;
int	   JOINT_SLIDER = 3;
int	   JOINT_WORLD = 4;
int	   JOINT_SPRING = 5;

#endif