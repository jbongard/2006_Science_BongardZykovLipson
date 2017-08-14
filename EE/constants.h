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

int			RANDOM_SEED			= 0;

// ----------------------------------------------------------------
//                           EE constants
// ----------------------------------------------------------------

int		MAX_TARGET_TRIALS		= 16;

int     MODELS_FOR_DISAGREEMENT = 5;

char	FILES_LOCATION[100]  = "..\\..\\Files";

char	TEMP_FILENAME[100] = "..\\..\\Files\\temp.dat";
char	TEMP2_FILENAME[100] = "..\\..\\Files\\temp2.dat";

//int		EVAL_PERIOD			= 300;
int		EVAL_PERIOD			= 100;

char	DATA_DIRECTORY[100] = "..\\..\\Data";

char    EXP_FILENAME[100] = "..\\..\\Data\\exp";
char    EST_FILENAME[100] = "..\\..\\Data\\est";

double	TEST_BANK_CUTOFF	= 5.0;

// ----------------------------------------------------------------
//                           GA constants
// ----------------------------------------------------------------

int			STAGNATION_LIMIT		= 1000000;

int			NUM_EST_GENERATIONS		= 200;
//int			NUM_EST_GENERATIONS		= 10;

int			EST_POPULATION_SIZE		= 15;

int			NUM_EXP_GENERATIONS		= 1;

int			EXP_POPULATION_SIZE		= 100;

int			HEADER_LENGTH		= 10;
int			WINDOW_LENGTH		= 5;

double		MUTATION_BIAS		= 5.0;

double	    CROSSOVER_PROBABILITY = 0.5;

int			TOURNAMENT_SIZE		= 2;

int			USE_DETERMINISTIC_CROWDING = true;

// ----------------------------------------------------------------
//                           Morphology constants
// ----------------------------------------------------------------

int			OBJECT_TO_MOVE		= 0;

int			NUM_STICKS			= 9;
int			RAW_WIDTH			= 9;

//double		MIN_LENGTH			= 0.01;
//double		MAX_LENGTH			= 0.1;
double		MIN_LENGTH			= 0.1;
double		MAX_LENGTH			= 2.0;

double		MAX_MASS_PER_PART   = 1.0;

double		CYLINDER_DIAMETER	= 0.14/2.0;

char        TARGET_DATA[100]   = "..\\..\\PermanentFiles";

char		TARGET_SOURCE[100] = "..\\..\\PermanentFiles\\Target.dat";
char		TARGET_DEST[100]   = "..\\..\\Files\\Body.dat";

char		SENSOR_FILENAME[100]   = "..\\..\\Files\\Sensors.dat";

int			NUM_TOUCH_SENSORS	= 4;
int			NUM_ANGLE_SENSORS	= NUM_STICKS-1;

double		Z_OFFSET	= 0.0;
//double		Z_OFFSET	= 1.0;

//int			MOTOR_STEP_SIZE = int(double(EVAL_PERIOD)/2.0);
int			MOTOR_STEP_SIZE = int(double(EVAL_PERIOD)/2.0);

//double MIN_RANGE_OF_MOTION = -45.0;
//double MAX_RANGE_OF_MOTION = 45.0;
double MIN_RANGE_OF_MOTION = -30.0;
double MAX_RANGE_OF_MOTION = 30.0;

double		DANGLER_LENGTH	= 0.1;

int			TOTAL_SENSORS = NUM_ANGLE_SENSORS + NUM_TOUCH_SENSORS + 2 + 1 + 1;

// ----------------------------------------------------------------
//                            NN constants
// ----------------------------------------------------------------

int			NUM_SENSORS			= NUM_TOUCH_SENSORS + NUM_ANGLE_SENSORS;
int			NUM_HIDDEN			= 3;
int			NUM_MOTORS			= 8;

char		NN_OUT_FILENAME[100]   = "Brain.dat";
char		BODY_OUT_FILENAME[100] = "Body.dat";

#endif