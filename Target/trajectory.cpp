/* ---------------------------------------------------
   FILE:     trajectory.cpp
	AUTHOR:   Josh Bongard
	DATE:     November 3, 2003
	FUNCTION: This class contains all information for
			  a three-dimensional trajectory.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _TRAJECTORY_CPP
#define _TRAJECTORY_CPP

#include "ode/ode.h"
#include "drawStuff/drawStuff.h"
#include "simParams.h"
#include "trajectory.h"

#ifdef dDOUBLE
#define dsDrawLine dsDrawLineD
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

extern double TRAJECTORY_POINT_RADIUS;
extern SIM_PARAMS *simParams;

TRAJECTORY::TRAJECTORY(int l) {

	length = l;

	x = new double[length];
	y = new double[length];
	z = new double[length];
}

TRAJECTORY::~TRAJECTORY(void) {

	delete[] x;
	x = NULL;

	delete[] y;
	y = NULL;

	delete[] z;
	z = NULL;
}

void TRAJECTORY::Draw(double r, double g, double b) {

	/*
	int currTime = 0;
	dReal pos[3];
	dReal orientation[12];

	simParams->SetDefaultRotation(orientation);

	while ( currTime < simParams->internalTimer ) {

		pos[0] = x[currTime];
		pos[1] = y[currTime];
		pos[2] = z[currTime];

		dsSetColor(r,g,b);
		dsDrawSphere(pos,orientation,TRAJECTORY_POINT_RADIUS);

		currTime = currTime + 10;
	}
	*/
}

void TRAJECTORY::Reset(void) {

	for (int l=0;l<length;l++) {
		x[l] = 0.0;
		y[l] = 0.0;
		z[l] = 0.0;
	}
}

void TRAJECTORY::Store(int t, double currX, double currY, double currZ) {

	x[t] = currX;
	y[t] = currY;
	z[t] = currZ;
}

#endif