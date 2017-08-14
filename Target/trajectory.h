/* ---------------------------------------------------
   FILE:     trajectory.h
	AUTHOR:   Josh Bongard
	DATE:     November 3, 2003
	FUNCTION: This class contains all information for
				 a three-dimensional trajectory.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

class TRAJECTORY {

public:
	int length;
	double *x;
	double *y;
	double *z;

public:
	TRAJECTORY(int l);
	~TRAJECTORY(void);
	void Draw(double r, double g, double b);
	void Reset(void);
	void Store(int t, double currX, double currY, double currZ);
};

#endif