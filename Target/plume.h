/* ---------------------------------------------------
   FILE:     plume.h
	AUTHOR:   Josh Bongard
	DATE:     February 12, 2002
	FUNCTION: This class contains all information for
				 an odour plume integrated into the
				 physical simulation
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"
#include "plumePoint.h"

#ifndef _PLUME_H
#define _PLUME_H

class PLUME {

public:
	double			height;

private:
	PLUME_POINT **plumePoints;
	double			gapLength;
	double			minDistToPtSource;
	double			maxDistToPtSource;
	double			ptSource[2];
	ofstream		*plumeFile;

public:
	PLUME(double plumeHeight);
	~PLUME(void);
	void	ChangePtSource(int evalIndex);
	double	GetConcAt(double x, double y);
	double	DistToPtSource(double x, double y);
	void	Draw(void);
	void    Reset(void);

private:
	void CalculatePtDists(void);
	void WriteToFile(void);
};

#endif