/* ---------------------------------------------------
   FILE:     plumePoint.h
	AUTHOR:   Josh Bongard
	DATE:     February 12, 2002
	FUNCTION: This class contains all information for
				 a single point in a plume.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"

#ifndef _PLUME_POINT_H
#define _PLUME_POINT_H

class PLUME_POINT {

public:
	double		distFromPointSource;
	double		conc;

private:
	int			indexX, indexY;
	double		posX, posY;
	double		sideLength;
	//RGraphic	*plumeGraphic1;
	//RGraphic	*plumeGraphic2;
	float 		colour[4];

public:
	PLUME_POINT(int x, int y, double centreX, double centreY);
	~PLUME_POINT(void);
	void CalcDistFromPtSource(double ptSrcX, double ptSrcY);
	void Draw(double minDist, double maxDist, double plumeHeight);
	void Write(ofstream *outFile);
};

#endif