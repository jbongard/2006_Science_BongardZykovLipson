/* ---------------------------------------------------
   FILE:     plumePoint.cpp
	AUTHOR:   Josh Bongard
	DATE:     February 12, 2002
	FUNCTION: This class contains all information for
				 a single point in an odour plume.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _PLUME_POINT_CPP
#define _PLUME_POINT_CPP

#include "plumePoint.h"
#include "simParams.h"
#include <math.h>
#include <drawstuff/drawstuff.h>

extern double		PLUME_AREA;
extern double		GROUND_LENGTH;
extern int			PLUME_RESOLUTION;
extern double		PLUME_HEIGHT;
extern SIM_PARAMS *simParams;

PLUME_POINT::PLUME_POINT(int x, int y, double centreX, double centreY) {
	
	indexX = x;
	indexY = y;
	posX = centreX;
	posY = centreY;
	sideLength = PLUME_AREA * (GROUND_LENGTH/double(PLUME_RESOLUTION));

	//plumeGraphic1 = NULL;
	//plumeGraphic2 = NULL;
}

PLUME_POINT::~PLUME_POINT(void) {

	/*
	RGraphicDelete(rc,plumeGraphic1,false);
	RGraphicDelete(rc,plumeGraphic2,false);

	delete plumeGraphic1;
	plumeGraphic1 = NULL;

	delete plumeGraphic2;
	plumeGraphic2 = NULL;
	*/
}

void PLUME_POINT::CalcDistFromPtSource(double ptSrcX, double ptSrcY) {

	double xComponent = (ptSrcX-posX)*(ptSrcX-posX);
	double yComponent = (ptSrcY-posY)*(ptSrcY-posY);

	distFromPointSource = sqrt( xComponent + yComponent );
}

void PLUME_POINT::Draw(double minDist, double maxDist, double plumeHeight) {

	conc = 1.0 - (distFromPointSource-minDist)/(maxDist-minDist);
	colour[0] = (float)conc;
	colour[1] = (float)conc;
	colour[2] = (float)conc;

	double origin[3], end[3];

	if ( distFromPointSource == minDist ) {

		colour[0] = 1.0 - colour[0];
		colour[1] = 1.0 - colour[1];
		colour[2] = 1.0 - colour[2];

		origin[1] = posX;
		origin[2] = plumeHeight;
		origin[0] = posY;
		end[1] = posX;
		end[2] = plumeHeight+0.2;
		end[0] = posY;

		dsSetColor(colour[0],colour[1],colour[2]);
		dsDrawLineD(origin,end);
	}

	origin[1] = posX - sideLength/2.0;
	origin[2] = plumeHeight;
	origin[0] = posY - sideLength/2.0;

	end[1] = posX + sideLength/2.0;
	end[2] = plumeHeight;
	end[0] = posY + sideLength/2.0;

	dsSetColor(colour[0],colour[1],colour[2]);
	dsDrawLineD(origin,end);

	origin[1] = posX + sideLength/2.0;
	origin[0] = posY - sideLength/2.0;

	end[1] = posX - sideLength/2.0;
	end[0] = posY + sideLength/2.0;

	dsDrawLineD(origin,end);
}

void PLUME_POINT::Write(ofstream *outFile) {

	(*outFile) << conc << ";";
}
#endif