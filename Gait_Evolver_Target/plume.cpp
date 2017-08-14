/* ---------------------------------------------------
   FILE:     plume.cpp
	AUTHOR:   Josh Bongard
	DATE:     February 12, 2002
	FUNCTION: This class contains all information for
				 a single odour plume integrated into the
				 physical simulation.
 -------------------------------------------------- */

#include "stdafx.h"
#include "math.h"

#ifndef _PLUME_CPP
#define _PLUME_CPP

#include "plume.h"
#include "simParams.h"

extern int		PLUME_RESOLUTION;
extern double	GROUND_LENGTH;
extern double	PLUME_PT_SRC_X[4];
extern double	PLUME_PT_SRC_Y[4];
extern SIM_PARAMS *simParams;

PLUME::PLUME(double plumeHeight) {

	int i,j;
	double currX, currY;

	height = plumeHeight;

	gapLength = GROUND_LENGTH/double(PLUME_RESOLUTION);

	plumePoints = new PLUME_POINT*[PLUME_RESOLUTION*PLUME_RESOLUTION];

	currY = (-GROUND_LENGTH/2.0) + (gapLength/2.0);

	for (i=0;i<PLUME_RESOLUTION;i++) {
		currX = (-GROUND_LENGTH/2.0) + (gapLength/2.0);
		for (j=0;j<PLUME_RESOLUTION;j++) {
			plumePoints[i*PLUME_RESOLUTION+j] = new PLUME_POINT(j,i,currX,currY);
			currX = currX + gapLength;
		}
		currY = currY + gapLength;
	}

	Reset();
}

PLUME::~PLUME(void) {

	int i,j;

	for (i=0;i<PLUME_RESOLUTION;i++)
		for (j=0;j<PLUME_RESOLUTION;j++)
			delete plumePoints[i*PLUME_RESOLUTION+j];

	delete[] plumePoints;
}

void  PLUME::ChangePtSource(int evalIndex) {

	ptSource[0] = (PLUME_PT_SRC_X[evalIndex]*GROUND_LENGTH) - (GROUND_LENGTH/2.0);
	ptSource[1] = (PLUME_PT_SRC_Y[evalIndex]*GROUND_LENGTH) - (GROUND_LENGTH/2.0);

	CalculatePtDists();
	WriteToFile();
}

double PLUME::DistToPtSource(double x, double y) {

	double xComp = (x-ptSource[0])*(x-ptSource[0]);
	double yComp = (y-ptSource[1])*(y-ptSource[1]);

	return( sqrt( xComp + yComp ) );
}

void PLUME::Draw(void) {

	for (int i=0;i<PLUME_RESOLUTION;i++)
		for (int j=0;j<PLUME_RESOLUTION;j++)
			plumePoints[i*PLUME_RESOLUTION+j]->Draw(minDistToPtSource,maxDistToPtSource,height);
}

double PLUME::GetConcAt(double x, double y) {

	int xIndex, yIndex;
	double conc;

	xIndex = int((x + (GROUND_LENGTH/2.0) + (gapLength/2.0)) / gapLength);
	yIndex = int((y + (GROUND_LENGTH/2.0) + (gapLength/2.0)) / gapLength);

	if ( (xIndex < 0) || (xIndex > (PLUME_RESOLUTION-1)) ||
		  (yIndex < 0) || (yIndex > (PLUME_RESOLUTION-1)) )
		return( 0.0 );
	else {
		conc = plumePoints[yIndex*PLUME_RESOLUTION+xIndex]->conc;
		return( conc );
	}
}

void PLUME::Reset(void) {

	int whichWall = simParams->RandInt(1,4);

	switch ( whichWall ) {
	case 1:
		ptSource[0] = -GROUND_LENGTH/2.0;
		ptSource[1] = (simParams->Rand(0.0,1.0)*GROUND_LENGTH) - (GROUND_LENGTH/2.0);
		break;
	case 2:
		ptSource[0] = GROUND_LENGTH/2.0;
		ptSource[1] = (simParams->Rand(0.0,1.0)*GROUND_LENGTH) - (GROUND_LENGTH/2.0);
		break;
	case 3:
		ptSource[0] = (simParams->Rand(0.0,1.0)*GROUND_LENGTH) - (GROUND_LENGTH/2.0);
		ptSource[1] = -GROUND_LENGTH/2.0;
		break;
	case 4:
		ptSource[0] = (simParams->Rand(0.0,1.0)*GROUND_LENGTH) - (GROUND_LENGTH/2.0);
		ptSource[1] = GROUND_LENGTH/2.0;
		break;
	}

	CalculatePtDists();
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void PLUME::CalculatePtDists(void) {

	int i,j;

	minDistToPtSource = 1000.0;
	maxDistToPtSource = -1000.0;

	for (i=0;i<PLUME_RESOLUTION;i++)
		for (j=0;j<PLUME_RESOLUTION;j++) {

			plumePoints[i*PLUME_RESOLUTION+j]->CalcDistFromPtSource(ptSource[0],ptSource[1]);
			
			if ( plumePoints[i*PLUME_RESOLUTION+j]->distFromPointSource < minDistToPtSource )
				minDistToPtSource = plumePoints[i*PLUME_RESOLUTION+j]->distFromPointSource;
			
			if ( plumePoints[i*PLUME_RESOLUTION+j]->distFromPointSource > maxDistToPtSource )
				maxDistToPtSource = plumePoints[i*PLUME_RESOLUTION+j]->distFromPointSource;	
		}
}

void PLUME::WriteToFile(void) {

	if ( simParams->plumeFile != NULL )

		for (int i=0;i<PLUME_RESOLUTION;i++) {
			for (int j=0;j<PLUME_RESOLUTION;j++)
				plumePoints[i*PLUME_RESOLUTION+j]->Write(simParams->plumeFile);
			(*simParams->plumeFile) << "\n";
		}
}

#endif