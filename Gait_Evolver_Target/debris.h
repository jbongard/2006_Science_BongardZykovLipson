/* ---------------------------------------------------
   FILE:     debris.h
	AUTHOR:   Josh Bongard
	DATE:     May 10, 2003
	FUNCTION: This class contains all information for
			  a single collection of debris.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _DEBRIS_H
#define _DEBRIS_H

#include "meobject.h"

class DEBRIS {

public:
	ME_OBJECT	**objects;
	ME_JOINT	**joints;
	int		type;
	int		numObjects;
	int		numJoints;

public:
	DEBRIS(int debrisType, double agentWidth, double agentLength);
	~DEBRIS(void);
	void	CreateODEStuff(dWorldID world, dSpaceID space);
	void	DestroyODEStuff(dSpaceID space);
	void	Draw(void);
	void	Move(dWorldID world, dSpaceID space, double *agentPos, double agentWidth, double agentLength);

private:
	void    CreateEmbeddedRubble(void);
	void	CreateFixedRubble(void);
	void    CreateLooseRubble(void);
	void	CreateRamp(void);
	void	CreateUnevenFloor(double agentWidth, double agentLength);
	void    GetCentreOfMass(double *com);
	void	RecordDebris(void);
	void	RollLeft(dWorldID world, dSpaceID space);
	void    RollRight(dWorldID world, dSpaceID space);
};

#endif