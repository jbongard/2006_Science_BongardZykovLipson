/* ---------------------------------------------------
   FILE:     bodyPlan.h
	AUTHOR:   Josh Bongard
	DATE:     October 2, 2000
	FUNCTION: This class contains all information for
				 a single physical segment of an organism
				 in the MathEngine environment.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _NEURAL_NETWORK_H
#define _NEURAL_NETWORK_H

#include "matrix.h"

class NEURAL_NETWORK {

public:
	MATRIX *motorCommands;

public:

	NEURAL_NETWORK(int numJoints);
	NEURAL_NETWORK(NEURAL_NETWORK *nn);
	~NEURAL_NETWORK(void);
	void Mutate(void);
	void Print(void);
	void Write(char *fileName);
	void Write(ofstream *outFile);
};

#endif