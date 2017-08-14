/* ---------------------------------------------------
   FILE:     bodyPlan.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 2, 2000
	FUNCTION: This class contains all information for
				 a single physical segment of an organism
				 in the MathEngine environment.
 -------------------------------------------------- */

#include "stdafx.h"
#include "stdlib.h"
#include "math.h"

#ifndef _MATRIX_CPP
#define _MATRIX_CPP

#include "matrix.h"
#include "simParams.h"

extern int			RANDOM_INIT;
extern SIM_PARAMS   *simParams;
extern char			TEMP_FILENAME[100];

MATRIX::MATRIX(int ln, int wd) {

	length = ln;
	width = wd;

	vals = new double[length*width];

	InitRandomly();
}

MATRIX::MATRIX(int ln, int wd, double val) {

	length = ln;
	width = wd;

	vals = new double[length*width];

	for (int i=0;i<length;i++)

		for (int j=0;j<width;j++)

			Set(i,j,val);
}

MATRIX::MATRIX(MATRIX *m) {

	length = m->length;
	width  = m->width;

	vals = new double[length*width];

	for (int i=0;i<length;i++)

		for (int j=0;j<width;j++)

			Set(i,j,m->Get(i,j));
}

MATRIX::MATRIX(ifstream *inFile) {

	double temp;

	(*inFile) >> length;
	(*inFile) >> width;

	vals = new double[length*width];

	for (int i=0;i<length;i++)

		for (int j=0;j<width;j++) {

			(*inFile) >> temp;
			Set(i,j,temp);
		}
}

MATRIX::~MATRIX(void) {

	delete[] vals;
	vals = NULL;
}

double MATRIX::AbsoluteDifference(MATRIX *m) {

	return( AbsoluteDifference(m,width) );
}

double MATRIX::AbsoluteDifference(MATRIX *m, int startWd, int endWd) {

	if ( (length==m->length) && (width==m->width) ) {

		double sum = 0.0;
		
		for (int i=0;i<length;i++)

			for (int j=startWd;j<=endWd;j++)
				
				sum = sum + fabs( double(Get(i,j)) - double(m->Get(i,j)) );

		return( sum / double(length*(endWd-startWd+1)) );
	}
	else
		return( 0.0 );
}

double MATRIX::AbsoluteDifference(MATRIX *m, int wd) {

	if ( (length==m->length) && (width==m->width) ) {

		double sum = 0.0;
		
		for (int i=0;i<length;i++)

			for (int j=0;j<wd;j++)
				
				sum = sum + fabs( double(Get(i,j)) - double(m->Get(i,j)) );

		return( sum / double(length*wd) );
	}
	else
		return( 0.0 );
}

double MATRIX::AbsoluteDifference(MATRIX *m, int wd, double normFactor) {

	return( AbsoluteDifference(m,wd) * double(length*wd) / normFactor );
}

double MATRIX::AbsoluteDifferenceOfTails(MATRIX *m) {

	if ( length==m->length ) {

		double sum = 0.0;
		
		for (int i=0;i<length;i++)
				
			sum = sum + fabs( double(Get(i,width-1)) - double(m->Get(i,m->width-1)) );

		return( sum / double(length) );
	}
	else
		return( 0.0 );
}

void MATRIX::Add(int i, int j, int val) {

	Set(i,j,Get(i,j)+val);
}

int  MATRIX::BinaryToDecimal(int i, int j, int ln) {

	int totalValue = 0;
	int placeValue = 128;
	int temp;

	for (int wd=j;wd<(j+ln);wd++) {

		temp = Get(i,wd);

		totalValue = totalValue + placeValue*Get(i,wd);
		placeValue = placeValue/2;
	}

	return( totalValue );
}

int  MATRIX::ComputeFSM(MATRIX *str, MATRIX *states, int i, int strIndex, int strLength, int q0, MATRIX *F) {

	states->Set(i,strIndex,q0);

	if ( strIndex == strLength ) {

		return( F->Get(0,q0) );
	}
	else {

		int currChar = str->Get(i,strIndex);

		return( ComputeFSM(str,states,i,++strIndex,strLength,Get(currChar,q0),F) );
	}
}

int MATRIX::Contains(double val) {

	int found = false;
	int i = 0;
	int j;

	while ( (i<length) && (!found) ) {

		j = 0;

		while ( (j<width) && (!found) ) {

			if ( Get(i,j) == val )
				found = true;

			j++;
		}

		i++;
	}

	return( found );
}

MATRIX *MATRIX::CountValues(int i1, int i2, int j1, int j2, int maxVal) {

	MATRIX *valueCount = new MATRIX(1,maxVal,0);

	for (int i=i1;i<i2;i++)

		for (int j=j1;j<j2;j++)

			valueCount->Add(0,Get(i,j),1);

	return( valueCount );
}

void MATRIX::CreateChain(int depth) {

	int i;
	int depthCounter;

	for (int j=0;j<width;j++) {

		depthCounter = depth;
		i = j;

		while ( depthCounter > 0 ) {

			Set(i,j,1);
			i = i + 1;
			if ( i==length )
				i = 0;
			depthCounter--;
		}
	}
}

void MATRIX::CreateIdentity(void) {

	for (int i=0;i<length;i++)
		for (int j=0;j<width;j++)
			if ( i == j )
				Set(i,j,1);
			else
				Set(i,j,0);
}

void MATRIX::CreateParity(void) {

	MATRIX *binaryOfColumn;
	int binarySum;

	for (int j=0;j<width;j++)
		
		for (int i=0;i<length;i++) {
			
			binaryOfColumn = DecimalToBinary(i,length-1);

			binarySum = binaryOfColumn->SumOfRow(0);
			
			if ( (binarySum%2) == 0 )
				Set(i,j,0);
			else
				Set(i,j,1);

			delete binaryOfColumn;
			binaryOfColumn = NULL;

		}
}

void MATRIX::DecreaseFullColumns(void) {

	for (int j=0;j<width;j++) {

		if ( SumOfColumn(j) == length )
			Set(simParams->RandInt(0,length-1),j,0);
	}
}

double MATRIX::EqualColumnVals(int col1, int col2) {

	double sumOfEquals = 0.0;

	for (int i=0;i<length;i++)

		if ( Get(i,col1) == Get(i,col2) )
			sumOfEquals++;

	return( sumOfEquals / double(length) );
}

void MATRIX::FillColumn(int myCol, int hisCol, MATRIX *m) {

	for (int i=0;i<length;i++)
		Set(i,myCol,m->Get(i,hisCol));
}

void MATRIX::FillRow(int myRow, int hisRow, MATRIX *m) {

	for (int j=0;j<width;j++)
		Set(myRow,j,m->Get(hisRow,j));
}

void MATRIX::Flip(int i, int j) {

	if ( Get(i,j) )
		Set(i,j,0);
	else
		Set(i,j,1);
}

void MATRIX::FlipRandomBit(void) {

	Flip(simParams->RandInt(0,length-1),simParams->RandInt(0,width-1));
}

void MATRIX::FlipRandomBitInRow(int i) {

	Flip(i,simParams->RandInt(0,width-1));
}


double MATRIX::Get(int i, int j) {

	return( vals[i*width + j] );
}

MATRIX *MATRIX::GetColumn(int j) {

	MATRIX *column = new MATRIX(length,1);

	for (int currRow=0;currRow<length;currRow++)
		column->Set(currRow,0,Get(currRow,j));

	return( column );
}

MATRIX *MATRIX::GetRow(int i) {

	MATRIX *row = new MATRIX(1,width);

	for (int currColumn=0;currColumn<width;currColumn++)
		row->Set(0,currColumn,Get(i,currColumn));

	return( row );
}

void MATRIX::IncreaseEmptyColumns(void) {

	for (int j=0;j<width;j++) {

		if ( SumOfColumn(j) == 0 )
			Set(simParams->RandInt(0,length-1),j,1);
	}
}

void MATRIX::InitColumns(int colSum) {

	SetAllTo(0);

	for (int j=0;j<width;j++)
		InitColumn(j,colSum);
}

int  MATRIX::MostSimilarRow(MATRIX *r, int i1, int i2) {

	int maxSimilarity = 10000;
	int rowDiff;

	for (int i=i1;i<i2;i++) {

		rowDiff = RowDifference(i,0,r);

		if ( rowDiff < maxSimilarity )
			maxSimilarity = rowDiff;
	}

	return( maxSimilarity );
}

void MATRIX::NormalizeToMass(void) {

	double fractionOfLengthChange;

	for (int i=1;i<length;i++)
		for (int j=8;j<8+4;j++) {
			
			fractionOfLengthChange = Get(i,j) / Get(0,j);
			Set(i,j,-53.7368*fractionOfLengthChange + 53.726);
		}
}

void MATRIX::Perturb(int maxVal) {

	int i, j;
		
	i = simParams->RandInt(0,length-1);
	j = simParams->RandInt(0,width-1);

	Set(i,j,simParams->RandInt(0,maxVal));
}

void MATRIX::Print(void) {

	for (int i=0;i<length;i++) {

		for (int j=0;j<width;j++) {
			printf("%3.3f ",vals[i*width+j]);
		}
		printf("\n");
	}

	char ch = getchar();
}

void MATRIX::PrintRow(int i) {

	for (int j=0;j<width;j++) {
		printf("%3.3f ",vals[i*width+j]);
	}
	printf("\n");

	char ch = getchar();
}

void MATRIX::Randomize(int maxVal) {

	for (int j=0;j<width;j++)
		RandomizeColumn(j,maxVal);
}

void MATRIX::RandomizeColumn(int j, int maxVal) {

	for (int i=0;i<length;i++)
		Set(i,j,simParams->RandInt(0,maxVal));
}

void MATRIX::RandomizeRow(int i, int maxVal) {

	for (int j=0;j<width;j++)
		Set(i,j,simParams->RandInt(0,maxVal));
}

void MATRIX::Replace(MATRIX *m) {

	if ( (length==m->length) && (width==m->width) ) {

		for (int i=0;i<length;i++)

			for (int j=0;j<width;j++)

				m->Set(i,j,Get(i,j));
	}
}

int  MATRIX::RowDifference(int myRow, int hisRow, MATRIX *m) {

	int diff = 0;

	for (int j=0;j<width;j++)
		diff = diff + abs( Get(myRow,j) - m->Get(hisRow,j) );

	return( diff );
}

void MATRIX::SelectUniquelyFrom(int maxVal) {

	MATRIX *chosen = new MATRIX(1,maxVal,0);
	int j;
	int chosenVal;

	for (j=0;j<width;j++) {

		chosenVal = simParams->RandInt(0,maxVal-1);

		while ( chosen->Get(0,chosenVal) )
			chosenVal = simParams->RandInt(0,maxVal-1);

		Set(0,j,chosenVal);
		chosen->Set(0,chosenVal,1);
	}

	delete chosen;
	chosen = NULL;
}

void MATRIX::Set(int i, int j, double val) {

	vals[i*width + j] = val;
}

void MATRIX::SetAllTo(int val) {

	for (int i=0;i<length;i++)

		for (int j=0;j<width;j++)

			Set(i,j,val);
}

void MATRIX::SetRow(int i, int val) {

	for (int j=0;j<width;j++)
		Set(i,j,val);
}


int MATRIX::SumOfColumn(int j) {

	int sum = 0;

	for (int i=0;i<length;i++)
		sum = sum + Get(i,j);

	return( sum );
}

int MATRIX::SumOfColumn(int i1, int i2, int j) {

	int sum = 0;

	for (int i=i1;i<i2;i++)
		sum = sum + Get(i,j);

	return( sum );
}

int MATRIX::SumOfIndices(MATRIX *indices, int i, int j1, int j2) {

	int sum = 0;

	for (int j=j1;j<j2;j++)
		sum = sum + Get(0,indices->Get(i,j));

	return( sum );
}

int MATRIX::SumOfRow(int i) {

	int sum = 0;

	for (int j=0;j<width;j++)
		sum = sum + Get(i,j);

	return( sum );
}

void MATRIX::WriteAndRename(char *fileName) {

	ofstream *outFile = new ofstream(TEMP_FILENAME);

	Write(outFile);

	outFile->close();
	delete outFile;
	outFile = NULL;

	char command[200];

	sprintf(command,"rename %s %s",TEMP_FILENAME,fileName);

	system(command);
}

void MATRIX::Write(ofstream *outFile) {

	(*outFile) << length << " " << width << "\n";

	for (int i=0;i<length;i++) {

		for (int j=0;j<width;j++)

			(*outFile) << Get(i,j) << " ";

		(*outFile) << "\n";
	}
}

void MATRIX::WriteRow(int i, ofstream *outFile) {

	(*outFile) << 1 << " " << width << "\n";

	for (int j=0;j<width;j++)

		(*outFile) << Get(i,j) << " ";

	(*outFile) << "\n";
}

void MATRIX::WriteRows(int numRows, ofstream *outFile) {

	(*outFile) << numRows << " " << width << "\n";

	for (int i=0;i<numRows;i++) {

		for (int j=0;j<width;j++)

			(*outFile) << Get(i,j) << " ";

		(*outFile) << "\n";
	}
}

void MATRIX::Zero(void) {

	for (int j=0;j<width;j++)
		ZeroColumn(j);
}

void MATRIX::ZeroColumn(int j) {

	for (int i=0;i<length;i++)
		Set(i,j,0);
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

int  MATRIX::BinaryToDecimal(int j) {

	int totalValue = 0;
	int placeValue = 1;

	for (int i=0;i<length;i++) {

		totalValue = totalValue + placeValue*Get(i,j);
		placeValue = placeValue*2;
	}

	return( totalValue );
}

MATRIX *MATRIX::DecimalToBinary(int val, int maxValue) {

	int power = 0;

	while ( (maxValue - pow(2,power)) >= 0 ) {

		power = power + 1;
	}

	MATRIX *binaryVal = new MATRIX(1,power,0);

	while ( val > 0 ) {

		power = 0;

		while ( (val - pow(2,power)) >= 0 ) {

			power = power + 1;
		}

		binaryVal->Set(0,binaryVal->width-power,1);
		val = val - int(pow(2,power-1));
	}

	return( binaryVal );
}

int  MATRIX::FindFirstValue(int j) {

	int found = false;
	int index = 0;

	while ( (index<length) && (!found) ) {

		if ( Get(index,j) )
			found = true;
		else
			index++;
	}

	return( index );
}

void MATRIX::InitColumn(int j, int colSum) {

	int i;

	for (int c=0;c<colSum;c++) {

		i = simParams->RandInt(0,length-1);

		while ( Get(i,j) > 0 )
			i = simParams->RandInt(0,length-1);

		Add(i,j,1);
	}
}

void MATRIX::InitRandomly(void) {

	for (int i=0;i<length;i++)

		for (int j=0;j<width;j++)

			Set(i,j,simParams->Rand(0,1));
}

double MATRIX::Mean(int i, int startJ, int endJ) {

	double mean = 0.0;

	for (int j=startJ;j<=endJ;j++)
		mean = mean + Get(i,j);

	return( mean/double(endJ-startJ+1) );
}

#endif