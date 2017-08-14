/* ---------------------------------------------------
   FILE:     bodyPlan.h
	AUTHOR:   Josh Bongard
	DATE:     October 2, 2000
	FUNCTION: This class contains all information for
				 a single physical segment of an organism
				 in the MathEngine environment.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"

#ifndef _MATRIX_H
#define _MATRIX_H

class MATRIX {

public:
	int length;
	int width;

private:
	double *vals;

public:
	MATRIX(int ln, int wd);
	MATRIX(int ln, int wd, double val);
	MATRIX(MATRIX *m);
	MATRIX(ifstream *inFile);
	~MATRIX(void);
	double   AbsoluteDifference(MATRIX *m);
	double   AbsoluteDifference(MATRIX *m, int startWd, int endWd);
	double   AbsoluteDifference(MATRIX *m, int wd);
	double   AbsoluteDifference(MATRIX *m, int wd, double normFactor);
	double   AbsoluteDifferenceOfTails(MATRIX *m);
	void	 Add(int i, int j, int val);
	int      BinaryToDecimal(int i, int j, int ln);
	int		 ComputeFSM(MATRIX *str, MATRIX *states, int i, int strIndex, int strLength, int q0, MATRIX *F);
	int	     Contains(double val);
	void	 CopyInto(MATRIX *source);
	MATRIX  *CountValues(int i1, int i2, int j1, int j2, int maxVal);
	void	 CreateChain(int depth);
	void	 CreateIdentity(void);
	void	 CreateParity(void);
	void	 DecreaseFullColumns(void);
	double   EqualColumnVals(int col1, int col2);
	void	 FillColumn(int myCol, int hisCol, MATRIX *m);
	void     FillRow(int myRow, int hisRow, MATRIX *m);
	void     Flip(int i, int j);
	void	 FlipRandomBit(void);
	void	 FlipRandomBitInRow(int i);
	double   Get(int i, int j);
	MATRIX  *GetColumn(int j);
	MATRIX  *GetRow(int i);
	void     IncreaseEmptyColumns(void);
	void	 InitColumns(int colSum);
	double   MaxOfColumn(int j);
	double	 MinOfColumn(int j);
	int		 MostSimilarRow(MATRIX *r, int i1, int i2);
	double	 NonZeroMeanOfColumn(int j);	
	void     NormalizeToMass(void);
	void	 Perturb(int maxVal);
	void	 Print(void);
	void     PrintRow(int i);
	void     Randomize(int maxVal);
	void     RandomizeColumn(int j, int maxVal);
	void     RandomizeRow(int i, int maxVal);
	void	 Replace(MATRIX *m);
	double	 RollingMeanDifference(MATRIX *m, int wd);
	int		 RowDifference(int myRow, int hisRow, MATRIX *m);
	void	 SelectUniquelyFrom(int maxVal);
	void	 Set(int i, int j, double val);
	void	 SetAllTo(int val);
	void     SetRow(int i, int val);
	double	 SumOfColumn(int j);
	double	 SumOfColumn(int i1, int i2, int j);
	double	 SumOfIndices(MATRIX *indices,int i, int j1, int j2);
	double	 SumOfRow(int i);
	void	 WriteAndRename(char *fileName);
	void	 Write(ofstream *outFile);
	void	 WriteRow(int i, ofstream *outFile);
	void	 WriteRows(int numRows, ofstream *outFile);
	void     Zero(void);
	void	 ZeroColumn(int j);

private:
	int		BinaryToDecimal(int j);
	MATRIX *DecimalToBinary(int val, int maxValue);
	int		FindFirstValue(int j);
	double  Mean(int i, int startJ, int endJ);
	void	InitColumn(int j, int colSum);
	void	InitRandomly(void);
};

#endif