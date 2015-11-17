#pragma once

#include <Windows.h>

class TrjGenerator
{
public:
	TrjGenerator(void);
	~TrjGenerator(void);

	void Initialize(char* fName, int nDim);
	bool InterpolateNextPoint(double* posDir);
	bool InterpolateNextPoint(double* posDir, double* fwdVel);
	bool GetNextPointStatic(double* posDir);
	//bool ReadNextCoord(double* coord, int type);
	double m_currT;	

private:
	double*		m_T;
	double** m_ptList;
	int m_idx;
	int m_nPt;
	int m_nDim;

	LARGE_INTEGER m_Freq;
	LARGE_INTEGER m_StartingTime;

};

