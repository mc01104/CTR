#include "stdafx.h"
#include "TrjGenerator.h"
#include <fstream>
#include <iostream>
#include <string>

TrjGenerator::TrjGenerator(void)
{
	QueryPerformanceFrequency(&m_Freq); 
	m_T = NULL;
	//for(int i=0; i<6; i++)	{	m_ptList[i] = NULL;		}
	m_ptList = NULL;
}


TrjGenerator::~TrjGenerator(void)
{
}


void TrjGenerator::Initialize(char* fName, int nDim)
{
	// CKim - nDim is the dimension of the coordinate (= number of elenments in each line)

	// CKim - Erase existing memory
	if(m_T)	{	delete m_T;		}
	for(int i=0; i<m_nDim; i++)	{	if(m_ptList[i]) {	delete m_ptList[i];		}	}
	m_nPt = 0;
	m_nDim = nDim;

	// CKim - Read file
	std::ifstream ifstr;	ifstr.open(fName);		std::string str;
	
	// CKim - Count number of lines
	while(1)
	{
		std::getline(ifstr,str);
		if(ifstr.eof())	{	break;	}
		m_nPt++;
	}
	ifstr.close();

	// CKim - Allocate memory
//	std::cout<<m_nPt<<std::endl;
	m_T = new double[m_nPt];
	m_ptList = new double* [m_nDim];
	for(int i=0; i<m_nDim; i++)	{	m_ptList[i] = new double[m_nPt];	}

	// CKim - Read position time file
	ifstr.open(fName);		double x;
	for(int i=0; i<m_nPt; i++)
	{
		ifstr>>x;	m_T[i] = x;	
		for(int j=0; j<m_nDim; j++) { 	ifstr>>x;	m_ptList[j][i] = x;		}
	}

	QueryPerformanceCounter(&m_StartingTime);
	m_idx = 0;
}


bool TrjGenerator::InterpolateNextPoint(double* posDir)
{
	// CKim - Get current time
	double t;		LARGE_INTEGER EndingTime, ElapsedMicroseconds;
	ElapsedMicroseconds.QuadPart = 0.0;

	QueryPerformanceCounter(&EndingTime);	
	ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - m_StartingTime.QuadPart;
	ElapsedMicroseconds.QuadPart *= 1000000;
	t = ElapsedMicroseconds.QuadPart / m_Freq.QuadPart;
	t /= 1000.0;
	m_currT = t;

	// CKim - Determine index and time boundary
	int idx1, idx2;		double t1,t2,pt1,pt2;		bool found = false;
	for(int i = m_nPt; i > m_idx; i--)
	{
		idx1 = (i-1);
		if(t>m_T[idx1])	
		{
			found = true;
			break;
		}
	}

	if(t < m_T[0])	// CKim - Did not reach starting time yet
	{
		for(int i=0; i<m_nDim; i++)	{	posDir[i] = m_ptList[i][0];		}
		return true;
	}
	else if(t > m_T[m_nPt-1])	// CKim - Past the final point
	{
		for(int i=0; i<m_nDim; i++)	{	posDir[i] = m_ptList[i][m_nPt-1];		}
		return false;
	}
	else
	{
		for(int i=idx1; i<m_nPt; i++)
		{
			idx2 = i;
			if(t<m_T[idx2])	{	break;	}
		}
	}

	t1 = m_T[idx1];		t2 = m_T[idx2];		m_idx = idx1;
//	std::cout<<idx1<<"\t"<<idx2<<"\n";

	// CKim - Interpolate 
	if(m_nDim!=6)
	{
		for(int i=0; i<m_nDim; i++)	{
			pt1 = m_ptList[i][idx1];		pt2 = m_ptList[i][idx2];
			posDir[i] = ( pt2*(t-t1) + pt1*(t2-t) ) / (t2-t1);				}
	}

	if(m_nDim==6)	// Assume pos dir combo
	{
		double sum = 0;

		for(int i=0; i<3; i++)	{
			pt1 = m_ptList[i][idx1];		pt2 = m_ptList[i][idx2];
			posDir[i] = ( pt2*(t-t1) + pt1*(t2-t) ) / (t2-t1);				}

		for(int i=3; i<6; i++)	{
			pt1 = m_ptList[i][idx1];		pt2 = m_ptList[i][idx2];
			posDir[i] = ( pt2*(t-t1) + pt1*(t2-t) ) / (t2-t1);				
			sum += (posDir[i]*posDir[i]);
		}

		for(int i=3; i<6; i++)	{	posDir[i] /= sqrt(sum);		}
	}



	return true;
}


bool TrjGenerator::GetNextPointStatic(double* posDir)
{
	if(m_idx == m_nPt)	{	return false;	}

	for(int i=0; i<m_nDim; i++)	{
		posDir[i] = m_ptList[i][m_idx];		}

	m_idx++;
	return true; 
}


bool TrjGenerator::InterpolateNextPoint(double* posDir, double* fwdVel)
{
	// CKim - Get current time
	double t;		LARGE_INTEGER EndingTime, ElapsedMicroseconds;
	ElapsedMicroseconds.QuadPart = 0.0;

	QueryPerformanceCounter(&EndingTime);	
	ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - m_StartingTime.QuadPart;
	ElapsedMicroseconds.QuadPart *= 1000000;
	t = ElapsedMicroseconds.QuadPart / m_Freq.QuadPart;
	t /= 1000.0;
	m_currT = t;

	// CKim - Determine index and time boundary
	int idx1, idx2;		double t1,t2,pt1,pt2;		bool found = false;
	for(int i = m_nPt; i > m_idx; i--)
	{
		idx1 = (i-1);
		if(t>m_T[idx1])	
		{
			found = true;
			break;
		}
	}

	if(t < m_T[0])	// CKim - Did not reach starting time yet
	{
		for(int i=0; i<m_nDim; i++)	{	posDir[i] = m_ptList[i][0];		}
		return true;
	}
	else if(t > m_T[m_nPt-1])	// CKim - Past the final point
	{
		for(int i=0; i<m_nDim; i++)	{	posDir[i] = m_ptList[i][m_nPt-1];		}
		return false;
	}
	else
	{
		for(int i=idx1; i<m_nPt; i++)
		{
			idx2 = i;
			if(t<m_T[idx2])	{	break;	}
		}
	}

	t1 = m_T[idx1];		t2 = m_T[idx2];		m_idx = idx1;
//	std::cout<<idx1<<"\t"<<idx2<<"\n";

	// CKim - Interpolate 
	if(m_nDim!=6)
	{
		for(int i=0; i<m_nDim; i++)	{
			pt1 = m_ptList[i][idx1];		pt2 = m_ptList[i][idx2];
			posDir[i] = ( pt2*(t-t1) + pt1*(t2-t) ) / (t2-t1);				}
	}

	if(m_nDim==6)	// Assume pos dir combo
	{
		double sum = 0;

		for(int i=0; i<3; i++)	{
			pt1 = m_ptList[i][idx1];		pt2 = m_ptList[i][idx2];
			posDir[i] = ( pt2*(t-t1) + pt1*(t2-t) ) / (t2-t1);				}

		for(int i=3; i<6; i++)	{
			pt1 = m_ptList[i][idx1];		pt2 = m_ptList[i][idx2];
			posDir[i] = ( pt2*(t-t1) + pt1*(t2-t) ) / (t2-t1);				
			sum += (posDir[i]*posDir[i]);
		}

		for(int i=3; i<6; i++)	{	posDir[i] /= sqrt(sum);		}

		for(int i=0; i<6; i++)	{
			pt1 = m_ptList[i][idx1];		pt2 = m_ptList[i][idx2];
			fwdVel[i] = (pt2-pt1)/((t2-t1)*0.001);									}
	}



	return true;
}