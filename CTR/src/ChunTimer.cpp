#include "stdafx.h"
#include "ChunTimer.h"


ChunTimer::ChunTimer(void)
{
	ElapsedMicroseconds.QuadPart = 0.0;
	QueryPerformanceFrequency(&Frequency); 
	int perfcnt = 0;	int navg = 50;		long loopTime = 0;
	
	QueryPerformanceCounter(&StartingTime);
}


ChunTimer::~ChunTimer(void)
{
}

long ChunTimer::GetTime()
{
	// CKim - Calculate the difference. 
	QueryPerformanceCounter(&CurrentTime);	
	ElapsedMicroseconds.QuadPart = CurrentTime.QuadPart - StartingTime.QuadPart;

	// CKim - Following calculation gives time in microseconds
	ElapsedMicroseconds.QuadPart *= 1000000;
	ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
					
	return ElapsedMicroseconds.QuadPart;
}


void ChunTimer::ResetTime()
{
	QueryPerformanceCounter(&StartingTime);
}