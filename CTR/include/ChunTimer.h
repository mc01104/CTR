// -------------------------------------------------------- //
// CKim - Simple class for measuring time
// -------------------------------------------------------- //

#pragma once
//#include "stdafx.h"

class ChunTimer
{
public:
	ChunTimer(void);
	~ChunTimer(void);

	// CKim - Returns time elapsed since last reset
	long GetTime();

	// CKim - Resets the timer
	void ResetTime();

private:

	// CKim - Parameters for loop speed measurement
	LARGE_INTEGER	StartingTime;
	LARGE_INTEGER	CurrentTime;
	LARGE_INTEGER	ElapsedMicroseconds;
	LARGE_INTEGER	Frequency;

};

