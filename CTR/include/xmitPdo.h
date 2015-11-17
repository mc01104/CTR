// ---------------------------------------------------------- //
// CKim - My own imlementation of CANopen Process Data Object
// using Copley Library. This is used to stream the position
// from the amps faster than what you can with default 
// Amp::GetPosition() function of the Copley library 
// This is 'transmission' from the amp's point of view
// ---------------------------------------------------------- //

#pragma once

#define USE_CAN

// CKim - Header files for Coley Motion Library
#include "CML.h"
#include "CML_PDO.h"
#include "can_kvaser.h"
#include <iostream>
#include <Windows.h>

CML_NAMESPACE_START()

class xmitPdo: public TPDO
{
	// CKim - Pointer to amp to which this Process Data Object is attached to
	class Amp *ampPtr;

	// CKim - Variable for storing received data.
	int m_cnt;

	// CKim - Synchronization objects
	CRITICAL_SECTION	m_cSection;		// CKim - Critical Section is used instead of mutex for synchroniation between threads. slightly fatser than mutexes

	// CKim - Performance measurement
	LARGE_INTEGER m_Freq;		LARGE_INTEGER m_StartT, m_EndT;		int m_perfcnt;		double m_xmitFreq;

public:
	// CKim - PDO mapping defines which varibles (index of the amp's object dictionary)
	// should be broadcasted (transmitted) or written (received). PmapXX class also provides
	// method of accessing the variables for sending and receiving
	Pmap32	m_Pos;	// for reciving position transmitted from the amp

	// CKim - Default constructor for this PDO
	xmitPdo()				{	 SetRefName( "xmitPdo" ); ampPtr=0;	}
	virtual ~xmitPdo()		{	 KillRef();							}
   
   	// CKim - Set up the PDO and connect it to the amp node
	const Error *Init( Amp &amp, uint16 slot );

	// CKim - This virtual function is called by network when the data is received from
	// the connected amp node. Implement your reaction here
	virtual void Received( void );

	// CKim - Get the copy of the position
	void GetPos(int& x);


	static HANDLE		m_hEvent;


private:
   
	// Private copy constructor (not supported)
	xmitPdo( const xmitPdo& );

	// Private assignment operator (not supported)
	xmitPdo& operator=( const xmitPdo& );

	void PrintMotionError(const char* msg, const CML::Error* err);
};

CML_NAMESPACE_END()