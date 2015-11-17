// ----------------------------------------------------------- //
// CKim - Receive Process Data Object to use in CANopen
// network to stream data to amp. It is 'Receive' from the 
// amp's point of view
// ----------------------------------------------------------- //

#pragma once

#define USE_CAN

// CKim - Header files for Coley Motion Library
#include "CML.h"
#include "CML_PDO.h"
#include "can_kvaser.h"
#include <iostream>
#include <Windows.h>

CML_NAMESPACE_START()

class rcvPdo: public RPDO
{
	// CKim - Pointer to amp to which this Process Data Object is attached to
	class Amp *ampPtr;

	// CKim - PDO mapping defines which varibles (index of the amp's object dictionary)
	// should be broadcasted (transmitted) or written (received). PmapXX class also provides
	// method of accessing the variables for sending and receiving
	Pmap32 cmdPosMap;	// for setting commanded position
	//Pmap16 ctrlBitMap;	// for initiating motion
	//Pmap32 cmdVelMap;	// for programming velocity

public:

	// CKim - Default constructor for this PDO
	rcvPdo()			{	SetRefName( "rcvPdo" ); ampPtr = 0;		}
	virtual ~rcvPdo()	{	KillRef();								}

	// CKim - Initialize and attach to the amp.
	const Error *Init( Amp &amp, uint16 slot );

	// CKim - This function sends the commanded position / velocity over the network in counts
	const Error* SetPos( const int& pos );
	const Error* StartMotion();
	const Error* SendPos( const int& pos, bool stop );
	//const Error* SendVel( int* vel );

private:
	// Private copy constructor (not supported)
	rcvPdo( const rcvPdo& );

	// Private assignment operator (not supported)
	rcvPdo& operator=( const rcvPdo& );

	void PrintMotionError(const char* msg, const CML::Error* err);

};

CML_NAMESPACE_END()