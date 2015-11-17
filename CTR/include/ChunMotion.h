// ---------------------------------------------------------------- //
// CKim - Class representing the motors of the robot joint.
// Uses Copley Motion library
// Last updated : Oct. 23, 2014 
// ---------------------------------------------------------------- //

// CKim - Set the position. Axis are mapped as follows.
// 0 : balance_pair_Translation (Tube 1 and 2)
// 1 : Inner_translation (Tube 3) + pushes tube forward
// 2 : Forceps_Actuation (Additional tool, currently disconnected)
// 3 : balance_pair_Outer_Rotation
// 4 : balance_pair_Innr_Rotation
// 5 : Inner_Rotation (Tube 3)
// 6 : Forceps_Rotation (Additional tool, currently disconnected)
// rotary 10.936 equals to one turn, 1 translation 3.175mm; (3.175 mm / motor command 1)

#pragma once

#define USE_CAN

// CKim - Header files for Coley Motion Library
#include "CML.h"
#include "can_kvaser.h"
#include "xmitPdo.h"
#include "rcvPdo.h"
#include <string>

// If a namespace has been defined in CML_Settings.h, this macros starts using it. 
CML_NAMESPACE_USE();
#define AMPCT 7

class ChunMotion
{
public:
	ChunMotion(void);
	~ChunMotion(void);

	bool Initialize();
	void PrintMotionError(const char* msg, const CML::Error* err);

	void GetMotorPos(double* cnt);
	void GetErrorFlag(int* flag);

	bool DoCoordMotion(double* p);
	void SetTeleOpMode(bool onoff);
	bool DoTeleOpMotion(double* p);

	void WaitMotionDone();
	void StopMotion();

private:
	
	// CKim - local data
	int32 canBPS;		// CAN network bit rate
	int16 canNodeID;	// CANopen node ID

	// CKim - Create an 'CanInterface' object used to access low level CAN network.
	// KvaserCAN is derived from CanInterface class, implementing the CAN network hardware provided by Kvaser
	KvaserCAN	m_CANHW;

	// CKim - CanOpen class represents CANopen network. It is initialized by calling
	// 'Open()' with CanInterface as a parameter.
	CanOpen		m_CANopenNet;
	   
	// CKim - 'Amp' object represents a Copley Controls amplifier on the CANopen network.
	// The Amp object can be used directly for fairly easy control of an amplifier on the CANopen network.
	// The object provides easy to use methods for setting and getting amplifier parameter blocks. 
	// It also handles many of the details of both point to point moves and the transfer of complex PVT profiles.
	Amp			m_Amp[AMPCT];

	// CKim - Using CanOpen Network Process Data Object enables communication with amps to be 
	// faster then when using Amp's member functions such as Amp::GetMotorPos.... that makes 
	// use of Service Data Objects

	// 1. A CanOpen "Receive Process Data Object" which will be connected to each amps
	// to control commanded velocity
	CML::RPDO	m_velRPDO[AMPCT];		Pmap32		m_velBitMap[AMPCT];

	// 2. "Transmit Process Data Object" which sends the position of the amps. This is derived 
	// from CML::TPDO class. 
	xmitPdo		m_TPDO[AMPCT];
	int			rpdoSlot, tpdoSlot;



	// CKim - 'Linkage' object allows coordinated motion between the motors. 
	// Create a linkage object holding these amps. 
	Linkage		m_LinkedAmps;

	// CKim - Position Limit
	SoftPosLimit	m_Lim[AMPCT];

	// CKim - Motion parameters
	ProfileConfigTrap	m_TrapMotionConfig[AMPCT];			// CKim - Trapezoidal position profile - used for TeleOp
	ProfileConfigScurve m_ScurveMotionConfig;		// CKim - conf

};

