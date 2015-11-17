#include "StdAfx.h"
#include "rcvPdo.h"

CML_NAMESPACE_START()

void rcvPdo::PrintMotionError(const char* msg, const CML::Error* err)
{
	CString errStr;
	errStr.Format("Motion Error. %s : %s",msg,err->toString());
	AfxMessageBox(errStr);
}


const Error* rcvPdo::Init( Amp &amp, uint16 slot )
{
	ampPtr = &amp;

	const Error *err = 0;

   	// CKim - Configure this'Receive Process Data Object' which will be added to the
	// amp 'Node' and 'transmits' (broadcasts) it position data to other nodes.

	// 1. Set the type (0-255) of transmission. They define the response of the node upon 
	// the reception of the data. If (0-240) received data is held until the next SYNC message.
	// When the SYNC message is received the data is applied. If (254-255)
	// The received data is applied to its mapped objects immediately upon reception.
	err = SetType(255);		// Received data is applied immediately

	if(err)	{	PrintMotionError("rcvPdo SetType error : ",err);	}

	// 2. Add 'Process Data Map' which instructs node what is the data that is received over this PDO
	// Construct map object with the 'index, subindex and the size' of the object to be received
	// and add it to PDO. 

	// CKim - Write to index 0x607A (=24698, type int32) is set point position adjust by DoMove()
	// Index 0x6040 (=24640, type int16) is a control bit which should be cleared to 
	// start motion. See manual p 58 and 202.
	//err = ctrlBitMap.Init(24640,0);	
	err = cmdPosMap.Init(24698,0);		

	// CKim - write to index 0X2341 (=9025, type int32) which is commanded velocity in counts. See manual p146
	//err = cmdVelMap.Init(9025,0);

	if(err)	{	PrintMotionError("rcvPdo Map Init error : ",err);	}
	
	// CKim - Map the PDO and add this RPDO to the Amp node
//	err = AddVar(ctrlBitMap);
	err = AddVar(cmdPosMap);	
	//err = AddVar(cmdVelMap);

	if(err)	{	PrintMotionError("rcvPdo AddVar error : ",err);		}

	err = ampPtr->PdoSet(slot,*this);
	if(err)	{	PrintMotionError("rcvPdo PdoSet error : ",err);		}

   return 0;
}


const Error* rcvPdo::SetPos(const int& pos)
{
	// CKim - Copy the commanded position
	int p = pos;	cmdPosMap.Set((byte*) &p);

	//// CKim - Control Bit is 16 bit binary. Transition of bit 4 (zero base) from 0 to 1
	//// initiates motion to commanded position. Bit 5 must be 1 so that the position will be 
	//// immediately updated. See manual p58 and 196.
	//int x = 0x000F;		// 0000 0000 000'0' 1111
	//ctrlBitMap.Set((byte*) &x);
   
	RefObjLocker<Network> net( ampPtr->GetNetworkRef() );
	if( !net ) return &NodeError::NetworkUnavailable;
	
	return RPDO::Transmit( *net );
}


const Error* rcvPdo::StartMotion()
{
	//// CKim - Control Bit is 16 bit binary. Transition of bit 4 (zero base) from 0 to 1
	//// initiates motion to commanded position. Bit 5 must be 1 so that the position will be 
	//// immediately updated. See manual p58 and 196.
	//int x = 0x003F;		// 0000 0000 001'1' 1111
	//ctrlBitMap.Set((byte*) &x);
	   
	//RefObjLocker<Network> net( ampPtr->GetNetworkRef() );
	//if( !net )
	//{
	//	return &NodeError::NetworkUnavailable;
	//}
	//return RPDO::Transmit( *net );
	return 0;
}





//const Error* rcvPdo::SendVel( int* vel )
//{
//   cmdVelMap.Set((byte*) vel);	//dat.Set(data);
//
//   RefObjLocker<Network> net( ampPtr->GetNetworkRef() );
//   if( !net ) return &NodeError::NetworkUnavailable;
//
//   return RPDO::Transmit( *net );
//}

CML_NAMESPACE_END()