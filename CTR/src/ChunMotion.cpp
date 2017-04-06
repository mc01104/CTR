#include "StdAfx.h"
#include "ChunMotion.h"
#include <fstream>
#include "Utilities.h"

ChunMotion::ChunMotion(void) : m_CANHW("CAN")
{
	// CKim - 'cml' is an Copley Motion Libraries utility object. This object defines a number of handy 
	// methods related to the libraries as a whole. A single global CML object is created by the libraries automatically
	//cml.SetDebugLevel( LOG_EVERYTHING );
	cml.SetDebugLevel( LOG_ERRORS );
	//cml.SetDebugLevel( LOG_WARNINGS);
	//cml.SetDebugLevel( LOG_NONE );
	
	// CKim - Setup CAN hardware and network
	canBPS = 1000000;             // CAN network bit rate
	canNodeID = 1;                // CANopen node ID
	m_CANHW.SetBaud(canBPS);

	// CKim - This slot number may be very important
	//rpdoSlot = 6;	tpdoSlot = 3;
	rpdoSlot = 3;//7;
	tpdoSlot = 6;//4;
}


ChunMotion::~ChunMotion(void)
{
	const CML::Error* err;

	for(int i=0; i<AMPCT; i++ )
	{
		// CKim - Connect PDO objects. Use slot 5 for TPDO and 3 for RPDO
		err = m_Amp[i].PdoDisable(tpdoSlot,m_TPDO[i]);
		//err = m_Amp[i].PdoDisable(rpdoSlot,m_posRPDO[i]);
		err = m_Amp[i].PdoDisable(rpdoSlot,m_velRPDO[i]);
		//err = m_Amp[i].PdoDisable(6,m_ctrlRPDO);
		err = m_Amp[i].Disable();	
		if(err)	{	PrintMotionError("Disabling Amps",err);		}
	}
}

bool ChunMotion::Initialize()
{
	const CML::Error* err = m_CANopenNet.Open(m_CANHW);
	if(err)	{	PrintMotionError("Opening CAN network",err);	return false;	}
	
	// CKim - Setup the amps. 'synchPeriod' (in microseconds) defines the frequency of SYNCH signal
	AmpSettings setting;	setting.guardTime = 0;		
	setting.synchPeriod = 5000;		// default is 10,000 micro sec = 10 ms. 1000 give Generic Can driver level at teleop motion
	MtrInfo mtrInfo;


	//::std::cout << "initial offset" << ::std::endl;
	//PrintCArray(offset, AMPCT);

	for(int i=0; i<AMPCT; i++ )
	{
		// CKim - initialize amp nodes on CANopen network
		err = m_Amp[i].Init( m_CANopenNet, 1+i, setting );
		if(err)	{	PrintMotionError("Initializing Amps",err);	return false;	}

		err = m_Amp[i].SetAmpMode(AMPMODE_CAN_PROFILE);
		if(err)	{	PrintMotionError("setting amp mode",err);	return false;	}

		// CKim - Get the motor info stored in the memory of the amp. How do you set it?
		err = m_Amp[i].GetMtrInfo( mtrInfo );		
		if(err)	{	PrintMotionError("Getting motor info",err);	return false;	}
		
		err = m_Amp[i].SetCountsPerUnit( mtrInfo.ctsPerRev );
		if(err)	{	PrintMotionError("Setting cpr",err);	return false;	}
		
		err= m_Amp[i].SetPositionActual(0);
		//err= m_Amp[i].SetPositionActual(offset[i]);
		//err= m_Amp[i].SetPositionMotor(offset[i]);

		if(err)	{	PrintMotionError("Resetting position",err);	return false;	}
	}

	// CKim - Initialize TPDO
	for(int i=0; i<AMPCT; i++ )
	{
		// CKim - Connect PDO objects. 
		m_TPDO[i].Init(m_Amp[i],tpdoSlot);
	}


	// --------------------------------------------- //
	// CKim - Initialize RPDO to receive 'programmed velocity', index 0X2341 (=9025, type int32)
	for(int i=0; i<AMPCT; i++)
	{
		// CKim - Received data will be applied immediately
		err = m_velRPDO[i].SetType(255);
		if(err)	{	PrintMotionError("velRPDO SetType error : ",err);	}

		// CKim - Set the mapping
		err = m_velBitMap[i].Init(9025,0);
		if(err)	{	PrintMotionError("velRPDO Map Init error : ",err);	}

		err = m_velRPDO[i].AddVar(m_velBitMap[i]);
		if(err)	{	PrintMotionError("velRPDO AddVar error : ",err);		}

		// CKim - Connect PDO objects. Use slot 5 for TPDO and 3 for RPDO
		err = m_Amp[i].PdoSet(rpdoSlot,m_velRPDO[i]);
		if(err)	{	PrintMotionError("velRPDO PdoSet error : ",err);		}
	}
	// --------------------------------------------- //


	// CKim - Homing, for right now, use current position as home position (COPLEY_HOME_METHOD::CHM_NONE)
	CML::HomeConfig cfgHome;
	cfgHome.method = COPLEY_HOME_METHOD::CHM_NONE;
	::std::ifstream f("crashDump.txt");
	CML::uunit offset[AMPCT] = {0};
	::std::string str;
    if(f.good())
	{
		::std::getline(f, str);
		memcpy(offset, DoubleVectorFromString(str).data(), sizeof(double) * AMPCT);
		f.close();
	}
	for (int i = 0; i < AMPCT; ++i)
	{
		cfgHome.offset = m_Amp[i].PosMtr2User(-offset[i]);
		
		m_Amp[i].GoHome(cfgHome);
	}

	::std::remove("crashDump.txt");

	// CKim - Set software limits - Software limit switches are not used until the amplifier has been homed. 
	// Set accel limit to zero to disable accel limit
	for(int i=0; i<AMPCT; i++)	{	m_Lim[i].accel = 0;		}
	m_Lim[0].neg = -100.0/3.175;	m_Lim[1].neg = -100.0/3.175;
	m_Lim[0].pos = 100.0/3.175;		m_Lim[1].pos = 100.0/3.175;
	for(int i=0; i<AMPCT; i++)	{	m_Amp[i].SetSoftLimits(m_Lim[i]);	}
	
	// CKim - Link all amps
	err = m_LinkedAmps.Init(AMPCT, m_Amp);
	if(err)	{	PrintMotionError("Linkage init",err);	return false;	}
	
	err = m_LinkedAmps.SetMoveLimits( 10, 100, 100, 100 );
	//err = m_LinkedAmps.SetMoveLimits( 1, 100, 100, 100 );
	//err = m_LinkedAmps.SetMoveLimits( 10000, 100000, 100000, 100000 );
	if(err)	{	PrintMotionError("Set limit",err);	return false;	}

	return true;

}


void ChunMotion::PrintMotionError(const char* msg, const CML::Error* err)
{
	CString errStr;
	errStr.Format("Motion Error. %s : %s",msg,err->toString());
	AfxMessageBox(errStr);
	this->DumpConfiguration();

}


void ChunMotion::SetTeleOpMode(bool onoff)
{
	// CKim - Coordinated motion using Linkage object automatically puts amps in 'AMPMODE_CAN_PVT' mode. 
	// This has to be explicitly changed to 'Programmed Velocity' mode for teleoperation.
	// This change is veeerrryyy important. Costed me a day to figure out. 
	
	const Error* err;
	if(onoff)
	{
		for(int i=0; i<AMPCT; i++ )
		{
			//err = m_Amp[i].SetAmpMode(AMPMODE_CAN_PROFILE);
			err = m_Amp[i].SetAmpMode(AMPMODE_PROG_VEL);
			if(err)	{	PrintMotionError("setting amp mode",err);	}
		}
	}
	else
	{
		for(int i=0; i<AMPCT; i++ )
		{
			err = m_Amp[i].SetAmpMode(AMPMODE_CAN_PROFILE);
			if(err)	{	PrintMotionError("setting amp mode",err);	}
		}
	}
}


bool ChunMotion::DoCoordMotion(double* p)
{
	const CML::Error* err;		Point<7> pos;
	for(int i=0; i<7; i++)	{	pos[i] = p[i];	}
	
	
	//err = m_LinkedAmps.MoveTo(pos);
	double vel = 120 * 10.9244/3.0;
	err = m_LinkedAmps.MoveTo(pos, vel, 10 * vel, 10 * vel, 100 * vel);

	this->WaitMotionDone();
	if(err)	{	PrintMotionError("Linkage start move",err);	return false;	}
	else	{	return true;	}
}


void ChunMotion::WaitMotionDone()
{
	m_LinkedAmps.WaitMoveDone();
}


void ChunMotion::DumpConfiguration()
{
	::std::ofstream os("crashDump.txt");
	int x = 0;
	for (int i = 0; i < AMPCT; ++i)
	{
		m_TPDO[i].GetPos(x);
		os << x << " ";
	}
	os << ::std::endl;
}


void ChunMotion::GetMotorPos(double* cnt)
{
	int x;

	// CKim - Use PDO. Static event handle m_hEvent is triggered by any of xmitPdos
	// upon update of the data
	DWORD res = WaitForSingleObject(xmitPdo::m_hEvent,1000);
	
	if(res == WAIT_OBJECT_0)	
	{
		for(int i=0; i<AMPCT; i++ )
		{
			m_TPDO[i].GetPos(x);
			cnt[i] = m_Amp[i].PosMtr2User(x);
		}
	}
	else	
	{	
		AfxMessageBox("GetPos time out!");		
		this->DumpConfiguration();
	}


	//// CKim - Use PDO
	//int x;
	//for(int i=0; i<AMPCT; i++ )
	//{
	//	m_TPDO[i].GetPos(x);
	//	cnt[i] = m_Amp[i].PosMtr2User(x);
	//}

	//// CKim - Using Amp's member function
	//for(int i=0; i<AMPCT; i++ )
	//{
	//	const CML::Error* err = m_Amp[i].GetPositionActual(cnt[i]);
	//}
}


void ChunMotion::GetErrorFlag(int* flag)
{
	const CML::Error* err;
	
	for(int i=0; i<7; i++)	
	{
		err = m_Amp[i].GetErrorStatus();	// CKim - This is done using PDO so no slow down...
		if(err)
		{
			flag[i] = err->GetID();		
			TRACE("Error %d : %s\n",i,err->toString());
			m_Amp[i].HaltMove();
		}
		else	{	flag[i] = 0;				}
	}
}


bool ChunMotion::DoTeleOpMotion(double* p)
{
	// CKim - Send Commanded velocity to each amps
	const CML::Error* err;		int vcnt;		int errcnt = 0;

	for(int i=0; i<AMPCT; i++)
	{
		vcnt = m_Amp[i].VelUser2Load(p[i]);
		m_velBitMap[i].Set((CML::byte*) &vcnt);	
		err = m_velRPDO[i].Transmit(m_CANopenNet);
		if(err)
		{
			errcnt++;	
			cml.Error("TeleOp Error amp %d : %s\n",i, err->toString());	
		}
	}

	return true;
	//if(errcnt!=0)	
	//{
	//	for(int i=0; i<AMPCT; i++)
	//	{
	//		err = m_Amp[i].SetAmpMode(AMPMODE_CAN_PROFILE);
	//		m_Amp[i].HaltMove();
	//	}
	//	return false;
	//}
	//else	{	return true;	}
}


void ChunMotion::StopMotion()
{
	for(int i=0; i<AMPCT; i++)
	{
		m_Amp[i].SetVelocityProgrammed(0);
	}
}
