#include "StdAfx.h"
#include "xmitPdo.h"
#include <iostream>


CML_NAMESPACE_START()

HANDLE			xmitPdo::m_hEvent = NULL;

void xmitPdo::PrintMotionError(const char* msg, const CML::Error* err)
{
	CString errStr;
	errStr.Format("Motion Error. %s : %s",msg,err->toString());
	AfxMessageBox(errStr);
}

const Error* xmitPdo::Init( Amp &amp, uint16 slot )
{
	// CKim - Initialize synchronization object
	InitializeCriticalSectionAndSpinCount(&m_cSection,16);
	if(!m_hEvent)  {	m_hEvent = CreateEvent(NULL,false,0,"eve");	}

	// CKim - Init performnce variable	
	QueryPerformanceFrequency(&m_Freq);		m_perfcnt = 0;
	
	ampPtr = &amp;

	// CKim - Configure this'Transmission Process Data Object' which will be added to the
	// amp 'Node' and 'transmits' (broadcasts) it position data to other nodes.

	// 1. Set the type (0-255) of transmission. Set it to N and then the PDO is transmitted 
	// every N SYNC messages. For example, a PDO with type code 7 would be transmitted on every
	// 7th SYNC message. set it to 254 and 255 and it will be transmitted upon event. See manual p36.
	const CML::Error *err = SetType(1);		// Send on every SYNC message

	if(err)	{	PrintMotionError("xmitPdo SetType error : ", err);	}

	// 2. Add 'Process Data Map' which instructs node what data will be broadcasted over this PDO.
	// Construct map object with the 'index, subindex and the size' of the object to be transmitted 
	// and add it to PDO. Single PDO can carry up to 8 byte of data and thus you can add as many 
	// map object as you wish as long as the total size of the transmitted data is below 8 byte.
	
	// CKim - Index of the 'Position' is 0x6064 and it is 32 bit integer (4 byte). See manual p137
	err = m_Pos.Init(24676,0);	// 0x6064 = 24676
	//err = m_Pos.Init(8513,0);	// 0x2141 = 8531 system time
	if(err)	{	PrintMotionError("xmitPdo Map Init error : ",err);	}

	err = AddVar(m_Pos);

	if(err)	{	std::cout<<"xmitPdo AddVar error : "<<err->toString()<<std::endl;	}

	// CKim - Add this TPDO to the Amp node
	err = ampPtr->PdoSet(slot,*this);
	
	return err;
}


void xmitPdo::Received()
{
//	if(m_perfcnt == 0)	{	QueryPerformanceCounter(&m_StartT);		}

//	EnterCriticalSection(&m_cSection);
	m_cnt = m_Pos.Read();
//	LeaveCriticalSection(&m_cSection);
	SetEvent(m_hEvent);

	//if(m_perfcnt == 1000)	
	//{
	//	QueryPerformanceCounter(&m_EndT);
	//	m_perfcnt = 0;	
	//	
	//	LARGE_INTEGER ElapsedMicroseconds;
	//	ElapsedMicroseconds.QuadPart = m_EndT.QuadPart - m_StartT.QuadPart;
	//	ElapsedMicroseconds.QuadPart *= 1000000;
	//	ElapsedMicroseconds.QuadPart /= m_Freq.QuadPart;
	//	m_xmitFreq = 1000000.0/ElapsedMicroseconds.QuadPart;
	//	m_xmitFreq*=1000.0;
	//}
	//else
	//{
	//	m_perfcnt++;
	//}

}

void xmitPdo::GetPos(int& x)
{
//	DWORD res = WaitForSingleObject(m_hEvent,1000);
	
//	if(res == WAIT_OBJECT_0)	
//	{
//		EnterCriticalSection(&m_cSection);
		x = m_cnt;		
		//LeaveCriticalSection(&m_cSection);
//	}
//	else	{	AfxMessageBox("GetPos time out!");		}
	
}

CML_NAMESPACE_END()