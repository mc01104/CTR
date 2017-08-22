
// CTRDoc.cpp : implementation of the CCTRDoc class
//

// #pragma comment (lib, "Mswsock.lib")


#include "stdafx.h"
#include "LieGroup.h"
#include <Eigen/Geometry> 
#include <time.h>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment (lib, "Ws2_32.lib")

// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "CTRApp.h"
#endif

#include "CTRDoc.h"
#include "ChunHaptic.h"
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduHapticDevice.h>
#include "ChunMotion.h"
#include "CTRKin.h"
#include "LWPRKinematics.h"
#include "ChunTracker.h"
#include "TrjGenerator.h"
#include "ChunTimer.h"

#include <propkey.h>
#include <fstream>
#include <math.h>
#include <sstream>
#include "VtkOnLinePlot.h"

// CKim - Eigen Header. Located at "C:\Chun\ChunLib"
#include <Eigen/Dense>
#include "Utilities.h"
#include "FilterLibrary.h"
#include "MechanicsBasedKinematics.h"
#include "CTRFactory.h"

// Heart Rate Monitoring from surgivet
#include "HeartRateMonitor.h"
#include "resource.h"

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"
#define WIN32_LEAN_AND_MEAN

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define OMNI_PLUGGED 
// CCTRDoc

IMPLEMENT_DYNCREATE(CCTRDoc, CDocument)

BEGIN_MESSAGE_MAP(CCTRDoc, CDocument)
	ON_COMMAND(ID_VIEW_STATUS_BAR, &CCTRDoc::OnViewStatusBar)
	ON_COMMAND(ID_VIEW_TELEOP, &CCTRDoc::OnViewTeleop)
	ON_COMMAND(ID_VIEW_PLOT, &CCTRDoc::OnViewPlot)

	ON_UPDATE_COMMAND_UI(ID_VIEW_PLOT, &CCTRDoc::OnUpdateViewPlot)
	ON_UPDATE_COMMAND_UI(ID_VIEW_TELEOP, &CCTRDoc::OnUpdateViewTeleop)

	ON_BN_CLICKED(IDC_INIT_EM, &CCTRDoc::OnBnClickedInitEm)
	ON_BN_CLICKED(IDC_REGST, &CCTRDoc::OnBnClickedRegst)
	ON_BN_CLICKED(IDC_BUTTON11, &CCTRDoc::OnBnClickedGoToApex)

	ON_BN_CLICKED(IDC_BTN_MOVE12, &CCTRDoc::OnBnClickedGoToFirst)
	ON_BN_CLICKED(IDC_BTN_MOVE13, &CCTRDoc::OnBnClickedGoToPrev)
	ON_BN_CLICKED(IDC_BTN_MOVE14, &CCTRDoc::OnBnClickedGoToLast)
	ON_BN_CLICKED(IDC_BTN_MOVE15, &CCTRDoc::OnBnClickedGoToNext)

	ON_BN_CLICKED(IDC_BTN_MOVE18, &CCTRDoc::OnBnClickedStartId)
	ON_BN_CLICKED(IDC_BTN_MOVE17, &CCTRDoc::OnBnClickedStopId)

	ON_BN_CLICKED(IDC_BTN_MOVE16, &CCTRDoc::OnBnClickedResetAutomation)


END_MESSAGE_MAP()


// CKim - Static variables
// Rotary 10.936 equals to one turn, 1 translation 3.175mm;
CRITICAL_SECTION	CCTRDoc::m_cSection;
double CCTRDoc::c_PI = acos(-1.0);
double CCTRDoc::c_CntToRad = -(2*c_PI)/(10.936 * 0.998888 * 1.000049260526897);
double CCTRDoc::c_CntToMM = 3.175;

// CCTRDoc construction/destruction

CCTRDoc::CCTRDoc()
{
	m_contact_response = false;
	// TODO: add one-time construction code here
	m_ioRunning = false;		m_teleOpMode = false;
	m_frequency_changed = false;
	m_control_mode = 0;
	m_freq_mode = 1;

	m_timer = new ChunTimer();
	m_radius = 10;
	m_position_gain = 1.0;
	m_orientation_gain = 1.0;
	m_position_gain_feedforward = 1.0;
	m_orientation_gain_feedforward = 1.0;

	for (int i = 0; i < 3; ++i)
		this->m_Status.tgtWorkspaceVelocity[i] = 0;
	m_date = GetDateString();
	m_compute_plane = false;
	m_camera_control = false;
	m_contact_control_normal << 0, 0, 1;
	m_logData = false;
	filters = new RecursiveFilter::MovingAverageFilter[3];

	this->cr_dot_filter = new RecursiveFilter::MovingAverageFilter(11);

	m_heartRateMonitor = new HeartRateMonitor();
	m_contactError = 0;
	m_contactError_prev = 0;
	for (int i = 0; i < 3; ++i)
		m_valve_center[i] = 0.0;

	// CKim - Initialize critical section
	// Initializes a critical section object and sets the spin count for the critical section.
	// When a thread tries to acquire a critical section that is locked, the thread spins: 
	// it enters a loop which iterates spin count times, checking to see if the lock is released. 
	// If the lock is not released before the loop finishes, the thread goes to sleep to wait for the lock to be released.
	InitializeCriticalSectionAndSpinCount(&m_cSection,16);

	// CKim - Initialize Haptic device
#ifdef OMNI_PLUGGED
	ChunHaptic::InitDevice();		m_Omni = new ChunHaptic();		m_Omni->StartLoop();
#endif

	// CKim - Initialize motor controller	
	m_motionCtrl = new ChunMotion();		m_motorConnected = false;
	m_motorConnected = m_motionCtrl->Initialize();
	cameraControlFlag = false;
	m_kinLib = new CTRKin;
	m_ref_set = false;
	// paths for LWPR models (TIP AND BALANCED PAIR)
	//::std::string pathToForwardModel("../models/model_ct_2016_4_6_17_25_29TIP.bin");
	::std::string pathToForwardModel("../models/lwpr_forward_2016_11_17_16_14_55.bin");
	::std::string pathToForwardModelBP("../models/model_ct_2016_4_7_14_48_43BP.bin");
	robot = CTRFactory::buildCTR("");
	kinematics = new MechanicsBasedKinematics(robot, 100);
	m_forceControlActivated = false;
	try
	{
		m_kinLWPR = new LWPRKinematics(pathToForwardModel);
		m_kinLWPR_BP = new LWPRKinematics(pathToForwardModelBP);
	}
	catch(LWPR_Exception& e)
	{
		::std::cout << ::std::string(e.getString()) << ::std::endl;
	}

	m_Tracker = new ChunTracker;
	m_TrjGen = new TrjGenerator;
	
	m_vtkPlot = NULL;

	m_hWndView = NULL;
	m_AdaptiveOn = false;
	m_FeedbackOn = false;
	m_InvKinOn = false;
	m_RegDataCollecting = false;
	m_playBack = false;
	m_bLogEMData = false;
	m_bStaticPlayBack = false;
	m_bRunExperiment = false;
	m_bCLIK = false;
	m_bDoUpdate = false;
	m_jointPlayback = false;
	m_adapt_LWPR = false;
	m_plotData = false;
	m_ContactUpdateReceived = false;
	m_contactGain = 0.0;
	m_contactDGain = 0.0;
	m_contactIGain = 0.0;

	m_contact_error_integral = 0.0;

	m_contactRatio = 0.0;
	m_contactRatioDesired = 0.0;

	m_fileStream = NULL;

	m_plane_coefficients(0) = 0.01;
	m_plane_coefficients(1) = 0.02;

	m_plane_covar = 0.01 * ::Eigen::Matrix<double, 2, 2>::Identity();
	m_plane_changed =false;
	m_Status.isTeleOpMoving = false;

	// visual servoing
	m_image_center(0) = 250 * 0.5;
	m_image_center(1) = 250 * 0.5;
	m_scaling_factor = 26.27;
	m_circumnavigation = false;

	m_valve_tangent_prev[0] = 0;
	m_valve_tangent_prev[1] = -1;

	m_valve_tangent[0] = 0;
	m_valve_tangent[1] = 0;

	m_line_detected = false;

	this->m_gain_center = 0.3;
	this->m_gain_tangent = 0.3;

	m_apex = false;
	for (int i = 0; i < 5; ++i)
		m_apex_coordinates[i] = 0;

	m_wall_detected = false;

	m_centroid_apex[0] = 0; 
	m_centroid_apex[1] = 0;

	this->m_apex_to_valve = false;
	m_useJacobianContactControl = false;

	periodsForCRComputation = 1;

	m_apex_theshold_min = 20;
	m_apex_theshold_max = 100;
	m_center_gainATV = 5.0/this->m_scaling_factor;
	m_forward_gainATV = 2.0;

	m_globalCR_gain = 1.0;

	aStatus = APEX_TO_VALVE_STATUS::LEFT;
	cStatus = CIRCUM_STATUS::LEFT_A;

	storeValvePoint = false;
	index = 0;
	switchToCircum = false;

	num_of_sins = 2;
	min_frequency = 0;
	max_frequency = 0;
	amplitude = 0;
	activateIdentification = false;

	//reset with play button
	timerId.ResetTime();
	reference_translation = 0;
	reference_translation_inner = 0;
	m_idMode = false;

	m_bias = 0;
	tangent_updates = 0;

	m_disturbance_amp = 0;

	centroid_velocity[0] = 0;
	centroid_velocity[1] = 0;
	tip_velocity[0] = 0;
	tip_velocity[1] = 0;

	centroid_prev[0] = 0;
	centroid_prev[1] = 0;

	tip_position_prev[0] = 0;
	tip_position_prev[1] = 0;

	retractRobot = false;
}

CCTRDoc::~CCTRDoc()
{
	m_ioRunning = false;		

	if( WaitForSingleObject(m_hTeleOpThread,1000) )	// CKim - Did not return 0
	{
		AfxMessageBox("Sparta!! IO thread");
	}

	DeleteCriticalSection(&m_cSection);

	delete robot;
	delete kinematics;
	delete m_kinLWPR;
	
}

void CCTRDoc::ChangeForceForTuning(double force)
{
	//this->m_Omni->SetForce(force);
	//std::cout << "set force in Doc" << force << ::std::endl;
	//m_force = force;
}

void CCTRDoc::SetForceGain(double forceGain, double forceGainD, double forceGainI)
{
	//this->m_kinLib->SetForceGain(forceGain);
	m_contactGain = forceGain;
	m_contactDGain = forceGainD;
	m_contactIGain = forceGainI;
}

void CCTRDoc::SetContactRatio(double ratio)
{
	m_contactRatioDesired = ratio;
	this->resetIntegral();
	::std::cout << "Contact Ratio was set to:" << m_contactRatioDesired << ::std::endl;
}

void CCTRDoc::UpdateDesiredPosition()
{
	double targetTmp[6];
	//EnterCriticalSection(&m_cSection);
	//memcpy(targetTmp, this->m_Status.tgtTipPosDir, 6 * sizeof(double));
	//LeaveCriticalSection(&m_cSection);

	//::std::cout << "initial target:" << targetTmp[2] << ::std::endl;

	//if (m_forceControlActivated)
	this->ComputeDesiredPosition(targetTmp);

	//::std::cout << "updated target:" << targetTmp[2] << ::std::endl;

	memcpy(m_desiredPosition, targetTmp, 6 * sizeof(double));
	//PrintCArray(m_desiredPosition, 6);
}

void CCTRDoc::ComputeDesiredVelocity()
{
	double contact_error_deriv = 0;
	if (m_deltaT > 0)
		contact_error_deriv = this->cr_dot_filter->step((m_contactError - m_contactError_prev)/m_deltaT);

	// this is the PD controller -> need to correctly propagate this goal to the position controller


	::Eigen::Vector3d local_vel = this->m_contact_control_normal *  (m_contactGain * m_contactError + m_contactDGain * contact_error_deriv + m_contactIGain * m_contact_error_integral);

	if (m_useJacobianContactControl)
		local_vel *= this->m_globalCR_gain * this->computeInverseApproxJacovianCR(this->m_contactRatio);

	//::std::cout << local_vel.transpose() << ::std::endl;
	memcpy(m_desiredPosition, local_vel.data(), 3 * sizeof(double));

	m_ContactUpdateReceived = !m_ContactUpdateReceived;
}

void CCTRDoc::ComputeDesiredPosition(double tmpPosition[6])
{
	//if(!m_ContactUpdateReceived)
	//{
	//	tmpPosition[2] = m_desiredPosition[2];
	//	return;
	//}

	//
	//m_deltaT = 1.0/40.0; // CHANGE to be computed by the network thread
	//// Implement in a more general way
	//tmpPosition[2] = m_contactGain * m_contactError * m_deltaT + m_desiredPosition[2];

	if(!m_ContactUpdateReceived)
	{
		memcpy(tmpPosition, m_desiredPosition, 3 * sizeof(double));
		return;
	}

	
	//m_deltaT = 1.0/40.0; // CHANGE to be computed by the network thread
	// Implement in a more general way
	
	double contact_error_deriv = 0;
	if (m_deltaT > 0)
		contact_error_deriv = (m_contactError - m_contactError_prev)/m_deltaT;

	// this is the PD controller -> need to correctly propagate this goal to the position controller
	::Eigen::Vector3d local_vel = this->m_contact_control_normal *  (m_contactGain * m_contactError - m_contactDGain * contact_error_deriv);
	
	::Eigen::Vector3d tmp = 100 * this->m_contact_control_normal *  m_contactGain * m_contactError * m_deltaT /*+ ::Eigen::Map<::Eigen::Vector3d> (m_desiredPosition,3)*/;
	memcpy(tmpPosition, tmp.data(), 3 * sizeof(double));

	m_ContactUpdateReceived = !m_ContactUpdateReceived;
}

void CCTRDoc::SaveModel()
{
	dynamic_cast<LWPRKinematics*> (this->m_kinLWPR)->SaveModel();
}

BOOL CCTRDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	return TRUE;
}


void CCTRDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}

#ifdef SHARED_HANDLERS

// Support for thumbnails
void CCTRDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// Modify this code to draw the document's data
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// Support for Search Handlers
void CCTRDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// Set search contents from document's data. 
	// The content parts should be separated by ";"

	// For example:  strSearchContent = _T("point;rectangle;circle;ole object;");
	SetSearchContent(strSearchContent);
}

void CCTRDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CCTRDoc diagnostics

#ifdef _DEBUG
void CCTRDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CCTRDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


void CCTRDoc::OnViewStatusBar()
{
}

void CCTRDoc::OnViewPlot()
{
	
	if(!m_vtkPlot)
	{
		// CKim - This creates modeless dialog using ChunVtkDlg class
		m_vtkPlot = new VtkOnLinePlot();
		m_vtkPlot->Create(VtkOnLinePlot::IDD);		
		
		// CKim - Move to (640,480) and don't change size. Show window and draw
		m_vtkPlot->SetWindowPos(&CWnd::wndTop,640,200,0,0,SWP_NOSIZE);		
		m_vtkPlot->ShowWindow(SW_SHOW);
		m_vtkPlot->Invalidate();
		m_plotData = true;
	}
	else
	{
		m_vtkPlot->DestroyWindow();		delete m_vtkPlot;		m_vtkPlot = NULL;
		m_plotData = false;
	}
}


void CCTRDoc::OnUpdateViewPlot(CCmdUI *pCmdUI)
{
	if(m_plotData)		{	pCmdUI->SetCheck(1);	}
	else				{	pCmdUI->SetCheck(0);	}
}

void CCTRDoc::OnViewTeleop()
{
	// CKim - Start TeleOp thread
	if(!m_ioRunning)	
	{
		if(m_motorConnected)	{												}
		else					
		{	
			AfxMessageBox("Motor not ready!");	
			//_beginthreadex(NULL, 0, CCTRDoc::NetworkCommunication, this, 0, NULL);

			return;	
		}
		m_ioRunning = true; 	
		m_hMtrCtrl = (HANDLE) _beginthreadex(NULL, 0, CCTRDoc::MotorLoop, this, 0, NULL);
		_beginthreadex(NULL, 0, CCTRDoc::NetworkCommunication, this, 0, NULL);
		_beginthreadex(NULL, 0, CCTRDoc::HeartRateMonitorThread, this, 0, NULL);
	}
}


void CCTRDoc::OnUpdateViewTeleop(CCmdUI *pCmdUI)
{
	if(m_ioRunning)	{	pCmdUI->SetCheck(1);	}
	else				{	pCmdUI->SetCheck(0);	}
}


void CCTRDoc::OnBnClickedInitEm()
{
	m_hEMTrck = (HANDLE) _beginthreadex(NULL, 0, CCTRDoc::EMLoop, this, 0, NULL);
}

unsigned int WINAPI	CCTRDoc::HeartRateMonitorThread(void* para)
{
	CCTRDoc* mySelf = (CCTRDoc*) para;

	mySelf->m_heartRateMonitor->run();

	return 0;
}

unsigned int WINAPI	CCTRDoc::NetworkCommunication(void* para)
{
	CCTRDoc* mySelf = (CCTRDoc*) para;	
	CTR_status	localStat;
	localStat.isTeleOpMoving = false;
	mySelf->m_timer->ResetTime();
	WSADATA wsaData;
    int iResult;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    int iSendResult;
    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;


    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }
	::std::cout << "in network" << ::std::endl;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket // this conflicts with using namespace std in LieGroup -> FIX IT!
    iResult = ::bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // No longer need server socket
    closesocket(ListenSocket);

	bool teleopOn = false;
	double force = 0.0;
	double desiredContactRatio = 0.0;
	double contactRatio = 0.0;
	double contactRatioError = 0.0;
	// Receive until the peer shuts down the connection
	clock_t start_loop, end_loop;
	start_loop = clock();

	//EnterCriticalSection(&m_cSection);
	//::std::ofstream* os;
	//LeaveCriticalSection(&m_cSection);
	double delta_t = 0;
	int counter = 0;
    do {
		::std::ostringstream ss;
		// update the local joint variables
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<5; i++)
			localStat.currJang[i] = mySelf->m_Status.currJang[i];		
		for(int i = 0; i < 6; i++)
			localStat.currTipPosDir[i] = mySelf->m_Status.currTipPosDir[i];
		teleopOn = mySelf->m_Status.isTeleOpMoving;
		desiredContactRatio = mySelf->m_contactRatioDesired;

		for(int i = 0; i < 6; i++)
			localStat.tgtTipPosDir[i] = mySelf->m_Status.tgtTipPosDir[i];

		LeaveCriticalSection(&m_cSection);
		//::std::cout << desiredContactRatio << ::std::endl;

		//update the buffer
		// first send the joint angles to be used for the image rotation and logging
		for(int i = 0; i < 5; ++i)
			ss << localStat.currJang[i] << " ";
		// send whether teleoperation is on/off
		ss << teleopOn << " ";

		// send the frequency from the monitor
		if (mySelf->m_freq_mode == 1)
			ss << mySelf->m_heartRateMonitor->getHeartRate() << " ";
		else
			ss << mySelf->m_frequency << " ";

		// target tip position/orientation
		for (int i = 0; i < 3; ++i)
			ss << localStat.tgtTipPosDir[i] << " ";

		if (mySelf->m_circumnavigation)
			ss << 1 << " ";
		else 
			ss << 0 << " ";

		if (mySelf->m_apex_to_valve)
			ss << 1 << " ";
		else 
			ss << 0 << " ";

		//::std::cout << mySelf->periodsForCRComputation << ::std::endl;
		ss << mySelf->periodsForCRComputation << " ";

		for(int i = 0; i < 3; ++i)
			ss << localStat.currTipPosDir[i] << " ";

		ss << mySelf->m_contactGain << " " << mySelf->m_contactDGain << " " << mySelf->m_contactIGain << " " << mySelf->m_forceControlActivated << " " << mySelf->m_contactRatioDesired << " ";

		ss << mySelf->GetMonitorBreathingFreq() << " ";

		if (mySelf->m_plane_changed)
		{
			ss << " " << 1 << " ";
			for (int j = 0; j < 3; ++j)
				ss << mySelf->m_contact_control_normal[j] << " ";
			for (int j = 0; j < 3; ++j)
				ss << mySelf->m_valve_center[j] << " ";
			
			ss << mySelf->m_radius << " "; 

			ss << mySelf->points_for_plane_estimation.size() << " ";

			for (int j = 0; j < mySelf->points_for_plane_estimation.size(); ++j)
				ss << mySelf->points_for_plane_estimation[j](0) << " " << mySelf->points_for_plane_estimation[j](1) << " " << mySelf->points_for_plane_estimation[j](2) << " "; 
			if(++counter > 200)
			{
				mySelf->m_plane_changed = false;
			}

		}
		else
		{
			ss << " " << 0 << " ";
			counter = 0;
		}
		ss << ::std::endl;

        // send data
        iSendResult = send( ClientSocket, ss.str().c_str(),  ss.str().size() + 1, 0 );
        if (iSendResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            WSACleanup();
            return 1;
        }
        else if (iSendResult == 0)
            printf("Connection closing...\n");
 
		iResult = recv(ClientSocket, recvbuf, DEFAULT_BUFLEN, 0);

		delta_t = (double) mySelf->m_timer->GetTime()/1000000.0;
		mySelf->m_timer->ResetTime();

		if (iResult > 0)
		{
			::std::string receivedStr = string(recvbuf);
			//::std::cout << "received:" << receivedStr <<::std::endl;
			if (receivedStr == "NOF")
			{
				//::std::cout << "NOF" << ::std::endl;
				EnterCriticalSection(&m_cSection);
				mySelf->m_ContactUpdateReceived = false;
				LeaveCriticalSection(&m_cSection);
				//::std::cout << "NOFORCE:" <<::std::endl;

			}
			else
			{
				//contactRatio = atof(recvbuf);
				::std::vector<double> msg = DoubleVectorFromString(::std::string(recvbuf));
				contactRatio = msg[0];
				contactRatioError = desiredContactRatio - contactRatio;


				EnterCriticalSection(&m_cSection);
				mySelf->m_ContactUpdateReceived = true;
				mySelf->m_contactError_prev = contactRatioError;
				mySelf->m_contactError = contactRatioError;
				mySelf->m_contactRatio = contactRatio;
				mySelf->m_contact_error_integral += contactRatioError * delta_t;
				mySelf->m_line_detected = msg[1];
				mySelf->m_contact_response = msg[2];


				mySelf->m_wall_detected = msg[7];
				mySelf->switchToCircum = msg[10];
				mySelf->m_apex = msg[11];

				// change all the following network depedencies!!!!!!!!!! //
				//mySelf->storeValvePoint = msg[10];
				// update the respective network function on the client side !!!!!///


				LeaveCriticalSection(&m_cSection);

				if (mySelf->m_line_detected && mySelf->m_circumnavigation)
				{
					mySelf->UpdateCircumnavigationParams(msg);
					mySelf->addPointOnValve();
					//::std::cout << "num of valve points added:" << mySelf->index << ::std::endl;
				}

				if (mySelf->m_wall_detected)
				{
					mySelf->m_centroid_apex[0] = msg[8];
					mySelf->m_centroid_apex[1] = msg[9];
				}

				//	::std::cout << "in network:" << msg[8] << " " << msg[9] << ::std::endl;
				if (mySelf->switchToCircum)
				{
					mySelf->m_apex_to_valve = false;
					mySelf->m_circumnavigation = true;
					::std::cout << "switching to circumnavigation" << ::std::endl;
				}

				if (mySelf->m_apex)
					memcpy(mySelf->m_apex_coordinates, &msg.data()[12], 5 * sizeof(double));

				end_loop = clock();
				//PrintCArray(msg.data(), msg.size());
				::std::cout << "CR:" << contactRatio << ::std::endl;
			}
			
		}
		//force *= 0.5;
		//force = 0.3;
		EnterCriticalSection(&m_cSection);
		//mySelf->m_Omni->SetForce(force);
		//mySelf->m_force = force;
		mySelf->m_deltaT = delta_t;
		LeaveCriticalSection(&m_cSection);

    } while (iResult > 0);

    closesocket(ClientSocket);
    WSACleanup();

	NetworkCommunication(para);
    return 0;

}

unsigned int WINAPI	CCTRDoc::EMLoop(void* para)
{
	// CKim - This loop is executed in the separate thread that starts with the 'InitEM' button. 
	// First, EM tracker is initialized and then it enters the loop where the EM tracker is continuously read (240 Hz)
	// and depending on the flags, collects data for the registration (if  m_RegDataCollecting is true) 
	// or calculates registered tip position and direction from the registration. It also calculates the predicted
	// tip position orientation from the model and fepending on the flag 'm_AdaptiveOn' it updates the model 
	// based on the measured data. 

	// CKim - Variables for measurement, log file, loop speed measurement...
	CCTRDoc* mySelf = (CCTRDoc*) para;		
	ChunTracker* self = mySelf->m_Tracker;
	ChunTimer timer;	
	
	double M[4][4];		double jAng[5];		double measTipPosDir[6];	double	predTipPosDir[6];
	int perfcnt = 0;	int navg = 0;		long loopTime = 0;			std::ofstream ofstr;

	int rCnt = 0;
	
	// CKim - Initialize the EM tracker. 
	if(!self->InitTracker())
	{
		AfxMessageBox(self->m_errMsg);	
		return 0;
	}
	else 
	{
		AfxMessageBox("Tracker Initialized");
	}
	
	// CKim - Enter the loop that reads from the sensors
	while(1)
	{
		// CKim - Read from the sensors and use data for registration or calculate registered tip position oreintation. 
		//if(self->GetTrackerMatrix(M,0))	
		if(self->GetTrackerMatrix(M,1))	
		{
			if(mySelf->m_RegDataCollecting) { 	mySelf->m_fStr<<M[0][3]<<" "<<M[1][3]<<" "<<M[2][3]<<"\n";		}
			else							{	self->GetMeasuredTipPosDir(measTipPosDir);						}
		}
		else {	AfxMessageBox(self->m_errMsg);		return 0;		}

		// CKim - Read/Write to shared variable here
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<4; i++)	{
			for(int j=0; j<4; j++)	{	mySelf->m_Status.emMat[i][j] = M[i][j];		}	}
		for(int i=0; i<6; i++)	{	mySelf->m_measPosforRec[i] = measTipPosDir[i];	}

		for(int i=0; i<6; i++)	{	mySelf->m_Status.sensedTipPosDir[i] = measTipPosDir[i];	}
		for(int i=0; i<5; i++)	{	jAng[i] = mySelf->m_Status.currJang[i];					}

		LeaveCriticalSection(&m_cSection);

		if(mySelf->m_playBack || mySelf->m_bStaticPlayBack || mySelf->m_bRunExperiment)
			mySelf->m_kinLib->UpdateFAC(jAng,measTipPosDir,predTipPosDir,mySelf->m_bDoUpdate);

		rCnt++;

		// ------------------------------------------------------------- //
		//// CKim - Log data if flag is true
		if(mySelf->m_bLogEMData)
		{
			// CKim - Calculate predicted tip pos, dir from the current kinematic model and update 
			// the model using the measured data if m_AdaptiveOn is true.
			mySelf->m_kinLib->UpdateFAC(jAng,measTipPosDir,predTipPosDir,0);
			
			if(!ofstr.is_open())	
			{	
				ofstr.open("C:\\03. OnlineCalibration\\OnlineCalib\\ExperimentData\\TrackerLog.txt");	timer.ResetTime();
			}
			if(perfcnt == navg)
			{
				loopTime = timer.GetTime();	
				ofstr<<loopTime<<" ";
				//for(int i=0; i<5; i++)	{	ofstr<<jAng[i]<<" ";	}
				for(int i=0; i<6; i++)	{	ofstr<<predTipPosDir[i]<<" ";	}
				for(int i=0; i<6; i++)	{	ofstr<<measTipPosDir[i]<<" ";	}
				ofstr<<"\n";
				perfcnt = 0;	//timer.ResetTime();
			}
			else	{		perfcnt++;			}
		}
		else
		{
			if(ofstr.is_open())	{	ofstr.close();	}
		}
		// ------------------------------------------------------------- //

		// CKim - Signal Event to wake up any loops that uses the EM tracker reading
		SetEvent(mySelf->m_hEMevent);
	}

	//ofstr.close();

	return 1;

}


void CCTRDoc::OnBnClickedRegst()
{
	// CKim - Rotate Motor, collect data
	double mtrAng[7] = { 0, 0, 0, 2*c_PI, 2*c_PI, 2*c_PI, 0 };		double mtrCnt[7];
	MtrAngToCnt(mtrAng, mtrCnt);

	m_fStr.open("RegPointSet.txt");
	m_RegDataCollecting = true;
	m_motionCtrl->DoCoordMotion(mtrCnt);
	m_motionCtrl->WaitMotionDone();
	m_RegDataCollecting = false;
	m_fStr.close();

	for(int i=0; i<7; i++)	{	mtrAng[i] = 0;	}
	MtrAngToCnt(mtrAng, mtrCnt);
	m_motionCtrl->DoCoordMotion(mtrCnt);

	
	char fName[100] = "RegPointSet.txt";

	Eigen::Matrix4d M;
	m_Tracker->Registration(fName,M);

	CString str;
	str.Format("%.3f %.3f %.3f %.2f \n %.3f %.3f %.3f %.2f \n %.3f %.3f %.3f %.2f \n %.3f %.3f %.3f %.2f",
		M(0,0), M(0,1), M(0,2), M(0,3), M(1,0), M(1,1), M(1,2), M(1,3), M(2,0), M(2,1), M(2,2), M(2,3), M(3,0), M(3,1), M(3,2), M(3,3));
	AfxMessageBox(str);

}


unsigned int WINAPI	CCTRDoc::TeleOpLoop(void* para)
{
	CCTRDoc* mySelf = (CCTRDoc*) para;		
	double tempSolution[5] = {0};
	// CKim - Robot status and Haptic device status
	CTR_status localStat;		
	
	// CKim - Flags
	bool teleOpCtrl = false;		bool safeToTeleOp = false;			double scl = 1.00;	double kp = 1.1;

	bool adaptModelFlag = false;

	// CKim - Get handle to current view window
	CFrameWnd * pFrame = (CFrameWnd *) (AfxGetApp()->m_pMainWnd);
	mySelf->m_hWndView = pFrame->GetActiveView()->m_hWnd;

	// CKim - Parameters for loop speed measurement
	ChunTimer timer;	int perfcnt = 0;	int navg = 5;		long loopTime = 0;

	//::std::string gainStr;
	//std::ostringstream strs;
	//strs << mySelf->m_contactGain;
	//gainStr = strs.str();
	//::std::string filename = "ExperimentData/" + GetDateString() + "-Gain" + gainStr + ".txt";
	
	//EnterCriticalSection(&m_cSection);
	//if (mySelf->m_fileStream)
	//	delete mySelf->m_fileStream;

	//mySelf->m_fileStream = new ::std::ofstream(filename);   
	//LeaveCriticalSection(&m_cSection);
	//ofstream os("debug_control_no_scaling.txt");
	RecursiveFilter::Filter* filters = new RecursiveFilter::MovingAverageFilter[3];


	// CKim - The Loop
	bool prevControl=false;
	bool currentControl = false;
	while(mySelf->m_teleOpMode)
	{
		prevControl = currentControl;
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<7; i++)	{	localStat.currMotorCnt[i] = mySelf->m_Status.currMotorCnt[i];	}
		for(int i=0; i<5; i++)	{	localStat.currJang[i] = mySelf->m_Status.currJang[i];	}
		for(int i=0; i<6; i++)	{	localStat.currTipPosDir[i] = mySelf->m_Status.currTipPosDir[i];	}
		for(int i = 0; i < 3; ++i) { localStat.hapticState.velocity[i] = mySelf->m_Status.hapticState.velocity[i];}
		for(int i = 0; i < 3; ++i) { localStat.hapticState.ang_velocity[i] = mySelf->m_Status.hapticState.ang_velocity[i];}
		currentControl = mySelf->m_forceControlActivated;
		LeaveCriticalSection(&m_cSection);

		// CKim - Synch with haptic device by executing function in haptic device scheduler
		// Exchange state with the device
#ifdef OMNI_PLUGGED		
		mySelf->m_Omni->SynchState(localStat);
#endif
		// CKim - Haptic device error handling		
		if (localStat.hapticState.err.errorCode != 0)
		{
			if(teleOpCtrl)	{
				mySelf->m_motionCtrl->StopMotion();			teleOpCtrl = false;		}
		}
	
		// --------------------------------------------------------------- //
		// CKim - Handle haptic device events. 
		// 0/1: Button 1 down/up, 2/3: Button 2 down/up
		// --------------------------------------------------------------- //
		while(!localStat.hapticState.eventQueue.empty())
		{
			HapticEventData ev = localStat.hapticState.eventQueue.front();
			
			if(ev.eventId == 0)	
			{
				// CKim - When button 1 is down, toggle teleoperation. Initialize inverse kinematics estimation
				teleOpCtrl = !teleOpCtrl;	
				if(teleOpCtrl)
				{	
					// CKim - Save the current configuration of the haptic device, and tip
					// which are used for master to slave transformation
					for(int i=0; i<16; i++)	{	localStat.M_T0[i] = ev.refMat[i];						}

					for(int i=0; i<6; i++)	
						localStat.refTipPosDir[i] = localStat.currTipPosDir[i];	
									
					// CKim - Initial point for the inverse kinematics 
					for(int i=0; i<5; i++)	{	localStat.initJang[i] = localStat.currJang[i];			}
					for(int i=0; i<5; i++)	{	localStat.initJAngLWPR[i] = localStat.currJang[i];			}
				
					// CKim - Update proxy location
					mySelf->SlaveToMaster(localStat, scl);

					// CKim - Set the force feedback
					localStat.hapticState.forceFlag = true;
					safeToTeleOp = false;		

					// CKim - Init timer
					timer.ResetTime();
				}
				else
				{
					localStat.hapticState.forceFlag = false;	

					// CKim - Stop velocity command
					mySelf->m_motionCtrl->StopMotion();
				}

			}

			if(ev.eventId == 1)		{			}
			if(ev.eventId == 2)		{
				PostMessage(mySelf->m_hWndView,WM_USER+2,NULL,NULL);		}
			if(ev.eventId == 3)		{			}

			localStat.hapticState.eventQueue.pop();
		}

		// ---------------------------------------------------------------------------------------- //
		// CKim - - For teleoperation, execute the rest of the loop in the function that is
		// scheduled and executed by the haptic device. Haptic device parameters are safely 
		// accessed inside this function teleoperation commands are handled here.
		// ---------------------------------------------------------------------------------------- //
		clock_t start_loop, end_loop;
		double delta_t;
		double vel[3] = {0};

		double camera_dir[3] = {0};
		double haptic_dir[3] = {0};
		double filtered_haptic_dir[3] = {0};
		if(teleOpCtrl)	
		{
			if (!mySelf->m_camera_control)
			{
				mySelf->MasterToSlave(localStat, scl);		// Updates robotStat.tgtTipPosDir
			
				//mySelf->UpdateDesiredPosition();
				mySelf->ComputeDesiredVelocity();

				// TODO
				//if (mySelf->m_compute_plane)
				//	mySelf->IncrementalPlaneUpdate();
				if (!currentControl && prevControl)
				{
					for(int i=0; i<6; i++)	
					{
						localStat.tgtTipPosDir[i] = localStat.currTipPosDir[i];	
						localStat.refTipPosDir[i] = localStat.currTipPosDir[i];	
					}
					for(int i=0; i<16; i++)	
						localStat.M_T0[i] = localStat.hapticState.tfMat[i];	
				}
				mySelf->SlaveToMaster(localStat, scl);		// Updates robotStat.hapticState.slavePos

				mySelf->m_bCLIK = true;
			}
			else
			{
				// compute haptic diplacement
				mySelf->computeHapticDisplacement(localStat, haptic_dir);
				for (int i = 0; i < 3; i++)
					filtered_haptic_dir[i] = filters[i].step(haptic_dir[i]);

				// compute direction of motion of the camera
				mySelf->computeCameraDesiredMotion(localStat, filtered_haptic_dir, camera_dir);
				memcpy(localStat.tgtWorkspaceVelocity, camera_dir, 3 * sizeof(double));

				mySelf->m_bCLIK = true;	

				for(int i=0; i<6; i++)	
				{
					localStat.tgtTipPosDir[i] = localStat.currTipPosDir[i];	
					localStat.refTipPosDir[i] = localStat.currTipPosDir[i];	
				}
				for(int i=0; i<16; i++)	
					localStat.M_T0[i] = localStat.hapticState.tfMat[i];	
				mySelf->SlaveToMaster(localStat, scl);

			}
			if (mySelf->m_circumnavigation || mySelf->m_apex_to_valve)
			{
								for(int i=0; i<6; i++)	
				{
					localStat.tgtTipPosDir[i] = localStat.currTipPosDir[i];	
					localStat.refTipPosDir[i] = localStat.currTipPosDir[i];	
				}
				for(int i=0; i<16; i++)	
					localStat.M_T0[i] = localStat.hapticState.tfMat[i];	
				mySelf->SlaveToMaster(localStat, scl);
			}
		}

					
		safeToTeleOp = teleOpCtrl && (localStat.hapticState.err.errorCode == 0);// && wasConverged;
		
		// --------------------------------------------------------------- //
		// CKim - Update shared variable
		// --------------------------------------------------------------- //
	
		EnterCriticalSection(&m_cSection);

		for(int i=0; i<6; i++)	{
			mySelf->m_Status.tgtTipPosDir[i] = localStat.tgtTipPosDir[i];		}
		for(int i=0; i<6; i++)	{
			mySelf->m_Status.refTipPosDir[i] = localStat.refTipPosDir[i];		}
		for(int i=0; i<5; i++)	{
			mySelf->m_Status.tgtJang[i] = localStat.tgtJang[i];		}
		for(int i=0; i<7; i++)	{
			mySelf->m_Status.tgtMotorCnt[i] = localStat.tgtMotorCnt[i];		}
		for(int i=0; i<3; i++)	{
			mySelf->m_Status.haptic_velocity[i] = localStat.haptic_velocity[i];		}
		for(int i=0; i<3; i++)	{
			mySelf->m_Status.tgtWorkspaceVelocity[i] = localStat.tgtWorkspaceVelocity[i];		}
		for(int i=0; i<3; i++)	{
			mySelf->m_Status.tgtWorkspaceAngVelocity[i] = localStat.tgtWorkspaceAngVelocity[i];		}

		mySelf->m_Status.invKinOK = localStat.invKinOK;
		mySelf->m_Status.limitOK = localStat.limitOK;
		mySelf->m_Status.isTeleOpMoving = teleOpCtrl;
		mySelf->m_Status.condNum = localStat.condNum;
		mySelf->m_Status.gain = kp;
		mySelf->m_Status.invKinErr[0] = localStat.invKinErr[0];
		mySelf->m_Status.invKinErr[1] = localStat.invKinErr[1];
		

		LeaveCriticalSection(&m_cSection);
	}
	for (int i = 0; i < 3; i++)
		mySelf->filters[i].resetFilter();

	mySelf->m_ref_set = false;
	mySelf->m_InvKinOn = false;
	//os.close();
	return 0;

}


unsigned int WINAPI	CCTRDoc::PlaybackLoop(void* para)
{
	// CKim - Playback from the 'Playback.txt' file. Perfomr online parameter estimation if instructed. Record data
	// CKim - Pointer to self
	CCTRDoc* mySelf = (CCTRDoc*) para;		

	// CKim - Log files
	::std::string filename = "ExperimentData/" + mySelf->m_date + "-Playback.txt";
	std::ofstream ofstr;	
	//ofstr.open("PlaybackLog.txt");	
	ofstr.open(filename);
	
	// CKim - Parameters for loop speed measurement
	ChunTimer timer;	int perfcnt = 0;	int navg = 0;		long loopTime = 0;

	// CKim - Robot status and Haptic device status
	CTR_status localStat;		
	
	// CKim - Gain
	double kp = 10.0;		double predTipPosDir[6];	double solvedTipPosDir[6];
	
	mySelf->m_kinLib->SetInvKinThreshold(0.1,3.0);

	// CKim - The Loop
	timer.ResetTime();

	for(int i=0; i<5; i++)	{	localStat.initJang[i] = mySelf->m_Status.currJang[i];	}

	while(mySelf->m_playBack)
	{
		// CKim - Synchronize on the update of the EM tracker data. Since the adaptive update will be 
		// meaningful only when new EM tracker data and predicted tip posDir is available
		// Use WaitForMultipleObjects to wait for this....
		//for(int cnt = 0; cnt<24; cnt++)
		//{
			int flag = WaitForSingleObject(mySelf->m_hEMevent,1000);
			if(flag == WAIT_TIMEOUT)	{
				AfxMessageBox("No event from EM Loop!");	return 0;
			}
		//}

		// CKim - Synch with haptic device. This is for spending 1 ms. 
		//mySelf->m_Omni->SynchState(localStat);
		
		// CKim - Read the shared variable (current cnt, jAng, .. ) that is used in this loop. 
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<7; i++)	{	localStat.currMotorCnt[i] = mySelf->m_Status.currMotorCnt[i];	}
		for(int i=0; i<5; i++)	{	localStat.currJang[i] = mySelf->m_Status.currJang[i];	}
		for(int i=0; i<6; i++)	{	localStat.currTipPosDir[i] = mySelf->m_Status.currTipPosDir[i];	}
		for(int i=0; i<6; i++)	{	localStat.sensedTipPosDir[i] = mySelf->m_Status.sensedTipPosDir[i];		}
		LeaveCriticalSection(&m_cSection);

		// ADAPT LWPR
		mySelf->m_kinLWPR->AdaptForwardModel(localStat.sensedTipPosDir, localStat.currJang);		// CKim - calculate predicted position and perform adaptive update if instructed
		
		// CKim - Read trajectory from the 'Playback.txt'. Returns false when the end of trajectory is reached
		mySelf->m_playBack = mySelf->m_TrjGen->InterpolateNextPoint(localStat.tgtTipPosDir, localStat.tgtMotorVel);	// Update tgtTipPosDir

		// CKim - Perform inverse kinematics to find joint angles that brings tip to the
		// target position and orientation. This calculates least square solution of the inverse kinematics
		// and applies joint limit to the solution. Two flags, invKinOK and limitOK will be raised
		// if least square error is larger than 1 and if joints has been limited. 
		mySelf->SolveInverseKin(localStat);			// Updates localStat.tgtMotorCnt, tgtJang
	
		mySelf->m_kinLWPR->TipFwdKin(localStat.tgtJang,localStat.solvedTipPosDir);

		// CKim - Log
		if(perfcnt==navg)
		{
			loopTime = timer.GetTime();
			//loopTime /= navg;
			ofstr<<loopTime<<" ";
			for(int i=0; i<6; i++)	{	ofstr<<localStat.currTipPosDir[i]<<" ";		}
			for(int i=0; i<6; i++)	{	ofstr<<predTipPosDir[i]<<" ";				}
			for(int i=0; i<6; i++)	{	ofstr<<localStat.sensedTipPosDir[i]<<" ";	}
			//for(int i=0; i<6; i++)	{	ofstr<<localStat.tgtTipPosDir[i]<<" ";		}
			//for(int i=0; i<5; i++)	{	ofstr<<localStat.tgtJang[i]<<" ";			}
			//for(int i=0; i<7; i++)	{	ofstr<<localStat.tgtMotorCnt[i]<<" ";	}
			//ofstr<<localStat.invKinOK<<" ";
			//ofstr<<localStat.condNum<<" ";
			//ofstr<<localStat.limitOK<<" ";
			//for(int i=0; i<6; i++)	{	ofstr<<localStat.solvedTipPosDir[i]<<" ";		}
			//ofstr<<localStat.exitCond<<" ";
			ofstr<<"\n";
		
			perfcnt = 0;		//timer.ResetTime();
		}
		else	{	perfcnt++;	}

		// --------------------------------------------------------------- //
		// CKim - Update shared variable
		// --------------------------------------------------------------- //
		EnterCriticalSection(&m_cSection);

		for(int i=0; i<6; i++)	{
			mySelf->m_Status.tgtTipPosDir[i] = localStat.tgtTipPosDir[i];		}
		for(int i=0; i<5; i++)	{
			mySelf->m_Status.tgtJang[i] = localStat.tgtJang[i];		}
		for(int i=0; i<7; i++)	{
			mySelf->m_Status.tgtMotorCnt[i] = localStat.tgtMotorCnt[i];		
			mySelf->m_Status.tgtMotorVel[i] = localStat.tgtMotorVel[i];		
		}

		mySelf->m_Status.invKinOK = localStat.invKinOK;
		mySelf->m_Status.limitOK = localStat.limitOK;
		mySelf->m_Status.isTeleOpMoving = mySelf->m_playBack;
		mySelf->m_Status.condNum = localStat.condNum;
		mySelf->m_Status.gain = kp;
		mySelf->m_Status.invKinErr[0] = localStat.invKinErr[0];
		mySelf->m_Status.invKinErr[1] = localStat.invKinErr[1];
	
		LeaveCriticalSection(&m_cSection);
	}
	mySelf->m_playBack = false;
	ofstr.close();
	
	return 0;

}


unsigned int WINAPI	CCTRDoc::StaticPlaybackLoop(void* para)
{
	// CKim - Implements Closed Loop Inverse Kinematics with feedback from sensor measurement. 

	// CKim - Pointer to self
	CCTRDoc* mySelf = (CCTRDoc*) para;		

	// CKim - Log files
	std::ofstream ofstr;	ofstr.open("C:\\03. OnlineCalibration\\OnlineCalib\\ExperimentData\\CLIK_Log.txt");	

	// CKim - Parameters for loop speed measurement
	ChunTimer timer;	int perfcnt = 0;		int navg = 200;		long loopTime = 0;		double nextTime = -1.0;

	// CKim - Robot status and Haptic device status
	CTR_status localStat;		
	
	double predTipPosDir[6];		double kp = 10.0;
	mySelf->m_kinLib->SetInvKinThreshold(0.1,3.0);
	for(int i=0; i<5; i++)	{	localStat.initJang[i] = mySelf->m_Status.currJang[i];	}

	// CKim - The Loop
	timer.ResetTime();

	//while(mySelf->m_playBack)
	while(mySelf->m_bStaticPlayBack)
	{
		//// CKim - Synchronize on the update of the EM tracker data. Since the adaptive update will be 
		//// meaningful only when new EM tracker data and predicted tip posDir is available
		//// Use WaitForMultipleObjects to wait for this....
		//int flag = WaitForSingleObject(mySelf->m_hEMevent,1000);
		//if(flag == WAIT_TIMEOUT)	{
		//	AfxMessageBox("No event from EM Loop!");	return 0;
		//}

		// CKim - Synch with haptic device. This is for spending 1 ms. 
		//mySelf->m_Omni->SynchState(localStat);
		
		// CKim - Read the shared variable (current cnt, jAng, .. ) that is used in this loop. 
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<7; i++)	{	localStat.currMotorCnt[i] = mySelf->m_Status.currMotorCnt[i];	}
		for(int i=0; i<5; i++)	{	localStat.currJang[i] = mySelf->m_Status.currJang[i];	}
		for(int i=0; i<6; i++)	{	localStat.currTipPosDir[i] = mySelf->m_Status.currTipPosDir[i];	}
		for(int i=0; i<6; i++)	{	localStat.sensedTipPosDir[i] = mySelf->m_Status.sensedTipPosDir[i];		}
		LeaveCriticalSection(&m_cSection);

		// CKim - Safety check. Stop and exit loop if joint angle is out of limit
		if( (localStat.currJang[2] > L31_MAX) || (localStat.currJang[2] < L31_MIN) || (fabs(localStat.currJang[4]) > 100.0) )
		{
			mySelf->m_Status.isTeleOpMoving = false;
			AfxMessageBox("Joint out of limit!");	
			break;
		}

		// CKim - Read trajectory from the 'Playback.txt'. Returns false when the end of trajectory is reached
		if(loopTime/1000 > nextTime)
		{
			mySelf->m_bStaticPlayBack = mySelf->m_TrjGen->GetNextPointStatic(localStat.tgtTipPosDir);	// Update tgtTipPosDir
			nextTime += 5000;
		}

		// CKim - In case of feed forward position control, solve inverse kinematics
		if(mySelf->m_InvKinOn)
		{
			mySelf->SolveInverseKin(localStat);			// Updates localStat.tgtMotorCnt, tgtJang
		}
		else	// CKim - In case of differential kinematics control, enable flag
		{
			mySelf->m_bCLIK = true;	
		}
		
		// --------------------------------------------------------------- //
		// CKim - Update shared variable
		// --------------------------------------------------------------- //
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<6; i++)	{
			mySelf->m_Status.tgtTipPosDir[i] = localStat.tgtTipPosDir[i];		}
		for(int i=0; i<5; i++)	{
			mySelf->m_Status.tgtJang[i] = localStat.tgtJang[i];		}
		for(int i=0; i<7; i++)	{
			mySelf->m_Status.tgtMotorCnt[i] = localStat.tgtMotorCnt[i];		}

		mySelf->m_Status.invKinOK = localStat.invKinOK;
		mySelf->m_Status.limitOK = localStat.limitOK;
		mySelf->m_Status.isTeleOpMoving = (mySelf->m_InvKinOn || mySelf->m_bCLIK);
		mySelf->m_Status.condNum = localStat.condNum;
		mySelf->m_Status.gain = kp;
		mySelf->m_Status.invKinErr[0] = localStat.invKinErr[0];
		mySelf->m_Status.invKinErr[1] = localStat.invKinErr[1];
	
		LeaveCriticalSection(&m_cSection);

		// CKim - Update the model
		//mySelf->m_kinLib->UpdateFAC(localStat.currJang,localStat.sensedTipPosDir,predTipPosDir,mySelf->m_AdaptiveOn);


		// --------------------------------------------------------------- //
		// CKim - Log the data that was read
		// --------------------------------------------------------------- //
		if(perfcnt==navg)
		{
			loopTime = timer.GetTime();
			ofstr<<loopTime<<" ";
			for(int i=0; i<6; i++)	{	ofstr<<localStat.currTipPosDir[i]<<" ";		}
			for(int i=0; i<6; i++)	{	ofstr<<localStat.sensedTipPosDir[i]<<" ";	}
			for(int i=0; i<6; i++)	{	ofstr<<localStat.tgtTipPosDir[i]<<" ";		}
			for(int i=0; i<5; i++)	{	ofstr<<localStat.tgtJang[i]<<" ";			}
			for(int i=0; i<7; i++)	{	ofstr<<localStat.tgtMotorCnt[i]<<" ";	}
			ofstr<<localStat.invKinOK<<" ";
			ofstr<<localStat.condNum<<" ";
			ofstr<<localStat.limitOK<<" ";
			ofstr<<"\n";		
			perfcnt = 0;		//timer.ResetTime();
		}
		else	{	perfcnt++;	}


	}
	mySelf->m_bCLIK = false;
	mySelf->m_playBack = false;
	
	ofstr.close();
	
	return 0;

}


unsigned int WINAPI	CCTRDoc::MotorLoop(void* para)
{
	CCTRDoc* mySelf = (CCTRDoc*) para;	
	CTR_status	localStat;

	bool circumnavigation = false;
	bool isInContact = false;
	bool apex_to_valve = false;
	// CKim - Variables for Feedforward position control
	bool safeToTeleOp = false;			
	double vel[7];			
	double MtrCntSetPt[7];				
	double kp = 10.0;		

	// CKim - Variables for Differential inverse kinematics control. Jacobian matrix
	Eigen::MatrixXd J(6,5);			
	Eigen::Matrix<double,6,1> err;
	err.setZero();
	double dq[5];		
	double dCnt[7];		
	
	//double K[6] = {5.0, 5.0, 5.0, 0.5, 0.5, 0.5 };	// working
	//double K[6] = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };		// working
	
	double K_image[6] = {1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0 };		// for image frame control
	double K[6] = {2.0, 2.0, 2.0, 5.0, 5.0, 5.0};		// working
	//double K[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};		// working
	//double K[6] = { 5.0, 5.0, 5.0, 50.0, 50.0, 50.0 };	
	//double K[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 };				// For sensor feedback + estimator
	//double K[6] = { 1.5, 1.5, 1.5, 0.1, 0.1, 0.1 };				// For sensor feedback + estimator
	//double K[6] = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5 };	// working	

	// CKim - Parameters for loop speed measurement
	ChunTimer timer;	
	int perfcnt = 0;	
	int navg = 50;		
	timer.ResetTime();	

	long endTime = 0;

	// JHa - disturbance related variables
	ChunTimer timer_dis;
	timer_dis.ResetTime();


	double	desiredPosition[6];

	while(mySelf->m_motorConnected)
	{

		// george -> test heart rate monitor
		//double heart_rate = mySelf->m_heartRateMonitor->getHeartRate();
		//::std::cout << "Heart Rate [bpm]: " << heart_rate << ::std::endl;


		// CKim - Read from the motors - blocking function
		mySelf->m_motionCtrl->GetMotorPos(localStat.currMotorCnt);	
		mySelf->m_motionCtrl->GetErrorFlag(localStat.errFlag);

		// CKim - Calculate current joint angle
		mySelf->MtrToJang(localStat.currMotorCnt, localStat.currJang);

		// TODO: learn an LWPR model for the balanced pair as well
		// CKim - Evaluate Kinematics Model for balanced pair position and orientation
		//mySelf->m_kinLib->BalancedPairFwdKin(localStat.currJang, localStat.bpTipPosDir);
		//mySelf->m_kinLWPR_BP->TipFwdKin(localStat.currJang, localStat.bpTipPosDir);

		// CKim - Apply Control Law to calculate joint velocity
		// CKim - Position FeedForward Control.
		
		//if(mySelf->m_InvKinOn || mySelf->m_teleOpMode)
		if(mySelf->m_InvKinOn)
		{
			// CKim - Read shared variable (Motor Count Setpoint and gain)
			// Motor count setpoint is updated from another loop that reads desired configuration from 
			// a) haptic device or b) trajectory point list and solves the inverse kinematics
			EnterCriticalSection(&m_cSection);
			for(int i=0; i<7; i++)	
				MtrCntSetPt[i] = mySelf->m_Status.tgtMotorCnt[i];

			kp = mySelf->m_Status.gain;

			safeToTeleOp = mySelf->m_Status.isTeleOpMoving;
			LeaveCriticalSection(&m_cSection);

			// CKim - Evaluate model
			//mySelf->m_kinLWPR->TipFwdKin(localStat.currJang, localStat.currTipPosDir);
			mySelf->m_kinLib->EvalCurrentKinematicsModelNumeric(localStat.currJang, localStat.currTipPosDir, J, mySelf->m_bCLIK);

			// CKim - Apply simple control law : vel = kp(target pos - current pos)
			if(safeToTeleOp)	
				for(int i=0; i<7; i++)	
				{
					vel[i] = -kp*(localStat.currMotorCnt[i]-MtrCntSetPt[i]);
	/*				if (vel[i] > 5.0)
						vel[i] = 5.0;
					else if (vel[i] < -5.0)
						vel[i] = -5.0;*/
				}
			else				
				for(int i=0; i<7; i++)	
					vel[i] = 0.0;									
		}
		// CKim - Differential Inverse Kinematics Control.
		else if(mySelf->m_bCLIK )	
		{
			::Eigen::Matrix<double, 3, 1> actualForce;
			actualForce.setZero();
			

			::Eigen::Vector3d planeNormal;
			// CKim - Read shared variables (sensed / target posdir and kinematics model)
			// Sensed posdir and kinematics model is updated from EM tracker loop,
			// target posdir is updated from 'Playback' loop  or 'TeleOp' loop
			EnterCriticalSection(&m_cSection);
			for (int i = 0; i < 6; ++i)
				desiredPosition[i] = mySelf->m_desiredPosition[i];
			for(int i=0; i<6; i++)
				localStat.tgtTipPosDir[i] = mySelf->m_Status.tgtTipPosDir[i];			
			for(int i=0; i<6; i++)	
				localStat.sensedTipPosDir[i] = mySelf->m_Status.sensedTipPosDir[i];		
			for(int i=0; i<6; i++)	
				localStat.tgtMotorVel[i] = mySelf->m_Status.tgtMotorVel[i];	

			// feedforward velocities
			for (int i = 0; i < 3; i++)
				localStat.tgtWorkspaceVelocity[i] = mySelf->m_Status.tgtWorkspaceVelocity[i];
			for (int i = 0; i < 3; i++)
				localStat.tgtWorkspaceAngVelocity[i] = mySelf->m_Status.tgtWorkspaceAngVelocity[i];

			actualForce[2] = mySelf->m_force;
			safeToTeleOp = mySelf->m_Status.isTeleOpMoving;
			for (int i = 0; i < 3; ++i)
				planeNormal(i) = mySelf->m_contact_control_normal(i);

			circumnavigation = mySelf->m_circumnavigation;
			apex_to_valve = mySelf->m_apex_to_valve;
			isInContact = mySelf->m_contact_response;
			LeaveCriticalSection(&m_cSection);
			//PrintCArray(localStat.tgtWorkspaceVelocity, 3);
			//PrintCArray(localStat.tgtWorkspaceAngVelocity, 3);
			::Eigen::Vector3d currentTangent = ::Eigen::Map<::Eigen::Vector3d> (&localStat.currTipPosDir[3], 3);
			::Eigen::Vector3d tangentVelocity = ::Eigen::Map<::Eigen::Vector3d> (localStat.tgtWorkspaceAngVelocity, 3).cross(currentTangent);
			//PrintCArray(tangentVelocity.data(), 3);

			//mySelf->m_kinLWPR->TipFwdKinJac(localStat.currJang, localStat.currTipPosDir, J,true);
			mySelf->m_kinLib->EvalCurrentKinematicsModelNumeric(localStat.currJang, localStat.currTipPosDir, J, mySelf->m_bCLIK);

			// Use sensor feedback
			if(mySelf->m_FeedbackOn)
			{
				for(int i=0; i<6; i++)	
					err(i,0) = K[i]*(localStat.tgtTipPosDir[i] - localStat.sensedTipPosDir[i]);	
			}
			// Use fwd kin output
			else
			{

				if (mySelf->m_forceControlActivated)
				{
					//localStat.tgtTipPosDir[3] = 0;
					//localStat.tgtTipPosDir[4] = 0;
					//localStat.tgtTipPosDir[5] = 1;
					tangentVelocity.setZero();
				}

				//PrintCArray(localStat.tgtWorkspaceVelocity, 3);
				double sum = 0;
				for(int i=0; i<3; i++)	
				{	
					if (!mySelf->m_camera_control)
						err(i,0) = mySelf->m_position_gain * K[i]*(localStat.tgtTipPosDir[i] - localStat.currTipPosDir[i]) + mySelf->m_position_gain_feedforward * localStat.tgtWorkspaceVelocity[i];
					else
						err(i, 0) = K_image[i] * localStat.tgtWorkspaceVelocity[i];   // this is to control the robot at the image-frame of the cardioscope
					::Eigen::Matrix<double, 6, 1> tmpVelocities;
					if (circumnavigation)
					{
						mySelf->computeCircumnavigationDirection(tmpVelocities);
						if (!isInContact) // tmp_fix
							err.block(0,0, 3, 1) = tmpVelocities;
						else
							err.setZero();
						//err.block(0,0, 3, 1) = tmpVelocities;
					}
					else if (apex_to_valve)
					{
						//::std::cout << "right before activating" << ::std::endl;
						//::std::cout << mySelf->aStatus << ::std::endl;
						mySelf->computeApexToValveMotion(err, mySelf->aStatus);
					}
				}
				for(int i=3; i<6; i++)
				{
					err(i,0) = mySelf->m_orientation_gain * K[i]*(localStat.tgtTipPosDir[i] - localStat.currTipPosDir[i]) + mySelf->m_orientation_gain_feedforward * tangentVelocity[i-3];
				}
				//::std::cout << "err from main loop: " << err.col(0).transpose() << ::std::endl;
			}
			// temporarily for debugging the higher level features
			//err.setZero();

			//project the error on the plane
			if (mySelf->m_forceControlActivated)
			{
				::Eigen::Matrix3d PlaneNullProjection = ::Eigen::Matrix3d::Identity() - planeNormal * planeNormal.transpose();
				err.block(0,0, 3, 1) = PlaneNullProjection * err.block(0,0, 3, 1);
			
				for (int i = 0; i < 3; i++)
					err(i, 0) += desiredPosition[i];   
				
				if (mySelf->retractRobot)
					err(3, 0) = -1.0; //value?
			}

			if (mySelf->m_control_mode == 0)
				mySelf->m_kinLib->ApplyKinematicControlNullspace(J,err,dq, localStat.currJang);
			else if (mySelf->m_control_mode ==1)
				mySelf->m_kinLib->ApplyKinematicControl(J,err,dq, localStat.currJang);

			// add sinusoidal disturbance
			double time = (double)timer_dis.GetTime()/1e6;
			double freq_dis = 6;	// BPM
			double vel_dis = mySelf->m_disturbance_amp * freq_dis/60 * 2 *M_PI * cos(freq_dis/60 * 2 *M_PI * time);
			dq[4] += vel_dis;

			// CKim - Convert dotq into motor velocity
			mySelf->dJangTodCnt(dq, dCnt);
	
			if(safeToTeleOp)	{	for(int i=0; i<7; i++)	{	vel[i] = dCnt[i];	}		}
			else				{	for(int i=0; i<7; i++)	{	vel[i] = 0.0;		}		}

		}
		else if (mySelf->m_jointPlayback)
		{
			EnterCriticalSection(&m_cSection);
			
			for(int i=0; i<7; i++)	
				MtrCntSetPt[i] = mySelf->m_Status.tgtMotorCnt[i];

			kp = mySelf->m_Status.gain;
			
			safeToTeleOp = mySelf->m_Status.isTeleOpMoving;

			LeaveCriticalSection(&m_cSection);

			if (safeToTeleOp)
			{
				for(int i=0; i<7; i++)	
					vel[i] = -kp*(localStat.currMotorCnt[i]-MtrCntSetPt[i]);
			}
			else
				for(int i=0; i<7; i++)	
					vel[i] = 0.0;	
		}
		else if(mySelf->activateIdentification)// && mySelf->m_idMode)
		{
			EnterCriticalSection(&m_cSection);
			

			double tgtTranslation = mySelf->reference_translation;
			double tgtTranslationVel = 0;
			double time = (double)mySelf->timerId.GetTime()/1e6;
			//::std::cout << "time: " << time << ::std::endl;
			for (int i = 0 ; i < mySelf->num_of_sins; i++)
			{
				double freq_rad_per_sec = mySelf->frequencies[i]/60 * 2 * M_PI;
				tgtTranslation += mySelf->amplitude/(double)mySelf->num_of_sins * sin(freq_rad_per_sec*time);

				tgtTranslationVel += mySelf->amplitude/(double)mySelf->num_of_sins * freq_rad_per_sec * cos(freq_rad_per_sec*time);
			}
			double tgtTranslation_inner = tgtTranslation - L31_MAX + mySelf->reference_translation_inner;

			
			MtrCntSetPt[0] = tgtTranslation/c_CntToMM;
			MtrCntSetPt[1] = tgtTranslation_inner/c_CntToMM;
			for(int i=2; i<7; i++)	
				MtrCntSetPt[i] = localStat.currMotorCnt[i];

			//kp = mySelf->m_Status.gain;
			//mySelf->m_Status.gain = 10;
			double kp_id = 10;
			
			//mySelf->m_Status.isTeleOpMoving = true;
			safeToTeleOp = mySelf->m_Status.isTeleOpMoving;

			LeaveCriticalSection(&m_cSection);

		

			//if (safeToTeleOp)
			if(true)
			{
				for(int i=0; i<7; i++)	
					vel[i] = -kp_id*(localStat.currMotorCnt[i]-MtrCntSetPt[i]);

				//PrintCArray(vel, 7);

				vel[0] += tgtTranslationVel/c_CntToMM;
				vel[1] += tgtTranslationVel/c_CntToMM;

				//for(int i=2; i<7; i++)
				//	vel[i] += tgtTranslationVel/c_CntToMM;
			}
			else
				for(int i=0; i<7; i++)	
					vel[i] = 0.0;	
			//::std::cout << "joint id velocity: ";
			//PrintCArray(vel, 7);

			//for(int i=0; i<7; i++)	
			//	vel[i] = -1.0;	

		}
		else	// CKim - When control is not running
		{
			//mySelf->m_kinLWPR->TipFwdKin(localStat.currJang, localStat.currTipPosDir);
			mySelf->m_kinLib->EvalCurrentKinematicsModel(localStat.currJang, localStat.currTipPosDir, J, mySelf->m_bCLIK);
			//PrintCArray(localStat.currTipPosDir, 3);
			for(int i=0; i<7; i++)	
				vel[i] = 0.0;		
	
			// CKim - Process user commands... should be in separate threads..
			mySelf->ProcessCommand(localStat);	
		}
		//PrintCArray(localStat.currTipPosDir, 6);
		// ----------------------------------------------------- //
		// CKim - Command joint velocity, update shared variable
		// ----------------------------------------------------- //
			for(int i=0; i<7; i++)	
				vel[i] = 0.0;		

		//::std::cout << "vel before commanding: ";
		//PrintCArray(vel,7);
		if(!mySelf->m_motionCtrl->DoTeleOpMotion(vel))	
		{
			// CKim - Stop velocity command
			AfxMessageBox("Motion Error!!");
			mySelf->m_motionCtrl->DumpConfiguration();
			break;
		}

		EnterCriticalSection(&m_cSection);
		for(int i=0; i<7; i++)	{
			mySelf->m_Status.currMotorCnt[i] = localStat.currMotorCnt[i];		mySelf->m_Status.errFlag[i] = localStat.errFlag[i];			}
		for(int i=0; i<5; i++)	{	mySelf->m_Status.currJang[i] = localStat.currJang[i];		}
		for(int i=0; i<6; i++)	
		{	
			mySelf->m_Status.currTipPosDir[i] = localStat.currTipPosDir[i];		mySelf->m_Status.bpTipPosDir[i] = localStat.bpTipPosDir[i];	
		}
		mySelf->m_Status.loopTime =  endTime / navg;
		LeaveCriticalSection(&m_cSection);
		//::std::cout << 1000000.0/(timer.GetTime()) << ::std::endl;
		timer.ResetTime();
		//if (timer.GetTime() < 2500)
		//	Sleep(1);
		// CKim - Measure loop speed
		if(perfcnt==navg)	{		endTime = timer.GetTime();		perfcnt = 0;		timer.ResetTime();		}
		else				{		perfcnt++;																	}
	}

	return 0;
}


unsigned int WINAPI	CCTRDoc::SettlingTestLoop(void* para)
{
	// CKim - Reinitialize the kinematics model. Move to a given joint / workspace coordinate
	// Start online estimation, record data. 

	// CKim - Pointer to self
	CCTRDoc* mySelf = (CCTRDoc*) para;		

	// CKim - Log files
	std::ofstream ofstr;	ofstr.open("C:\\03. OnlineCalibration\\OnlineCalib\\ExperimentData\\SettlingLog.txt");	

	// CKim - Parameters for loop speed measurement
	ChunTimer timer;	int perfcnt = 20;	int navg = 20;		long loopTime = 0;

	// CKim - Robot status and Haptic device status
	CTR_status localStat;		
	
	// CKim - Variables
	double predTipPosDir[6];		double setPt[6];		double kp = 5.0;
	
	mySelf->m_kinLib->SetInvKinThreshold(0.1,3.0);
	for(int i=0; i<5; i++)	{	localStat.initJang[i] = mySelf->m_Status.currJang[i];	}
	
	// CKim - Read the set point
	for(int i=0; i<6; i++)	{	setPt[i] = mySelf->m_SetPt[i];		}

	// CKim - Reset timer
	timer.ResetTime();

	while(1)
	{
		// CKim - Synch with haptic device. This is for spending 1 ms. 
		//mySelf->m_Omni->SynchState(localStat);
		
		// CKim - Read the shared variable (current cnt, jAng, .. ) that is used in this loop. 
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<7; i++)	{	localStat.currMotorCnt[i] = mySelf->m_Status.currMotorCnt[i];	}
		for(int i=0; i<5; i++)	{	localStat.currJang[i] = mySelf->m_Status.currJang[i];	}
		for(int i=0; i<6; i++)	{	localStat.currTipPosDir[i] = mySelf->m_Status.currTipPosDir[i];	}
		for(int i=0; i<6; i++)	{	localStat.sensedTipPosDir[i] = mySelf->m_Status.sensedTipPosDir[i];		}
		LeaveCriticalSection(&m_cSection);

		// CKim - Safety check. Stop and exit loop if joint angle is out of limit
		if( (localStat.currJang[2] > L31_MAX) || (localStat.currJang[2] < L31_MIN) || (fabs(localStat.currJang[4]) > 100.0) )
		{
			mySelf->m_Status.isTeleOpMoving = false;
			AfxMessageBox("Joint out of limit!");	
			break;
		}
	
		// CKim - Read the target
		for(int i=0; i<6; i++)	{	localStat.tgtTipPosDir[i] = setPt[i];	}
		
		// CKim - In case of feed forward position control, solve inverse kinematics
		if(mySelf->m_InvKinOn)
		{
			mySelf->SolveInverseKin(localStat);			// Updates localStat.tgtMotorCnt, tgtJang
		}
		else	// CKim - In case of differential kinematics control, enable flag
		{
			mySelf->m_bCLIK = true;	
		}

		// --------------------------------------------------------------- //
		// CKim - Log the data that was read
		// --------------------------------------------------------------- //
		if(perfcnt==navg)
		{
			loopTime = timer.GetTime();
			ofstr<<loopTime<<" ";
			for(int i=0; i<6; i++)	{	ofstr<<localStat.currTipPosDir[i]<<" ";		}
			for(int i=0; i<6; i++)	{	ofstr<<localStat.sensedTipPosDir[i]<<" ";	}
			for(int i=0; i<6; i++)	{	ofstr<<localStat.tgtTipPosDir[i]<<" ";		}
			
			

			//for(int i=0; i<5; i++)	{	ofstr<<localStat.tgtJang[i]<<" ";			}
			//for(int i=0; i<7; i++)	{	ofstr<<localStat.tgtMotorCnt[i]<<" ";	}
			ofstr<<localStat.invKinOK<<" ";
			ofstr<<localStat.condNum<<" ";
			ofstr<<localStat.limitOK<<" ";
			ofstr<<"\n";		
			perfcnt = 0;		//timer.ResetTime();
		}
		else	{	perfcnt++;	}

			
		// --------------------------------------------------------------- //
		// CKim - Update shared variable
		// --------------------------------------------------------------- //
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<6; i++)	{
			mySelf->m_Status.tgtTipPosDir[i] = localStat.tgtTipPosDir[i];		}
		for(int i=0; i<5; i++)	{
			mySelf->m_Status.tgtJang[i] = localStat.tgtJang[i];		}
		for(int i=0; i<7; i++)	{
			mySelf->m_Status.tgtMotorCnt[i] = localStat.tgtMotorCnt[i];		}

		mySelf->m_Status.invKinOK = localStat.invKinOK;
		mySelf->m_Status.limitOK = localStat.limitOK;
		mySelf->m_Status.isTeleOpMoving = (mySelf->m_InvKinOn || mySelf->m_bCLIK);
		mySelf->m_Status.condNum = localStat.condNum;
		mySelf->m_Status.gain = kp;
		mySelf->m_Status.invKinErr[0] = localStat.invKinErr[0];
		mySelf->m_Status.invKinErr[1] = localStat.invKinErr[1];
	
		LeaveCriticalSection(&m_cSection);

		if(mySelf->m_AdaptiveOn)	{	mySelf->m_bDoUpdate = true;		}
		else						{	mySelf->m_bDoUpdate = false;	}


		// CKim - Exit loop if error between the setpoint and measured position is less than xx
		if( (loopTime/1000.0) > 20000.0 )		{		break;		}

	}
	mySelf->m_bDoUpdate = false;
	mySelf->m_bCLIK = false;
	ofstr.close();
	
	AfxMessageBox("Done");
	return 0;

}


unsigned int WINAPI	CCTRDoc::ClosedLoopControlLoop(void* para)
{
	// CKim - Implements Closed Loop Inverse Kinematics with feedback from sensor measurement. 

	// CKim - Pointer to self
	CCTRDoc* mySelf = (CCTRDoc*) para;		

	// CKim - Log files
	::std::string filename = "ExperimentData/" + mySelf->m_date + "-ClosedLoopChun.txt";

	std::ofstream ofstr;	
	ofstr.open(filename);

	// CKim - Parameters for loop speed measurement
	ChunTimer timer;	int perfcnt = 20;		int navg = 20;		long loopTime = 0;

	// CKim - Robot status and Haptic device status
	CTR_status localStat;		
	
	double predTipPosDir[6];		double kp = 10.0;

	mySelf->m_kinLib->SetInvKinThreshold(0.1,3.0);

	for(int i=0; i<5; i++)	{	localStat.initJang[i] = mySelf->m_Status.currJang[i];	}

	// CKim - The Loop
	timer.ResetTime();
	//::std::cout << "closed loop control" <<  ::std::endl;
	while(mySelf->m_playBack)
	{
		//::std::cout << "in playbacl loop " << ::std::endl;
		//// CKim - Synchronize on the update of the EM tracker data. Since the adaptive update will be 
		//// meaningful only when new EM tracker data and predicted tip posDir is available
		//// Use WaitForMultipleObjects to wait for this....
		int flag = WaitForSingleObject(mySelf->m_hEMevent,1000);
		if(flag == WAIT_TIMEOUT)	{
			AfxMessageBox("No event from EM Loop!");	return 0;
		}

		// CKim - Synch with haptic device. This is for spending 1 ms. 
		//mySelf->m_Omni->SynchState(localStat);
		
		// CKim - Read the shared variable (current cnt, jAng, .. ) that is used in this loop. 
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<7; i++)	{	localStat.currMotorCnt[i] = mySelf->m_Status.currMotorCnt[i];	}
		for(int i=0; i<5; i++)	{	localStat.currJang[i] = mySelf->m_Status.currJang[i];	}
		for(int i=0; i<6; i++)	{	localStat.currTipPosDir[i] = mySelf->m_Status.currTipPosDir[i];	}
		//for(int i=0; i<6; i++)	{	localStat.sensedTipPosDir[i] = mySelf->m_Status.sensedTipPosDir[i];		}
		for(int i=0; i<6; i++)	{	localStat.sensedTipPosDir[i] = mySelf->m_measPosforRec[i];		}
		LeaveCriticalSection(&m_cSection);

		// CKim - Safety check. Stop and exit loop if joint angle is out of limit
		if( (localStat.currJang[2] > L31_MAX) || (localStat.currJang[2] < L31_MIN) || (fabs(localStat.currJang[4]) > 100.0) )
		{
			mySelf->m_Status.isTeleOpMoving = false;
			AfxMessageBox("Joint out of limit!");	
			break;
		}

		//if (mySelf->m_adapt_LWPR)
		//	mySelf->m_kinLWPR->AdaptForwardModel(localStat.sensedTipPosDir, localStat.currJang);

		//mySelf->m_kinLWPR->TipFwdKin(localStat.currJang, predTipPosDir);

		// CKim - Read trajectory from the 'Playback.txt'. Returns false when the end of trajectory is reached
		mySelf->m_playBack = mySelf->m_TrjGen->InterpolateNextPoint(localStat.tgtTipPosDir);	// Update tgtTipPosDir
		
		// CKim - In case of feed forward position control, solve inverse kinematics
		if(mySelf->m_InvKinOn)
		{
			mySelf->SolveInverseKin(localStat);			// Updates localStat.tgtMotorCnt, tgtJang
		}
		else	// CKim - In case of differential kinematics control, enable flag
		{
			mySelf->m_bCLIK = true;	
		}
		
		if (mySelf->m_adapt_LWPR)
			mySelf->m_kinLWPR->AdaptForwardModel(localStat.sensedTipPosDir, localStat.currJang);

		// --------------------------------------------------------------- //
		// CKim - Log the data that was read
		// --------------------------------------------------------------- //
		if(perfcnt==navg)
		{
			loopTime = timer.GetTime();
			ofstr<<loopTime<<" ";
			for(int i=0; i<6; i++)	{	ofstr<<localStat.currTipPosDir[i]<<" ";		}
			for(int i=0; i<6; i++)	{	ofstr<<localStat.sensedTipPosDir[i]<<" ";	}
			for(int i = 0; i < 5; i++) { ofstr  << localStat.currJang[i] << " "; }
			//for(int i=0; i<6; i++)	{	ofstr<<predTipPosDir[i]<<" ";		}
			//for(int i=0; i<5; i++)	{	ofstr<<localStat.tgtJang[i]<<" ";			}
			//for(int i=0; i<7; i++)	{	ofstr<<localStat.tgtMotorCnt[i]<<" ";	}
			//ofstr<<localStat.invKinOK<<" ";
			//ofstr<<localStat.condNum<<" ";
			//ofstr<<localStat.limitOK<<" ";
			ofstr<<"\n";		
			perfcnt = 0;		//timer.ResetTime();
		}
		else	{	perfcnt++;	}


		// --------------------------------------------------------------- //
		// CKim - Update shared variable
		// --------------------------------------------------------------- //
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<6; i++)	{
			mySelf->m_Status.tgtTipPosDir[i] = localStat.tgtTipPosDir[i];		}
		for(int i=0; i<5; i++)	{
			mySelf->m_Status.tgtJang[i] = localStat.tgtJang[i];		}
		for(int i=0; i<7; i++)	{
			mySelf->m_Status.tgtMotorCnt[i] = localStat.tgtMotorCnt[i];		}

		mySelf->m_Status.invKinOK = localStat.invKinOK;
		mySelf->m_Status.limitOK = localStat.limitOK;
		mySelf->m_Status.isTeleOpMoving = (mySelf->m_InvKinOn || mySelf->m_bCLIK);
		mySelf->m_Status.condNum = localStat.condNum;
		mySelf->m_Status.gain = kp;
		mySelf->m_Status.invKinErr[0] = localStat.invKinErr[0];
		mySelf->m_Status.invKinErr[1] = localStat.invKinErr[1];
	
		LeaveCriticalSection(&m_cSection);
		
		if(mySelf->m_AdaptiveOn)	{	mySelf->m_bDoUpdate = true;		}
		else						{	mySelf->m_bDoUpdate = false;	}
		
		//////if(loopTime/1000.0 > 25000)
		//if(loopTime/1000.0 > 10000)
		//{	
		//	if(mySelf->m_FeedbackOn) {	mySelf->m_FeedbackOn = false;	}	
		//	if(mySelf->m_AdaptiveOn) {	mySelf->m_AdaptiveOn = false;	}
		//}


	}
	mySelf->m_bDoUpdate = false;
	mySelf->m_bCLIK = false;
	mySelf->m_playBack = false;
	ofstr.close();
	
	return 0;
	
}

unsigned int WINAPI	CCTRDoc::JointSpacePlayback(void* para)
{
	CCTRDoc* mySelf = (CCTRDoc*) para;		
	
	// CKim - Robot status and Haptic device status
	CTR_status localStat;		
	
	// CKim - Flags
	bool teleOpCtrl = false;		bool safeToTeleOp = false;			double scl = 1.0;	double kp = 10;

	bool adaptModelFlag = false;

	// CKim - Parameters for loop speed measurement
	ChunTimer timer;	int perfcnt = 0;	int navg = 50;		long loopTime = 0;

	for(int i=0; i<5; i++)	{	localStat.initJang[i] = mySelf->m_Status.currJang[i];	}

	// CKim - The Loop
	timer.ResetTime();
	double pos[5];

	while(mySelf->m_jointPlayback)
	{
	
		// CKim - Synch with haptic device. This is for spending 1 ms. 
		//mySelf->m_Omni->SynchState(localStat);
		
		// CKim - Read the shared variable (current cnt, jAng, .. ) that is used in this loop. 
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<7; i++)	{	localStat.currMotorCnt[i] = mySelf->m_Status.currMotorCnt[i];	}
		for(int i=0; i<5; i++)	{	localStat.currJang[i] = mySelf->m_Status.currJang[i];	}
		for(int i=0; i<6; i++)	{	localStat.currTipPosDir[i] = mySelf->m_Status.currTipPosDir[i];	}
		for(int i=0; i<6; i++)	{	localStat.sensedTipPosDir[i] = mySelf->m_measPosforRec[i];		}
		LeaveCriticalSection(&m_cSection);

		// CKim - Safety check. Stop and exit loop if joint angle is out of limit
		if( (localStat.currJang[2] > L31_MAX) || (localStat.currJang[2] < L31_MIN) || (fabs(localStat.currJang[4]) > 100.0) )
		{
			mySelf->m_Status.isTeleOpMoving = false;
			AfxMessageBox("Joint out of limit!");	
			break;
		}

		mySelf->m_TrjGen->InterpolateNextPoint(pos);
		pos[0] *= M_PI/180.0; pos[1] *= M_PI/180.0; pos[3] *= M_PI/180.0;

		for (int i = 0; i < 5; ++i)
			localStat.tgtJang[i] = pos[i];
		//memcpy(localStat.tgtJang , pos, 5 * sizeof(double));

		bool isInLim = true;
		mySelf->InvKinJangToMtr(pos, localStat.currMotorCnt, localStat.tgtJang, localStat.tgtMotorCnt, isInLim);

/*		::std::cout << "motors" << ::std::endl;		
		for (int i = 0; i < 7 ; ++i) {::std::cout << localStat.currMotorCnt[i] << " "; }
		::std::cout << ::std::endl;

		for (int i = 0; i < 7 ; ++i) {::std::cout << localStat.tgtMotorCnt[i] << " "; }
		::std::cout << ::std::endl;
		::std::cout << "angles" << ::std::endl;
		for (int i = 0; i < 5 ; ++i) {::std::cout << localStat.currJang[i] << " "; }
		::std::cout << ::std::endl;

		for (int i = 0; i < 5 ; ++i) {::std::cout << localStat.tgtJang[i] << " "; }
		::std::cout << ::std::endl;
*/
		if (isInLim) {safeToTeleOp = true;}

		//mySelf->MtrAngToCnt(localStat.tgtJang, localStat.tgtMotorCnt);

		// --------------------------------------------------------------- //
		// CKim - Update shared variable
		// --------------------------------------------------------------- //
		EnterCriticalSection(&m_cSection);
		for(int i=0; i<6; i++)	
			mySelf->m_Status.tgtTipPosDir[i] = localStat.tgtTipPosDir[i];	
		for(int i=0; i<5; i++)	
			mySelf->m_Status.tgtJang[i] = localStat.tgtJang[i];	
		for(int i=0; i<7; i++)	
			mySelf->m_Status.tgtMotorCnt[i] = localStat.tgtMotorCnt[i];	

		mySelf->m_Status.limitOK = localStat.limitOK;
		mySelf->m_Status.isTeleOpMoving = safeToTeleOp;
		mySelf->m_Status.condNum = localStat.condNum;
		mySelf->m_Status.gain = kp;
	
		LeaveCriticalSection(&m_cSection);
		
	}

	mySelf->m_bCLIK = false;
	
	
	return 0;

}

void CCTRDoc::MtrAngToCnt(const double* jA, double* cnt)
{
	cnt[0] = jA[0]/c_CntToMM;		cnt[1] = jA[1]/c_CntToMM;
	cnt[3] = jA[3]/c_CntToRad;		cnt[4] = jA[4]/c_CntToRad;		cnt[5] = jA[5]/c_CntToRad;
	cnt[2] = cnt[6] = 0;
}


void CCTRDoc::MtrCntToAng(const double* cnt, double* jA)
{
	jA[0] = cnt[0]*c_CntToMM;		jA[1] = cnt[1]*c_CntToMM;
	jA[3] = cnt[3]*c_CntToRad;		jA[4] = cnt[4]*c_CntToRad;		jA[5] = cnt[5]*c_CntToRad;
	jA[2] = jA[6] = 0;
}


void CCTRDoc::InvKinJangToMtr(const double* jA, const double* currCnt, double* tgtJang, double* tgtCnt, bool& isInLimFlag)
{
	double a1, a21, a31, L1, L31, L3;		double tmp;		double delta;

	// CKim - Joint angle solution from the inverse kinemtatics can be any of the periodic solution of the truncated fouriere series. 
	// So first, normalize the solution in their range. 

	// a1 : Rotation of tube 1 (outer tube of the balanced pair), Joint Angle [3], Motor Count [3]
	// This is in range of 0 to 2pi, so normalize. sin(th) = sin(th + 2npi), cos(th) = cos(th + 2npi)
	tgtJang[3] = a1 = jA[3] - 2.0*c_PI*floor(jA[3]/(2.0*c_PI));
	
	// a21 : Rotation of tube 2 (inner tube of the balanced pair) w.r.t. tube 1, Joint Angle [0], Motor Count [4-3]
	// This is in range of -pi to pi, so normalize using atan2, sin(th) = atan2(sin(th),cos(th)), cos(th) = atan2(sin(th),cos(th))
	tgtJang[0] = a21 = atan2(sin(jA[0]),cos(jA[0]));
	
	// a31 : Rotation of tube 3 w.r.t. tube 1, Joint Angle [1], Motor Count [5-3]
	// This is in range of -pi to pi, so normalize using atan2, sin(th) = atan2(sin(th),cos(th)), cos(th) = atan2(sin(th),cos(th))
	tgtJang[1] = a31 = atan2(sin(jA[1]),cos(jA[1]));
	
	// CKim - Calculating the motor count based on the normalized joint angle needs to consider current motor position. 
	// If current angular position is -539 degree (= -179 degree) and next position is 899 degree (= 179 degree), motor should turn 2 degree clockwise
	// instead of turning 358 degree counter clockwise. This is accomplished by using atan2 on difference.
	
	// Motor Count [3] : Rotation of tube 1 (outer tube of the balanced pair), a1 = jA[3]
	// Since a1 is in [0, 2pi], motor should move either clockwise or counter clockwise which ever is close from current location. 
	// Calculate the difference between current and new joint angle, normalize in [-pi, pi] using atan2, convert to counts, and add
	tmp = a1 - currCnt[3]*c_CntToRad;
	delta = atan2(sin(tmp),cos(tmp));
	tgtCnt[3] = currCnt[3] + delta/c_CntToRad;

	// Motor Count [4] : Rotation of tube 2 (inner tube of the balanced pair), a21 + a1 = jA[3+0]
	tmp = a21 + a1 - currCnt[4]*c_CntToRad;
	delta = atan2(sin(tmp),cos(tmp));
	tgtCnt[4] = currCnt[4] + delta/c_CntToRad;

	// Motor Count [5] : Rotation of tube 3, a31 + a1 = jA[3+1]
	tmp = a31 + a1 - currCnt[5]*c_CntToRad;
	delta = atan2(sin(tmp),cos(tmp));
	tgtCnt[5] = currCnt[5] + delta/c_CntToRad;


	// L1 : Translation of balanced pair, Joint Angle [4], Motor Count [0], No normalizatio needed for this. 
	L1 = jA[4];

	// L31 : Relative protrusion length of the tube 3 w.r.t. tube 1, Joint Angle [2] = Initial protrusion 'L31_MAX' -  Motor Count [0-1] 
	// This is normalized L31/L31_MAX*0.5*pi and passed to Fourier series. Therefore it could be possible that L31 we get from
	// inverse kinematics solution can be L31' such that L31'/L31_MAX*0.5*pi = L31/L31_MAX*0.5*pi + 2pi

	// POSSIBLE BUG!!!!
	tmp = jA[2]/L31_MAX*0.5*c_PI;
	L31 = atan2(sin(tmp),cos(tmp)) / (0.5*c_PI) * L31_MAX;
	//L31 = jA[2];

	// Motor Count [0] : Translation of balanced pair, L1 = jA[4].
	// Motor Count [1] : translation of tube 3, L3. L31 = L31_MAX - (L1 - L3). 
	// L3 should be determined from L1 and L31. But first, we apply joint angle limits	L31_MIN < L31 < L31_MAX
	if(L31 > L31_MAX)		{	L31 = L31_MAX;	isInLimFlag = false;		::std::cout << "L31 >"  <<std::endl;			}
	if(L31 < L31_MIN)		{	L31 = L31_MIN;	isInLimFlag = false;		::std::cout << "L31 <"  <<std::endl;			}
	
	// Second, since L1 and L3  is controlled by translation stage 0 and 1, both are limited to +-100 mm.
	// We limit L1 such that -100 < L1, L3 < 100.0
	// L3 < L1 = L31_MAX - L31 + L3 < 100.0
	// -100 < L3 = L31 - L31_MAX + L1 < L1, this leads to -100 + L31_MAX - L31 < L1
	if(L1 > 100.0)						{	L1 = 100.0;						isInLimFlag = false;	::std::cout << "L1 >"  <<std::endl;}
	if(L1 < (-100.0 + L31_MAX - L31))	{	L1 = (-100.0 + L31_MAX - L31);	isInLimFlag = false;	::std::cout << "L1 <"  <<std::endl;}
	
	// Calculate L3. Update joint angles and motor counts 
	L3 = L31 - L31_MAX + L1;
	tgtJang[2] = L31;			tgtJang[4] = L1;
	tgtCnt[0] = L1/c_CntToMM;	tgtCnt[1] = L3/c_CntToMM;

	// Motor 2,6 are not used
	tgtCnt[2] = tgtCnt[6] = 0;
		
}


void CCTRDoc::MtrToJang(const double* cnt, double* jA)
{
	double currAng[7];		MtrCntToAng(cnt,currAng);

	jA[0] = (currAng[4] - currAng[3]);			// a21 = tube 2 - tube 1 rotation
	jA[1] = (currAng[5] - currAng[3]);			// a31 = tube 3 - tube 1 rotation
	jA[2] = (currAng[1] - currAng[0]) + L31_MAX;	// t3 = tube 3 translation w.r.t balanced pair 
	jA[3] = currAng[3];							// a1 = balanced pair rotation
	jA[4] = currAng[0];							// l1 = nalanced pair translation

	// CKim - Normalize....
	jA[0] = atan2(sin(jA[0]),cos(jA[0]));
	jA[1] = atan2(sin(jA[1]),cos(jA[1]));
	jA[3] = jA[3] - 2.0*c_PI*floor(jA[3]/(2.0*c_PI));	
}


void CCTRDoc::dJangTodCnt(const double* dJ, double* dCnt)
{
	// CKim - dJ = { da21, da31, dL31, da1, dL1 }, dCnt = { L1, L3, 0, a1, a2, a3, 0 }
	double da1, da2, da3, dL1, dL3;			
	double scl, tmp;		double maxLinVel = 150.0;		double maxRotVel = 2.0*3.141592;

	da1 = dJ[3];			da2 = dJ[0] + da1;			da3 = dJ[1] + da1;
	dL1 = dJ[4];			dL3 = dJ[2] + dL1;

	dCnt[0] = dL1/c_CntToMM;		dCnt[1] = dL3/c_CntToMM;		dCnt[2] = 0;
	dCnt[3] = da1/c_CntToRad;		dCnt[4] = da2/c_CntToRad;		dCnt[5] = da3/c_CntToRad;		dCnt[6] = 0;

	scl = fabs(dL1)/maxLinVel;
	tmp = fabs(dL3)/maxLinVel;		if(scl < tmp)	{	scl = tmp;	}
	tmp = fabs(da1)/maxRotVel;		if(scl < tmp)	{	scl = tmp;	}
	tmp = fabs(da2)/maxRotVel;		if(scl < tmp)	{	scl = tmp;	}
	tmp = fabs(da3)/maxRotVel;		if(scl < tmp)	{	scl = tmp;	}
	
	if(scl > 1.0)
	{
		for(int i=0; i<7; i++)	{	dCnt[i]/=scl;	}
	}
}


void CCTRDoc::GetCurrentStatus(CTR_status& stat)
{
	// CKim - Update status variable. This is shared with the UI update. We need mutex here.
	// Using CriticalSection as this is slightly faster than mutex.
	EnterCriticalSection(&m_cSection);
	for(int i=0; i<6; i++)	{	
		stat.currTipPosDir[i] = m_Status.currTipPosDir[i];		stat.tgtTipPosDir[i] = m_Status.tgtTipPosDir[i];
		stat.bpTipPosDir[i] = m_Status.bpTipPosDir[i];		
	}
	for(int i=0; i<5; i++)	{
		stat.currJang[i] = m_Status.currJang[i];			stat.tgtJang[i] = m_Status.tgtJang[i];			}
	for(int i=0; i<7; i++)	{
		stat.currMotorCnt[i] = m_Status.currMotorCnt[i];	stat.tgtMotorCnt[i] =m_Status.tgtMotorCnt[i];
		stat.errFlag[i] = m_Status.errFlag[i];
	}
	for(int i=0; i<16; i++)	{	stat.hapticState.tfMat[i] = m_Status.hapticState.tfMat[i];		}
	stat.isTeleOpMoving = m_Status.isTeleOpMoving;
	stat.loopTime = m_Status.loopTime;
	stat.invKinOK = m_Status.invKinOK;
	stat.limitOK = m_Status.limitOK;
	stat.condNum = m_Status.condNum;
	stat.invKinErr[0] = m_Status.invKinErr[0];
	stat.invKinErr[1] = m_Status.invKinErr[1];

	for(int i=0; i<4; i++)	{
		for(int j=0; j<4; j++)	{	stat.emMat[i][j] = m_Status.emMat[i][j];	}	}

	for(int i=0; i<6; i++)	{	stat.sensedTipPosDir[i] = m_Status.sensedTipPosDir[i];	}
	LeaveCriticalSection(&m_cSection);
}


void CCTRDoc::SendCommand(int type, const double* para)
{
	CTR_cmd cmd;
	cmd.cmdType = type;	
	for(int i=0; i<10; i++)	{	cmd.para[i] = para[i];	}

	// CKim - Protected access to queue
	EnterCriticalSection(&m_cSection);
	m_cmdQueue.push(cmd);
	LeaveCriticalSection(&m_cSection);
}

void CCTRDoc::ClearCommandQueue()
{
	EnterCriticalSection(&m_cSection);
	m_cmdQueue.empty();
	LeaveCriticalSection(&m_cSection);
}

void CCTRDoc::SwitchTeleOpMode(bool onoff)
{
	m_motionCtrl->SetTeleOpMode(onoff);	
	m_teleOpMode = onoff;

	// Added
	if(onoff)
	{
		m_hTeleOpThread = (HANDLE) _beginthreadex(NULL, 0, CCTRDoc::TeleOpLoop, this, 0, NULL);
		
	}
	else
	{
		// switch by default force control
		//this->m_forceControlActivated = false;

		if( WaitForSingleObject(m_hTeleOpThread,1000) )	// CKim - Did not return 0
		{
			AfxMessageBox("Sparta!! TeleOp thread");
			this->m_motionCtrl->DumpConfiguration();
		}
		this->m_bCLIK = false;
	}
}


void CCTRDoc::SwitchPlayBackMode(bool onoff)
{
	m_motionCtrl->SetTeleOpMode(onoff);	
	
	//m_teleOpMode = onoff;


}



void CCTRDoc::SwitchJointPlayBackMode(bool onoff)
{
	m_jointPlayback = onoff;	
	m_motionCtrl->SetTeleOpMode(onoff);	
}

void CCTRDoc::SwitchStaticPlayBackMode(bool onoff)
{

	m_bStaticPlayBack = onoff;	
	m_motionCtrl->SetTeleOpMode(onoff);	
}


bool CCTRDoc::ProcessCommand(CTR_status& stat)
{
	if(m_cmdQueue.empty())	{	return true;		}

	// CKim - Access command queue
	EnterCriticalSection(&m_cSection);
	CTR_cmd cmd = m_cmdQueue.front();
	m_cmdQueue.pop();
	LeaveCriticalSection(&m_cSection);
			
	// CKim - I need to check type of command but now it is just commanded joint angle
	if(cmd.cmdType == 0)		// CKim - Commanded joint angle
	{
		double jAng[5];
		for(int i=0; i<5; i++)	{	jAng[i] = cmd.para[i];	}
		bool isInLimit = (jAng[2] >= L31_MIN && jAng[2]<=L31_MAX && jAng[4] <= 100 && (-24-jAng[2]) <= jAng[4] );
		isInLimit = true;

		if(isInLimit)
		{
			for(int i=0; i<5; i++)	{	stat.tgtJang[i] = jAng[i];	}
			InvKinJangToMtr(stat.tgtJang, stat.currMotorCnt, stat.tgtJang, stat.tgtMotorCnt, stat.limitOK);
			
			if(!m_motionCtrl->DoCoordMotion(stat.tgtMotorCnt))	
			{
				AfxMessageBox("Error during commanded motion!!");
				this->m_motionCtrl->DumpConfiguration();
				return false;
			}
			
			// Added.... 
			EnterCriticalSection(&m_cSection);
			for(int i=0; i<5; i++)	{	m_Status.tgtJang[i] = jAng[i];	}
			for(int i=0; i<7; i++)	{	m_Status.tgtMotorCnt[i] = stat.tgtMotorCnt[i];	}
			LeaveCriticalSection(&m_cSection);

		}
	}

	else if(cmd.cmdType == 1)	// CKim - Commanded tip configuration
	{
		for(int i=0; i<5; i++)	{	stat.initJang[i] = stat.currJang[i];	}

		for(int i=0; i<3; i++)	
		{	stat.tgtTipPosDir[i] = cmd.para[i];		stat.tgtTipPosDir[i+3] = cmd.para[i+3];		}
				
		SolveInverseKin(stat);
		
		if(stat.invKinOK)
		{
			if(!m_motionCtrl->DoCoordMotion(stat.tgtMotorCnt))	
			{
				AfxMessageBox("Error during commanded motion!!");
				return false;
			}
		}
		
		EnterCriticalSection(&m_cSection);

		for(int i=0; i<6; i++)	{
			m_Status.tgtTipPosDir[i] = stat.tgtTipPosDir[i];		}
		for(int i=0; i<5; i++)	{
			m_Status.tgtJang[i] = stat.tgtJang[i];		}
		for(int i=0; i<7; i++)	{
			m_Status.tgtMotorCnt[i] = stat.tgtMotorCnt[i];		}

		m_Status.invKinOK = stat.invKinOK;
		m_Status.limitOK = stat.limitOK;
		m_Status.condNum = stat.condNum;
		m_Status.invKinErr[0] = stat.invKinErr[0];
		m_Status.invKinErr[1] = stat.invKinErr[1];
	
		LeaveCriticalSection(&m_cSection);

	}
	
	return true;
}


void CCTRDoc::SolveInverseKin(CTR_status& stat)
{
	// CKim - Numerically solves inverse kinematics, converts it to joint angle and checks limit. 
	double jAng[5];		bool isConverged = false;		double mtrCnt[7];		bool isInLimit = true;
	double Err[3];		int exitCond;					double maxPosErr, maxOrtErr;
	
	bool isInLimitLWPR = true;

	// here append code for inverse kinematics!!!!
	m_kinLib->GetInvKinThreshold(maxPosErr,maxOrtErr);

	// CKim - Update initial point - 1: as current joint angle
	for(int i=0; i<5; i++)	{	stat.initJang[i] = stat.currJang[i];		}

	//m_kinLWPR->InverseKinematicsLSQ(stat.tgtTipPosDir, stat.initJang, jAng, Err, exitCond);
	//m_kinLib->InverseKinematicsLSQ(stat.tgtTipPosDir, stat.initJang, jAng, Err, exitCond);
	//m_kinLWPR->runOptimizationController(stat.currJang, stat.tgtTipPosDir, jAng);
	
	//memcpy(tempSolution, localStat.tgtJang, 5 * sizeof(double));
	stat.condNum = Err[0];		stat.invKinErr[0] = Err[1];		stat.invKinErr[1] = Err[2];
	//::std::cout << Err[0] << ::std::endl;
	// CKim - Check convergence from the position and orientation error magnitude
	if( (stat.invKinErr[0] > maxPosErr) || (stat.invKinErr[1] > maxOrtErr)  )	{	isConverged = false;	}
	else																		{	isConverged = true;		}
	

	// CKim - Update initial point - 2: to a last solution
	for(int i=0; i<5; i++)	{	stat.initJang[i] = stat.currJang[i];		}
	
	// CKim - Normalize, Apply Joint limits and covert to motor counts. 
	InvKinJangToMtr(jAng,stat.currMotorCnt,stat.tgtJang,stat.tgtMotorCnt, isInLimit);
	//for (int i = 0; i < 5; ++i)
	//	::std::cout << (stat.currMotorCnt[i] - stat.tgtMotorCnt[i]) * c_CntToRad << " ";
	//
	//::std::cout << ::std::endl;
	stat.exitCond = exitCond;		stat.invKinOK = isConverged;		stat.limitOK = isInLimit;
}
void CCTRDoc::computeMechanicsKinematics(CTR_status stat)
{
	double configuration[5] = {0};
	memcpy(configuration, stat.currJang, 5 * sizeof(double));

	double rotation[3] = {0};
	double translation[3] = {0};
	MechanicsBasedKinematics::RelativeToAbsolute(this->robot, configuration, rotation, translation);
	this->kinematics->ComputeKinematics(rotation, translation);

}

void CCTRDoc::GetTipTransformation(::Eigen::Matrix<double, 3, 3>& trans)
{
	double configuration[5] = {0};
	memcpy(configuration, this->m_Status.currJang, 5 * sizeof(double));

	double rotation[3] = {0};
	double translation[3] = {0};
	MechanicsBasedKinematics::RelativeToAbsolute(this->robot, configuration, rotation, translation);
	this->kinematics->ComputeKinematics(rotation, translation);

	SE3 tipFrame;
	this->kinematics->GetBishopFrame(tipFrame);

	SO3ToEigen(tipFrame.GetOrientation(), trans);	

}

void CCTRDoc::GetImageToCameraTransformation(::Eigen::Matrix<double, 3, 3>& trans)
{
	trans = ::Eigen::AngleAxis<double>(-this->kinematics->GetInnerTubeRotation(), ::Eigen::Vector3d::UnitZ());
}


void CCTRDoc::MasterToSlave(CTR_status& stat, double scl, bool absolute)
{

	//::std::cout << stat.hapticState.velocity[0] << " " << stat.hapticState.velocity[1] << " " << stat.hapticState.velocity[2] << ::std::endl;
	// CKim - Relative tip position - displacement of the stylus T from its reference point T0
	// (location when the button was pressed, w.r.t master csys, stored in M_T and M_T0)
	// is scaled and mapped to position of the slave end-effector
	Eigen::Vector3d to, t, p, tipPos;	Eigen::Matrix3d MtoS, MtoSOr;
	for(int i=0; i<3; i++)	
	{
		to(i) = stat.M_T0[12+i];
		t(i) = stat.hapticState.tfMat[12+i];
		p(i) = stat.refTipPosDir[i];	
	}
	
	// update current and previous positions
	memcpy(stat.hapticState.previousPosition , stat.hapticState.position, 3  * sizeof(double));
	memcpy(stat.hapticState.position, t.data(), 3  * sizeof(double));


	MtoS(0,0) =	0;		MtoS(0,1) =	1;		MtoS(0,2) = 0;
	MtoS(1,0) =	1;		MtoS(1,1) =	0;		MtoS(1,2) =	0;
	MtoS(2,0) =	0;		MtoS(2,1) = 0;		MtoS(2,2) = -1;
	
	MtoSOr(0,0) =	0;		MtoSOr(0,1) =	1;		MtoSOr(0,2) = 0;
	MtoSOr(1,0) =	0;		MtoSOr(1,1) =	0;		MtoSOr(1,2) =	-1;
	MtoSOr(2,0) =	-1;		MtoSOr(2,1) = 0;		MtoSOr(2,2) = 0;

	MtoS = MtoSOr;
	tipPos = MtoS*scl*(t-to) + p;

	::Eigen::Vector3d velocityRobotFrame = MtoS * scl * ::Eigen::Map<::Eigen::Vector3d>(stat.hapticState.velocity,3);
	::Eigen::Vector3d ang_velocityRobotFrame = MtoS * ::Eigen::Map<::Eigen::Vector3d>(stat.hapticState.ang_velocity,3);

	// CKim - Relative tip orientation - Rotation of the stylus R from its reference orientation Ro
	// (location when the button was pressed, w.r.t reference stylus csys, Ro'*R.
	// Ref tip direction (v) is mapped transfromed by Ro'*R in slave csys. (MtoS*Ro'*R*MtoS');
	Eigen::Vector3d v,z, tipDir;	Eigen::Matrix3d Ro, R;		z(0) = z(1) = 0;	z(2) = 1;
	for(int i=0; i<3; i++)	{	v(i) = stat.refTipPosDir[i+3];
		for(int j=0; j<3; j++)	
		{
			R(j,i) = stat.hapticState.tfMat[4*i+j];	
			Ro(j,i) = stat.M_T0[4*i+j];
	}		}
	
	absolute = true;

	if(absolute)
	{
		tipDir = MtoSOr*R*(-z);
		//tipDir = MtipToBase*MtoS*R*(-z);
	}
	else
	{
		tipDir = MtoS*Ro.transpose()*R*MtoS.transpose()*v;
	}
	//::std::cout << velocityRobotFrame.transpose() << ::std::endl; 
	for(int i=0; i<3; i++)	{	stat.tgtTipPosDir[i] = tipPos(i);		stat.tgtTipPosDir[i+3] = tipDir(i);		}

	memcpy(stat.tgtWorkspaceVelocity, velocityRobotFrame.data(), 3  * sizeof(double));
	memcpy(stat.tgtWorkspaceAngVelocity, ang_velocityRobotFrame.data(), 3  * sizeof(double));
	//PrintCArray(stat.tgtWorkspaceAngVelocity, 3);
}


void CCTRDoc::MasterToSlave(const CTR_status& stat, double* slavePosDir, double scl, bool absolute)
{
	// CKim - Relative tip position - displacement of the stylus T from its reference point T0
	// (location when the button was pressed, w.r.t master csys, stored in M_T and M_T0)
	// is scaled and mapped to position of the slave end-effector
	Eigen::Vector3d to, t, p, tipPos;	
	for(int i=0; i<3; i++)	{
		to(i) = stat.M_T0[12+i];	t(i) = stat.hapticState.tfMat[12+i];	p(i) = stat.refTipPosDir[i];	}
	


	Eigen::Matrix3d MtoS;
	MtoS(0,0) =	0;		MtoS(0,1) =	1;		MtoS(0,2) = 0;
	MtoS(1,0) =	0;		MtoS(1,1) =	0;		MtoS(1,2) =	-1;
	MtoS(2,0) =	-1;		MtoS(2,1) = 0;		MtoS(2,2) = 0;

	tipPos = MtoS*scl*(t-to) + p;

	// CKim - Relative tip orientation - Rotation of the stylus R from its reference orientation Ro
	// (location when the button was pressed, w.r.t reference stylus csys, Ro'*R.
	// Ref tip direction (v) is mapped transfromed by Ro'*R in slave csys. (MtoS*Ro'*R*MtoS');
	Eigen::Vector3d v,z, tipDir;	Eigen::Matrix3d Ro, R;		z(0) = z(1) = 0;	z(2) = 1;
	for(int i=0; i<3; i++)	{	v(i) = stat.refTipPosDir[i+3];
		for(int j=0; j<3; j++)	{
			R(j,i) = stat.hapticState.tfMat[4*i+j];		Ro(j,i) = stat.M_T0[4*i+j];		}		}
	
	absolute = true;

	if(absolute)	{	tipDir = MtoS*R*(-z);	}
	else
	{
		tipDir = MtoS*Ro.transpose()*R*MtoS.transpose()*v;
		//tipDir = MtoS*Ro.transpose()*R*MtoS.transpose()*(-z);
	}

	for(int i=0; i<3; i++)	{	slavePosDir[i] = tipPos(i);		slavePosDir[i+3] = tipDir(i);		}
}


void CCTRDoc::SlaveToMaster(CTR_status& stat, double scl)
{
	// CKim - Relative tip position - displacement of the stylus T from its reference point T0
	// (location when the button was pressed, w.r.t master csys, stored in M_T and M_T0)
	// is scaled and mapped to position of the slave end-effector
	Eigen::Vector3d to, t, p, tipPos;	Eigen::Matrix3d MtoS;
	for(int i=0; i<3; i++)	
	{
		to(i) = stat.M_T0[12+i];	p(i) = stat.refTipPosDir[i];	tipPos(i) = stat.currTipPosDir[i];
	}
	MtoS(0,0) =	0;		MtoS(0,1) =	1;		MtoS(0,2) = 0;
	MtoS(1,0) =	1;		MtoS(1,1) =	0;		MtoS(1,2) =	0;
	MtoS(2,0) =	0;		MtoS(2,1) = 0;		MtoS(2,2) = -1;

	::Eigen::Matrix<double, 3, 3> MtipToBase;
	//this->GetTipTransformation(MtipToBase);
	MtipToBase.setIdentity();
	//MtoS(0,0) =	0;		MtoS(0,1) =	1;		MtoS(0,2) = 0;
	//MtoS(1,0) =	0;		MtoS(1,1) =	0;		MtoS(1,2) =	-1;
	//MtoS(2,0) =	-1;		MtoS(2,1) = 0;		MtoS(2,2) = 0;

	//t = MtoS.transpose()*(tipPos - p)/scl + to;
	t =  MtoS.transpose() * MtipToBase.transpose()*(tipPos - p)/scl + to;

	for(int i=0; i<3; i++)	
	{
		stat.hapticState.slavePos[i] = t(i);
	}
}


void CCTRDoc::SlaveToMaster(const CTR_status& stat, double* proxyPos, double scl, bool absolute)
{
	// CKim - Relative tip position - displacement of the stylus T from its reference point T0
	// (location when the button was pressed, w.r.t master csys, stored in M_T and M_T0)
	// is scaled and mapped to position of the slave end-effector
	Eigen::Vector3d to, t, p, tipPos;	Eigen::Matrix3d MtoS;
	for(int i=0; i<3; i++)	
	{
		to(i) = stat.M_T0[12+i];	p(i) = stat.refTipPosDir[i];	tipPos(i) = stat.currTipPosDir[i];
	}

	MtoS(0,0) =	0;		MtoS(0,1) =	1;		MtoS(0,2) = 0;
	MtoS(1,0) =	0;		MtoS(1,1) =	0;		MtoS(1,2) =	-1;
	MtoS(2,0) =	-1;		MtoS(2,1) = 0;		MtoS(2,2) = 0;

	t = MtoS.transpose()*(tipPos - p)/scl + to;
	for(int i=0; i<3; i++)	{	proxyPos[i] = t(i);		}
}


bool CCTRDoc::TeleOpSafetyCheck()
{
	//// CKim - Check the distance between the current and released configuration
	//// It issafe to teleop if the frame is brought back close to where it was released
	//// Trace(A'*B) = 1+2cos(th) must be close to 3
	//if( teleOpCtrl && !safeToTeleOp)
	//{
	//	double distOrt = 0;		double distTrans = 0;
	//	for(int i=0; i<3; i++)	{	distTrans += pow((tgtPos[i] - currPos[i]),2);	}
	//	for(int i=0; i<3; i++)	{	distOrt += (tgtOrt[i]*currOrt[i]);	}
	//	distOrt = acos( distOrt ) * 180.0/3.141592;
	//
	//	if( ( sqrt(distTrans) < 5.0 ) && ( fabs(distOrt) < 30.0 )  )
	//	{
	//		safeToTeleOp = true;	
	//		// End haptic guidance
	//	}
	//}
	return true;
}	

void CCTRDoc::ToggleForceControl()
{
	this->m_forceControlActivated = !this->m_forceControlActivated;
	::std::cout << this->m_forceControlActivated << ::std::endl;
	//EnterCriticalSection(&m_cSection);
	//memcpy(this->m_Status.tgtTipPosDir, this->m_Status.currTipPosDir, 6 * sizeof(double));
	////memcpy(this->m_Status.refTipPosDir, this->m_Status.currTipPosDir, 6 * sizeof(double));
	//LeaveCriticalSection(&m_cSection);
//
//	this->SlaveToMaster(this->m_Status, 1);		// Updates robotStat.hapticState.slavePos}
//}
}

void CCTRDoc::TogglePlaneEstimation()
{
	this->m_compute_plane = !this->m_compute_plane;
}

void CCTRDoc::ToggleCameraControl()
{
	this->m_camera_control = !this->m_camera_control;
}

::Eigen::Vector3d
CCTRDoc::GetTipPosition()
{
	return Eigen::Map<::Eigen::Vector3d> (this->m_Status.currTipPosDir, 3);
}

void CCTRDoc::IncrementalPlaneUpdate()
{
	double kappa = 1.0;

	::Eigen::Vector3d tmpPosition = this->GetTipPosition();
	::Eigen::Vector2d phi_k = tmpPosition.segment(0, 2);

	double epsilon_k = tmpPosition(2) - m_plane_coefficients.transpose() * phi_k;

	m_plane_covar = 1.0/kappa * (m_plane_covar - 1.0/(kappa + phi_k.transpose() * m_plane_covar * phi_k) * m_plane_covar * phi_k * phi_k.transpose() * m_plane_covar);

	m_plane_coefficients += epsilon_k * m_plane_covar * m_plane_coefficients;

	::std::cout << m_plane_coefficients << ::std::endl;
}


void
CCTRDoc::computeHapticDisplacement(CTR_status stat, double dP[3])
{
	for(int i = 0; i < 3; ++i)
		dP[i] = stat.hapticState.position[i] - stat.hapticState.previousPosition[i];
}

void
CCTRDoc::computeCameraDesiredMotion(CTR_status stat, const double haptic_dir[3], double camera_dir[3])
{
	// from haptic to image frame
	::Eigen::Matrix3d RImageToHaptic;
	RImageToHaptic << 0, 1, 0,
					  0, 0,-1,
					 -1, 0, 0;
	::Eigen::Vector3d image_dir = RImageToHaptic * ::Eigen::Map < ::Eigen::Vector3d> (const_cast<double*> (haptic_dir), 3);

	/* from image to camera frame */
	// solve kinematics
	this->computeMechanicsKinematics(stat);

	// compute transformation
	::Eigen::Matrix3d RCameraToImage;
	GetImageToCameraTransformation(RCameraToImage);

	::Eigen::Matrix3d RTipToCamera;
	RTipToCamera = ::Eigen::AngleAxis<double>(-9.9 * M_PI/180.0, ::Eigen::Vector3d::UnitY());

	// compute camera desired direction of motion
	::Eigen::Matrix3d RbaseToTip;
	SE3 bodyFrame;
	this->kinematics->GetBodyFrame(bodyFrame);
	SO3ToEigen(bodyFrame.GetOrientation(), RbaseToTip);
	::Eigen::Vector3d camera_dir_Eigen = RbaseToTip * RTipToCamera * RCameraToImage * image_dir;
	memcpy(camera_dir, camera_dir_Eigen.data(), 3 * sizeof(double));
}

void
CCTRDoc::computeCameraJacobian(CTR_status stat)
{
}

void 
CCTRDoc::setContactControlNormal(const ::Eigen::Vector3d& computedNormal, const ::Eigen::Vector3d& center, double radius, ::std::vector<::Eigen::Vector3d> pts)
{
	for(int i = 0; i < 3; i++)
	{
		this->m_contact_control_normal(i) = computedNormal(i);
		this->m_valve_center[i] = center(i);
		this->m_radius = radius;
	}
	points_for_plane_estimation = pts;
	this->m_plane_changed = true;
}

void 
CCTRDoc::ToggleLog()
{
	

	if (this->m_logData)
	{
		//this->m_fileStream->close();
		this->m_heartRateMonitor->toggleLog(false);
		this->m_logData = false;
	}
	else
	{
		//::std::string filename = GetDateString() + "1"+ ".txt";
		//this->m_fileStream = new ::std::ofstream(filename);
		this->m_logData = true;
		this->m_heartRateMonitor->toggleLog(true);
	}
}


void CCTRDoc::UpdateGains(double position, double orientation, double position_forward, double orientation_forward)
{
	this->m_position_gain = position;
	this->m_orientation_gain = orientation;
	this->m_position_gain_feedforward = position_forward;
	this->m_orientation_gain_feedforward = orientation_forward;
}

void CCTRDoc::SwitchControlMode(int mode)
{
	this->m_control_mode = mode;

}

void CCTRDoc::SwitchFreqMode(int mode)
{
	this->m_freq_mode = mode;
}

double CCTRDoc::GetMonitorFreq()
{
	return this->m_heartRateMonitor->getHeartRate();
}

void CCTRDoc::computeCircumnavigationDirection(Eigen::Matrix<double,6,1>& err)
{

	//if (!m_line_detected)
	//{
	//	err.setZero();
	//	return;
	//}

	::std::cout << "centroid:[" << m_centroid[0] << "," << m_centroid[1] << "]" << ::std::endl; 
	::std::cout << "tangent:[" << m_valve_tangent[0] << "," << m_valve_tangent[1] << "]" << ::std::endl; 

	// later this needs to be computed from the plane normal
	::Eigen::Matrix3d rot = ::Eigen::Matrix3d::Identity();

	::Eigen::Vector2d error;
	error = m_image_center - ::Eigen::Map<::Eigen::Vector2d> (m_centroid, 2);
	error /= m_scaling_factor;
	error *= -this->m_gain_center;

	m_direction = 1;
	error += this->m_gain_tangent * m_direction * ::Eigen::Map<::Eigen::Vector2d> (m_valve_tangent,2);

	::Eigen::Vector3d error3D;
	error3D.segment(0, 2) = error;
	error3D(2) = 0.0;

	err.block(0, 0, 3, 1) = rot * error3D;
	
	::std::cout << "velocities:" << err.block(0, 0, 3, 1).transpose() << ::std::endl;
	//err.setZero();
}

// BE CAREFULL TO SWITCH ON/OFF THE CIRCUM
void CCTRDoc::ToggleCircumnavigation()
{
	this->m_circumnavigation = !this->m_circumnavigation;

	if (this->m_circumnavigation)
		memcpy(this->tip_position_prev, this->m_Status.currTipPosDir, 2 *sizeof(double));

	if (this->m_circumnavigation)
		this->tangent_updates = 0;

	if (this->m_circumnavigation)
	{
		this->valve_points_visited.clear();
		this->index = 0;
	}
	::std::cout << "circumnavigation: ";
	(this->m_circumnavigation ?	::std::cout << "ON" : ::std::cout << "OFF");
	::std::cout << ::std::endl;
}

void CCTRDoc::ToggleApexToValve()
{


	this->m_apex_to_valve = !this->m_apex_to_valve;


	::std::cout << "apex-to-valve navigation: ";
	(this->m_apex_to_valve ?	::std::cout << "ON" : ::std::cout << "OFF");
	::std::cout << ::std::endl;
}

void CCTRDoc::UpdateCircumnavigationParams(::std::vector<double>& msg)
{
	this->retractRobot = false;

	double dt = static_cast<double> (circumTimer.GetTime())/1.e06;
	circumTimer.ResetTime();

	this->centroid_prev[0] = m_centroid[0];
	this->centroid_prev[1] = m_centroid[1];

	for (int i = 0; i < 2; ++i)
		this->tip_velocity[i] = (this->m_Status.currTipPosDir[i] - this->tip_position_prev[i])/dt;

	m_centroid[0] = msg[3];
	m_centroid[1] = msg[4];

	for (int i = 0; i < 2; ++i)
		this->centroid_velocity[i] = (this->m_centroid[i] - this->centroid_prev[i])/dt/this->m_scaling_factor; // centroid velocity in mm/sec

	::Eigen::Vector2d vel_cen = ::Eigen::Map<::Eigen::Vector2d> (this->centroid_velocity, 2);
	::Eigen::Vector2d vel_tip = ::Eigen::Map<::Eigen::Vector2d> (this->tip_velocity, 2);

	if (vel_cen.norm() < 0.5 * vel_tip.norm())
		this->retractRobot = true;

	memcpy(m_valve_tangent_prev, m_valve_tangent, 2 * sizeof(double));

	m_valve_tangent[0]  = msg[5];
	m_valve_tangent[1]  = msg[6];

	double tmp = m_valve_tangent_prev[0] * m_valve_tangent[0] + m_valve_tangent_prev[1] * m_valve_tangent[1];

	if (tmp < 0)
	{
		m_valve_tangent[0] *= -1;
		m_valve_tangent[1] *= -1;
	}
	if (tangent_updates == 0)
		computeInitialDirection();
	tangent_updates++;

	memcpy(this->tip_position_prev, this->m_Status.currTipPosDir, 2 *sizeof(double));
}

void CCTRDoc::SetVSGains(double gain_center, double gain_tangent) 
{
	this->m_gain_center = gain_center; 
	this->m_gain_tangent = gain_tangent;
	::std::cout << m_gain_center << ::std::endl;
	::std::cout << m_gain_tangent << ::std::endl;
}

void CCTRDoc::OnBnClickedGoToApex()
{
	::std::cout << "Go to apex" << ::std::endl; 
	PrintCArray(this->m_apex_coordinates,5);

	if (this->m_apex)
	{
		::std::cout <<"active"<<::std::endl;
		this->SendCommand(0, m_apex_coordinates);
	}
}

void CCTRDoc::computeApexToValveMotion(Eigen::Matrix<double,6,1>& err, APEX_TO_VALVE_STATUS aStatus)
{
	//::std::cout << "activating circumnavigation" << ::std::endl;
	switch(aStatus)
	{
	case APEX_TO_VALVE_STATUS::LEFT:
			//::std::cout << "activate left" << ::std::endl;
			computeATVLeft(err);
			break;
	case APEX_TO_VALVE_STATUS::TOP:
			computeATVTop(err);
			break;
	case APEX_TO_VALVE_STATUS::BOTTOM:
			computeATVBottom(err);
			break;
	}

}

void CCTRDoc::computeATVLeft(Eigen::Matrix<double,6,1>& err)
{
	//::std::cout << "left Apex-to-valve" << ::std::endl;
	// left bias
	err[1] = -this->m_bias;

	// forward velocity
	err[2] = m_forward_gainATV;    // mm/sec
	//::std::cout <<"centroid in controller  :" <<  this->m_centroid_apex[0] << " " << this->m_centroid_apex[1] << ::std::endl;
	// check controller
	if (this->m_centroid_apex[1] >= m_apex_theshold_max)
		err[1] += m_center_gainATV/m_scaling_factor * (this->m_centroid_apex[1] - m_apex_theshold_max);
	else if (this->m_centroid_apex[1] <= m_apex_theshold_min)
		err[1] += m_center_gainATV/m_scaling_factor * (this->m_centroid_apex[1] - m_apex_theshold_min);

	//::std::cout << "vel from aTV: " << err.transpose() << ::std::endl;
}


void CCTRDoc::computeATVTop(Eigen::Matrix<double,6,1>& err)
{
	// Upward bias
	err[0] = this->m_bias;

	// forward velocity
	err[2] = m_forward_gainATV;    // mm/sec

	// check controller

	if (this->m_centroid_apex[0] <= 250 - m_apex_theshold_max)
		err[0] += m_center_gainATV/m_scaling_factor * (this->m_centroid_apex[0] - 255 + m_apex_theshold_max);
	else if (this->m_centroid_apex[0] >= m_apex_theshold_min)
		err[0] += m_center_gainATV/m_scaling_factor * (this->m_centroid_apex[0] - 255 + m_apex_theshold_min);
}

void CCTRDoc::computeATVBottom(Eigen::Matrix<double,6,1>& err)
{
	// Downward bias
	err[0] = -this->m_bias;

	// forward velocity
	err[2] = m_forward_gainATV;    // mm/sec

	// check controller
	if (this->m_centroid_apex[0] >= m_apex_theshold_max)
		err[0] += m_center_gainATV/m_scaling_factor * (this->m_centroid_apex[0] - m_apex_theshold_max);
	else if (this->m_centroid_apex[1] <= m_apex_theshold_min)
		err[0] += m_center_gainATV/m_scaling_factor * (this->m_centroid_apex[0] - m_apex_theshold_min);

}


double	CCTRDoc::computeInverseApproxJacovianCR(double currentCR)
{
	double result = 0;
	double coeffs[6] = {-4.30643464102747e-17, -5.95983404918811e-16, 2.47429266802908, 0.100012827571416, -2.62290365923007, 1.04859816362958};
    if (currentCR > 0.01 && currentCR < 0.99)
        result = coeffs[2] + 2*coeffs[3] * currentCR + 3*coeffs[4] * ::std::pow(currentCR, 2)+ 4*coeffs[5] * ::std::pow(currentCR, 3)+ 5*coeffs[6] * ::std::pow(currentCR, 4);
    else
        result = 0.1;
	
	return result;
}


void	CCTRDoc::ToggleJacobianContactControl()
{
	this->m_useJacobianContactControl = !this->m_useJacobianContactControl;
	
	::std::cout << "use global gains: ";
	(this->m_useJacobianContactControl ?	::std::cout << "ON" : ::std::cout << "OFF");
	::std::cout << ::std::endl;
}

void	CCTRDoc::UpdateGainsApexToValve(double center, double forward, double threshold_min, double threshold_max)
{
	this->m_apex_theshold_min = threshold_min;
	this->m_apex_theshold_max = threshold_max;
	this->m_center_gainATV = center;
	this->m_forward_gainATV = forward;

	::std::cout << "Apex-to-valve gains updated" << ::std::endl;
	::std::cout << "minimum threshold:" << this->m_apex_theshold_min << ::std::endl;
	::std::cout << "maximum threshold:" << this->m_apex_theshold_max << ::std::endl;

	::std::cout << "center gain:" << this->m_center_gainATV << ::std::endl;
	::std::cout << "forward gain:" << this->m_forward_gainATV << ::std::endl;
}

void CCTRDoc::OnBnClickedGoToFirst()
{
	if (this->valve_points_visited.size() > 0)
	{
		this->SendCommand(0, this->valve_points_visited.front().data());
		this->index = 0;
	}
}

void CCTRDoc::OnBnClickedGoToLast()
{
	if (this->valve_points_visited.size() > 0)
	{
		this->SendCommand(0, this->valve_points_visited.back().data());
		this->index = this->valve_points_visited.size() - 1;
	}
}

void CCTRDoc::OnBnClickedGoToPrev()
{
	if (this->valve_points_visited.size() > 0)
	{
		this->index--;
		if (this->index >= 0)
			this->SendCommand(0, this->valve_points_visited[this->index].data());
	}
}

void CCTRDoc::OnBnClickedGoToNext()
{
	if (this->valve_points_visited.size() > 0)
	{
		this->index++;
		if (this->index < this->valve_points_visited.size())
			this->SendCommand(0, this->valve_points_visited[this->index].data());
	}
}

void CCTRDoc::OnBnClickedStartId()
{
	m_motionCtrl->SetTeleOpMode(true);	

	activateIdentification = true;
	timerId.ResetTime();
	reference_translation = m_Status.currJang[4];
	reference_translation_inner = m_Status.currJang[2];
	::std::cout << m_idMode << " " << activateIdentification << ::std::endl;
	::std::cout << "reference translation: " << reference_translation << ::std::endl;
}

void CCTRDoc::OnBnClickedStopId()
{
	m_motionCtrl->SetTeleOpMode(false);	

	activateIdentification = false;
	double joints[5];
	memcpy(joints, m_Status.currJang, 5*sizeof(double));
	joints[4] = reference_translation;
	joints[2] = reference_translation_inner;
	
	SendCommand(0, joints);
}

void CCTRDoc::UpdateIDParams(double min_freq, double max_freq, double amplitude, int num_of_sins)
{
	this->min_frequency = min_freq;
	this->max_frequency = max_freq;
	this->amplitude = amplitude;
	this->num_of_sins = num_of_sins;
	
	if (num_of_sins < 2)
		num_of_sins = 2;

	frequencies.clear();
	frequencies.resize(num_of_sins);

	for(int i = 0 ; i < num_of_sins; ++i)
	{
		frequencies[i] = min_frequency + (max_frequency - min_frequency) * (double)i / (double)(num_of_sins-1);
		::std::cout << frequencies[i] << ::std::endl;
	}


}

void 
CCTRDoc::OnBnClickedResetAutomation()
{
	CFrameWnd * pFrame = (CFrameWnd *) (AfxGetApp()->m_pMainWnd);
	pFrame->GetActiveView()->CheckDlgButton(IDC_EDIT1, 0);
	pFrame->GetActiveView()->CheckDlgButton(IDC_EDIT2, 0);
	pFrame->GetActiveView()->CheckDlgButton(IDC_EDIT4, 0);

	this->m_apex_to_valve = false;
	this->m_circumnavigation = false;
	this->m_forceControlActivated = false;
	this->tangent_updates = 0;
	this->valve_points_visited.clear();
	this->index = 0;
}

void CCTRDoc::addPointOnValve()
{
	int numOfPoints = this->valve_points_visited.size();
	
	double dist = 10000;
	double tmp = 0;
	::Eigen::VectorXd tmpPoint(5);

	for (int i = 0; i < 5; ++i)
		tmpPoint(i) = this->m_Status.currJang[i];

	for (int i = 0; i < numOfPoints; ++i)
	{
		tmp = (tmpPoint - this->valve_points_visited[i]).norm();
		if (tmp < dist)
			dist = tmp;
	}

	if (dist > 0.5) 
		this->valve_points_visited.push_back(tmpPoint);

	this->index = this->valve_points_visited.size();
}

void CCTRDoc::computeInitialDirection()
{
	double tmp_position[2];
	for (int i = 0; i < 2; ++i)
		tmp_position[i] = this->m_Status.currTipPosDir[i] + m_valve_tangent[i] * 10;
	//::std::cout << "temp position :";
	//PrintCArray(tmp_position, 2);
	//::std::cout << "tangent:";
	//PrintCArray(m_valve_tangent, 2);
	//::std::cout << "current:" << this->m_Status.currTipPosDir[0] << " " << this->m_Status.currTipPosDir[1] << ::std::endl;
	//::std::cout << "direction selection" << ::std::endl;
	switch (this->cStatus)
	{
		case CIRCUM_STATUS::LEFT_A:
		{
			//::std::cout << " left" << ::std::endl;
			if (tmp_position[1] > this->m_Status.currTipPosDir[1])
			{
				//::std::cout << "flip" << ::std::endl;
				m_valve_tangent[0] *= -1;
				m_valve_tangent[1] *= -1;
				
			}
			break;
		}
		case CIRCUM_STATUS::UP:
		{
			//::std::cout << " up" << ::std::endl;
			if (tmp_position[0] < this->m_Status.currTipPosDir[0])
			{
								//::std::cout << "flip" << ::std::endl;
				m_valve_tangent[0] *= -1;
				m_valve_tangent[1] *= -1;
			}
			break;
		}
		case CIRCUM_STATUS::RIGHT:
		{
			if (tmp_position[1] < this->m_Status.currTipPosDir[1])
			{
								//::std::cout << "flip" << ::std::endl;
				m_valve_tangent[0] *= -1;
				m_valve_tangent[1] *= -1;
			}
			break;
		}
		case CIRCUM_STATUS::DOWN:
		{
			if (tmp_position[0] > this->m_Status.currTipPosDir[0])
			{
								//::std::cout << "flip" << ::std::endl;
				m_valve_tangent[0] *= -1;
				m_valve_tangent[1] *= -1;
			}
			break;
		}
	}
}


double 
CCTRDoc::GetMonitorBreathingFreq()
{
	return this->m_heartRateMonitor->getBreathingRate();
}