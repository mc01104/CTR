
// CTRDoc.h : interface of the CCTRDoc class
//


#pragma once

#include <queue>
#include "CTR_Common.h"
#include <fstream>
#include <Eigen/Dense>


#include "VtkOnLinePlot.h"

// CKim - Declare the classes that will be maintained inside the 'Doc' class
// They are the classes that encapsulates functionalities of the robot software
class ChunHaptic;
class ChunMotion;
class CTRKin;
class LWPRKinematics;
class ChunTracker;
class TrjGenerator;


class CCTRDoc : public CDocument
{
protected: // create from serialization only
	CCTRDoc();
	DECLARE_DYNCREATE(CCTRDoc)

// Attributes
public:
	::std::queue<::std::string, ::std::deque< ::std::string> > setPointsQ;
	double durationLWPR;

private:

	// CKim - Robot Status
	CTR_status		m_Status;
	::std::ofstream adaptiveExperimentLog;
	// CKim - Command Queue
	std::queue<CTR_cmd>	m_cmdQueue; 

	// CKim - Robot state parameters - position of the haptic device, motor
	bool	m_motorConnected;

	// CKim - Pointers and handles to vrious components
	ChunHaptic*			m_Omni;
	HWND				m_hWndView;		// CKim - handle to view window
	ChunMotion*			m_motionCtrl;
	CTRKin*				m_kinLib;
	LWPRKinematics*		m_kinLWPR;
	ChunTracker*		m_Tracker;
	TrjGenerator*		m_TrjGen;

	// CKim - Variables for threads
	bool				m_ioRunning;
	bool				m_teleOpMode;
	bool				m_TrackerRunning;
	bool				m_playBack;
	bool				m_jointPlayback;
	bool				m_RegDataCollecting;
	bool				m_bLogEMData;
	bool				m_AdaptiveOn;
	bool				m_bDoUpdate;
	bool				m_FeedbackOn;
	bool				m_InvKinOn;
	bool				m_bStaticPlayBack;
	bool				m_bRunExperiment;
	bool				m_bCLIK;

	HANDLE				m_hTeleOpThread;	// CKim - Handle to the thread for teleoperation
	HANDLE				m_hEMTrck;
	HANDLE				m_hEMevent;
	HANDLE				m_hAdaptive;
	HANDLE				m_hMtrCtrl;
	double				m_SetPt[6];
	double				m_measPosforRec[6];
	bool				m_plotData;
	VtkOnLinePlot*		m_vtkPlot;
//	bool				
	std::ofstream		m_fStr;

	static CRITICAL_SECTION	m_cSection;		// CKim - Critical Section is used instead of mutex for synchroniation between threads. slightly fatser than mutexes

	// CKim - Transmission ratio
	static double c_CntToRad;		static double c_CntToMM;	static double c_PI;

// Operations
public:
	bool				m_adapt_LWPR;
	::std::string		m_date;
// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// Implementation
public:
	virtual ~CCTRDoc();
	void StartUIupdate();

	// CKim - Get thread safe copy of the robot status
	void	GetCurrentStatus(CTR_status& stat);

	VtkOnLinePlot*	GetVtkPlot(){return this->m_vtkPlot;};

	// CKim - Send command to robot
	void	SendCommand(int type, const double* para);
	
	// CKim - Set tele op mode on/off
	void	SwitchTeleOpMode(bool onoff);
	void	SwitchPlayBackMode(bool onoff);
	void	SwitchStaticPlayBackMode(bool onoff);
	void	SwitchJointPlayBackMode(bool onoff);

	void	SetForgettingFactor(double val);

	void	SaveModel();

#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif


protected:

	// CKim - Conversion between motor angle and motor count by transmission ratios
	void	MtrAngToCnt(const double* jA, double* cnt);  
	void	MtrCntToAng(const double* cnt, double* jA);  

	// CKim - Conversion from counts of each motor to joint parameter
	void	MtrToJang(const double* cnt, double* jA);  

	// CKim - Calculate new motor count from Inverse Kinematics Solution. This depends on current motor count
	void	InvKinJangToMtr(const double* jA, const double* currCnt, double* tgtJang, double* tgtCnt, bool& isInLimFlag);
	
	// CKim - Joint angle velocity to Motor count velocity
	void	dJangTodCnt(const double* dJ, double* dCnt);

	// CKim - Initializes EM Tracker and enters loop that reads from the tracker
	static unsigned int WINAPI	EMLoop(void* para);
	
	// CKim - Loop running during teleoperation. Processes haptic device input and updates the setpoint of the motor
	// Decouples inverse kinematics loop from main control loop
	static unsigned int WINAPI	TeleOpLoop(void* para);

	// CKim - Loop running motor control
	static unsigned int WINAPI	MotorLoop(void* para);

	// CKim - Loop running playback and adaptation
	static unsigned int WINAPI	PlaybackLoop(void* para);

	// CKim - Loop running playback and adaptation for point by point trajectory
	static unsigned int WINAPI	StaticPlaybackLoop(void* para);

	// CKim - Loop for testing settling time 
	static unsigned int WINAPI	SettlingTestLoop(void* para);

	// CKim - Loop for 'closed loop inverse kinematics' with sensor feedback.
	static unsigned int WINAPI	ClosedLoopControlLoop(void* para);

	// George - joint space trajectory playback
	static unsigned int WINAPI	JointSpacePlayback(void* para);

	// CKim - Following functions encapsulates various caclulations performed inside the loop
	// It takes reference to the CTR_status structure and updates it

	// CKim - Process command Queue
	bool	ProcessCommand(CTR_status& stat);

	// CKim - Solve inverse kinematic and check results, 'tgtJang', 'tgtMotorCnt', 'invKinOk', 'invKinErr'
	// 'limitOK', 'exitCond' is updated 
	void	SolveInverseKin(CTR_status& stat);
	
	// CKim - Reads master input and calculates desired endeffector position, depending on teleOp mode
	void	MasterToSlave(CTR_status& stat, double scale, bool absolute = true);
	void	MasterToSlave(const CTR_status& stat, double* slavePosDir, double scale, bool absolute = true);

	void	SlaveToMaster(CTR_status& stat, double scale);
	void	SlaveToMaster(const CTR_status& stat, double* proxyPos, double scale, bool absolute = true);

	bool	TeleOpSafetyCheck();



// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// Helper function that sets search content for a Search Handler
	void SetSearchContent(const CString& value);
#endif // SHARED_HANDLERS
public:
	afx_msg void OnViewStatusBar();
	afx_msg void OnViewTeleop();
	afx_msg void OnUpdateViewTeleop(CCmdUI *pCmdUI);
	afx_msg void OnBnClickedInitEm();
	afx_msg void OnBnClickedRegst();
	afx_msg void OnBnClickedChkAdapt();
	afx_msg void OnBnClickedBtnPlay();
	afx_msg void OnBnClickedExp();
	afx_msg void OnBnClickedBtnMdlreset();
	afx_msg void OnBnClickedChkFeedback();
	afx_msg void OnBnClickedChkInvkinon();

	afx_msg void OnViewPlot();
	afx_msg void OnUpdateViewPlot(CCmdUI *pCmdUI);

};
