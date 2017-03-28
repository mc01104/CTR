
// CTRDoc.h : interface of the CCTRDoc class
//


#pragma once

#include <queue>
#include "CTR_Common.h"
#include <fstream>
#include <Eigen/Dense>
#include "time.h"
#include "FilterLibrary.h"
#include "ChunTimer.h"
#include "VtkOnLinePlot.h"

// CKim - Declare the classes that will be maintained inside the 'Doc' class
// They are the classes that encapsulates functionalities of the robot software
class ChunHaptic;
class ChunMotion;
class CTRKin;
class LWPRKinematics;
class ChunTracker;
class TrjGenerator;
class MechanicsBasedKinematics;
class CTR;
class HeartRateMonitor;

//class RecursiveFilter::Filter;

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
	CTR* robot;
	MechanicsBasedKinematics* kinematics;
	RecursiveFilter::Filter* filters;

	// George - heart rate monitor
	HeartRateMonitor*   m_heartRateMonitor;
	double			m_valve_center[3];

	// CKim - Robot state parameters - position of the haptic device, motor
	bool	m_motorConnected;

	// CKim - Pointers and handles to vrious components
	ChunHaptic*			m_Omni;
	HWND				m_hWndView;		// CKim - handle to view window
	ChunMotion*			m_motionCtrl;
	CTRKin*				m_kinLib;

	LWPRKinematics*		m_kinLWPR;
	LWPRKinematics*		m_kinLWPR_BP;

	bool				m_ref_set;
	ChunTracker*		m_Tracker;
	TrjGenerator*		m_TrjGen;
	ChunTimer*			m_timer;

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
	int					m_traj_type;
	static CRITICAL_SECTION	m_cSection;		// CKim - Critical Section is used instead of mutex for synchroniation between threads. slightly fatser than mutexes

	// CKim - Transmission ratio
	static double c_CntToRad;		static double c_CntToMM;	static double c_PI;

	double				m_force;
	bool				m_forceControlActivated;
	double				m_desiredPosition[3];
	bool				m_ContactUpdateReceived;
	double				m_deltaT;
	double				m_contactError;
	double				m_contactGain;
	double				m_contactDGain;
	double				m_contactRatio;
	double				m_contactRatioDesired;
	double				m_contactError_prev;
	::std::ofstream*	m_fileStream;
	bool				m_logData;

	// plane computation
	bool				m_compute_plane;
	void				IncrementalPlaneUpdate();
	::Eigen::Vector2d	m_plane_coefficients;
	::Eigen::Matrix2d   m_plane_covar;
	::Eigen::Vector3d	m_contact_control_normal;


	void				TogglePlaneEstimation();
	

	// new teleoperation control
	void				computeHapticDisplacement(CTR_status stat, double dP[3]);
	void				computeCameraDesiredMotion(CTR_status stat, const double image_dir[3], double camera_dir[3]);
	void				computeCameraJacobian(CTR_status stat);

	void				computeMechanicsKinematics(CTR_status stat);
	bool				cameraControlFlag;
	bool				m_camera_control;
	
	



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

	void	UpdateGains(double position, double orientation, double position_forward, double orientation_forward);
	void	SwitchControlMode(int mode);

	void SwitchAllControlFlagsOff();
	void				ToggleCameraControl();
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
	void	SetTrajectoryType(int traj_type) {m_traj_type = traj_type;};
	void	SetForgettingFactor(double val);
	LWPRKinematics* GetKinematics() { return this->m_kinLWPR;};
	void	ToggleForceControl();
	void	ToggleLog();
	void	SetFrequency(double frequency) {this->m_frequency = frequency; this->m_frequency_changed = true;};

	void	ChangeForceForTuning(double force);
	void	SetForceGain(double forceGain, double forceGainD = 0);
	void	SetContactRatio(double ratio);

	void	SaveModel();
	void	ClearCommandQueue();
	ChunMotion*	GetMotionController() {return this->m_motionCtrl;};

	/////////////// CONTACT ////////////
	void	UpdateDesiredPosition();
	void	ComputeDesiredPosition(double tmpPosition[6]);

	Eigen::Vector3d GetTipPosition();
	void setContactControlNormal(const ::Eigen::Vector3d& computedNormal);

#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif


protected:
	void GetTipTransformation(::Eigen::Matrix<double, 3, 3>& trans);
	void GetImageToCameraTransformation(::Eigen::Matrix<double, 3, 3>& trans);

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

	// George - joint space trajectory playback
	static unsigned int WINAPI	NetworkCommunication(void* para);

	// George - thread to read heart rate from surgivet monitor
	static unsigned int WINAPI	HeartRateMonitorThread(void* para);

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

	double	m_frequency;
	bool	m_frequency_changed;
	bool	m_plane_changed;
	double	m_position_gain;
	double	m_orientation_gain;

	double	m_position_gain_feedforward;
	double	m_orientation_gain_feedforward;

	int		m_control_mode;

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

	afx_msg void OnViewPlot();
	afx_msg void OnUpdateViewPlot(CCmdUI *pCmdUI);

};
