
// CTRDoc.h : interface of the CCTRDoc class
//


#pragma once

#include <queue>
#include <map>
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
	enum APEX_TO_VALVE_STATUS 
	{
		LEFT,
		TOP,
		BOTTOM,
		USER
	};

	enum CIRCUM_STATUS 
	{
		UP,
		LEFT_A,
		RIGHT,
		DOWN
	};

	enum CIRCUM_DIRECTION 
	{
		CW,
		CCW
	};

private:

	APEX_TO_VALVE_STATUS aStatus;
	CIRCUM_STATUS		cStatus;
	CIRCUM_DIRECTION dStatus;

	// CKim - Robot Status
	CTR_status		m_Status;
	::std::ofstream adaptiveExperimentLog;
	// CKim - Command Queue
	std::queue<CTR_cmd>	m_cmdQueue; 
	CTR* robot;
	MechanicsBasedKinematics* kinematics;
	RecursiveFilter::Filter* filters;

	RecursiveFilter::Filter*		cr_dot_filter;
	// George - heart rate monitor
	HeartRateMonitor*   m_heartRateMonitor;
	double			m_valve_center[3];
	double			m_radius;
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
	
	double				desiredWallClock;
	bool				deactivateTest;
	// visual servoing stuff - these need to be read from the network
	double				m_centroid[2];
	double				m_centroid_apex[2];
	double				m_valve_tangent[2];
	double				m_valve_tangent_prev[2];
	int					m_direction;
	::Eigen::Vector2d	m_image_center;
	double				m_scaling_factor;
	bool				m_circumnavigation;
	bool				m_line_detected;
	bool				m_wall_detected;
	double				m_gain_center;
	double				m_gain_tangent;
	bool				m_apex_to_valve;

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

	// PID gains for CR
	double				m_contactGain;
	double				m_contactDGain;
	double				m_contactIGain;
	double				m_contact_error_integral;

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
	
	void				resetIntegral(){m_contact_error_integral = 0.0;};

	bool				switchToCircum;

	double				desiredClockfacePosition;
	double				actualClockfacePosition;
	bool				goToClockFace;
	double				registrationOffset;
	
// Operations
public:
	void				computeShortestDirection();
	void				setDesiredClockfacePosition(double desClock){this->desiredClockfacePosition = desClock; };
	void				setDesiredWallClockfacePosition(double desClock) {this->desiredWallClock = desClock;};
	void				activateClockfaceTask(){this->goToClockFace = true; tangent_updates = 0;};
	double				computeAngle(double clockfacePosition);
	void				setRegistrationOffset(double offset) {this->registrationOffset = offset;};

	bool				m_adapt_LWPR;
	::std::string		m_date;
	int					m_HRSource;
	bool				isModelRegistered;
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

	double GetMonitorFreq();
	double GetMonitorBreathingFreq();
	void	setHRSource(int source){this->m_HRSource = source;};
	void	UpdateGains(double position, double orientation, double position_forward, double orientation_forward);
	void	UpdateGainsApexToValve(double center, double forward, double threshold_min, double threshold_max);
	void	UpdateGlobalGain(double gain) {this->m_globalCR_gain = gain; ::std::cout << this->m_globalCR_gain << ::std::endl; };
	void	SwitchControlMode(int mode);
	void	SwitchFreqMode(int mode);
	void	ToggleClockface();
	void	SwitchAllControlFlagsOff();
	void	ToggleCameraControl();
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
	void	ToggleCircumnavigation();
	void	ToggleApexToValve();
	void	ToggleJacobianContactControl();

	void	SetFrequency(double frequency) {this->m_frequency = frequency; this->m_frequency_changed = true;};
	void	SetScalingFactor(double scaling_factor) {this->m_scaling_factor = scaling_factor;};
	void	SetCentroid(double x, double y) {this->m_centroid[0] = x; this->m_centroid[1] = y;};
	void	SetTangent(double tangent[2]) {memcpy(m_valve_tangent, tangent, 2 * sizeof(double));};
	void	SetDirection(int direction){this->m_direction = direction;};
	void	SetVSGains(double gain_center, double gain_tangent);
	void	SetSamplingPeriod(int samplingPeriod) {this->periodsForCRComputation = samplingPeriod;};
	void	ChangeForceForTuning(double force);
	void	SetContacControlGains(double forceGain, double forceGainD = 0, double forceGainI = 0);
	void	SetContactRatio(double ratio);

	void	UpdateCircumnavigationParams(::std::vector<double>& msg);

	void	SaveModel();
	void	ClearCommandQueue();
	ChunMotion*	GetMotionController() {return this->m_motionCtrl;};

	/////////////// CONTACT ////////////

	void	ComputeDesiredPosition(double tmpPosition[6]);
	void	ComputeDesiredVelocity();
	::std::vector<::Eigen::Vector3d> points_for_plane_estimation;
	Eigen::Vector3d GetTipPosition();
	void setContactControlNormal(const ::Eigen::Vector3d& computedNormal, const ::Eigen::Vector3d& center, double radius, ::std::vector<::Eigen::Vector3d> pts);

	void computeCircumnavigationDirection(Eigen::Matrix<double,6,1>& err);
	void computeApexToValveMotion(Eigen::Matrix<double,6,1>& err, APEX_TO_VALVE_STATUS aStatus = LEFT);

	void computeATVLeft(Eigen::Matrix<double,6,1>& err);
	void computeATVTop(Eigen::Matrix<double,6,1>& err);
	void computeATVBottom(Eigen::Matrix<double,6,1>& err);
	void computeATVUser(Eigen::Matrix<double,6,1>& err);

	double getInitialPositionOnValve();

	void addPointOnValve();
	double getClockPosition() {return this->actualClockfacePosition;};
	bool storeValvePoint;
	::std::vector<::Eigen::VectorXd> valve_points_visited;

	int	 periodsForCRComputation;
	void	SwitchApexToValveStatus(APEX_TO_VALVE_STATUS sts) {this->aStatus = sts;};
	void	SwitchCircumStatus(CIRCUM_STATUS sts) {this->cStatus = sts; this->computeInitialDirection();};
	void	SwitchCircumStatus(CIRCUM_DIRECTION sts) {this->dStatus = sts; this->computeInitialDirection();};

	void	computeInitialDirection();
	void	checkDirection(::Eigen::Matrix<double, 6, 1>& err);

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
	int		m_freq_mode;

	bool	m_apex;
	double	m_apex_coordinates[5];

	double	m_apex_theshold_min;
	double	m_apex_theshold_max;
	double  m_center_gainATV;
	double  m_forward_gainATV;

	double	m_globalCR_gain;

	bool	m_contact_response;
	double	computeInverseApproxJacovianCR(double currentCR);
	bool	m_useJacobianContactControl;

	// identification variables
	int		num_of_sins;
	double	min_frequency;				// 	[BPM]
	double	max_frequency;				// 	[BPM]
	double	amplitude;
	bool	activateIdentification;		// Set this to be true at 'Start' and false at 'Stop'.
	ChunTimer timerId;					// Reset this at 'Start'.
	double  reference_translation;		// Set this at 'Start'. Bring the robot back to this point at 'Stop'.
	double	reference_translation_inner;

	::std::vector<double> frequencies;	// Set this at 'OnKillFocus..' as linspace(min_frq, max_frq, num_sin).	[BPM]
	bool	m_idMode;
	int tangent_updates;

	double centroid_prev[2];
	double centroid_velocity[2];
	double tip_velocity[2];
	double tip_position_prev[2];
	double commanded_vel[2];
	RecursiveFilter::Filter* filter_centroid;
	RecursiveFilter::Filter*	filter_tip;

	::Eigen::Vector2d error_circ;
	::Eigen::Vector3d error3D;
	::Eigen::Matrix<double, 6, 1> tmpVelocities;
	::Eigen::Vector3d commandedVel;
	::Eigen::Vector3d tipPosition;
	::Eigen::Vector3d axisToTipPositionVector;
	::Eigen::Vector3d robot_axis;
	::Eigen::Vector3d commandedDirection;
	::Eigen::Vector3d tangent_vec;
	::Eigen::Vector3d tangent_vel;

	::Eigen::Vector3d center_vel;
	::Eigen::Vector3d final_vel;

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
	afx_msg void OnBnClickedGoToApex();

	afx_msg void OnBnClickedGoToFirst();
	afx_msg void OnBnClickedGoToLast();
	afx_msg void OnBnClickedGoToPrev();
	afx_msg void OnBnClickedGoToNext();

	afx_msg void OnBnClickedDumpConf();

	// identification
	afx_msg void OnBnClickedStartId();
	afx_msg void OnBnClickedStopId();
	afx_msg void OnBnClickedKillFocusId();
	afx_msg void OnBnClickedStraight();
	afx_msg void OnBnClickedHome();
	void UpdateIDParams(double min_freq, double max_freq, double amplitude, int num_of_sins);
	void switchIDMode(bool onoff) {this->m_idMode = onoff;};
	int index;

	void resetAutomation();

	double m_bias;
	void SetBias(double bias){this->m_bias = bias;};
	double m_disturbance_amp;
	void SetDisturbance(double disturbance){this->m_disturbance_amp = disturbance;};

	ChunTimer circumTimer;
	bool retractRobot;
	bool m_usePullBack;

	void TogglePullback();
	void ToggleTest();
};
