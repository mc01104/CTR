
// CTRView.h : interface of the CCTRView class
//

#pragma once

#include "resource.h"
class ChunVtkDlg;	// CKim - Encapsulates graphic rendering of the robot using VTK library

#include <fstream>
#include "afxwin.h"

class CCTRView : public CFormView
{
protected: // create from serialization only
	CCTRView();
	DECLARE_DYNCREATE(CCTRView)

public:
	enum{ IDD = IDD_CTR_FORM };

// Attributes
public:
	CCTRDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void OnInitialUpdate(); // called first time after construct

// Implementation
public:
	virtual ~CCTRView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnEnKillfocusTgt();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg LRESULT OnUserMsg1(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnUserMsg2(WPARAM wParam, LPARAM lParam);
	afx_msg void OnDestroy();
	afx_msg void OnClickCopy();
	afx_msg void OnKillFocusInc();
	afx_msg void OnKillFocusGain();
	afx_msg void OnClickedBtnHome();
	afx_msg void OnClickedBtnMove();
	afx_msg void OnClickedBtnLeft();
	afx_msg void OnClickedBtnRight();
	afx_msg void OnClickedBtnRecConf();
	afx_msg void OnClickedBtnGoToRecConf();

	afx_msg void OnClickedBtnStartLog();
	afx_msg void OnClickedBtnStopLog();

	afx_msg void OnClickedBtnRecPoint();
	afx_msg void OnClickedBtnPopPoint();
	afx_msg void OnClickedBtnCleanAll();
	afx_msg void OnClickedBtnComputePlane();

	afx_msg void OnKillFocusForce();
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnViewGraphicwin();
	afx_msg void OnKillFocusContactRatio();
	afx_msg void OnKillFocusSamplingPeriods();

	afx_msg void OnUpdateViewGraphicwin(CCmdUI *pCmdUI);
	afx_msg void OnBnClickedRadioModes();
	afx_msg void OnBnClickedRadioModesPlane();
	afx_msg void OnClickedBtnUpdate();
	afx_msg void OnKillFocusUpdateFrequency();

	afx_msg void UpdateGains();
	afx_msg void UpdateVSGains();
	afx_msg void UpdateScalingFactor();
	afx_msg void UpdateCentroid();
	afx_msg void UpdateTangent();
	afx_msg void OnBnClickedRadioModesController();
	afx_msg void OnBnClickedRadioModesFreq();
	afx_msg void OnBnClickedRadioModesCircum();
	afx_msg void OnBnClickedRadioModesATV();
	afx_msg void UpdateGainsATV();
	afx_msg void UpdateGlovalGain();
	afx_msg void OnBnClickedKillFocusId();

	//afx_msg void GoToApex();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	
	int m_PlaneEstimationMode;
	int m_controlMode;
	int m_frequencyMode;

	bool logDataFlag;
	::std::ofstream logStream;

	::std::vector<::Eigen::Vector3d> points_for_plane_estimation;

	double transIncrement;
	double recConfiguration[5];

	// Ckim - Control IDs and variables
	static int		m_idActMotor[7];		CString		m_actMotor[7];
	static int		m_idCmdMotor[7];		CString		m_cmdMotor[7];
	static int		m_idActJang[5];			CString		m_actJang[5];	
	static int		m_idCmdJang[5];			CString		m_cmdJang[5];
	static int		m_idActConfig[6];		CString		m_actConfig[6];
	static int		m_idCmdConfig[6];		CString		m_cmdConfig[6];
	static int		m_idErrFlag[7];			int			m_errFlag[7];
	static int		m_idEMMat[12];			CString		m_emMat[12];
	static int		m_idSensConfig[6];		CString		m_sensConfig[6];

	static int		m_id_monitor_freq;		CString		m_monitor_freq;

	static int		manual_point_ENABLE[15];
	static int		manual_point_DISABLE[15];
	static int		manual_ENABLE[15];
	static int		manual_DISABLE[15];
	static int		online_ENABLE[15];
	static int		online_DISABLE[15];

	// CKim - 3D graphic dialog using vtk
	ChunVtkDlg*		m_vtkDlg;

	// CKim - When something in document changes, call UpdatAllView() and this OnUpdata() is called in the view
	// Perfor UI updates here.
	virtual void OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/);

	void updateGUIActivationState(int handlesToActivate[], int handlesToDectivate[]);
	CString m_sysMsg;
	int m_ctrlMode;
	bool m_Warning[2];
	bool eStopPressed;
	std::ofstream	m_logfstr;
	bool			m_blogData;

	LARGE_INTEGER m_Stime, m_Etime, m_Elapsed, m_Freq;

	afx_msg void OnEnKillfocusForget();
	afx_msg void OnEnKillFocusBias();
	afx_msg void OnEnKillFocusDisturbance();
	//afx_msg void OnBnClickedCheckLWPR();
	//afx_msg void OnBnClickedButtonSave();
	//afx_msg void OnCheckTraj();
	void ToggleForceChkbox(bool flag);
	void ToggleForceControl();
	void ToggleCameraControl();
	void ToggleCircumnavigation();
	void ToggleApexToValve();
	void ToggleGlobalGains();

	void computeCircle(::Eigen::Matrix3d rot, ::Eigen::Vector3d& center, double& radius);
	bool m_freqUpdated;

	CComboBox m_traj_type;
	CString IDC_CIRCLE;

	::Eigen::Vector3d normal;
	::Eigen::Vector3d center;
	double radius;

	int m_direction;

	int m_apex_wall;
	bool m_transition;
};

#ifndef _DEBUG  // debug version in CTRView.cpp
inline CCTRDoc* CCTRView::GetDocument() const
   { return reinterpret_cast<CCTRDoc*>(m_pDocument); }
#endif


