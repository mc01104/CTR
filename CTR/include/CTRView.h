
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
	afx_msg void OnClickedBtnHome();
	afx_msg void OnClickedBtnMove();
	afx_msg void OnClickedBtnLeft();
	afx_msg void OnClickedBtnRight();
	afx_msg void OnClickedBtnRecConf();
	afx_msg void OnClickedBtnGoToRecConf();
	afx_msg void OnClickedBtnStartLog();
	afx_msg void OnClickedBtnStopLog();
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnViewGraphicwin();
	
	afx_msg void OnUpdateViewGraphicwin(CCmdUI *pCmdUI);
	afx_msg void OnBnClickedRadioModes();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	
	bool logDataFlag;
	void LogData(::std::ofstream& os);
	::std::ofstream logStream;

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

	// CKim - 3D graphic dialog using vtk
	ChunVtkDlg*		m_vtkDlg;

	// CKim - When something in document changes, call UpdatAllView() and this OnUpdata() is called in the view
	// Perfor UI updates here.
	virtual void OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/);
	
	CString m_sysMsg;
	int m_ctrlMode;
	bool m_Warning[2];

	std::ofstream	m_logfstr;
	bool			m_blogData;

	LARGE_INTEGER m_Stime, m_Etime, m_Elapsed, m_Freq;

	afx_msg void OnEnKillfocusForget();
	//afx_msg void OnBnClickedCheckLWPR();
	//afx_msg void OnBnClickedButtonSave();
	//afx_msg void OnCheckTraj();

	CComboBox m_traj_type;
	CString IDC_CIRCLE;
};

#ifndef _DEBUG  // debug version in CTRView.cpp
inline CCTRDoc* CCTRView::GetDocument() const
   { return reinterpret_cast<CCTRDoc*>(m_pDocument); }
#endif

