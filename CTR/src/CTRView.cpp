
// CTRView.cpp : implementation of the CCTRView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "CTR.h"
#endif

#include "CTRDoc.h"
#include "CTRView.h"
#include "ChunVtkDlg.h"
#include "ChunHaptic.h"
#include "ChunMotion.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


int CCTRView::m_idActMotor[7] = { IDC_AX0_ACT, IDC_AX1_ACT, IDC_AX2_ACT, IDC_AX3_ACT, IDC_AX4_ACT, IDC_AX5_ACT, IDC_AX6_ACT  };
int CCTRView::m_idCmdMotor[7] = { IDC_AX0_CMD, IDC_AX1_CMD, IDC_AX2_CMD, IDC_AX3_CMD, IDC_AX4_CMD, IDC_AX5_CMD, IDC_AX6_CMD	 };
int CCTRView::m_idActJang[5] = { IDC_A12_ACT, IDC_A13_ACT, IDC_L3_ACT, IDC_A1_ACT, IDC_L1_ACT };
int CCTRView::m_idCmdJang[5] = { IDC_A12_CMD, IDC_A13_CMD, IDC_L3_CMD, IDC_A1_CMD, IDC_L1_CMD };
int CCTRView::m_idActConfig[6] = { IDC_TIPX_ACT, IDC_TIPY_ACT, IDC_TIPZ_ACT, IDC_TIPNX_ACT, IDC_TIPNY_ACT, IDC_TIPNZ_ACT   };
int CCTRView::m_idCmdConfig[6] = { IDC_TIPX_CMD, IDC_TIPY_CMD, IDC_TIPZ_CMD, IDC_TIPNX_CMD, IDC_TIPNY_CMD, IDC_TIPNZ_CMD  };
int CCTRView::m_idErrFlag[7] = {  IDC_CHK_ERR0, IDC_CHK_ERR1, IDC_CHK_ERR2, IDC_CHK_ERR3, IDC_CHK_ERR4, IDC_CHK_ERR5, IDC_CHK_ERR6  };
int CCTRView::m_idEMMat[12] = { IDC_MAT00, IDC_MAT01, IDC_MAT02, IDC_MAT10, IDC_MAT11, IDC_MAT12, IDC_MAT20, IDC_MAT21, IDC_MAT22, IDC_MATX, IDC_MATY, IDC_MATZ };
int CCTRView::m_idSensConfig[6] = { IDC_TIPX_SENS, IDC_TIPY_SENS, IDC_TIPZ_SENS, IDC_TIPNX_SENS, IDC_TIPNY_SENS, IDC_TIPNZ_SENS };


IMPLEMENT_DYNCREATE(CCTRView, CFormView)

BEGIN_MESSAGE_MAP(CCTRView, CFormView)
	ON_EN_KILLFOCUS(IDC_TGTX, &CCTRView::OnEnKillfocusTgt)
	ON_EN_KILLFOCUS(IDC_TGTY, &CCTRView::OnEnKillfocusTgt)
	ON_EN_KILLFOCUS(IDC_TGTZ, &CCTRView::OnEnKillfocusTgt)
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_MESSAGE(WM_USER+1,OnUserMsg1)
	ON_MESSAGE(WM_USER+2,OnUserMsg2)
	ON_BN_CLICKED(IDC_BTN_MOVE, &CCTRView::OnClickedBtnMove)
	ON_WM_CREATE()

	ON_COMMAND(ID_VIEW_GRAPHICWIN, &CCTRView::OnViewGraphicwin)
	//ON_COMMAND(ID_VIEW_PLOT, &CCTRView::OnViewGraphicwin)
	
	ON_UPDATE_COMMAND_UI(ID_VIEW_GRAPHICWIN, &CCTRView::OnUpdateViewGraphicwin)
	//ON_UPDATE_COMMAND_UI(ID_VIEW_PLOT, &CCTRView::OnUpdateViewGraphicwin)

	ON_BN_CLICKED(IDC_RADIO_JA, &CCTRView::OnBnClickedRadioModes)
	ON_BN_CLICKED(IDC_RADIO_TIP, &CCTRView::OnBnClickedRadioModes)
	ON_BN_CLICKED(IDC_RADIO_TELE, &CCTRView::OnBnClickedRadioModes)
	ON_BN_CLICKED(IDC_RADIO_PLAYBACK, &CCTRView::OnBnClickedRadioModes)
	ON_BN_CLICKED(IDC_RADIO_STATPB, &CCTRView::OnBnClickedRadioModes)
	ON_BN_CLICKED(IDC_RADIO_JSPB, &CCTRView::OnBnClickedRadioModes)
	ON_WM_CTLCOLOR()
	ON_EN_KILLFOCUS(IDC_FORGET, &CCTRView::OnEnKillfocusForget)
	ON_BN_CLICKED(IDC_CHECK2, &CCTRView::OnBnClickedCheckLWPR)
	ON_BN_CLICKED(IDC_BUTTON5, &CCTRView::OnBnClickedButtonSave)
	ON_BN_CLICKED(IDC_BUTTON6, &CCTRView::OnClickedBtnHome)
END_MESSAGE_MAP()

// CCTRView construction/destruction

CCTRView::CCTRView()
	: CFormView(CCTRView::IDD)
	, m_sysMsg(_T(""))
	, m_ctrlMode(0)
{
	// TODO: add construction code here
	for(int i=0;i<7; i++)	{	m_errFlag[i] = 0;	}
	m_vtkDlg = NULL;
	m_Warning[0] = m_Warning[1] = false;

	m_blogData = false;
}

CCTRView::~CCTRView()
{
}

void CCTRView::DoDataExchange(CDataExchange* pDX)
{
	// CKim - This function is called UpdateData()to automatically update the control variable
	// by  to update control variables
	// depending on te operating mode, some controls should not be automatically updated based on the 
	CFormView::DoDataExchange(pDX);

	for(int i=0;i<7; i++)
	{
		DDX_Text(pDX, m_idActMotor[i], m_actMotor[i]);
		DDX_Text(pDX, m_idCmdMotor[i], m_cmdMotor[i]);
		DDX_Check(pDX, m_idErrFlag[i], m_errFlag[i]);
	}

	for(int i=0; i<12; i++)
	{
		DDX_Text(pDX,m_idEMMat[i], m_emMat[i]);
	}

	for(int i=0; i<5; i++)	{	DDX_Text(pDX, m_idActJang[i], m_actJang[i]);	}

	if(m_ctrlMode!= 0)	// CKim - 0: joint angle, 1: tip orientation, 2: teleoperation
	{
		for(int i=0; i<5; i++)	{	DDX_Text(pDX, m_idCmdJang[i], m_cmdJang[i]);	}
	}

	for(int i=0; i<6; i++)
	{
		DDX_Text(pDX, m_idActConfig[i], m_actConfig[i]);	
		DDX_Text(pDX, m_idSensConfig[i],m_sensConfig[i]);	
	}

	if(m_ctrlMode!= 1)	// CKim - 0: joint angle, 1: tip orientation, 2: teleoperation
	{
		for(int i=0; i<6; i++)	{	DDX_Text(pDX, m_idCmdConfig[i], m_cmdConfig[i]);	}
	}

	DDX_Text(pDX, IDC_SYSMSG, m_sysMsg);
	DDX_Radio(pDX, IDC_RADIO_JA, m_ctrlMode);

	DDV_MinMaxInt(pDX, m_ctrlMode, 0, 5);
}

BOOL CCTRView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CFormView::PreCreateWindow(cs);
}

void CCTRView::OnInitialUpdate()
{
	
	CFormView::OnInitialUpdate();
	GetParentFrame()->RecalcLayout();
	ResizeParentToFit();
	GetParentFrame()->SetWindowPos(&CWnd::wndTop,0,0,850,950, SWP_SHOWWINDOW );


	// CKim - Initialize controls here
	this->SetDlgItemTextA(m_idCmdJang[0],"180");		this->SetDlgItemTextA(m_idCmdJang[1],"180");	
	this->SetDlgItemTextA(m_idCmdJang[2],"43");			this->SetDlgItemTextA(m_idCmdJang[3],"0");			this->SetDlgItemTextA(m_idCmdJang[4],"0");
	
	//// CKim - Open the graphics dialog
	//if(!m_vtkDlg)
	//{
	//	// CKim - This creates modeless dialog using ChunVtkDlg class
	//	m_vtkDlg = new ChunVtkDlg();		m_vtkDlg->Create(ChunVtkDlg::IDD);		
	//	
	//	// CKim - Move to (640,480) and don't change size. Show window and draw
	//	m_vtkDlg->SetWindowPos(&CWnd::wndTop,900,200,0,0,SWP_NOSIZE);		
	//	m_vtkDlg->ShowWindow(SW_SHOW);
	//	m_vtkDlg->Invalidate();
	//}

	this->SetTimer(100,30,NULL);
	QueryPerformanceFrequency(&m_Freq);


}


// CCTRView diagnostics

#ifdef _DEBUG
void CCTRView::AssertValid() const
{
	CFormView::AssertValid();
}

void CCTRView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}

CCTRDoc* CCTRView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CCTRDoc)));
	return (CCTRDoc*)m_pDocument;
}
#endif //_DEBUG


// CCTRView message handlers


void CCTRView::OnEnKillfocusTgt()
{
	CString str;
	//this->GetDlgItemTextA(IDC_TGTX,str);	double x = atof(str);
	//this->GetDlgItemTextA(IDC_TGTY,str);	double y = atof(str);
	//this->GetDlgItemTextA(IDC_TGTZ,str);	double z = atof(str);
	//this->GetDocument()->m_vtkDlg->MoveCursor(x,y,z);
	//float rgb[3];	rgb[0] = 0;		rgb[1] = 1;		rgb[2] = 0;
	//this->GetDocument()->m_vtkDlg->SetCursorColor(rgb);
	//this->GetDocument()->m_vtkDlg->SetCursorSize(3);
	//this->GetDocument()->m_vtkDlg->Invalidate();
	// TODO: Add your control notification handler code here
}


void CCTRView::OnTimer(UINT_PTR nIDEvent)
{
	// CKim - Read the status of the robot. This is thread safe
	CTR_status stat;
	this->GetDocument()->GetCurrentStatus(stat);

	// CKim - Update dialog
	for(int i=0; i<7; i++)	{
		m_actMotor[i].Format("%.3f",stat.currMotorCnt[i]);		
		m_cmdMotor[i].Format("%.3f",stat.tgtMotorCnt[i]);	
		m_errFlag[i] = (stat.errFlag[i] != 0);
	}


	m_actJang[0].Format("%.3f",stat.currJang[0]*180.0/3.141592);	m_actJang[1].Format("%.3f",stat.currJang[1]*180.0/3.141592);		
	m_actJang[2].Format("%.3f",stat.currJang[2]);					m_actJang[3].Format("%.3f",stat.currJang[3]*180.0/3.141592);		
	m_actJang[4].Format("%.3f",stat.currJang[4]);		

	if(m_ctrlMode != 0)
	{
		m_cmdJang[0].Format("%.3f",stat.tgtJang[0]*180.0/3.141592);		m_cmdJang[1].Format("%.3f",stat.tgtJang[1]*180.0/3.141592);
		m_cmdJang[2].Format("%.3f",stat.tgtJang[2]);					m_cmdJang[3].Format("%.3f",stat.tgtJang[3]*180.0/3.141592);	
		m_cmdJang[4].Format("%.3f",stat.tgtJang[4]);	
	}

	for(int i=0; i<6; i++)	{	m_actConfig[i].Format("%.3f",stat.currTipPosDir[i]);		}
	//for(int i=0; i<6; i++)	{	m_actConfig[i].Format("%.3f",stat.bpTipPosDir[i]);		}
	
	if(m_ctrlMode != 1)
	{
		for(int i=0; i<6; i++)	{	m_cmdConfig[i].Format("%.3f",stat.tgtTipPosDir[i]);		}
	}

	CString wtf;
	wtf.Format("%.2f",stat.invKinErr[0]);		this->SetDlgItemTextA(IDC_POSERR,wtf);
	wtf.Format("%.2f",stat.invKinErr[1]);		this->SetDlgItemTextA(IDC_ORTERR,wtf);

	long int dt = stat.loopTime;		double freq = 1000000.0/dt;
	m_sysMsg.Format("Loop running at %.2f Hz. //  TeleOp %d  //  Inv kin %d // Limit %d // Cond %.2f", 
		             freq, stat.isTeleOpMoving, stat.invKinOK, stat.limitOK, stat.condNum);

	m_Warning[0] = !stat.invKinOK;	//(!stat.isTeleOpMoving) && (m_ctrlMode==2);
	m_Warning[1] = !stat.limitOK;	//(!stat.isTeleOpMoving) && (m_ctrlMode==2);

	// CKim - Read EM Tracker data, Update UI
	for(int i=0; i<9; i++)	{	m_emMat[i].Format("%.3f",stat.emMat[i%3][i/3]);			}
	for(int i=9; i<12; i++)	{	m_emMat[i].Format("%.1f",stat.emMat[i%3][i/3]);			}
	for(int i=0; i<6; i++)	{	m_sensConfig[i].Format("%.3f",stat.sensedTipPosDir[i]);	}

	double adpPosErr = 0;	double adpOrtErr = 0;
	for(int i=0; i<3; i++)	{
		adpPosErr += ((stat.sensedTipPosDir[i]-stat.currTipPosDir[i])*(stat.sensedTipPosDir[i]-stat.currTipPosDir[i]));
		adpOrtErr += (stat.sensedTipPosDir[i+3]*stat.currTipPosDir[i+3]);
	}
	wtf.Format("%.2f",sqrt(adpPosErr));					SetDlgItemText(IDC_ADP_POSERR,wtf);
	wtf.Format("%.2f",acos(adpOrtErr)*180.0/3.141592);	SetDlgItemText(IDC_ADP_ORTERR,wtf);


	UpdateData(false);

	if(m_blogData)
	{
		QueryPerformanceCounter(&m_Etime);	
		m_Elapsed.QuadPart = m_Etime.QuadPart - m_Stime.QuadPart;
		m_Elapsed.QuadPart *= 1000000;
		m_Elapsed.QuadPart /= m_Freq.QuadPart;
		double dt = m_Elapsed.QuadPart/1000;

		for(int i=0; i<6; i++)	{
			m_logfstr<<stat.tgtTipPosDir[i]<<"\t";		}
		m_logfstr<<stat.invKinOK<<"\t"<<dt;
		m_logfstr<<"\n";

		for(int i=0; i<6; i++)	{
			m_logfstr<<stat.currTipPosDir[i]<<"\t";		}
		m_logfstr<<stat.invKinOK<<"\t"<<dt;
		m_logfstr<<"\n";

		for(int i=0; i<6; i++)	{
			m_logfstr<<stat.sensedTipPosDir[i]<<"\t";		}
		m_logfstr<<stat.invKinOK<<"\t"<<dt;
		m_logfstr<<"\n";


	}

	// CKim - Update 3D graphics
	if(m_vtkDlg)
	{
		//m_vtkDlg->RenderBalancedPair(stat.bpTipPosDir);
		//m_vtkDlg->RenderTip(stat.bpTipPosDir,stat.currTipPosDir);
		//m_vtkDlg->RenderProxy(stat.tgtTipPosDir);
			
		//m_vtkDlg->MoveCursor(stat.bpTipPosDir[0], stat.bpTipPosDir[1], stat.bpTipPosDir[2]);
		//if(stat.mode ==1)	// CKim - under teleop
		//{
		//	m_vtkDlg->SetCursorColor(0,1,0);
		//	m_vtkDlg->MoveRefCsys(stat.currOmniInput);
		//}
		//else
		//{
		//	m_vtkDlg->SetCursorColor(1,0,0);
		//}

		m_vtkDlg->Invalidate();

				//if( (!omni.btn[0]) && (omni.btn[1]) )
		//{
		//	// CKim - Update camera here
		//		}
		//	else
		//{
			// CKim - Function for drawing robot should be here
			//m_vtkDlg->MoveCsys(stat.currOmniInput);
			//m_vtkDlg->MovePointer(stat.currOmniInput);//		}

	}
	
	
	if (this->GetDocument()->GetVtkPlot()) 
		this->GetDocument()->GetVtkPlot()->PlotData(stat.sensedTipPosDir, stat.currTipPosDir);

}


void CCTRView::OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/)
{
	// TODO: Add your message handler code here and/or call default
	UpdateData(false);	// False 0 to set control value, true to retrieve.

	// TODO: Add your specialized code here and/or call the base class
}


int CCTRView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CFormView::OnCreate(lpCreateStruct) == -1)
		return -1;

	return 0;
}


void CCTRView::OnDestroy()
{
	//delete[] m_ctrlLabels;
	CFormView::OnDestroy();
	KillTimer(100);

	// TODO: Add your message handler code here
}


void CCTRView::OnClickedBtnMove()
{
	// TODO: Add your control notification handler code here
	//UpdateData(true);
	CString str;	double p[10];

	if(m_ctrlMode == 0)	//0: joint angle, 1: tip configuration, 2: tele op
	{
		for(int i=0; i<5; i++)	{
			this->GetDlgItemTextA(m_idCmdJang[i],str);		p[i] = 	atof(str);		p[i+5] = 0;		}

		p[0] *= (3.141592/180.0);	p[1] *= (3.141592/180.0);	p[3] *= (3.141592/180.0);
		this->GetDocument()->SendCommand(0,p);
	}

	if(m_ctrlMode == 1)	//0: joint angle, 1: tip configuration, 2: tele op
	{
		for(int i=0; i<6; i++)	{
			this->GetDlgItemTextA(m_idCmdConfig[i],str);		p[i] = 	atof(str);		}
			this->GetDocument()->SendCommand(1,p);
	}

	if(m_vtkDlg)
		m_vtkDlg->ResetCam();
}


void CCTRView::OnViewGraphicwin()
{
	// CKim - Open/Close the graphics dialog
	if(!m_vtkDlg)
	{
		// CKim - This creates modeless dialog using ChunVtkDlg class
		m_vtkDlg = new ChunVtkDlg();		m_vtkDlg->Create(ChunVtkDlg::IDD);		
		
		// CKim - Move to (640,480) and don't change size. Show window and draw
		m_vtkDlg->SetWindowPos(&CWnd::wndTop,640,200,0,0,SWP_NOSIZE);		
		m_vtkDlg->ShowWindow(SW_SHOW);
		m_vtkDlg->Invalidate();
	}
	else
	{
		m_vtkDlg->DestroyWindow();		delete m_vtkDlg;		m_vtkDlg = NULL;
	}
}


void CCTRView::OnUpdateViewGraphicwin(CCmdUI *pCmdUI)
{
	if(m_vtkDlg)	{	pCmdUI->SetCheck(1);		}
	else			{	pCmdUI->SetCheck(0);		}
	// TODO: Add your command update UI handler code here
}


void CCTRView::OnBnClickedRadioModes()
{
	int prevMode = m_ctrlMode;
	
	UpdateData(true);	// CKim - Get the status
	
	if(m_ctrlMode == 0)	{
		for(int i=0; i<5; i++)	{	
			CEdit* pWnd = (CEdit*) this->GetDlgItem(m_idCmdJang[i]);		pWnd->SetReadOnly(0);		}	}
	else {
		for(int i=0; i<5; i++)	{	
			CEdit* pWnd = (CEdit*) this->GetDlgItem(m_idCmdJang[i]);		pWnd->SetReadOnly(1);		}	}

	if(m_ctrlMode == 1)	{
		for(int i=0; i<6; i++)	{	
			CEdit* pWnd = (CEdit*) this->GetDlgItem(m_idCmdConfig[i]);		pWnd->SetReadOnly(0);	}	}
	else {
		for(int i=0; i<6; i++)	{	
			CEdit* pWnd = (CEdit*) this->GetDlgItem(m_idCmdConfig[i]);		pWnd->SetReadOnly(1);	}	}

	if(m_ctrlMode == 2)		{	this->GetDocument()->SwitchTeleOpMode(1);	return;		}
	else if(prevMode == 2)	{	this->GetDocument()->SwitchTeleOpMode(0);	return;		}

	if(m_ctrlMode == 3)		{	this->GetDocument()->SwitchPlayBackMode(1);	return;		}
	else if(prevMode == 3)	{	this->GetDocument()->SwitchPlayBackMode(0);	return;		}

	if(m_ctrlMode == 4)		{	this->GetDocument()->SwitchStaticPlayBackMode(1);	return;		}
	else if(prevMode == 4)	{	this->GetDocument()->SwitchStaticPlayBackMode(0);	return;		}

	if(m_ctrlMode == 5)		{	this->GetDocument()->SwitchJointPlayBackMode(1);	return;		}
	else if(prevMode == 5)	{	this->GetDocument()->SwitchJointPlayBackMode(0);	return;		}
	
	// TODO: Add your control notification handler code here
}


LRESULT CCTRView::OnUserMsg1(WPARAM wParam, LPARAM lParam)
{
	// CKim - 
	return 0;
}

LRESULT CCTRView::OnUserMsg2(WPARAM wParam, LPARAM lParam)
{ 
	// CKim - OnUserMsg2 is called by OS when WM_USER+2 message is generated from the teleoperation loop.
	// This message is currently generated when haptic device button 1 released
	//m_vtkDlg->ResetCam();

	// CKim - Record time
	m_blogData = !m_blogData;
	if(m_blogData)	{	m_logfstr.open("LogDataFile.txt");		}
	else
	{
		m_logfstr.close();
	}

	m_Elapsed.QuadPart = 0.0;
	QueryPerformanceCounter(&m_Stime);	
	
	return 0;
}

HBRUSH CCTRView::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	// Call the base class implementation first! Otherwise, it may 
	// undo what we're trying to accomplish here.
	HBRUSH hbr = CFormView::OnCtlColor(pDC, pWnd, nCtlColor);
	
	// Are we painting the IDC_MYSTATIC control? We can use 
	// CWnd::GetDlgCtrlID() to perform the most efficient test. 
	if(m_ctrlMode ==2)
	{
		for(int i=0; i<6; i++)
		{
			if (pWnd->GetDlgCtrlID() == m_idCmdConfig[i])
			{
				if(m_Warning[0])	{	pDC->SetTextColor(RGB(255, 0, 0));	}
				else			{	pDC->SetTextColor(RGB(0, 0, 0));	}
			}
		}

		for(int i=0; i<5; i++)
		{
			if (pWnd->GetDlgCtrlID() == m_idCmdJang[i])
			{
				if(m_Warning[1])	{	pDC->SetTextColor(RGB(255, 0, 0));	}
				else				{	pDC->SetTextColor(RGB(0, 0, 0));	}
			}
		}

	}
	// TODO:  Change any attributes of the DC here

	// TODO:  Return a different brush if the default is not desired
	return hbr;
}


void CCTRView::OnEnKillfocusForget()
{
	CString str;
	this->GetDlgItemTextA(IDC_FORGET, str);		
	this->GetDocument()->SetForgettingFactor(atof(str));

	// TODO: Add your control notification handler code here
}


void CCTRView::OnBnClickedCheckLWPR()
{
	// TODO: Add your control notification handler code here
	this->GetDocument()->m_adapt_LWPR = !this->GetDocument()->m_adapt_LWPR;
	::std::cout << "changing mode" << ::std::endl;
}

void CCTRView::OnBnClickedButtonSave()
{
	this->GetDocument()->SaveModel();
}

void CCTRView::OnClickedBtnHome()
{
	CButton* pButton = (CButton*) GetDlgItem(IDC_RADIO_JA);
	m_ctrlMode = 0;
	pButton->SetCheck(true);

	double p[10] = {0, 0, 86.34, 0 , 0,};

	p[0] *= (3.141592/180.0);	p[1] *= (3.141592/180.0);	p[3] *= (3.141592/180.0);
	this->GetDocument()->SendCommand(0,p);
}