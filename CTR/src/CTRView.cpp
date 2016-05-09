
// CTRView.cpp : implementation of the CCTRView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "CTRApp.h"
#endif

#include "CTRDoc.h"
#include "CTRView.h"
#include "ChunVtkDlg.h"
#include "ChunHaptic.h"
#include "ChunMotion.h"
#include "Utilities.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

int CCTRView::m_idActJang[5] = { IDC_A12_ACT, IDC_A13_ACT, IDC_L3_ACT, IDC_A1_ACT, IDC_L1_ACT };
int CCTRView::m_idCmdJang[5] = { IDC_A12_CMD, IDC_A13_CMD, IDC_L3_CMD, IDC_A1_CMD, IDC_L1_CMD };


IMPLEMENT_DYNCREATE(CCTRView, CFormView)

BEGIN_MESSAGE_MAP(CCTRView, CFormView)
	ON_WM_TIMER()
	ON_WM_DESTROY()

	ON_MESSAGE(WM_USER+1,OnUserMsg1)
	ON_MESSAGE(WM_USER+2,OnUserMsg2)

	ON_BN_CLICKED(IDC_BTN_MOVE, &CCTRView::OnClickedBtnMove)
	ON_WM_CREATE()

	ON_COMMAND(ID_VIEW_GRAPHICWIN, &CCTRView::OnViewGraphicwin)
	ON_UPDATE_COMMAND_UI(ID_VIEW_GRAPHICWIN, &CCTRView::OnUpdateViewGraphicwin)

	ON_BN_CLICKED(IDC_RADIO_JA, &CCTRView::OnBnClickedRadioModes)	
	ON_BN_CLICKED(IDC_RADIO_TELE, &CCTRView::OnBnClickedRadioModes)

	ON_BN_CLICKED(IDC_BUTTON10, &CCTRView::OnClickedBtnLeft)
	ON_BN_CLICKED(IDC_BUTTON8, &CCTRView::OnClickedBtnRight)
	ON_BN_CLICKED(IDC_BUTTON9, &CCTRView::OnClickedBtnRecConf)
	ON_BN_CLICKED(IDC_BUTTON7, &CCTRView::OnClickedBtnGoToRecConf)
	ON_BN_CLICKED(IDC_BUTTON1, &CCTRView::OnClickCopy)
	
	ON_EN_KILLFOCUS(IDC_EDIT1, &CCTRView::OnKillFocus)

	ON_BN_CLICKED(IDC_BTN_MOVE3, &CCTRView::OnClickedBtnStartLog)
	ON_BN_CLICKED(IDC_BTN_MOVE4, &CCTRView::OnClickedBtnStopLog)

	ON_WM_CTLCOLOR()
	ON_BN_CLICKED(IDC_BUTTON6, &CCTRView::OnClickedBtnHome)
END_MESSAGE_MAP()



CCTRView::CCTRView()
	: CFormView(CCTRView::IDD)
	, m_sysMsg(_T(""))
	, m_ctrlMode(0)
	, IDC_CIRCLE(_T(""))
{
	// TODO: add construction code here
	for(int i=0;i<7; i++)	{	m_errFlag[i] = 0;	}
	m_vtkDlg = NULL;
	m_Warning[0] = m_Warning[1] = false;
	this->transIncrement = 5.0;
	m_blogData = false;
	this->logDataFlag = false;

	this->eStopPressed = false;
	
	for(int i = 0; i < 5; ++i)
		this->recConfiguration[i] = 0.0;
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

	for(int i=0; i<5; i++)	{	DDX_Text(pDX, m_idActJang[i], m_actJang[i]);	}

	if(m_ctrlMode!= 0)	// CKim - 0: joint angle, 1: tip orientation, 2: teleoperation
	{
		for(int i=0; i<5; i++)	{	DDX_Text(pDX, m_idCmdJang[i], m_cmdJang[i]);	}
	}

	if (!this->eStopPressed)
		DDX_Radio(pDX, IDC_RADIO_JA, m_ctrlMode);

	DDV_MinMaxInt(pDX, m_ctrlMode, 0, 1);


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
	this->SetDlgItemTextA(m_idCmdJang[2],"10");			this->SetDlgItemTextA(m_idCmdJang[3],"0");			this->SetDlgItemTextA(m_idCmdJang[4],"0");
	
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


void CCTRView::OnTimer(UINT_PTR nIDEvent)
{

	CTR_status stat;
	this->GetDocument()->GetCurrentStatus(stat);

	double jAng[5];
	memcpy(jAng, stat.currJang, 5 * sizeof(double));
	jAng[0] *= 180.0/3.141592;
	jAng[1] *= 180.0/3.141592;
	jAng[3] *= 180.0/3.141592;
	
	for (int i = 0; i < 5; ++i)
		m_actJang[i].Format("%.3f",jAng[i]);


	UpdateData(false);

	if(this->logDataFlag)
	{
		QueryPerformanceCounter(&m_Etime);	
		m_Elapsed.QuadPart = m_Etime.QuadPart - m_Stime.QuadPart;
		m_Elapsed.QuadPart *= 1000000;
		m_Elapsed.QuadPart /= m_Freq.QuadPart;
		double dt = m_Elapsed.QuadPart/1000;

		for(int i=0; i<6; i++)	
			this->logStream << stat.tgtTipPosDir[i] << "\t";		
		//this->logStream << ::std::endl;

		for(int i = 0; i < 5; ++i)
			this->logStream << stat.currJang[i] << "\t";
		this->logStream << ::std::endl;

	}

	///////////// FIX TO UPDATE GRAPHICS //////////////////
	// CKim - Update 3D graphics
	if(m_vtkDlg)
	{
		m_vtkDlg->RenderBalancedPair(stat.bpTipPosDir);
		m_vtkDlg->RenderTip(stat.bpTipPosDir,stat.currTipPosDir);
		m_vtkDlg->RenderProxy(stat.tgtTipPosDir);
			
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

		//}
		//	else
		//{
		//	 CKim - Function for drawing robot should be here
		//	m_vtkDlg->MoveCsys(stat.currOmniInput);
		//	m_vtkDlg->MovePointer(stat.currOmniInput);//		}

	}

}


void CCTRView::OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/)
{
	UpdateData(false);	// False 0 to set control value, true to retrieve.
}


int CCTRView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CFormView::OnCreate(lpCreateStruct) == -1)
		return -1;

	return 0;
}


void CCTRView::OnDestroy()
{
	CFormView::OnDestroy();
	KillTimer(100);

}


void CCTRView::OnClickCopy()
{
	CString str;
	for (int i = 0; i < 5; ++i)
	{
		this->GetDlgItemTextA(m_idActJang[i], str);		
		this->SetDlgItemTextA(m_idCmdJang[i], str);
	}
}

void CCTRView::OnKillFocus()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT1, str);		
	this->transIncrement = atof(str);
	::std::cout << "increment changed to:" << this->transIncrement << ::std::endl;
}

void CCTRView::OnClickedBtnMove()
{
	CString str;	double p[10];

	if(m_ctrlMode == 0)	//0: joint angle, 1: tip configuration, 2: tele op
	{
		for(int i=0; i<5; i++)	
		{
			this->GetDlgItemTextA(m_idCmdJang[i], str);		

			p[i] = 	atof(str);		

			p[i+5] = 0;		
		}

		p[0] *= (3.141592/180.0);	p[1] *= (3.141592/180.0);	p[3] *= (3.141592/180.0);

		this->GetDocument()->SendCommand(0,p);

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
}


void CCTRView::OnBnClickedRadioModes()
{
	int prevMode = m_ctrlMode;
	
	UpdateData(true);	// CKim - Get the status
	
	if(m_ctrlMode == 0)	
		for(int i=0; i<5; i++)	
		{	
			CEdit* pWnd = (CEdit*) this->GetDlgItem(m_idCmdJang[i]);
			pWnd->SetReadOnly(0);	
		}	
	else 
		for(int i=0; i<5; i++)	
		{	
			CEdit* pWnd = (CEdit*) this->GetDlgItem(m_idCmdJang[i]);		
			pWnd->SetReadOnly(1);		
		}	
	if (m_ctrlMode == 1)
	{	
		this->GetDocument()->SwitchTeleOpMode(1);	
		return;		
	}
	else if(prevMode == 1)	
	{	
		this->GetDocument()->SwitchTeleOpMode(0);	
		return;		
	}

}


LRESULT CCTRView::OnUserMsg1(WPARAM wParam, LPARAM lParam)
{
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
	HBRUSH hbr = CFormView::OnCtlColor(pDC, pWnd, nCtlColor);
	
	if(m_ctrlMode ==1)
	{
		for(int i=0; i<5; i++)
			if (pWnd->GetDlgCtrlID() == m_idCmdJang[i])
			{
				if(m_Warning[1])	{	pDC->SetTextColor(RGB(255, 0, 0));	}
				else				{	pDC->SetTextColor(RGB(0, 0, 0));	}
			}
	}
	return hbr;
}

// TODO: refactor that to move tubes one by one
void CCTRView::OnClickedBtnHome()
{

	double home[5] =  {M_PI, 0, 5, 0, 0};

	CString str;
	double p[10] = {0};
	for (int i = 0; i < 5; ++i)
	{
		this->GetDlgItemTextA(m_idActJang[i], str);		
		p[i] = atof(str);
	}
	p[2] = home[2];
	this->GetDocument()->SendCommand(0, p);
	
	Sleep(3000);
	p[1] = home[1];
	p[0] = home[0];
	this->GetDocument()->SendCommand(0, p);
	 
	Sleep(1000);
	p[3] = home[3];
	p[4] = home[4];
	this->GetDocument()->SendCommand(0, p);
}

void CCTRView::OnClickedBtnLeft()
{
	CTR_status stat;
	this->GetDocument()->GetCurrentStatus(stat);

	CString str;
	double p[10] = {0};
	memcpy(p, stat.currJang, sizeof(double) * 5); 

	if(m_ctrlMode == 0)	
	{
		p[4] += this->transIncrement;
		this->GetDocument()->SendCommand(0,p);
	}
}

void CCTRView::OnClickedBtnRight()
{
	CTR_status stat;
	this->GetDocument()->GetCurrentStatus(stat);

	CString str;
	double p[10] = {0};
	memcpy(p, stat.currJang, sizeof(double) * 5); 

	if(m_ctrlMode == 0)	
	{
		p[4] -= this->transIncrement;
		this->GetDocument()->SendCommand(0,p);
	}
}

void CCTRView::OnClickedBtnRecConf()
{
	CTR_status stat;
	this->GetDocument()->GetCurrentStatus(stat);

	CString str;
	memcpy(this->recConfiguration, stat.currJang, sizeof(double) * 5); 
}

void CCTRView::OnClickedBtnGoToRecConf()
{
	if (m_ctrlMode == 0)
		this->GetDocument()->SendCommand(0, this->recConfiguration);
}

void CCTRView::OnClickedBtnStartLog()
{
	::std::string filename = GetDateString() + ".txt";
	this->logStream  = ::std::ofstream(filename.c_str());
	
	this->logDataFlag = true;
	
	GetDlgItem(IDC_BTN_MOVE3)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_MOVE4)->EnableWindow(TRUE);
}

void CCTRView::OnClickedBtnStopLog()
{
	this->logStream.close();
	GetDlgItem(IDC_BTN_MOVE4)->EnableWindow(FALSE);
	GetDlgItem(IDC_BTN_MOVE3)->EnableWindow(TRUE);
}

