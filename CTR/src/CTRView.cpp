
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

int CCTRView::manual_point_ENABLE[15] = {IDC_BTN_MOVE3, IDC_BTN_MOVE4, IDC_BTN_MOVE5, IDC_BTN_MOVE6, IDC_BTN_MOVE7, -1};
int CCTRView::manual_point_DISABLE[15] = {IDC_EDIT8, IDC_EDIT9, IDC_EDIT10, IDC_BTN_MOVE8, IDC_EDIT11, IDC_EDIT12, IDC_EDIT13, IDC_BTN_MOVE8, -1};

int CCTRView::manual_ENABLE[15] = {IDC_EDIT8, IDC_EDIT9, IDC_EDIT10, IDC_BTN_MOVE8, -1};
int CCTRView::manual_DISABLE[15] = {IDC_BTN_MOVE3, IDC_BTN_MOVE4, IDC_BTN_MOVE5, IDC_BTN_MOVE6, -1};

int CCTRView::online_ENABLE[15] = {IDC_EDIT11, IDC_EDIT12, IDC_EDIT13, IDC_BTN_MOVE9, -1};
int CCTRView::online_DISABLE[15] = {IDC_EDIT8, IDC_EDIT9, IDC_EDIT10, IDC_BTN_MOVE8, -1};

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

	// plane radio controls
	ON_BN_CLICKED(IDC_RADIO_JA2, &CCTRView::OnBnClickedRadioModesPlane)	
	ON_BN_CLICKED(IDC_RADIO_TELE2, &CCTRView::OnBnClickedRadioModesPlane)
	ON_BN_CLICKED(IDC_RADIO_TELE3, &CCTRView::OnBnClickedRadioModesPlane)


	ON_BN_CLICKED(IDC_BUTTON10, &CCTRView::OnClickedBtnLeft)
	ON_BN_CLICKED(IDC_BUTTON8, &CCTRView::OnClickedBtnRight)
	ON_BN_CLICKED(IDC_BUTTON9, &CCTRView::OnClickedBtnRecConf)
	ON_BN_CLICKED(IDC_BUTTON7, &CCTRView::OnClickedBtnGoToRecConf)
	ON_BN_CLICKED(IDC_BUTTON1, &CCTRView::OnClickCopy)
	
	ON_EN_KILLFOCUS(IDC_EDIT1, &CCTRView::OnKillFocusInc)
	ON_EN_KILLFOCUS(IDC_EDIT2, &CCTRView::OnKillFocusGain)
	ON_EN_KILLFOCUS(IDC_EDIT3, &CCTRView::OnKillFocusContactRatio)

	ON_BN_CLICKED(IDC_BTN_MOVE3, &CCTRView::OnClickedBtnRecPoint)
	ON_BN_CLICKED(IDC_BTN_MOVE4, &CCTRView::OnClickedBtnPopPoint)
	ON_BN_CLICKED(IDC_BTN_MOVE6, &CCTRView::OnClickedBtnCleanAll)
	ON_BN_CLICKED(IDC_BTN_MOVE5, &CCTRView::OnClickedBtnComputePlane)
	ON_BN_CLICKED(IDC_BTN_MOVE7, &CCTRView::OnClickedBtnUpdate)
	ON_BN_CLICKED(IDC_BTN_MOVE8, &CCTRView::OnClickedBtnUpdate)
	ON_BN_CLICKED(IDC_BTN_MOVE9, &CCTRView::OnClickedBtnUpdate)


	ON_BN_CLICKED(IDC_CHECK1, &CCTRView::ToggleForceControl)
	ON_BN_CLICKED(IDC_CHECK3, &CCTRView::ToggleCameraControl)

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

	points_for_plane_estimation.resize(0);
	m_PlaneEstimationMode = 0;

}

CCTRView::~CCTRView()
{
}

void CCTRView::OnKillFocusGain()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT2, str);		
	double forceGain = atof(str);
	::std::cout << "requested gain: " << forceGain << ::std::endl;
	this->GetDocument()->SetForceGain(forceGain);

}

void CCTRView::OnKillFocusContactRatio()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT3, str);		
	double contactRatio = atof(str);
	::std::cout << "requested ratio: " << contactRatio << ::std::endl;
	this->GetDocument()->SetContactRatio(contactRatio);
}

void CCTRView::OnKillFocusForce()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT3, str);
	double force = atof(str);
	this->GetDocument()->ChangeForceForTuning(force);
	std::cout << "set force" <<  force << ::std::endl;
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

	// update number of collected points
	CString tmp;
	tmp.Format("%d",this->points_for_plane_estimation.size());
	DDX_Text(pDX, IDC_EDIT4, tmp);
	
	DDX_Radio(pDX, IDC_RADIO_JA2, m_PlaneEstimationMode);

	m_ctrlMode != 1 ? GetDlgItem(IDC_CHECK1)->EnableWindow(false) : GetDlgItem(IDC_CHECK1)->EnableWindow(true);
	m_ctrlMode != 1 ? GetDlgItem(IDC_CHECK3)->EnableWindow(false) : GetDlgItem(IDC_CHECK3)->EnableWindow(true);

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

	CString tmp;
	tmp.Format("%d",this->points_for_plane_estimation.size());
	this->SetDlgItemTextA(IDC_EDIT4,tmp);

	// CKim - Initialize controls here
	this->SetDlgItemTextA(m_idCmdJang[0],"105.324");		this->SetDlgItemTextA(m_idCmdJang[1],"125.421");	
	this->SetDlgItemTextA(m_idCmdJang[2],"5");			this->SetDlgItemTextA(m_idCmdJang[3],"180.576");			this->SetDlgItemTextA(m_idCmdJang[4],"0");
	
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

	double jAngCmd[5];
	if (m_ctrlMode == 1)
	{
		memcpy(jAngCmd, stat.tgtJang, 5 * sizeof(double));
		jAngCmd[0] *= 180.0/3.141592;
		jAngCmd[1] *= 180.0/3.141592;
		jAngCmd[3] *= 180.0/3.141592;
	
		for (int i = 0; i < 5; ++i)
			m_cmdJang[i].Format("%.3f",jAngCmd[i]);
	}

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
	::std::cout << "copy configuration" << ::std::endl;
}

void CCTRView::OnKillFocusInc()
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

void CCTRView::OnClickedBtnRecPoint()
{
	::Eigen::Vector3d tmpPoint = this->GetDocument()->GetTipPosition();
	this->points_for_plane_estimation.push_back(tmpPoint);

}

void CCTRView::OnClickedBtnPopPoint()
{
	this->points_for_plane_estimation.pop_back();
}

void CCTRView::OnClickedBtnCleanAll()
{
	this->points_for_plane_estimation.clear();
}

void CCTRView::ToggleForceChkbox(bool flag)
{
	GetDlgItem(IDC_CHECK1)->EnableWindow(flag);
}

void CCTRView::ToggleForceControl()
{
	if (m_ctrlMode == 1)
		this->GetDocument()->ToggleForceControl();
}

void CCTRView::ToggleCameraControl()
{
	if (m_ctrlMode == 1)
		this->GetDocument()->ToggleForceControl();
}

void CCTRView::OnClickedBtnComputePlane()
{
	::Eigen::MatrixXd data(3, this->points_for_plane_estimation.size());
	for (int i = 0; i < this->points_for_plane_estimation.size(); ++i)
		data.col(i) = this->points_for_plane_estimation[i];
	
	// not sure if this is correct
	::Eigen::Vector3d mu = data.rowwise().mean();
	::Eigen::Matrix3Xd points_centered = data.colwise() - mu;

	int setting = Eigen::ComputeFullU | Eigen::ComputeFullU;
	Eigen::JacobiSVD<Eigen::Matrix3Xd> svd = points_centered.jacobiSvd(setting);
	::Eigen::MatrixXd U = svd.matrixU();
	Eigen::Vector3d normal_tmp = U.col(2);

	if (normal_tmp(2) < 0)
		normal_tmp = -normal_tmp;

	::std::cout << normal_tmp << ::std::endl;
	this->normal = normal_tmp;;
	::std::cout << this->normal << ::std::endl;
}


void CCTRView::OnBnClickedRadioModesPlane()
{

	UpdateData(true);	
	
	if(m_PlaneEstimationMode == 0)	
	{
		::std::cout << "Estimate plane from number of points" << ::std::endl;
		this->updateGUIActivationState(manual_point_ENABLE, manual_point_DISABLE);
	}
	else if (m_PlaneEstimationMode == 1)
	{
		::std::cout << "Manual input of plane normal" << ::std::endl;
		this->updateGUIActivationState(manual_ENABLE, manual_DISABLE);
	}
	else if (m_PlaneEstimationMode == 2)
	{
		::std::cout << "Online plane estimation!" << ::std::endl;
		this->updateGUIActivationState(manual_point_ENABLE, manual_point_DISABLE);
	}

	return;		
}

void CCTRView::updateGUIActivationState(int handlesToActivate[], int handlesToDectivate[])
{
	int value = 0;
	int i = 0;
	while (value >= 0)
	{
		value = handlesToActivate[i];
		GetDlgItem(value)->EnableWindow(TRUE);
		i++;
	}

	value = 0;
	i = 0;
	while (value >= 0)
	{
		value = handlesToDectivate[i];
		GetDlgItem(value)->EnableWindow(FALSE);
		i++;
	}
}

void CCTRView::OnClickedBtnUpdate()
{
	UpdateData(true);	
	
	CString str;
	//Except from updating the displays, I need to push the computed plane normal to the contact controller -> TODO;
	switch(m_PlaneEstimationMode)
	{
	case 0:
		str.Format("%1.4f", this->normal(0));
		this->SetDlgItemTextA(IDC_EDIT5, str);
		str.Format("%1.4f", this->normal(1));
		this->SetDlgItemTextA(IDC_EDIT6, str);
		str.Format("%1.4f", this->normal(2));
		this->SetDlgItemTextA(IDC_EDIT7, str);
		this->GetDocument()->setContactControlNormal(this->normal);
		break;
	case 1:
		this->GetDlgItemTextA(IDC_EDIT8, str);		
		this->normal[0] = atof(str);
		this->GetDlgItemTextA(IDC_EDIT9, str);		
		this->normal[1] = atof(str);
		this->GetDlgItemTextA(IDC_EDIT10, str);		
		this->normal[2] = atof(str);
		break;
	case 2:
		break;
	}

	return;		
}