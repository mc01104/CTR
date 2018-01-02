
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
#include <filesystem>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

int CCTRView::m_idActJang[5] = { IDC_A12_ACT, IDC_A13_ACT, IDC_L3_ACT, IDC_A1_ACT, IDC_L1_ACT };
int CCTRView::m_idCmdJang[5] = { IDC_A12_CMD, IDC_A13_CMD, IDC_L3_CMD, IDC_A1_CMD, IDC_L1_CMD };

int CCTRView::manual_point_ENABLE[15] = {IDC_BTN_MOVE3, IDC_BTN_MOVE4, IDC_BTN_MOVE5, IDC_BTN_MOVE6, IDC_BTN_MOVE7, -1};
int CCTRView::manual_point_DISABLE[15] = {IDC_EDIT8, IDC_EDIT9, IDC_EDIT10, IDC_BTN_MOVE8, IDC_EDIT11, IDC_EDIT12, IDC_BTN_MOVE8, -1};

int CCTRView::manual_ENABLE[15] = {IDC_EDIT8, IDC_EDIT9, IDC_EDIT10, IDC_BTN_MOVE8, -1};
int CCTRView::manual_DISABLE[15] = {IDC_BTN_MOVE7, IDC_BTN_MOVE3, IDC_BTN_MOVE4, IDC_BTN_MOVE5, IDC_BTN_MOVE6, -1};

int CCTRView::online_ENABLE[15] = {IDC_EDIT11, IDC_EDIT12, IDC_BTN_MOVE9, -1};
int CCTRView::online_DISABLE[15] = {IDC_EDIT8, IDC_EDIT9, IDC_EDIT10, IDC_BTN_MOVE8, -1};

int CCTRView::m_id_monitor_freq = IDC_A12_ACT2;
int CCTRView::m_id_monitor_freq_breath = IDC_A12_ACT3;

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
	//ON_BN_CLICKED(IDC_RADIO_TELE9, &CCTRView::OnBnClickedRadioModes)

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
	ON_EN_KILLFOCUS(IDC_EDIT19, &CCTRView::OnKillFocusGain)
	ON_EN_KILLFOCUS(IDC_EDIT20, &CCTRView::OnKillFocusGain)

	ON_EN_KILLFOCUS(IDC_EDIT3, &CCTRView::OnKillFocusContactRatio)
	ON_EN_KILLFOCUS(IDC_EDIT14, &CCTRView::OnKillFocusUpdateFrequency)

	ON_BN_CLICKED(IDC_BTN_MOVE3, &CCTRView::OnClickedBtnRecPoint)
	ON_BN_CLICKED(IDC_BTN_MOVE4, &CCTRView::OnClickedBtnPopPoint)
	ON_BN_CLICKED(IDC_BTN_MOVE6, &CCTRView::OnClickedBtnCleanAll)
	ON_BN_CLICKED(IDC_BTN_MOVE5, &CCTRView::OnClickedBtnComputePlane)
	ON_BN_CLICKED(IDC_BTN_MOVE7, &CCTRView::OnClickedBtnUpdate)
	ON_BN_CLICKED(IDC_BTN_MOVE8, &CCTRView::OnClickedBtnUpdate)
	//ON_BN_CLICKED(IDC_BTN_MOVE9, &CCTRView::OnClickedBtnUpdate)

	ON_BN_CLICKED(IDC_BTN_MOVE10, &CCTRView::OnClickedBtnStartLog)
	ON_BN_CLICKED(IDC_BTN_MOVE11, &CCTRView::OnClickedBtnStopLog)

	ON_BN_CLICKED(IDC_CHECK1, &CCTRView::ToggleForceControl)
	ON_BN_CLICKED(IDC_CHECK2, &CCTRView::ToggleCircumnavigation)
	ON_BN_CLICKED(IDC_CHECK3, &CCTRView::ToggleGlobalGains)
	ON_BN_CLICKED(IDC_CHECK4, &CCTRView::ToggleApexToValve)
	ON_BN_CLICKED(IDC_CHECK5, &CCTRView::TogglePullBack)

	ON_EN_KILLFOCUS(IDC_EDIT15, &CCTRView::UpdateGains)
	ON_EN_KILLFOCUS(IDC_EDIT16, &CCTRView::UpdateGains)
	ON_EN_KILLFOCUS(IDC_EDIT17, &CCTRView::UpdateGains)
	ON_EN_KILLFOCUS(IDC_EDIT18, &CCTRView::UpdateGains)

	ON_EN_KILLFOCUS(IDC_EDIT21, &CCTRView::UpdateScalingFactor)
	ON_EN_KILLFOCUS(IDC_EDIT22, &CCTRView::UpdateVSGains)
	ON_EN_KILLFOCUS(IDC_EDIT24, &CCTRView::UpdateVSGains)	
	ON_EN_KILLFOCUS(IDC_EDIT23, &CCTRView::OnKillFocusSamplingPeriods)	
	ON_EN_KILLFOCUS(IDC_EDIT25, &CCTRView::UpdateGainsATV)	
	ON_EN_KILLFOCUS(IDC_EDIT26, &CCTRView::UpdateGainsATV)	
	ON_EN_KILLFOCUS(IDC_EDIT27, &CCTRView::UpdateGainsATV)	
	ON_EN_KILLFOCUS(IDC_EDIT29, &CCTRView::UpdateGainsATV)	

	ON_EN_KILLFOCUS(IDC_EDIT28, &CCTRView::UpdateGlovalGain)	

	ON_BN_CLICKED(IDC_RADIO_JA3, &CCTRView::OnBnClickedRadioModesController)	
	ON_BN_CLICKED(IDC_RADIO_TELE4, &CCTRView::OnBnClickedRadioModesController)

	ON_BN_CLICKED(IDC_RADIO_JA4, &CCTRView::OnBnClickedRadioModesFreq)	
	ON_BN_CLICKED(IDC_RADIO_TELE5, &CCTRView::OnBnClickedRadioModesFreq)

	//ON_BN_CLICKED(IDC_RADIO_JA5, &CCTRView::OnBnClickedRadioModesCircum)	
	//ON_BN_CLICKED(IDC_RADIO_TELE6, &CCTRView::OnBnClickedRadioModesCircum)
	//ON_BN_CLICKED(IDC_RADIO_TELE10, &CCTRView::OnBnClickedRadioModesCircum)
	//ON_BN_CLICKED(IDC_RADIO_TELE11, &CCTRView::OnBnClickedRadioModesCircum)

	ON_BN_CLICKED(IDC_RADIO_JA5, &CCTRView::OnBnClickedRadioModesCWCCW)	
	ON_BN_CLICKED(IDC_RADIO_TELE6, &CCTRView::OnBnClickedRadioModesCWCCW)

	ON_BN_CLICKED(IDC_RADIO_JA6, &CCTRView::OnBnClickedRadioModesATV)	
	ON_BN_CLICKED(IDC_RADIO_TELE7, &CCTRView::OnBnClickedRadioModesATV)
	ON_BN_CLICKED(IDC_RADIO_TELE8, &CCTRView::OnBnClickedRadioModesATV)
	ON_BN_CLICKED(IDC_RADIO_TELE9, &CCTRView::OnBnClickedRadioModesATV)



	ON_WM_CTLCOLOR()
	ON_BN_CLICKED(IDC_BUTTON6, &CCTRView::OnClickedBtnHome)

	//ON_EN_KILLFOCUS(IDC_EDIT30, &CCTRView::OnBnClickedKillFocusId)
	////ON_EN_KILLFOCUS(IDC_EDIT31, &CCTRView::OnBnClickedKillFocusId)
	//ON_EN_KILLFOCUS(IDC_EDIT32, &CCTRView::OnBnClickedKillFocusId)
	//ON_EN_KILLFOCUS(IDC_EDIT33, &CCTRView::OnBnClickedKillFocusId)
	ON_EN_KILLFOCUS(IDC_EDIT34, &CCTRView::OnEnKillFocusBias)

	//ON_EN_KILLFOCUS(IDC_EDIT35, &CCTRView::OnEnKillFocusDisturbance)
	ON_BN_CLICKED(IDC_BTN_MOVE16, &CCTRView::OnBnClickedResetAutomation)

	ON_EN_KILLFOCUS(IDC_EDIT36, &CCTRView::OnKillFocusHour)
	ON_BN_CLICKED(IDC_BTN_MOVE13, &CCTRView::OnClickedBtnGo)

	ON_EN_KILLFOCUS(IDC_EDIT37, &CCTRView::OnEnKillFocusATV)

	ON_BN_CLICKED(IDC_BTN_MOVE9, &CCTRView::OnClickedBtnGetPoint)
	ON_BN_CLICKED(IDC_BTN_MOVE22, &CCTRView::OnClickedBtnDiscardPoint)
	ON_BN_CLICKED(IDC_BTN_MOVE23, &CCTRView::OnClickedBtnRecordPoint)
	ON_BN_CLICKED(IDC_BTN_MOVE24, &CCTRView::OnClickedBtnSaveAll)
	
	ON_EN_KILLFOCUS(IDC_EDIT32, &CCTRView::OnKillFocusRegOffset)


	ON_CBN_SELENDOK(IDC_COMBO1, &CCTRView::OnCbnSelchangeCombo1)

	ON_BN_CLICKED(IDC_CHECK6, &CCTRView::OnClickedBtnClockFace)
	ON_BN_CLICKED(IDC_CHECK7, &CCTRView::OnClickedBtnTEST)
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

	angleBetweenPlaneAndRobot = 0;

	this->azimuth = 0;
	this->altitude = 0;

	this->eStopPressed = false;
	
	for(int i = 0; i < 5; ++i)
		this->recConfiguration[i] = 0.0;

	points_for_plane_estimation.resize(0);
	m_PlaneEstimationMode = 0;
	m_controlMode = 0;
	m_frequencyMode = 1;
	m_direction = 1;			// I am not sure if this is taken into account
	m_apex_wall = 0;
	offsetBetweenValveCenterAndRobotAxis = 0;
	m_transition = false;
	::std::ifstream f("plane_points.txt");
	::std::vector<::std::string> points_on_plane;
	::std::vector<double> tmp_values;
	if (f.good())
	{
		points_on_plane = ReadLinesFromFile("plane_points.txt");
		this->points_for_plane_estimation.clear();
		for (int i = 0; i < points_on_plane.size(); ++i)
		{
			tmp_values = DoubleVectorFromString(points_on_plane[i], ',');
			this->points_for_plane_estimation.push_back(::Eigen::Map<::Eigen::Vector3d> (tmp_values.data(), 3));
		}

	}
	::std::string filename = "plane_coordinates_" + GetDateString() + ".txt";
	::std::remove("plane_points.txt");

}

CCTRView::~CCTRView()
{
}

void CCTRView::OnKillFocusGain()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT2, str);		
	double forceGain = atof(str);
		
	this->GetDlgItemTextA(IDC_EDIT19, str);		
	double forceGainD = atof(str);

	this->GetDlgItemTextA(IDC_EDIT20, str);		
	double forceGainI = atof(str);

	::std::cout << "requested P-gain: " << forceGain << ::std::endl;
	::std::cout << "requested D-gain: " << forceGainD << ::std::endl;
	::std::cout << "requested I-gain: " << forceGainI << ::std::endl;

	this->GetDocument()->SetContacControlGains(forceGain, forceGainD, forceGainI);

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
	//this->GetDocument()->ChangeForceForTuning(force);
	std::cout << "set force" <<  force << ::std::endl;
}

void CCTRView::DoDataExchange(CDataExchange* pDX)
{
	// CKim - This function is called UpdateData()to automatically update the control variable
	// by  to update control variables
	// depending on te operating mode, some controls should not be automatically updated based on the 
	CFormView::DoDataExchange(pDX);

	for(int i=0; i<5; i++)	{	DDX_Text(pDX, m_idActJang[i], m_actJang[i]);	}

	DDX_Text(pDX, m_id_monitor_freq, m_monitor_freq);
	DDX_Text(pDX, m_id_monitor_freq_breath, m_monitor_freq_breath);

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

	CString tmp2;
	tmp2.Format("%d",this->current_measurement.size());
	DDX_Text(pDX, IDC_EDIT13, tmp2);

	CString tmp3;
	tmp3.Format("%f2.1",this->GetDocument()->getClockPosition());
	DDX_Text(pDX, IDC_EDIT31, tmp3);

	CString tmp4;
	tmp4.Format("%f4.2", this->angleBetweenPlaneAndRobot);
	DDX_Text(pDX, IDC_EDIT33, tmp4);

	tmp4.Format("%f2.1",this->offsetBetweenValveCenterAndRobotAxis);
	DDX_Text(pDX, IDC_EDIT35, tmp4);


	tmp4.Format("%f2.1", this->center[0]);
	DDX_Text(pDX, IDC_EDIT38, tmp4);

	tmp4.Format("%f2.1", this->center[1]);
	DDX_Text(pDX, IDC_EDIT39, tmp4);

	tmp4.Format("%f2.1", this->center[2]);
	DDX_Text(pDX, IDC_EDIT40, tmp4);

	tmp4.Format("%f2.1", this->azimuth);
	DDX_Text(pDX, IDC_EDIT42, tmp4);

	tmp4.Format("%f2.1", this->altitude);
	DDX_Text(pDX, IDC_EDIT41, tmp4);


	DDX_Radio(pDX, IDC_RADIO_JA2, m_PlaneEstimationMode);
	DDX_Radio(pDX, IDC_RADIO_JA3, m_controlMode);
	DDX_Radio(pDX, IDC_RADIO_JA4, m_frequencyMode);
	DDX_Radio(pDX, IDC_RADIO_JA5, m_direction);

	DDX_Radio(pDX, IDC_RADIO_JA6, m_apex_wall);
	m_ctrlMode != 1 ? GetDlgItem(IDC_CHECK1)->EnableWindow(false) : GetDlgItem(IDC_CHECK1)->EnableWindow(true);
	//	m_ctrlMode != 1 ? GetDlgItem(IDC_CHECK3)->EnableWindow(false) : GetDlgItem(IDC_CHECK3)->EnableWindow(true);

	DDX_Control(pDX, IDC_COMBO1, m_combo);
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

	//CString str("ECG");
	//m_combo.AddString(str);


	//str.Format("HR Art");
	//m_combo.AddString(str);
	//str.Format("SPO2");
	//m_combo.AddString(str);


	CString tmp;
	tmp.Format("%d",this->points_for_plane_estimation.size());
	this->SetDlgItemTextA(IDC_EDIT4,tmp);

	// CKim - Initialize controls here
	this->SetDlgItemTextA(m_idCmdJang[0],"180");		this->SetDlgItemTextA(m_idCmdJang[1],"0");	
	this->SetDlgItemTextA(m_idCmdJang[2],"5");			this->SetDlgItemTextA(m_idCmdJang[3],"0");			this->SetDlgItemTextA(m_idCmdJang[4],"0");
	
	//this->updateGUIActivationState(manual_point_ENABLE, manual_point_DISABLE);

	this->SetTimer(100,30,NULL);
	QueryPerformanceFrequency(&m_Freq);

	tmp.Format("%f",1.0);
	this->SetDlgItemTextA(IDC_EDIT15,tmp);
	tmp.Format("%f",0.7);
	this->SetDlgItemTextA(IDC_EDIT16,tmp);

	tmp.Format("%0.2f",0.7);
	this->SetDlgItemTextA(IDC_EDIT17,tmp);
	tmp.Format("%0.2f",0.0);
	this->SetDlgItemTextA(IDC_EDIT18,tmp);

	tmp.Format("%0.2f",10.0);
	this->SetDlgItemTextA(IDC_EDIT2,tmp);
	tmp.Format("%0.2f",0.0);
	this->SetDlgItemTextA(IDC_EDIT19,tmp);

	tmp.Format("%0.2f",0.0);
	this->SetDlgItemTextA(IDC_EDIT3,tmp);

	tmp.Format("%0.2f",26.27);
	this->SetDlgItemTextA(IDC_EDIT21,tmp);

	tmp.Format("%0.2f",0.3);
	this->SetDlgItemTextA(IDC_EDIT22,tmp);
	this->SetDlgItemTextA(IDC_EDIT24,tmp);

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
	m_monitor_freq.Format("%3.1f", this->GetDocument()->GetMonitorFreq());
	m_monitor_freq_breath.Format("%2.1f", this->GetDocument()->GetMonitorBreathingFreq());

	UpdateData(false);

	//if(this->logDataFlag)
	//{
	//	QueryPerformanceCounter(&m_Etime);	
	//	m_Elapsed.QuadPart = m_Etime.QuadPart - m_Stime.QuadPart;
	//	m_Elapsed.QuadPart *= 1000000;
	//	m_Elapsed.QuadPart /= m_Freq.QuadPart;
	//	double dt = m_Elapsed.QuadPart/1000;

	//	for(int i=0; i<6; i++)	
	//		this->logStream << stat.tgtTipPosDir[i] << "\t";		
	//	//this->logStream << ::std::endl;

	//	for(int i = 0; i < 5; ++i)
	//		this->logStream << stat.currJang[i] << "\t";
	//	this->logStream << ::std::endl;

	//}

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

	if (m_ctrlMode == 2)
	{
		this->GetDocument()->switchIDMode(1);
		return;
	}
	else if (prevMode == 2)
	{
		this->GetDocument()->switchIDMode(0);
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
		this->GetDocument()->ToggleCameraControl();
}

void CCTRView::OnClickedBtnComputePlane()
{
	::Eigen::MatrixXd data(3, this->points_for_plane_estimation.size());

	for (int i = 0; i < this->points_for_plane_estimation.size(); ++i)
		data.col(i) = this->points_for_plane_estimation[i];
	

	::Eigen::Vector3d mu = data.rowwise().mean();
	::Eigen::Matrix3Xd points_centered = data.colwise() - mu;

	int setting = Eigen::ComputeFullU | Eigen::ComputeFullU;
	Eigen::JacobiSVD<Eigen::Matrix3Xd> svd = points_centered.jacobiSvd(setting);
	::Eigen::MatrixXd U = svd.matrixU();
	Eigen::Vector3d normal_tmp = U.col(2);

	// the computed plane normal is always pointing to the positive z-direction 
	if (normal_tmp(2) < 0)
		normal_tmp = -normal_tmp;

	this->normal = normal_tmp;;
	this->normal.normalize();

	::Eigen::Vector3d center;
	double radius;
	this->computeCircle(U, center, radius);
	center += mu;

	this->center = center;

	this->radius = radius;
	double tmpInnerProduct = this->normal.transpose() * ::Eigen::Vector3d(0, 0, 1);
	this->angleBetweenPlaneAndRobot = acos(tmpInnerProduct) * 180.0/M_PI;

	this->offsetBetweenValveCenterAndRobotAxis = this->center.segment(0, 2).norm();

	::Eigen::Vector3d YZ(1, 0, 0);
	double lambda = this->normal.transpose() * YZ;
	::Eigen::Vector3d normalYZ = this->normal - lambda * YZ;

	tmpInnerProduct = this->normal.transpose() * normalYZ;
	this->azimuth = acos(tmpInnerProduct) * 180.0/M_PI;

	::Eigen::Vector3d XZ(0, 1, 0);
	lambda = this->normal.transpose() * XZ;
	::Eigen::Vector3d normalXZ = this->normal - lambda * XZ;
	tmpInnerProduct = this->normal.transpose() * normalXZ;
	this->altitude = acos(tmpInnerProduct) * 180.0/M_PI;

	this->dumpPlanePoints();
}

void CCTRView::computeCircle(::Eigen::Matrix3d rot, ::Eigen::Vector3d& center, double& radius)
{
	::Eigen::MatrixXd data(3, this->points_for_plane_estimation.size());
	for (int i = 0; i < this->points_for_plane_estimation.size(); ++i)
		data.col(i) = this->points_for_plane_estimation[i];
	
	// not sure if this is correct
	::Eigen::Vector3d mu = data.rowwise().mean();
	::Eigen::Matrix3Xd points_centered = data.colwise() - mu;

	::std::vector<::Eigen::Vector3d> rotated_points;
	
	for(int i = 0; i < points_for_plane_estimation.size(); ++i)
		rotated_points.push_back(rot*points_centered.col(i));

	fitCircle(rotated_points, center, radius);

	center = rot.transpose() * center;
}

void CCTRView::OnBnClickedRadioModesPlane()
{

	UpdateData(true);	
	
	if(m_PlaneEstimationMode == 0)	
	{
		::std::cout << "Estimate plane from number of points" << ::std::endl;
		//this->updateGUIActivationState(manual_point_ENABLE, manual_point_DISABLE);
	}
	else if (m_PlaneEstimationMode == 1)
	{
		::std::cout << "Manual input of plane normal" << ::std::endl;
		//this->updateGUIActivationState(manual_ENABLE, manual_DISABLE);
	}
	else if (m_PlaneEstimationMode == 2)
	{
		::std::cout << "Online plane estimation!" << ::std::endl;
		//this->updateGUIActivationState(manual_point_ENABLE, manual_point_DISABLE);
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
		this->GetDocument()->setContactControlNormal(this->normal, this->center, radius, this->points_for_plane_estimation);
		break;
	case 1:
		this->GetDlgItemTextA(IDC_EDIT8, str);		
		this->normal[0] = atof(str);
		this->GetDlgItemTextA(IDC_EDIT9, str);		
		this->normal[1] = atof(str);
		this->GetDlgItemTextA(IDC_EDIT10, str);		
		this->normal[2] = atof(str);
		this->GetDocument()->setContactControlNormal(this->normal, this->center, radius, this->points_for_plane_estimation);
		break;
	case 2:
		break;
	}

	return;		
}

void CCTRView::OnClickedBtnStartLog()
{
	this->GetDlgItem(IDC_BTN_MOVE10)->EnableWindow(FALSE);
	this->GetDlgItem(IDC_BTN_MOVE11)->EnableWindow(TRUE);
	this->GetDocument()->ToggleLog();
}

void CCTRView::OnClickedBtnStopLog()
{
	this->GetDlgItem(IDC_BTN_MOVE10)->EnableWindow(TRUE);
	this->GetDlgItem(IDC_BTN_MOVE11)->EnableWindow(FALSE);

	this->GetDocument()->ToggleLog();

}

void CCTRView::OnKillFocusUpdateFrequency()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT14, str);		
	this->GetDocument()->SetFrequency(atof(str));
	::std::cout << "in callback" << ::std::endl;
	::std::cout << str << ::std::endl;
}

void CCTRView::UpdateGains()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT15, str);		
	double position_gain = atof(str);
	
	this->GetDlgItemTextA(IDC_EDIT16, str);		
	double orientation_gain = atof(str);

	this->GetDlgItemTextA(IDC_EDIT17, str);		
	double position_gain_feedforward = atof(str);
	
	this->GetDlgItemTextA(IDC_EDIT18, str);		
	double orientation_gain_feedforward = atof(str);


	this->GetDocument()->UpdateGains(position_gain, orientation_gain, position_gain_feedforward, orientation_gain_feedforward);
}

void CCTRView::OnBnClickedRadioModesController()
{
	UpdateData(true);	
	
	if(m_controlMode == 0)	
	{
		::std::cout << "Nullspace Controller" << ::std::endl;
	}
	else if (m_controlMode == 1)
	{
		::std::cout << "Weighted Cotnroller" << ::std::endl;
	}

	this->GetDocument()->SwitchControlMode(m_controlMode);
	return;		
}

void CCTRView::OnBnClickedRadioModesFreq()
{
	UpdateData(true);	
	
	if(m_frequencyMode == 0)	
	{
		::std::cout << "Manual Frequency input activated" << ::std::endl;
	}
	else if (m_frequencyMode == 1)
	{
		::std::cout << "Heart rate frequency is streamed from SurgiVet" << ::std::endl;
	}

	this->GetDocument()->SwitchFreqMode(m_frequencyMode);
	return;		
	
}

void CCTRView::ToggleCircumnavigation()
{
	UpdateData(true);

	bool circumnavigation_status = this->IsDlgButtonChecked(IDC_CHECK2);
	bool contact_control_status = this->IsDlgButtonChecked(IDC_CHECK1);
	bool apex_to_valve_status = this->IsDlgButtonChecked(IDC_CHECK4);
	
	UpdateData(false);

	if (circumnavigation_status == 0)
		this->CheckDlgButton(IDC_CHECK1, 0);
	else
	{
		this->CheckDlgButton(IDC_CHECK1, 1);
		this->CheckDlgButton(IDC_CHECK4, 0);
	}

	//this->GetDocument()->ToggleForceControl();
	this->GetDocument()->ToggleCircumnavigation();
}

void CCTRView::ToggleApexToValve()
{
	UpdateData(true);

	bool apex_to_valve_status = this->IsDlgButtonChecked(IDC_CHECK4);
	//bool contact_control_status = this->IsDlgButtonChecked(IDC_CHECK1);
	bool circum_status = this->IsDlgButtonChecked(IDC_CHECK2);

	UpdateData(false);

	//if (apex_to_valve_status == 0)
	//	this->CheckDlgButton(IDC_CHECK1, 0);
	//else
	if (apex_to_valve_status == 1)
	{
		this->CheckDlgButton(IDC_CHECK1, 0);
		this->CheckDlgButton(IDC_CHECK2, 0);
	}
	this->GetDocument()->ToggleApexToValve();
	//this->GetDocument()->ToggleForceControl();
}

void CCTRView::UpdateScalingFactor()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT21, str);		
	this->GetDocument()->SetScalingFactor(atof(str));
}

void CCTRView::UpdateCentroid()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT22, str);		
	double x = atof(str);
	this->GetDlgItemTextA(IDC_EDIT24, str);		
	double y = atof(str);
	this->GetDocument()->SetCentroid(x, y);
}

void CCTRView::UpdateTangent()
{
	//double tangent[2];
	//CString str;
	//this->GetDlgItemTextA(IDC_EDIT23, str);	
	//tangent[0] = atof(str);
	//this->GetDlgItemTextA(IDC_EDIT25, str);	
	//tangent[1] = atof(str);
	//this->GetDocument()->SetTangent(tangent);
}

void CCTRView::OnBnClickedRadioModesCWCCW()
{
	UpdateData(true);	
	CCTRDoc::CIRCUM_DIRECTION sts;
	if(m_direction == 0)	
	{
		::std::cout << "Circumnavigation direction: clockwise" << ::std::endl;
		sts = CCTRDoc::CIRCUM_DIRECTION::CW;
	}
	else if (m_direction == 1)
	{
		::std::cout << "Circumnavigation direction: counter-clockwise" << ::std::endl;
		sts = CCTRDoc::CIRCUM_DIRECTION::CCW;
	}

	this->GetDocument()->SwitchCircumStatus(sts);
	return;		

}

void CCTRView::OnBnClickedRadioModesCircum()
{
	UpdateData(true);	
	CCTRDoc::CIRCUM_STATUS sts;
	if(m_direction == 0)	
	{
		::std::cout << "Circumnavigation direction: up" << ::std::endl;
		sts = CCTRDoc::CIRCUM_STATUS::UP;
	}
	else if (m_direction == 1)
	{
		::std::cout << "Circumnavigation direction: left" << ::std::endl;
		sts = CCTRDoc::CIRCUM_STATUS::LEFT_A;
	}
	else if (m_direction == 2)
	{
		::std::cout << "Circumnavigation direction: down" << ::std::endl;
		sts = CCTRDoc::CIRCUM_STATUS::DOWN;
	}
	else if (m_direction == 3)
	{
		::std::cout << "Circumnavigation direction: right" << ::std::endl;
		sts = CCTRDoc::CIRCUM_STATUS::RIGHT;
	}

	this->GetDocument()->SwitchCircumStatus(sts);
	return;		

}


void CCTRView::UpdateVSGains()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT22, str);	
	double gain_center = atof(str);
	this->GetDlgItemTextA(IDC_EDIT24, str);	
	double gain_tangent = atof(str);
	this->GetDocument()->SetVSGains(gain_center, gain_tangent);
}

void CCTRView::ToggleGlobalGains()
{
	this->GetDocument()->ToggleJacobianContactControl();
}

void CCTRView::OnKillFocusSamplingPeriods()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT23, str);	
	int samplingPeriods = atoi(str);
	//::std::cout << "sampling periods: " << samplingPeriods << ::std::endl;
	this->GetDocument()->SetSamplingPeriod(samplingPeriods);
}


void CCTRView::UpdateGainsATV()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT25, str);	
	double gain_forward = atof(str);
	this->GetDlgItemTextA(IDC_EDIT26, str);	
	double gain_center = atof(str);
	this->GetDlgItemTextA(IDC_EDIT27, str);	
	double thres_min = atof(str);
	this->GetDlgItemTextA(IDC_EDIT29, str);	
	double thres_max = atof(str);

	this->GetDocument()->UpdateGainsApexToValve(gain_center, gain_forward, thres_min, thres_max);


}

void CCTRView::UpdateGlovalGain()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT28, str);
	double gain = atof(str);
	this->GetDocument()->UpdateGlobalGain(gain);
}

void CCTRView::OnBnClickedRadioModesATV()
{
	UpdateData(true);	

	if(m_apex_wall == 0)	
	{
		::std::cout << "Apex-to-valve direction: left" << ::std::endl;
		this->GetDocument()->SwitchApexToValveStatus(CCTRDoc::APEX_TO_VALVE_STATUS::LEFT);
	}
	else if (m_apex_wall == 1)
	{
		::std::cout << "Apex-to-valve direction: top" << ::std::endl;
		this->GetDocument()->SwitchApexToValveStatus(CCTRDoc::APEX_TO_VALVE_STATUS::TOP);
	}
	else if (m_apex_wall == 2)
	{
		::std::cout << "Apex-to-valve direction: bottom" << ::std::endl;
		this->GetDocument()->SwitchApexToValveStatus(CCTRDoc::APEX_TO_VALVE_STATUS::BOTTOM);
	}
	else if (m_apex_wall == 3)
	{
		::std::cout << "Apex-to-valve direction: user-defined" << ::std::endl;
		this->GetDocument()->SwitchApexToValveStatus(CCTRDoc::APEX_TO_VALVE_STATUS::USER);
	}
	else
	{
		::std::cout << "undefined direction for apex-to-valve navigation -> switching to default (LEFT)" << ::std::endl;
		this->GetDocument()->SwitchApexToValveStatus(CCTRDoc::APEX_TO_VALVE_STATUS::LEFT);
	}	
	return;		

}


void CCTRView::OnBnClickedKillFocusId()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT30, str);	
	double min_freq = atof(str);
	//this->GetDlgItemTextA(IDC_EDIT31, str);	
	double max_freq = atof(str);
	this->GetDlgItemTextA(IDC_EDIT32, str);	
	double amplitude = atof(str);
	this->GetDlgItemTextA(IDC_EDIT33, str);	
	int num_of_sins = atof(str);

	::std::cout << "minimum frequency " <<  min_freq << " " << "max_frq: " << max_freq << ::std::endl;
	::std::cout << "amplitude: " << amplitude << " num_of_sins: " << num_of_sins << ::std::endl;
	this->GetDocument()->UpdateIDParams(min_freq, max_freq, amplitude, num_of_sins);

}

void CCTRView::OnEnKillFocusBias()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT34, str);	
	double bias = atof(str);
	this->GetDocument()->SetBias(bias);
	::std::cout << "bias:" << bias <<::std::endl;
}

void CCTRView::OnEnKillFocusDisturbance()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT35, str);	
	double disturbance = atof(str);
	this->GetDocument()->SetDisturbance(disturbance);
	::std::cout << "disturbance amplitude:" << disturbance <<::std::endl;
}

void CCTRView::OnBnClickedResetAutomation()
{
	::std::cout << "resetting " << ::std::endl;
	UpdateData(false);
	this->CheckDlgButton(IDC_CHECK1, 0);
	this->CheckDlgButton(IDC_CHECK2, 0);
	this->CheckDlgButton(IDC_CHECK4, 0);
	this->GetDocument()->resetAutomation();
}

void CCTRView::TogglePullBack()
{
	this->GetDocument()->TogglePullback();
}

void
CCTRView::dumpPlanePoints()
{
	::std::ofstream os("plane_points.txt");
	::std::ofstream os2("plane_" + GetDateString() + ".txt");
	for (int i = 0; i < this->points_for_plane_estimation.size(); ++i)
	{
		os << this->points_for_plane_estimation[i](0) << ", " << this->points_for_plane_estimation[i](1) << ", " << this->points_for_plane_estimation[i](2) <<::std::endl;
		os2 << this->points_for_plane_estimation[i](0) << ", " << this->points_for_plane_estimation[i](1) << ", " << this->points_for_plane_estimation[i](2) <<::std::endl;
		
	}

	os.close();
	os2.close();
}

void 
CCTRView::OnKillFocusHour()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT36, str);	
	double desClockPosition = atof(str);

	this->GetDocument()->setDesiredClockfacePosition(desClockPosition);
}

void
CCTRView::OnEnKillFocusATV()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT37, str);	
	double desClockPosition = atof(str);

	this->GetDocument()->setDesiredWallClockfacePosition(desClockPosition);
}


void 
CCTRView::OnClickedBtnGo()
{
	this->GetDocument()->activateClockfaceTask();
}


void 
CCTRView::OnClickedBtnGetPoint()
{
	::Eigen::Vector3d tmp  = this->GetDocument()->GetTipPosition();
	if (this->current_measurement.size() < 2)
		this->current_measurement.push_back(tmp);
}

void 
CCTRView::OnClickedBtnDiscardPoint()
{
	::Eigen::Vector3d tmp  = this->GetDocument()->GetTipPosition();
	if (this->current_measurement.size() > 0);
		this->current_measurement.pop_back();

}

void 
CCTRView::OnClickedBtnRecordPoint()
{
	if (this->current_measurement.size() == 2)
	{
		this->leak_measurements.push_back(this->current_measurement);
		this->current_measurement.clear();		
	}
}


void 
CCTRView::OnClickedBtnSaveAll()
{

	::std::ofstream os("leak_measurements_" + GetDateString() + ".txt");
	for (int i = 0; i < this->leak_measurements.size(); ++i)
		os << this->leak_measurements[i][0].transpose() << " " << this->leak_measurements[i][1].transpose() << ::std::endl;

	os.close();

}

void 
CCTRView::OnKillFocusRegOffset()
{
	CString str;
	this->GetDlgItemTextA(IDC_EDIT32, str);	
	double offset = atof(str);

	this->GetDocument()->setRegistrationOffset(offset);
}


void CCTRView::OnCbnSelchangeCombo1() 
{
   int nIndex = m_combo.GetCurSel();
	CString strCBText;
	m_combo.GetLBText( nIndex, strCBText);

   UpdateData(FALSE);
   std::string s((LPCTSTR) strCBText);
   ::std::cout << s << ::std::endl;

   int n = 0;
   if (s == "ECG")
		n = 1;
   else if  (s == "HR Art")
	   n = 4;
   else
	   n = 8;

   ::std::cout << n << ::std::endl;
   this->GetDocument()->setHRSource(n);
}

void CCTRView::OnClickedBtnClockFace()
{
	this->GetDocument()->ToggleClockface();
}

void CCTRView::OnClickedBtnTEST()
{
	this->GetDocument()->ToggleTest();
}