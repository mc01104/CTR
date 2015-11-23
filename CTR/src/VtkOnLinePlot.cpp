// VtkOnLinePlot.cpp : implementation file
//

#include "stdafx.h"
#include "CTR.h"
#include "VtkOnLinePlot.h"
#include "afxdialogex.h"

#include <iostream>

// VtkOnLinePlot dialog

IMPLEMENT_DYNAMIC(VtkOnLinePlot, CDialog)

VtkOnLinePlot::VtkOnLinePlot(CWnd* pParent /*=NULL*/)
	: CDialog(VtkOnLinePlot::IDD, pParent),
	timeWindowSize(200)
{
	m_renWin = vtkWin32OpenGLRenderWindow::New();
	m_Renderer = vtkRenderer::New();
	m_iren = vtkWin32RenderWindowInteractor::New();

	m_view = vtkSmartPointer<vtkContextView>::New();

	m_view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
 

    m_chart = vtkSmartPointer<vtkChartXY>::New();
	m_chart->AutoAxesOff();

	m_chart->GetAxis(vtkAxis::LEFT)->SetRange(0,200);	
	m_chart->GetAxis(vtkAxis::LEFT)->SetBehavior(vtkAxis::FIXED);
	m_chart->GetAxis(vtkAxis::BOTTOM)->SetRange(0,200);	
	m_chart->GetAxis(vtkAxis::BOTTOM)->SetBehavior(vtkAxis::FIXED);

	m_view->GetScene()->AddItem(m_chart);
	m_line = m_chart->AddPlot(vtkChart::LINE);

	m_table =  vtkSmartPointer<vtkTable>::New();

	vtkSmartPointer<vtkFloatArray> arrX =  vtkSmartPointer<vtkFloatArray>::New();

	arrX->SetName("X Axis");
	m_table->AddColumn(arrX);
 
	vtkSmartPointer<vtkFloatArray> arrC =  vtkSmartPointer<vtkFloatArray>::New();
	arrC->SetName("Position Error");
	m_table->AddColumn(arrC);

	m_table->SetNumberOfRows(timeWindowSize);

	// find a way to maintain this data as a member variable and remove the buffer
	for (int i = 0; i < timeWindowSize; ++i)
	{
		m_table->SetValue(i, 0, i);
		m_table->SetValue(i, 1, 0);
	}


}

VtkOnLinePlot::~VtkOnLinePlot()
{
	m_Renderer->Delete();
	m_iren->Delete();
	m_renWin->Delete();
}

void VtkOnLinePlot::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


void VtkOnLinePlot::TesterFunction()
{
	::std::cout << "I am in the event handler!!" << ::std::endl;
}

BEGIN_MESSAGE_MAP(VtkOnLinePlot, CDialog)
	ON_WM_CREATE()
	ON_WM_PAINT()
	ON_WM_SIZE()
	ON_WM_ERASEBKGND()
	ON_WM_DESTROY()
END_MESSAGE_MAP()

int VtkOnLinePlot::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialog::OnCreate(lpCreateStruct) == -1)
		return -1;

	// CKim - OnCreate is called by OS when the window is created by CDialog->Create(resource id, pointer to parent window)
	// Connect renderer to render window and render window to this dialog
	m_renWin->AddRenderer(m_Renderer);
	m_renWin->SetParentId(this->GetSafeHwnd());
	m_iren->SetRenderWindow(m_renWin);

	//m_view->GetInteractor()->Initialize();
	//m_view->GetInteractor()->Start();

	return 0;
}

void VtkOnLinePlot::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	// Do not call CDialog::OnPaint() for painting messages

	// CKim - OnPaint() is called by OS whenever UI area needs to be updated. 
	// All actors that are added to the renderer are rendered here

	// CKim - For the first time, initialize render interactor, set the 
	// size of the render window and reset the camera of the renderer
	if(!m_iren->GetInitialized())
	{
		m_iren->Initialize();

	}

	m_view->Update();

	// CKim - Rendering only starts when 'render()' is called. Render() should be called through
	// renderwindow or interactor class, not straight from rendere
	m_iren->Render();
}

void VtkOnLinePlot::OnSize(UINT nType, int cx, int cy)
{
	CDialog::OnSize(nType, cx, cy);

	// CKim - This function is called when the window is resized
	// Adjust the size of the renderwindow accordingly
	CRect rect;		this->GetClientRect(&rect);
	m_renWin->SetSize(rect.right-rect.left,rect.bottom-rect.top);
}

BOOL VtkOnLinePlot::OnEraseBkgnd(CDC* pDC)
{
	// TODO: Add your message handler code here and/or call default
	//return CDialog::OnEraseBkgnd(pDC);
	return true;
}

void VtkOnLinePlot::OnDestroy()
{
	CDialog::OnDestroy();
	
	// TODO: Add your message handler code here
	m_renWin->Finalize();
}

void VtkOnLinePlot::PlotData(const double* sensorData, const double* predictionData)
{

	//::std::cout << "predicted z = "<< predictionData[2] << ::std::endl;
	// update the table
	m_table->RemoveRow(0);
	m_table->InsertNextBlankRow();
	m_table->SetValue(timeWindowSize - 1, 0, timeWindowSize);
	m_table->SetValue(timeWindowSize - 1, 1, predictionData[2]);
  
	m_line->SetInput(m_table, 0, 1);

	m_line->SetColor(0, 255, 0, 255);
	m_line->SetWidth(1.0);

}
// VtkOnLinePlot message handlers
