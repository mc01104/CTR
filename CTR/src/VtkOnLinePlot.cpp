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

	this->dataBuffer = new double[timeWindowSize];

	for (int i = 0; i < this->timeWindowSize; ++i)
		this->dataBuffer[i] = 0.0;

	this->start = clock();
}

VtkOnLinePlot::~VtkOnLinePlot()
{
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
	// update the dataBuffer
	memmove(this->dataBuffer, this->dataBuffer + 1, (this->timeWindowSize - 1) * sizeof(double));
	this->dataBuffer[this->timeWindowSize - 1] = sensorData[2];


	vtkSmartPointer<vtkTable> table = vtkSmartPointer<vtkTable>::New();
	vtkSmartPointer<vtkFloatArray> arrX =  vtkSmartPointer<vtkFloatArray>::New();

	arrX->SetName("X Axis");
	table->AddColumn(arrX);
 
	vtkSmartPointer<vtkFloatArray> arrC =  vtkSmartPointer<vtkFloatArray>::New();
	arrC->SetName("Position Error");
	table->AddColumn(arrC);
 
  // Fill in the table with some example values

  table->SetNumberOfRows(timeWindowSize);
  // find a way to maintain this data as a member variable and remove the buffer
  for (int i = 0; i < timeWindowSize; ++i)
  {
    table->SetValue(i, 0, i);
    table->SetValue(i, 1, this->dataBuffer[i]);
  }
 
  // Set up the view
  vtkSmartPointer<vtkContextView> view = vtkSmartPointer<vtkContextView>::New();
  view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
 
  // Add multiple line plots, setting the colors etc
  vtkSmartPointer<vtkChartXY> chart = vtkSmartPointer<vtkChartXY>::New();
  view->GetScene()->AddItem(chart);
  vtkPlot *line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
  line->SetInput(table, 0, 1);
#else
  line->SetInputData(table, 0, 1);
#endif
  line->SetColor(0, 255, 0, 255);
  line->SetWidth(1.0);
  line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
  line->SetInput(table, 0, 2);
#else
  line->SetInputData(table, 0, 2);
#endif
  line->SetColor(255, 0, 0, 255);
  line->SetWidth(5.0);
 
  // For dotted line, the line type can be from 2 to 5 for different dash/dot
  // patterns (see enum in vtkPen containing DASH_LINE, value 2):
#ifndef WIN32
  line->GetPen()->SetLineType(vtkPen::DASH_LINE);
#endif
  // (ifdef-ed out on Windows because DASH_LINE does not work on Windows
  //  machines with built-in Intel HD graphics card...)
 
  //view->GetRenderWindow()->SetMultiSamples(0);
 
  // Start interactor
  view->GetInteractor()->Initialize();
  view->GetInteractor()->Start();
 
}
// VtkOnLinePlot message handlers
