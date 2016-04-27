// VtkOnLinePlot.cpp : implementation file
//

#include "stdafx.h"
#include "CTRApp.h"
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

}

VtkOnLinePlot::~VtkOnLinePlot()
{
	m_Renderer->Delete();
	m_iren->Delete();
	m_renWin->Delete();

	delete this->dataBuffer;
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

	if(!m_iren->GetInitialized())
	{
		m_iren->Initialize();

	}

	//m_view->Update();
	//view->GetInteractor()->Initialize();
	//view->GetInteractor()->Render();
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

	memmove(this->dataBuffer, this->dataBuffer + 1, (this->timeWindowSize - 1)*sizeof(double)); 
	this->dataBuffer[this->timeWindowSize - 1] = predictionData[2];

  // Create a table with some points in it
  vtkSmartPointer<vtkTable> table =  vtkSmartPointer<vtkTable>::New();
 
  vtkSmartPointer<vtkFloatArray> arrX = vtkSmartPointer<vtkFloatArray>::New();
  arrX->SetName("X Axis");
  table->AddColumn(arrX);
 
  vtkSmartPointer<vtkFloatArray> arrC =  vtkSmartPointer<vtkFloatArray>::New();
  arrC->SetName("Cosine");  table->AddColumn(arrC);
 
 
  table->SetNumberOfRows(timeWindowSize);
  for (int i = 0; i < timeWindowSize; ++i)
  {
    table->SetValue(i, 0, i );
    table->SetValue(i, 1, dataBuffer[i]);
  }
 
  // Set up the view
  vtkSmartPointer<vtkContextView> view = 
    vtkSmartPointer<vtkContextView>::New();
  view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
 
  // Add multiple line plots, setting the colors etc
  vtkSmartPointer<vtkChartXY> chart = 
    vtkSmartPointer<vtkChartXY>::New();
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

  // Start interactor
  view->GetInteractor()->Initialize();
}
// VtkOnLinePlot message handlers
