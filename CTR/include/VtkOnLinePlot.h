#pragma once

#include "vtkRenderer.h"
#include "vtkWin32OpenGLRenderWindow.h"
#include "vtkWin32RenderWindowInteractor.h"


#include <vtkVersion.h>

#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkPlot.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>

#include <ctime>

class VtkOnLinePlot : public CDialog
{
	DECLARE_DYNAMIC(VtkOnLinePlot)

	vtkWin32OpenGLRenderWindow*		m_renWin;
	vtkRenderer*					m_Renderer;
	vtkWin32RenderWindowInteractor*	m_iren;

	vtkSmartPointer<vtkContextView> m_view;
	vtkSmartPointer<vtkChartXY>		m_chart;
	vtkPlot*						m_line;
	int timeWindowSize;

	clock_t start;
	clock_t end;

	vtkSmartPointer<vtkTable>		m_table;

public:
	VtkOnLinePlot(CWnd* pParent = NULL);   // standard constructor
	virtual ~VtkOnLinePlot();

// Dialog Data
	enum { IDD = ID_VTKPLOT };

	void TesterFunction();
	void PlotData(const double* sensorData, const double* predictionData);


protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()

public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnDestroy();

};
