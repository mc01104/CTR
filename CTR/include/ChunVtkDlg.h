// ---------------------------------------------------------------- //
// CKim - Dialog box that can render graphics using VTK
// Provides methods for rendering various geometry and medical images
// inside the dialog. Could have been CView instead of CDialog....
// Maybe that interfaces better with CDocuments
// Last updated : Oct. 17, 2014 
// ---------------------------------------------------------------- //

#pragma once

// CKim - VTK Headers for basic rendering window
#include "vtkRenderer.h"
#include "vtkWin32OpenGLRenderWindow.h"
#include "vtkWin32RenderWindowInteractor.h"

// CKim - VTK Headers for drawing objects
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkCylinderSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkAxesActor.h"
#include "vtkTransform.h"
#include "vtkMatrix4x4.h"
#include "vtkProperty.h"
#include "vtkAssembly.h"
#include "vtkCamera.h"
#include "vtkPolyData.h"

#include "vtkPoints.h"
#include "vtkCellArray.h"
#include "vtkTubeFilter.h"


// ChunVtkDlg dialog

class ChunVtkDlg : public CDialog
{
	DECLARE_DYNAMIC(ChunVtkDlg)

public:
	ChunVtkDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~ChunVtkDlg();

	// CKim - Function that manpulates graphical objects
	void MoveCursor(double x, double y, double z);
	void SetCursorSize(double rad);		
	void SetCursorColor(const float r, const float g, const float b);

	void MoveCsys(double* tf);

	void InitWorkSpace();

	
	void InitObjects();		// CKim - Initializ all the objects used in the scene

	void RenderBalancedPair(const double* bpTipPosDir);
	void RenderTip(const double* bpTipPosDir, const double* currTipPosDir);
	void RenderProxy(const double* tgtTipPosDir);
	void ResetCam();
	

// Dialog Data
	enum { IDD = IDD_CHUNVTKDLG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	// CKim - VTK rendering window components
	vtkWin32OpenGLRenderWindow*		m_renWin;
	vtkRenderer*					m_Renderer;
	vtkWin32RenderWindowInteractor*	m_iren;

	// CKim - VTK Props for representing various objects
	vtkSmartPointer<vtkActor>		m_pCursor;
	vtkSmartPointer<vtkActor>		m_pCurrTip;		// CKim - Cylinder representing current tip position and orientation from FwdKin
	vtkSmartPointer<vtkAxesActor>	m_pRefCsys;

	vtkSmartPointer<vtkAxesActor>	m_pCsys;
	vtkSmartPointer<vtkActor>		m_pBalancePair;
	vtkSmartPointer<vtkActor>		m_pThirdTube;
	vtkSmartPointer<vtkActor>		m_pWkSpcInside;
	vtkSmartPointer<vtkActor>		m_pWkSpcOutside;
	vtkSmartPointer<vtkActor>		m_pProxy;		// CKim - Cylinder representing the tip position and orientation commanded by Omni

	DECLARE_MESSAGE_MAP()
public:

	// CKim - message handler OnCreate() OnPaint() OnSize() OnEraseBkgnd() are basic 
	// message handlers that needs to be modified
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnDestroy();
};
