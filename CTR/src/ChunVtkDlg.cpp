// ---------------------------------------------------------------- //
// CKim - Dialog box that can render graphics using VTK
// Provides methods for rendering various geometry and medical images
// inside the dialog. Could have been CView instead of CDialog....
// Maybe that interfaces better with CDocuments
// Last updated : Oct. 17, 2014 
// ---------------------------------------------------------------- //
// ChunVtkDlg.cpp : implementation file
//

#include "stdafx.h"
#include "CTRApp.h"
#include "ChunVtkDlg.h"
#include "afxdialogex.h"

#include "ChunKinematics.h"

// CKim - Eigen Header. Located at "C:\Chun\ChunLib"
#include <Eigen/Dense>


// ChunVtkDlg dialog

IMPLEMENT_DYNAMIC(ChunVtkDlg, CDialog)

ChunVtkDlg::ChunVtkDlg(CWnd* pParent /*=NULL*/)
	: CDialog(ChunVtkDlg::IDD, pParent)
{
	// CKim - Create window, renderer and interactor
	m_renWin = vtkWin32OpenGLRenderWindow::New();
	m_Renderer = vtkRenderer::New();
	m_iren = vtkWin32RenderWindowInteractor::New();
}

ChunVtkDlg::~ChunVtkDlg()
{
	// CKim - Clean up
	m_Renderer->Delete();
	m_iren->Delete();
	m_renWin->Delete();
}

void ChunVtkDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(ChunVtkDlg, CDialog)
	ON_WM_CREATE()
	ON_WM_PAINT()
	ON_WM_SIZE()
	ON_WM_ERASEBKGND()
	ON_WM_DESTROY()
END_MESSAGE_MAP()


// ChunVtkDlg message handlers


int ChunVtkDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialog::OnCreate(lpCreateStruct) == -1)
		return -1;

	// CKim - OnCreate is called by OS when the window is created by CDialog->Create(resource id, pointer to parent window)
	// Connect renderer to render window and render window to this dialog
	m_renWin->AddRenderer(m_Renderer);
	m_renWin->SetParentId(this->GetSafeHwnd());
	m_iren->SetRenderWindow(m_renWin);
	
	// CKim - This turns off the use of default meassage handler provided by vtkWin32OpenGLRenderWindow.
	//m_iren->InstallMessageProcOff();

	// CKim - Initialize objects, vtkAxesActor is convenient class for drawing coordinate system
	m_pCursor = vtkSmartPointer<vtkActor>::New();
	m_pRefCsys = vtkSmartPointer<vtkAxesActor>::New();	
	m_pCurrTip = vtkSmartPointer<vtkActor>::New();
	
	m_pCsys = vtkSmartPointer<vtkAxesActor>::New();
	m_pBalancePair = vtkSmartPointer<vtkActor>::New();
	m_pThirdTube = vtkSmartPointer<vtkActor>::New();
	m_pProxy = vtkSmartPointer<vtkActor>::New();

	return 0;
}

void ChunVtkDlg::OnPaint()
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

		CRect rect;			this->GetClientRect(&rect);
		m_renWin->SetSize(rect.right-rect.left,rect.bottom-rect.top);
		
		// CKim - Setup Camera. Camera is located at 'position' looking at 'focal point'
		// Then set up the up direction
		m_Renderer->ResetCamera();		vtkCamera* pCam = m_Renderer->GetActiveCamera();
		pCam->SetPosition(0,0,-80);		pCam->SetFocalPoint(0,0,80);		pCam->ComputeViewPlaneNormal();
		pCam->SetViewUp(1,0,0);			pCam->OrthogonalizeViewUp();
		
		pCam->SetClippingRange(-40,40);	pCam->ParallelProjectionOff();	pCam->SetViewAngle(60);
		//pCam->ParallelProjectionOn();	pCam->SetParallelScale(10);
		
		pCam->Zoom(10);
		
		// CKim - Setup objects
		InitObjects();
		InitWorkSpace();	
	//	AddPointer();
	}
	
	// CKim - Rendering only starts when 'render()' is called. Render() should be called through
	// renderwindow or interactor class, not straight from rendere
	m_iren->Render();
}

void ChunVtkDlg::OnSize(UINT nType, int cx, int cy)
{
	CDialog::OnSize(nType, cx, cy);

	// CKim - This function is called when the window is resized
	// Adjust the size of the renderwindow accordingly
	CRect rect;		this->GetClientRect(&rect);
	m_renWin->SetSize(rect.right-rect.left,rect.bottom-rect.top);
}

BOOL ChunVtkDlg::OnEraseBkgnd(CDC* pDC)
{
	// TODO: Add your message handler code here and/or call default
	//return CDialog::OnEraseBkgnd(pDC);
	return true;
}

void ChunVtkDlg::OnDestroy()
{
	CDialog::OnDestroy();
	
	// TODO: Add your message handler code here
	m_renWin->Finalize();
}

// ----------------------------------------------------------------- //

void ChunVtkDlg::MoveCursor(double x, double y, double z)
{
	m_pCursor->SetPosition(x,y,z);
}

void ChunVtkDlg::SetCursorSize(double rad)
{
	m_pCursor->SetScale(rad);
}

void ChunVtkDlg::SetCursorColor(const float r, const float g, const float b)
{
	m_pCursor->GetProperty()->SetColor(r,g,b);
}
// ----------------------------------------------------------------- //

// ----------------------------------------------------------------- //

void ChunVtkDlg::MoveCsys(double* tf)
{
	// CKim - 16 element array tf stores 4 by 4 transformation matrix columnwise. 
	// That is, tf[0,1,2,3] is first column of the tfMat
	vtkSmartPointer<vtkMatrix4x4> tfMat = vtkSmartPointer<vtkMatrix4x4>::New();  
	for(int i=0; i<4; i++)	{
		for(int j=0; j<4; j++)	{		tfMat->SetElement(i,j,tf[i+4*j]);	}	}

	// CKim - Orientation of the AxesActor can only be controlled by setting 'UserTransform'
	// which is set by vtkMatrix
	vtkSmartPointer<vtkTransform> Xfrm = vtkSmartPointer<vtkTransform>::New();
	Xfrm->SetMatrix(tfMat);		m_pCsys->SetUserTransform(Xfrm);

}

// ----------------------------------------------------------------- //
void ChunVtkDlg::ResetCam()
{
	this->m_Renderer->ResetCamera();
}

void ChunVtkDlg::InitObjects()
{
	vtkSmartPointer<vtkPolyDataMapper> pMapper;

	// ---------------------------------------- //
	// CKim - 1. Initialize Sphere. General workflow. 'Source' generates list of vertices 
	// representing the object, 'mapper' converts vertices to GL primitives, 
	// which is stored in 'actor' with other 'properties'
	vtkSmartPointer<vtkSphereSource> pSphere =  vtkSmartPointer<vtkSphereSource>::New();
	pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

	// CKim - Set Sphere
	pSphere->SetRadius(1.0);			pSphere->SetThetaResolution(18);
	pSphere->SetPhiResolution(18);		pSphere->LatLongTessellationOn();
	
	// Link the mapper
    pMapper->SetInputConnection(pSphere->GetOutputPort());

    // Link the actor. set properties. Add to renderer
	m_pCursor->SetMapper(pMapper);		m_Renderer->AddActor(m_pCursor);

	// ---------------------------------------- //
	// CKim - 2. Initialize coordinate system actor. Set axis properties
	m_pCsys->SetTotalLength(20,20,20);
	m_pCsys->GetXAxisShaftProperty()->SetColor(1,0,0);			m_pCsys->GetYAxisShaftProperty()->SetColor(0,1,0);
	m_pCsys->GetZAxisShaftProperty()->SetColor(0,0,1);			m_pCsys->SetAxisLabels(0);
	m_pCsys->SetConeRadius(0.2);

	// Add it to the renderer
	m_Renderer->AddActor(m_pCsys);

	// ---------------------------------------- //
	// CKim - 3. Initialize cylinder representing proxy
	vtkSmartPointer<vtkCylinderSource> pCylinder =  vtkSmartPointer<vtkCylinderSource>::New();
	pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

	// CKim - Set Cylinder
	pCylinder->SetRadius(0.5);			pCylinder->SetHeight(10.0);		pCylinder->SetResolution(9);		
	
	// Link the mapper
    pMapper->SetInputConnection(pCylinder->GetOutputPort());

    // Link the actor. set property and add it to the renderer
	m_pProxy->SetMapper(pMapper);		m_pProxy->GetProperty()->SetColor(0,0,1);
	m_Renderer->AddActor(m_pProxy);

}


void ChunVtkDlg::RenderProxy(const double* tgtTipPosDir)
{
	// CKim - Cylinder's position is defined by its center with initial direction aligned to Y axis
	// Draw cylinder based on position and direction by setting transformation matrix that
	// aligns and positions cylinder to given direction and location
	Eigen::Vector4d p, dir, y, w;		Eigen::Matrix4d R;	
	y(0) = y(2) = 0;	y(1) = 1;	double h = 10.0;
	vtkSmartPointer<vtkMatrix4x4> tfMat = vtkSmartPointer<vtkMatrix4x4>::New();		
	for(int i=0; i<3; i++)	{	p(i) = tgtTipPosDir[i];	dir(i) = tgtTipPosDir[i+3];	}

	// CKim - The location
	tfMat->SetElement(0,3,p(0));	tfMat->SetElement(1,3,p(1));	
	tfMat->SetElement(2,3,p(2));	tfMat->SetElement(3,3,1.0);
	
	// CKim - Define a rotation matrix that rotates Y axis (0,1,0) to given direction
	w = ChunKinematics::Cross(y,dir);	w(3) = 0;	double th = asin(w.norm());		w.normalize();
	R = ChunKinematics::Rodrigues(w,th);

	for(int i=0; i<3; i++)	{
		for(int j=0; j<3; j++)	{	tfMat->SetElement(i,j,R(i,j));	}	}
	tfMat->SetElement(3,0,0);		tfMat->SetElement(3,1,0);		tfMat->SetElement(3,2,0);

	// CKim - Orientation of the AxesActor can only be controlled by setting 'UserTransform'
	// which is set by vtkMatrix
	vtkSmartPointer<vtkTransform> Xfrm = vtkSmartPointer<vtkTransform>::New();
	Xfrm->SetMatrix(tfMat);		
	
	m_pProxy->SetUserTransform(Xfrm);

}

void ChunVtkDlg::RenderBalancedPair(const double* bpTipPosDir)
{
	vtkSmartPointer<vtkPoints> pPt = vtkPoints::New();
	vtkSmartPointer<vtkCellArray> pLineCell = vtkCellArray::New();
	vtkSmartPointer<vtkPolyData> pData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkTubeFilter> pTube = vtkSmartPointer<vtkTubeFilter>::New();
	vtkSmartPointer<vtkPolyDataMapper> pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

	// CKim - Create list of points for arc.
	Eigen::Vector3d a,b,v,p,x,y,z,pt;	
	a(0) = 0;	a(1) = 0;	a(2) = 0;
	v(0) = 0;	v(1) = 0;	v(2) = 1;

	for(int i=0; i<3; i++)	{	b(i) = bpTipPosDir[i];	}
	p = b-a;		double dist = p.norm();		p.normalize();
	y = v.cross(p);		y.normalize();		x = y.cross(v);	
	
	double th = 2.0*fabs(acos(p.dot(v)));		double r = dist/2.0/sin(th/2.0);
	
	int npt = 10;
	pPt->SetNumberOfPoints(npt+1);
	for(int i=0; i<=npt; i++)
	{
		double ith = i/10.0*th;
		pt = a + r*(1-cos(ith))*x + r*sin(ith)*v;
		pPt->SetPoint(i,pt(0),pt(1),pt(2));
	}

	// CKim - Set the cell array describing the connection of the points in the line
	// 1. Insert a cell to cell array. The cell will contain index of npt points
	// 2. Insert npt indexes to the cell. This will define how the points are connected to form a line
	pLineCell->InsertNextCell(npt+1);	
	for(int i=0; i<=npt; i++)	{	pLineCell->InsertCellPoint(i);	}

	// CKim - Create polydata using the points and cell array
	pData->SetPoints(pPt);
	pData->SetLines(pLineCell);
 
	// CKim - Tube Filter creates tube for given line
	pTube->SetRadius(1.3);
	pTube->SetNumberOfSides(20);
	pTube->SetInput(pData);

	// CKim - Usual mapper and actor for rendering 
	pMapper->SetInputConnection(pTube->GetOutputPort());
	
	m_pBalancePair->SetMapper(pMapper);
	m_pBalancePair->GetProperty()->SetColor(1,1,0);
	m_Renderer->AddActor(m_pBalancePair);
}


void ChunVtkDlg::RenderTip(const double* bpTipPosDir, const double* currTipPosDir)
{
	vtkSmartPointer<vtkPoints> pPt = vtkPoints::New();
	vtkSmartPointer<vtkCellArray> pLineCell = vtkCellArray::New();
	vtkSmartPointer<vtkPolyData> pData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkTubeFilter> pTube = vtkSmartPointer<vtkTubeFilter>::New();
	vtkSmartPointer<vtkPolyDataMapper> pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

	// CKim - Create list of points for arc.
	Eigen::Vector3d a,b,v,p,x,y,z,pt;	

	//for(int i=0; i<3; i++)	{	a(i) = currTipPosDir[i];	b(i) = bpTipPosDir[i];	v(i) = -currTipPosDir[i+3];	}
	for(int i=0; i<3; i++)	{	a(i) = bpTipPosDir[i];	b(i) = currTipPosDir[i];	v(i) = bpTipPosDir[i+3];	}
	p = b-a;		double dist = p.norm();		p.normalize();
	y = v.cross(p);		y.normalize();		x = y.cross(v);	
	
	double th = 2.0*fabs(acos(p.dot(v)));		double r = dist/2.0/sin(th/2.0);
	
	int npt = 10;
	pPt->SetNumberOfPoints(npt+1);
	for(int i=0; i<=npt; i++)
	{
		double ith = i/10.0*th;
		pt = a + r*(1-cos(ith))*x + r*sin(ith)*v;
		pPt->SetPoint(i,pt(0),pt(1),pt(2));
	}

	// CKim - Set the cell array describing the connection of the points in the line
	// 1. Insert a cell to cell array. The cell will contain index of npt points
	// 2. Insert npt indexes to the cell. This will define how the points are connected to form a line
	pLineCell->InsertNextCell(npt+1);	
	for(int i=0; i<=npt; i++)	{	pLineCell->InsertCellPoint(i);	}

	// CKim - Create polydata using the points and cell array
	pData->SetPoints(pPt);
	pData->SetLines(pLineCell);
 
	// CKim - Tube Filter creates tube for given line
	pTube->SetRadius(0.7);
	pTube->SetNumberOfSides(20);
	pTube->SetInput(pData);

	// CKim - Usual mapper and actor for rendering 
	pMapper->SetInputConnection(pTube->GetOutputPort());
	
	m_pThirdTube->SetMapper(pMapper);
	m_pThirdTube->GetProperty()->SetColor(0,1,1);
	m_Renderer->AddActor(m_pThirdTube);
}

void ChunVtkDlg::InitWorkSpace()
{
	m_pWkSpcInside = vtkSmartPointer<vtkActor>::New();
	m_pWkSpcOutside = vtkSmartPointer<vtkActor>::New();

	// CKim - Polydata and mapper
	vtkSmartPointer<vtkPoints> pPt;			vtkSmartPointer<vtkCellArray> pCell;
	vtkSmartPointer<vtkPolyData> pData;		vtkSmartPointer<vtkPolyDataMapper> pMapper;

	for(int nnn = 0; nnn<2; nnn++)
	{
		// CKim - Init	
		pPt = vtkSmartPointer<vtkPoints>::New();		pCell = vtkSmartPointer<vtkCellArray>::New();
		pData = vtkSmartPointer<vtkPolyData>::New();	pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		
		// CKim - Read file, create point list
		ifstream ifstr;		double r;	double z;	double th;	int cnt = 0;	int nSec = 0;	int nPt = 360/6;
		if(nnn==0)
		{
			ifstr.open("C:\\01. ConcentricTubeRobots\\CTR\\wkspc_outer.txt");
		}
		else
		{
			ifstr.open("C:\\01. ConcentricTubeRobots\\CTR\\wkspc_inner.txt");
		}

		while(1)
		{
			ifstr>>r;	ifstr>>z;
			if(ifstr.eof())	{	break;	}
			else			
			{
				for(int i=-180; i<180; i+=6)
				{
					th = i*3.141592/180.0;
					pPt->InsertPoint(cnt,r*cos(th), r*sin(th),z);
					cnt++;
				}
				nSec++;
			}
		}
		ifstr.close();

		// CKim - Create triangle strip cell. It is list of indexes [id0 id1 id2 id3...] 
		// From the list, triangles are defined by 'id_012' 'id_123' ... 
		for(int j=0; j<nPt; j++)
		{
			pCell->InsertNextCell(nSec*2);	int a,b;
		
			for(int i=0; i<nSec; i++)
			{
				a = nPt*i+j;	b = nPt*i+((j+1)%nPt);
				pCell->InsertCellPoint(a);
				pCell->InsertCellPoint(b);
			}
		}	

		// CKim - Create polydata using the points and cell array
		pData->SetPoints(pPt);		pData->SetStrips(pCell);	

		// CKim - Mapper and then renderer
		pMapper->SetInput(pData);
		if(nnn==0)
		{
			m_pWkSpcOutside->SetMapper(pMapper);			m_pWkSpcOutside->GetProperty()->SetColor(0.7,0.7,0.7);
			m_pWkSpcOutside->GetProperty()->SetOpacity(0.2);	m_Renderer->AddActor(m_pWkSpcOutside);
		}
		else
		{
			m_pWkSpcInside->SetMapper(pMapper);				m_pWkSpcInside->GetProperty()->SetColor(0.7,0.7,0.7);
			m_pWkSpcInside->GetProperty()->SetOpacity(0.2);	m_Renderer->AddActor(m_pWkSpcInside);
		}
	}

}
