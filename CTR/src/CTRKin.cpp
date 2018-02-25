#include "StdAfx.h"
#include "CTRKin.h"
#include "ChunTimer.h"
#include "Utilities.h"
#include <math.h>

CTRKin::CTRKin(void)
{
	std::string fName = "";

	// CKim - Coefficient file for Tip
	//fName = "CTR_TIP_FAC.txt";
	//fName = "model_FT_2017_3_23_12_44_48_3.txt";
	fName = "Fourier_newrobot_order3.txt";
	//fName = "Fourier_oldrobot_order3.txt";
	//fName = "Fourier_oldrobotrepaired_order3.txt";
	//fName = "Fourier_3rd robot_order3.txt";

	os.open("conditionNumber.txt");
	if (readCTR_FAC_file(fName, m_Tip_px, m_Tip_py, m_Tip_pz, m_Tip_ox, m_Tip_oy, m_Tip_oz) == false) //file read error
	{
		AfxMessageBox("Sparta!!!!");
	}

	// CKim - Initialize local variable used in adaptive update.
	m_tmpMat.resize(125,6);
	for(int i=0; i<125; i++)	
	{
		m_tmpMat(i,0) = m_Tip_px[i];		m_tmpMat(i,1) = m_Tip_py[i];		m_tmpMat(i,2) = m_Tip_pz[i];
		m_tmpMat(i,3) = m_Tip_ox[i];		m_tmpMat(i,4) = m_Tip_oy[i];		m_tmpMat(i,5) = m_Tip_oz[i];
	}

	// CKim  Coefficients for Balance Pair
	fName = "C:\\01. ConcentricTubeRobots\\CTR\\CTR_BP_FAC.txt";
	
	//if (readCTR_FAC_file(fName, m_BP_px, m_BP_py, m_BP_pz, m_BP_ox, m_BP_oy, m_BP_oz) == false) //file read error
	//{
	//	AfxMessageBox("Sparta!!!!");
	//}

	// CKim - Initialize matrices for recursive least square
	for(int i=0; i<6; i++)	{
		F[i].resize(125,125);	F[i].setIdentity();		F[i] = 0.1*F[i];		}
	m_forceGain = 0.0;
	Fzero.resize(125,125);

	SetInvKinThreshold(0.1,3.0);
	m_forgettingFactor = 1.0;

	m_hFACMutex = CreateMutex(NULL,false,"FAC_Mutex");

}


CTRKin::~CTRKin(void)
{
}


void CTRKin::ReInitializeEstimator()
{
	// CKim - Initialize matrices for recursive least square from file. 
	double val;
	std::ifstream ifstr;		ifstr.open("Init_M_for_Estimator.txt");
	//if(ifstr.fail())		{	return false;	}
	
	for(int i=0; i<125; i++)	{
		for(int j=0; j<125; j++)	{	ifstr >> val;
			for(int k=0; k<6; k++)		{	F[k](i,j) = val;	}	}	}


	//// CKim - Initialize matrices for recursive least square
	//for(int i=0; i<6; i++)	{
	//	F[i].resize(125,125);	F[i].setIdentity();		F[i] = 0.1*F[i];		}

	//Fzero.resize(125,125);		Fzero.setZero();
}


void CTRKin::ReInitializeModel()
{
	std::string fName = "";

	// CKim - Coefficient file for Tip
	//fName = "C:\\01. ConcentricTubeRobots\\CTR\\CTR_TIP_FAC.txt";
	fName = "FK_new_parameters.txt";

	if (readCTR_FAC_file(fName, m_Tip_px, m_Tip_py, m_Tip_pz, m_Tip_ox, m_Tip_oy, m_Tip_oz) == false) //file read error
	{
		AfxMessageBox("Sparta!!!!");
	}

	// CKim - Initialize local variable used in adaptive update.
	m_tmpMat.resize(125,6);
	for(int i=0; i<125; i++)	
	{
		m_tmpMat(i,0) = m_Tip_px[i];		m_tmpMat(i,1) = m_Tip_py[i];		m_tmpMat(i,2) = m_Tip_pz[i];
		m_tmpMat(i,3) = m_Tip_ox[i];		m_tmpMat(i,4) = m_Tip_oy[i];		m_tmpMat(i,5) = m_Tip_oz[i];
	}


	// CKim  Coefficients for Balance Pair
	fName = "C:\\01. ConcentricTubeRobots\\CTR\\CTR_BP_FAC.txt";
	
	if (readCTR_FAC_file(fName, m_BP_px, m_BP_py, m_BP_pz, m_BP_ox, m_BP_oy, m_BP_oz) == false) //file read error
	{
		AfxMessageBox("Sparta!!!!");
	}
}


bool CTRKin::readCTR_FAC_file(std::string fileName,  double px[125], double py[125],  double pz[125],  double ox[125],  double oy[125],  double oz[125])
{
	std::string junkS;	
	std::ifstream CTR_FAC_id;	CTR_FAC_id.open(fileName);
	if(CTR_FAC_id.fail())		{	return false;	}
	
	for(int i = 0; i <125; i++)
	{
		CTR_FAC_id >> junkS >> junkS >> junkS >> junkS;
		CTR_FAC_id >> px[i] >>  junkS >> py[i] >> junkS >> pz[i] >> junkS >> ox[i] >> junkS >> oy[i] >> junkS >> oz[i];
		CTR_FAC_id >> junkS;
	}

	//// CKim - Random numbers....
	//for(int i = 0; i <125; i++)
	//{
	//	Eigen::MatrixXd M = Eigen::MatrixXd::Random(6,1);
	//	px[i] = M(0,0);		py[i] = M(1,0);		pz[i] = M(2,0);	
	//	//ox[i] = M(3,0);		oy[i] = M(4,0);		oz[i] = M(5,0);

	//	//CTR_FAC_id >> junkS >> junkS >> junkS >> junkS;
	//	//CTR_FAC_id >> px[i] >>  junkS >> py[i] >> junkS >> pz[i] >> junkS >> ox[i] >> junkS >> oy[i] >> junkS >> oz[i];
	//	//CTR_FAC_id >> junkS;
	//}

	CTR_FAC_id.close();
	return true;
}


void CTRKin::GetFAC(Eigen::MatrixXd& Coeff)
{
	WaitForSingleObject(m_hFACMutex,INFINITE);
	for(int i=0; i<6; i++)
	{
		if(i==0)	{
			for(int j=0; j<125; j++)	{	Coeff(j,i) = m_Tip_px[j];	}	}
		if(i==1)	{
			for(int j=0; j<125; j++)	{	Coeff(j,i) = m_Tip_py[j];	}	}
		if(i==2)	{
			for(int j=0; j<125; j++)	{	Coeff(j,i) = m_Tip_pz[j];	}	}
		if(i==3)	{
			for(int j=0; j<125; j++)	{	Coeff(j,i) = m_Tip_ox[j];	}	}
		if(i==4)	{
			for(int j=0; j<125; j++)	{	Coeff(j,i) = m_Tip_oy[j];	}	}
		if(i==5)	{
			for(int j=0; j<125; j++)	{	Coeff(j,i) = m_Tip_oz[j];	}	}
	}
	ReleaseMutex(m_hFACMutex);
}


bool CTRKin::TipFwdKin(const double* jAng, double* posOrt)
{
	// CKim - Evaluate functional approximation at jAng. a21, a31 relative rotation of tube 2 and 3
	// w.r.t. tube 1. L31 is relative protrusion of the tube w.r.t. balanced pair
	double a21 = jAng[0];		double a31 = jAng[1];		double L31 = jAng[2];

	// 1. First evaluate harmonic basis function at joint angle. A = [1, cos(a21), sin(a21), cos(2*a21), sin(2*a21) .... ]
	// The length of protrusion is normalized so that its range falls into [0,pi/2]. L31_max = 80.0;
	double A[5], B[5], C[5];	A[0] = B[0] = C[0] = 1.0;
	double L_normalized;		int n,r;	

	for(int i=1; i<5; i++)	
	{
		//n = i/2;	r = i%2;	L_normalized = L31/80.0*0.5*3.141592;
		n = i/2;	r = i%2;	L_normalized = L31/L31_MAX*0.5*3.141592;
		if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
		else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
	}

	WaitForSingleObject(m_hFACMutex,INFINITE);

	// 2. Multiply coefficients
	double p[3] = { 0, 0, 0 };		double v[3] = { 0, 0, 0 };		double val = 0;
	for(int i=0; i<5; i++)	{
		for(int j=0; j<5; j++)	{
			for(int k=0; k<5; k++)	
			{	
				val = A[i]*B[j]*C[k];
				p[0] += (val*m_Tip_px[25*i+5*j+k]);		p[1] += (val*m_Tip_py[25*i+5*j+k]);		p[2] += (val*m_Tip_pz[25*i+5*j+k]);	
				v[0] += (val*m_Tip_ox[25*i+5*j+k]);		v[1] += (val*m_Tip_oy[25*i+5*j+k]);		v[2] += (val*m_Tip_oz[25*i+5*j+k]);
			}
		}
	}

	ReleaseMutex(m_hFACMutex);


	val = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= val;		v[1] /= val;		v[2] /= val;

	// 3. Transform the result by rotation about z axis by a1 and translation along z axis by L1
	double a1 = jAng[3];		double L1 = jAng[4];
	
	posOrt[0] = cos(a1)*p[0] - sin(a1)*p[1];		posOrt[3] = cos(a1)*v[0] - sin(a1)*v[1];
	posOrt[1] = sin(a1)*p[0] + cos(a1)*p[1];		posOrt[4] = sin(a1)*v[0] + cos(a1)*v[1];
	posOrt[2] = p[2] + L1;							posOrt[5] = v[2];

	for (int i = 0; i < 3; ++i)
	{
		posOrt[i] += 20 * posOrt[i+3];
	}
	return true;
}


void CTRKin::TipFwdKinEx(const double* jAng, const Eigen::MatrixXd& Coeff, double* posOrt)
{
	// CKim - Evaluate functional approximation at jAng. a21, a31 relative rotation of tube 2 and 3
	// w.r.t. tube 1. L31 is relative protrusion of the tube w.r.t. balanced pair
	double a21 = jAng[0];		double a31 = jAng[1];		double L31 = jAng[2];

	// 1. First evaluate harmonic basis function at joint angle. A = [1, cos(a21), sin(a21), cos(2*a21), sin(2*a21) .... ]
	// The length of protrusion is normalized so that its range falls into [0,pi/2]. L31_max = 80.0;
	double A[5], B[5], C[5];	A[0] = B[0] = C[0] = 1.0;
	double L_normalized;		int n,r;	

	for(int i=1; i<5; i++)	
	{
		n = i/2;	r = i%2;	L_normalized = L31/L31_MAX*0.5*3.141592;
		if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
		else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
	}


    // 2. Multiply coefficients
	double p[3] = { 0, 0, 0 };		double v[3] = { 0, 0, 0 };		double val = 0;
	for(int i=0; i<5; i++)	{
		for(int j=0; j<5; j++)	{
			for(int k=0; k<5; k++)	
			{	
				val = A[i]*B[j]*C[k];
				p[0] += (val*Coeff(25*i+5*j+k,0));		p[1] += (val*Coeff(25*i+5*j+k,1));	
				p[2] += (val*Coeff(25*i+5*j+k,2));		v[0] += (val*Coeff(25*i+5*j+k,3));
				v[1] += (val*Coeff(25*i+5*j+k,4));		v[2] += (val*Coeff(25*i+5*j+k,5));
			}
		}
	}

	val = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= val;		v[1] /= val;		v[2] /= val;

	// 3. Transform the result by rotation about z axis by a1 and translation along z axis by L1
	double a1 = jAng[3];		double L1 = jAng[4];
	
	posOrt[0] = cos(a1)*p[0] - sin(a1)*p[1];		posOrt[3] = cos(a1)*v[0] - sin(a1)*v[1];
	posOrt[1] = sin(a1)*p[0] + cos(a1)*p[1];		posOrt[4] = sin(a1)*v[0] + cos(a1)*v[1];
	posOrt[2] = p[2] + L1;							posOrt[5] = v[2];

	//for (int i = 0; i < 3; ++i)
	//{
	//	posOrt[i] += 20 * posOrt[i+3];
	//}
}


bool CTRKin::BalancedPairFwdKin(const double* jAng, double* posOrt)
{
	// CKim - Evaluate functional approximation at jAng. a21, a31 relative rotation of tube 2 and 3
	// w.r.t. tube 1. L31 is relative protrusion of the tube w.r.t. balanced pair
	double a21 = jAng[0];		double a31 = jAng[1];		double L31 = jAng[2];

	// 1. First evaluate harmonic basis function at joint angle. A = [1, cos(a21), sin(a21), cos(2*a21), sin(2*a21) .... ]
	// The length of protrusion is normalized so that its range falls into [0,pi/2]. L31_max = 80.0;
	double A[5], B[5], C[5];	A[0] = B[0] = C[0] = 1.0;
	double L_normalized;		int n,r;	

	for(int i=1; i<5; i++)	
	{
		//n = i/2;	r = i%2;	L_normalized = L31/80.0*0.5*3.141592;
		n = i/2;	r = i%2;	L_normalized = L31/L31_MAX*0.5*3.141592;
		if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
		else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
	}

	// 2. Multiply coefficients
	double p[3] = { 0, 0, 0 };		double v[3] = { 0, 0, 0 };		double val = 0;
	for(int i=0; i<5; i++)	{
		for(int j=0; j<5; j++)	{
			for(int k=0; k<5; k++)	
			{	
				val = A[i]*B[j]*C[k];
				p[0] += (val*m_BP_px[25*i+5*j+k]);		p[1] += (val*m_BP_py[25*i+5*j+k]);		p[2] += (val*m_BP_pz[25*i+5*j+k]);	
				v[0] += (val*m_BP_ox[25*i+5*j+k]);		v[1] += (val*m_BP_oy[25*i+5*j+k]);		v[2] += (val*m_BP_oz[25*i+5*j+k]);
			}
		}
	}

	val = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= val;		v[1] /= val;		v[2] /= val;

	// 3. Transform the result by rotation about z axis by a1 and translation along z axis by L1
	double a1 = jAng[3];		double L1 = jAng[4];
	
	posOrt[0] = cos(a1)*p[0] - sin(a1)*p[1];		posOrt[3] = cos(a1)*v[0] - sin(a1)*v[1];
	posOrt[1] = sin(a1)*p[0] + cos(a1)*p[1];		posOrt[4] = sin(a1)*v[0] + cos(a1)*v[1];
	posOrt[2] = p[2] + L1;							posOrt[5] = v[2];
		
	return true;
}


void CTRKin::SetInvKinThreshold(const double MaxPosErr, const double MaxOrtErr)
{
	m_MaxPosErr = MaxPosErr;		m_MaxOrtErr = MaxOrtErr;	// in degree
	m_Thresh = m_MaxPosErr;
}


void CTRKin::GetInvKinThreshold(double& MaxPosErr, double& MaxOrtErr)
{
	MaxPosErr = m_MaxPosErr;		MaxOrtErr = m_MaxOrtErr;	// in degreeMax
}


void CTRKin::InverseKinematicsRootFinding(const double* tgtPosOrt, const double* init, double* jAng, double& sqErr, int& exitCond)
{
	// CKim - Solve Inverse kinematics by 'root finding' 
	// Tip position '[x,y,z]' and direction 'n', are function of joint angle 'jAng'
	// given the desired tip position '[xd,yd,zd]' and direction 'nd' and maximum position error 'pmax' and angle error 'thmax', 
	// inverse kinematics is solved by finding the root of the equation 
	// f(jAng) = [ x-xd, y-yd, z-zd, pmax/sin(thmax)*cross(n,nd) ] = 0
	// exitCond 1: lsqErr below threshold, 2: update magnitude too small, 3: Not decreasing, 4: max iteration

	// CKim - Parameters for root finding
	int iter = 0;	int maxiter = 100;		double eps = m_Thresh;		double lambda = 0.001;		double stepSz = 1.0;	
		
	Eigen::Matrix<double,4,5> J;		Eigen::Matrix<double,4,1> fx;		Eigen::Matrix<double,4,1> fxnew;		
	Eigen::Matrix<double,5,1> update;	Eigen::Matrix<double,5,1> b;		Eigen::Matrix<double,5,5> JtJ;		
	Eigen::Matrix<double,5,5> A;		double temp[5];

	// CKim - Load current model, initialize
	Eigen::MatrixXd Coeff(125,6);		GetFAC(Coeff);
	for(int i=0; i<5; i++)	{	jAng[i] = init[i];		}
	lambda = 0.001;		exitCond = 0;

	// CKim - Root finding iterations 
	for(iter = 0; iter < maxiter; iter++)
	{
		// CKim - Evaluate function at curent joint angle
		EvalF_RootFinding(jAng,tgtPosOrt,Coeff,fx);
		sqErr = fx.norm();

		// CKim - Exit if distance is less than threshold
		if(sqErr < eps)	{		exitCond = 1;	break;		}
		
		// CKim - Numerically evalaluate the jacobian dy/dx
		EvalJ_RootFinding(jAng,tgtPosOrt,Coeff,J);
		
		// CKim - In Newton-Rhapson method of root finding, update direction update = -Jinv*fx
		// in our case m != n so it is pseudo inverse, so solve JtJ update = -Jt * fx
		// to avoid singularity, you may also damp JtJ by adding lamda * identity
		b = -1.0 * J.transpose()*fx;
		JtJ = J.transpose()*J;		A = JtJ;
		for(int i=0; i<5; i++)	{	A(i,i) += lambda*JtJ(i,i);	}
		//for(int i=0; i<5; i++)	{	A(i,i) += lambda;	}

		// CKim - Calculate Update step. Use JacobiSVD.solve to get least square solution
		Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> Jsvd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);
		update = Jsvd.solve(b);

		// CKim - If the magnitude of the update direction is small, exit
		if(update.norm() < 0.001)	{	exitCond = 2;	break;		}

		// CKim - For convergence to root, it is very important to decide how much we move in update direction
		// Decide how much in the update direction we should move. Full (1) to minimum (0.1)
		// this depends on if |error|^2 is decreasing
		stepSz = 1.0;			
		while(1)
		{
			// CKim - Move by step size. starting from stepSize = 1.0;
			for(int i=0; i<5; i++)	{	temp[i] = jAng[i] + stepSz*update(i,0);	}
		
			// CKim - Evalualte function at new updated point
			EvalF_RootFinding(temp,tgtPosOrt,Coeff,fxnew);

			// CKim - If the norm^2 of the function decreases, accept the step
			if(fxnew.squaredNorm() < fx.squaredNorm())
			{
				for(int i=0; i<5; i++)	{	jAng[i] = temp[i];	}
				break;
			}
			else
			{
				stepSz *= 0.5;	
			}
			
			// CKim - If function is not decreasing, 
			if(stepSz < 0.0005)	
			{
				exitCond = 3;
				break;	
			}
		}

		if(exitCond == 3)	{		break;			}
	}
	if(iter == maxiter)	{		exitCond = 4;		}
}


void CTRKin::EvalF_RootFinding(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F)
{
	// CKim - Function whose root is sought for inverse kinematics solution. 
	// f(jAng) = [ x-xd, y-yd, z-zd, pmax/sin(thmax)*cross(n,nd) ] = 0
	// Tip position '[x,y,z]' and direction 'n', are function of joint angle 'jAng'
	// given the desired tip position '[xd,yd,zd]' and direction 'nd' and maximum position error 'pmax' and angle error 'thmax', 

	double posOrt[6];	double xprod[3];	double pmax = m_MaxPosErr;		double thmax = m_MaxOrtErr*3.141592/180.0;		double sum = 0.0;
	
	// CKim - Evaluate function
	TipFwdKinEx(jAng,Coeff,posOrt);
	xprod[0] = posOrt[4]*tgtPosOrt[5] - posOrt[5]*tgtPosOrt[4];
	xprod[1] = posOrt[5]*tgtPosOrt[3] - posOrt[3]*tgtPosOrt[5];
	xprod[2] = posOrt[3]*tgtPosOrt[4] - posOrt[4]*tgtPosOrt[3];
	for(int i=0; i<3; i++)	{	
		F(i,0) = posOrt[i] - tgtPosOrt[i];				
		sum += (xprod[i]*xprod[i]);
	}
	F(3,0) = pmax/sin(thmax)*sqrt(sum);
}


void CTRKin::EvalJ_RootFinding(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,5>& J)
{
	// CKim - Numerically evalaluate the jacobian dy/dx
	Eigen::Matrix<double,4,1> Fxo;		Eigen::Matrix<double,4,1> Fx;
	
	double x[5];	double dx = FLT_EPSILON;

	// CKim - First evaluate at current location
	EvalF_RootFinding(jAng,tgtPosOrt,Coeff,Fxo);

	// CKim - Perturb each jAng
	for(int col=0; col<5; col++)
	{
		for(int i=0; i<5; i++)
		{
			if(i==col)
			{
				dx = FLT_EPSILON*fabs(jAng[i]);
				if(dx==0.0) {	dx = FLT_EPSILON;	}
				
				// CKim - This code is said to reduce finite-precision error
				x[i] = jAng[i] + dx;
				dx = x[i] - jAng[i];
			}
			else		{	x[i] = jAng[i];		}
		}
		
		EvalF_RootFinding(x,tgtPosOrt,Coeff,Fx);

		for(int i=0; i<4; i++)	{	J(i,col) = (Fx(i,0) - Fxo(i,0))/dx;	}
	}
}


void CTRKin::InverseKinematicsLSQ(const double* tgtPosOrt, const double* init, double* jAng, double* Err, int& exitCond)
{
	// CKim - Solve Inverse kinematics by 'non-linear least square' 
	// Tip position '[x,y,z]' and direction 'n', are function of joint angle 'jAng'
	// given the desired tip position '[xd,yd,zd]' and direction 'nd' and maximum position error 'pmax' and angle error 'thmax', 
	// inverse kinematics is solved by minimizing the objective function 
	// f(jAng) = (x-xd)^2 + (y-yd)^2 + (z-zd)^2 + pmax/(1-cos(thmax))*(1 - dot(n,nd))^2
	// by using Lenvenberg-Marquardt algorithm which is the variation of the gauss-Newton algorithm for optimization. 
	// exitCond 1: lsqErr below threshold, 2: update magnitude too small, 3: max iteration

	int iter = 0;	int maxiter = 100;	double eps = m_Thresh;		double lambda = 0.001;		double stepSz = 1.0;
	
	Eigen::Matrix<double,4,5> J;		Eigen::Matrix<double,4,1> fx;		Eigen::Matrix<double,4,1> fxnew;		
	Eigen::Matrix<double,5,1> update;	Eigen::Matrix<double,5,1> b;		Eigen::Matrix<double,5,5> JtJ;
	Eigen::Matrix<double,5,5> A;		double temp[5];						
	
	// CKim - Initialize
	Eigen::MatrixXd Coeff(125,6);		GetFAC(Coeff);
	for(int i=0; i<5; i++)	{	jAng[i] = init[i];		}
	lambda = 0.001;		exitCond = 0;
	::Eigen::Matrix<double, 3, 5> Jp;
	::Eigen::Matrix<double, 1, 5> Jo;
	::Eigen::Matrix<double, 3, 3> tmpMat;
	::Eigen::Matrix<double, 5, 5> IdMat;
	IdMat.setIdentity();

	// CKim - Iterate
	for(iter = 0; iter < maxiter; iter++)
	{
		// CKim - Evaluate function at curent joint angle
		EvalF_LSQ(jAng,tgtPosOrt,Coeff,fx);		
		Err[0] = fx.norm(); 
		Err[1] = sqrt( fx(0,0)*fx(0,0) + fx(1,0)*fx(1,0) + fx(2,0)*fx(2,0) );
		Err[2] = acos(1.0 - fx(3,0)*(1-cos(m_MaxOrtErr*3.141592/180.0))/m_MaxPosErr)*180.0/3.141592;
		
		if(Err[0] < eps)	{	exitCond = 1;	break;		}
	
		EvalJ_LSQ(jAng,tgtPosOrt,Coeff,J);
		
		Jp = J.block(0,0, 3, 5);
		Jp.col(0) *= M_PI / 180.0;
		Jp.col(1) *= M_PI / 180.0;
		Jp.col(3) *= M_PI / 180.0;
		Jo = J.row(3);

		tmpMat = (Jp * Jp.transpose());
		for (int i = 0; i < 3; ++i)
			tmpMat(i, i) += 0.0001;
		J.col(0) *= M_PI / 180.0;
		J.col(1) *= M_PI / 180.0;
		J.col(3) *= M_PI / 180.0;
		b = -1.0 * J.transpose()*fx;
		//b = -Jp.transpose() * fx.segment(0, 3) - (IdMat - Jp.transpose() * tmpMat.inverse() * Jp) * Jo.transpose() * fx[3];
	/*	b[2] *= 100;
		b[4] *= 100;*/

		// CKim - Calculate Update step. Use JacobiSVD.solve to get least square solution
		//Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> Jsvd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);
		//update = Jsvd.solve(b);
		update = 0.001*b;
	
		// CKim - If the magnitude of the update direction is small, exit
		if(update.norm() < 0.001)	{	exitCond = 2;	break;		}

		// CKim - Decide how much in the update direction we should move. Full (1) to minimum (0.1)
		// this depends on if |error|^2 is decreasing
		//stepSz = 0.2;
		stepSz = 1.0;
		//for(int i=0; i<5; i++)	{	jAng[i] = jAng[i] + stepSz*update(i,0);	}
		//EvalF_LSQ(jAng,tgtPosOrt,Coeff,fxnew);
		while(1)
		{
			for(int i=0; i<5; i++)	{	temp[i] = jAng[i] + stepSz*update(i,0);	}
		
			// CKim - Evalualte distance at new updated point
			EvalF_LSQ(temp,tgtPosOrt,Coeff,fxnew);	

			// CKim - If the norm^2 of the function decreases, accept the step
			if(fxnew.squaredNorm() < fx.squaredNorm())
			{
				for(int i=0; i<5; i++)	{	jAng[i] = temp[i];	}
				break;
			}
			else
				stepSz *= 0.5;	
			
			// CKim - If function is not decreasing, 
			if(stepSz < 0.005)	
				break;
		}
	}
	//::std::cout << fxnew[0] << " " <<  fxnew[1] << " " << fxnew[2] << " " << fxnew[3] << ::std::endl;

	if(iter == maxiter)	{		exitCond = 3;		}

	//::std::cout << "iter = " << iter << ::std::endl;
}


void CTRKin::EvalF_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F)
{
	// CKim - Function whose root is sought for inverse kinematics solution. 
	// f(jAng) = [ x-xd, y-yd, z-zd, pmax/sin(thmax)*cross(n,nd) ] = 0
	// Tip position '[x,y,z]' and direction 'n', are function of joint angle 'jAng'
	// given the desired tip position '[xd,yd,zd]' and direction 'nd' and maximum position error 'pmax' and angle error 'thmax', 

	double posOrt[6];	double pmax = m_MaxPosErr;		double thmax = m_MaxOrtErr*3.141592/180.0;		double sum = 0.0;
	
	// CKim - Evaluate function
	TipFwdKinEx(jAng,Coeff,posOrt);
	for(int i=0; i<3; i++)	{	
		F(i,0) = posOrt[i] - tgtPosOrt[i];				
		sum += (posOrt[i+3]*tgtPosOrt[i+3]);
	}
	F(3,0) = 0.3 * pmax/(1-cos(thmax))*(1-sum);
	//F(3,0) = 0;
}


void CTRKin::EvalJ_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,5>& J)
{
	// CKim - Numerically evalaluate the jacobian dy/dx
	Eigen::Matrix<double,4,1> Fxo;		Eigen::Matrix<double,4,1> Fx;
	
	double x[5];	double dx = FLT_EPSILON;

	// CKim - First evaluate at current location
	EvalF_LSQ(jAng,tgtPosOrt,Coeff,Fxo);

	// CKim - Perturb each jAng
	for(int col=0; col<5; col++)
	{
		for(int i=0; i<5; i++)
		{
			if(i==col)
			{
				dx = FLT_EPSILON*fabs(jAng[i]);
				if(dx==0.0) {	dx = FLT_EPSILON;	}
				
				// CKim - This code is said to reduce finite-precision error
				x[i] = jAng[i] + dx;
				dx = x[i] - jAng[i];
			}
			else		{	x[i] = jAng[i];		}
		}
		
		EvalF_LSQ(x,tgtPosOrt,Coeff,Fx);

		for(int i=0; i<4; i++)	{	J(i,col) = (Fx(i,0) - Fxo(i,0))/dx;	}
	}
}


void CTRKin::UpdateFAC(const double jAng[5], const double measTipPosDir[6], double predTipPosDir[6], bool doUpdate)
{
	// CKim - Perform recursive least square update on coefficients....
	// 0. Input vector [ 1, cos(L31), sin(L31) ..... sin(a21)*sin(a31)*sin(L31) ];
	Eigen::Matrix<double,125,1> x;		double a21 = jAng[0];		double a31 = jAng[1];		double L31 = jAng[2];

	double A[5], B[5], C[5];	A[0] = B[0] = C[0] = 1.0;
	double L_normalized;		int n,r;	

	for(int i=1; i<5; i++)	
	{
		n = i/2;	r = i%2;	L_normalized = L31/L31_MAX*0.5*3.141592;
		if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
		else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
	}

	int cnt = 0;
	for(int i=0; i<5; i++)	{
		for(int j=0; j<5; j++)	{
			for(int k=0; k<5; k++)	{	x(cnt,0) = A[i]*B[j]*C[k];	cnt++;	}	}	}

	
	// 0.5. In measuredData, compensate for the rigid rotation/translation about/along z axis
	double a1 = jAng[3];		double L1 = jAng[4];	double meas[6];		double pred[6];		double err;
	
	meas[0] = cos(a1)*measTipPosDir[0] + sin(a1)*measTipPosDir[1];
	meas[1] = -sin(a1)*measTipPosDir[0] + cos(a1)*measTipPosDir[1];
	meas[2] = measTipPosDir[2] - L1;
	meas[3] = cos(a1)*measTipPosDir[3] + sin(a1)*measTipPosDir[4];
	meas[4] = -sin(a1)*measTipPosDir[3] + cos(a1)*measTipPosDir[4];
	meas[5] = measTipPosDir[5];

	// Actual Update
	m_forgettingFactor = 1.00;	//0.998;	//1.00;	//0.997;	//1.00;	//1.0;	//0.98;	//0.6;
	for(int i=0; i<6; i++)		
	{
		// 1. Calculate prediction error
		pred[i] = x.dot(m_tmpMat.col(i));
		err = meas[i] - pred[i];
		
		//if(doUpdate && (i < 3))		// i < 3 to update position only
		if(doUpdate && (i < 6))		// i < 6 to update both position and orientation
		{
			// 2. Update Matrix F
			Eigen::MatrixXd Fold = F[i];
			double tmp = x.transpose()*Fold*x;

			F[i] = (1/m_forgettingFactor)*(Fold - (1/(m_forgettingFactor+tmp)) * Fold*(x*x.transpose())*Fold);

			// 3. Update coefficients
			Eigen::VectorXd v = err*F[i]*x;
			m_tmpMat.col(i) += v;
		}

	}
	
	predTipPosDir[0] = cos(a1)*pred[0] - sin(a1)*pred[1];
	predTipPosDir[1] = sin(a1)*pred[0] + cos(a1)*pred[1];
	predTipPosDir[2] = pred[2] + L1;
	predTipPosDir[3] = cos(a1)*pred[3] - sin(a1)*pred[4];
	predTipPosDir[4] = sin(a1)*pred[3] + cos(a1)*pred[4];
	predTipPosDir[5] = pred[5];

	// CKim - Copy the results to the shared coefficients
	if(doUpdate)
	{
		WaitForSingleObject(m_hFACMutex,INFINITE);
		for(int i=0; i<6; i++)		// To update position and orientation
		//for(int i=0; i<3; i++)	// To update position only
		{
			if(i==0)	{
				for(int j=0; j<125; j++)	{	m_Tip_px[j] = m_tmpMat(j,i);	}	}
			if(i==1)	{
				for(int j=0; j<125; j++)	{	m_Tip_py[j] = m_tmpMat(j,i);	}	}
			if(i==2)	{
				for(int j=0; j<125; j++)	{	m_Tip_pz[j] = m_tmpMat(j,i);	}	}
			if(i==3)	{
				for(int j=0; j<125; j++)	{	m_Tip_ox[j] = m_tmpMat(j,i);	}	}
			if(i==4)	{
				for(int j=0; j<125; j++)	{	m_Tip_oy[j] = m_tmpMat(j,i);	}	}
			if(i==5)	{
				for(int j=0; j<125; j++)	{	m_Tip_oz[j] = m_tmpMat(j,i);	}	}
		}
		ReleaseMutex(m_hFACMutex);
	}

}


void CTRKin::UpdateInitM(const double jAng[5], bool inv)
{
	//Eigen::Matrix<double,125,1> x;		double a21 = jAng[0];		double a31 = jAng[1];		double L31 = jAng[2];

	//double A[5], B[5], C[5];	A[0] = B[0] = C[0] = 1.0;
	//double L_normalized;		int n,r;	

	//if(inv)
	//{
	//	for(int i=0; i<6; i++)	{	F[i] = Fzero.inverse();		}

	//	std::ofstream f;	f.open("Asdf.txt");
	//	for(int i=0; i<125; i++)	{  f<<"\n";
	//		for(int j=0; j<125; j++)	{
	//			f<<F[0](i,j)<<"\t";
	//		}	}

	//	return;
	//}

	//for(int i=1; i<5; i++)	
	//{
	//	n = i/2;	r = i%2;	L_normalized = L31/L31_MAX*0.5*3.141592;
	//	if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
	//	else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
	//}

	//int cnt = 0;
	//for(int i=0; i<5; i++)	{
	//	for(int j=0; j<5; j++)	{
	//		for(int k=0; k<5; k++)	{	x(cnt,0) = A[i]*B[j]*C[k];	cnt++;	}	}	}

	//// CKim - Initialize matrices for recursive least square
	//Fzero += (x*x.transpose());	

}


void CTRKin::EvalAnalyticJacobian(const double* jAng, const Eigen::MatrixXd& Coeff, Eigen::MatrixXd& J)
{
	// CKim - Evaluate the jacobian of the functional approximation at jAng. 
	// Jacobian will be 6 by 5 matrix, that is
	// J = [ dpx/da21, dpx/da31, dpx/dL31, dpx/da1, dpx/dL1 ;
	//       dpy/da21,		  ................		dpy/dL1	;
	//			:		      ................		   :	;
	//		 dvz/da21,        ................      dvz/dL1   ];
	
	double a21 = jAng[0];			double a31 = jAng[1];		double L31 = jAng[2];		
	double a1 = jAng[3];			double L1 = jAng[4];		double Ln = 0.5*3.141592/L31_MAX;

	// 1. First evaluate the partial derivate of the harmonic basis function at joint angle. 
	// For example, derivative w.r.t a21 will be dA = [0, -sin(a21), cos(a21), -2*sin(2*a21), 2*cos(2*a21) .... ]
	// The length of protrusion is normalized so that its range falls into [0,pi/2]. L31_max = 80.0;
	double A[5], B[5], C[5];	double L_normalized = L31*Ln;		int n,r;

	for(int col=0; col<5; col++)
	{
		if(col==0)	{	A[0] = 0.0;		B[0] = C[0] = 1.0;		}	 // derive w.r.t a21
		if(col==1)	{	B[0] = 0.0;		A[0] = C[0] = 1.0;		}	 // derive w.r.t a31
		if(col==2)	{	C[0] = 0.0;		A[0] = B[0] = 1.0;		}	 // derive w.r.t L31
	
		for(int i=1; i<5; i++)	
		{
			n = i/2;	r = i%2;	
			if(col==0)	// derive by a21
			{
				if(r==0)	{	A[i] = n*cos(n*a21);			B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
				else		{	A[i] = -(n+1)*sin((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
			}

			if(col==1)	// derive by a31
			{
				if(r==0)	{	A[i] = sin(n*a21);		B[i] = n*cos(n*a31);			C[i] = sin(n*L_normalized);		}
				else		{	A[i] = cos((n+1)*a21);	B[i] = -(n+1)*sin((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
			}

			if(col==2)	// derive by L31
			{
				if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = n*Ln*cos(n*L_normalized);			}
				else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = -(n+1)*Ln*sin((n+1)*L_normalized);	}
			}

			if(col > 2)	// derive by a1 and L1
			{
				if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
				else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
			}
		}

		// 2. Multiply coefficients
		double dp[3] = { 0, 0, 0 };		double dv[3] = { 0, 0, 0 };		double val = 0;
		for(int i=0; i<5; i++)	{
			for(int j=0; j<5; j++)	{
				for(int k=0; k<5; k++)	
				{	
					val = A[i]*B[j]*C[k];
					dp[0] += (val*Coeff(25*i+5*j+k,0));		dp[1] += (val*Coeff(25*i+5*j+k,1));		
					dp[2] += (val*Coeff(25*i+5*j+k,2));		dv[0] += (val*Coeff(25*i+5*j+k,3));
					dv[1] += (val*Coeff(25*i+5*j+k,4));		dv[2] += (val*Coeff(25*i+5*j+k,5));
				}
			}
		}

		// 3. Fill the columns
		if(col < 3)	
		{
			J(0,col) = cos(a1)*dp[0] - sin(a1)*dp[1];		J(3,col) = cos(a1)*dv[0] - sin(a1)*dv[1];		
			J(1,col) = sin(a1)*dp[0] + cos(a1)*dp[1];		J(4,col) = sin(a1)*dv[0] + cos(a1)*dv[1];		
			J(2,col) = dp[2];								J(5,col) = dv[2];	
		}

		if(col == 3)
		{
			J(0,col) = -sin(a1)*dp[0] - cos(a1)*dp[1];		J(3,col) = -sin(a1)*dv[0] - cos(a1)*dv[1];		
			J(1,col) = cos(a1)*dp[0] - sin(a1)*dp[1];		J(4,col) = cos(a1)*dv[0] - sin(a1)*dv[1];		
			J(2,col) = 0;									J(5,col) = 0;
		}

		if(col == 4)
		{
			J(0,col) = 0;		J(3,col) = 0;
			J(1,col) = 0;		J(4,col) = 0;
			J(2,col) = 1;		J(5,col) = 0;
		}
	}
		
}


//void CTRKin::EvalAnalyticJacobian2(const double* jAng, const double* nd, Eigen::MatrixXd& J)
//{
//	// CKim - Evaluate the jacobian of the functional approximation at jAng. 
//	// Jacobian will be 6 by 5 matrix, that is
//	// J = [ dpx/da21, dpx/da31, dpx/dL31, dpx/da1, dpx/dL1 ;
//	//       dpy/da21,		  ................		dpy/dL1	;
//	//			:		      ................		   :	;
//	//		 dvz/da21,        ................      dvz/dL1   ];
//	
//	double a21 = jAng[0];			double a31 = jAng[1];		double L31 = jAng[2];		
//	double a1 = jAng[3];			double L1 = jAng[4];		double Ln = 0.5*3.141592/L31_MAX;
//	Eigen::MatrixXd Coeff(125,6);	GetFAC(Coeff);
//
//	// 1. First evaluate the partial derivate of the harmonic basis function at joint angle. 
//	// For example, derivative w.r.t a21 will be dA = [0, -sin(a21), cos(a21), -2*sin(2*a21), 2*cos(2*a21) .... ]
//	// The length of protrusion is normalized so that its range falls into [0,pi/2]. L31_max = 80.0;
//	double A[5], B[5], C[5];	double L_normalized = L31*Ln;		int n,r;
//
//	for(int col=0; col<5; col++)
//	{
//		if(col==0)	{	A[0] = 0.0;		B[0] = C[0] = 1.0;		}	 // derive w.r.t a21
//		if(col==1)	{	B[0] = 0.0;		A[0] = C[0] = 1.0;		}	 // derive w.r.t a31
//		if(col==2)	{	C[0] = 0.0;		A[0] = B[0] = 1.0;		}	 // derive w.r.t L31
//	
//		for(int i=1; i<5; i++)	
//		{
//			n = i/2;	r = i%2;	
//			if(col==0)	// derive by a21
//			{
//				if(r==0)	{	A[i] = n*cos(n*a21);			B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
//				else		{	A[i] = -(n+1)*sin((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
//			}
//
//			if(col==1)	// derive by a31
//			{
//				if(r==0)	{	A[i] = sin(n*a21);		B[i] = n*cos(n*a31);			C[i] = sin(n*L_normalized);		}
//				else		{	A[i] = cos((n+1)*a21);	B[i] = -(n+1)*sin((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
//			}
//
//			if(col==2)	// derive by L31
//			{
//				if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = n*Ln*cos(n*L_normalized);			}
//				else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = -(n+1)*Ln*sin((n+1)*L_normalized);	}
//			}
//
//			if(col > 2)	// derive by a1 and L1
//			{
//				if(r==0)	{	A[i] = sin(n*a21);		B[i] = sin(n*a31);		C[i] = sin(n*L_normalized);		}
//				else		{	A[i] = cos((n+1)*a21);	B[i] = cos((n+1)*a31);	C[i] = cos((n+1)*L_normalized);	}
//			}
//		}
//
//		// 2. Multiply coefficients
//		double dp[3] = { 0, 0, 0 };		double dv[3] = { 0, 0, 0 };		double val = 0;
//		for(int i=0; i<5; i++)	{
//			for(int j=0; j<5; j++)	{
//				for(int k=0; k<5; k++)	
//				{	
//					val = A[i]*B[j]*C[k];
//					dp[0] += (val*Coeff(25*i+5*j+k,0));		dp[1] += (val*Coeff(25*i+5*j+k,1));		
//					dp[2] += (val*Coeff(25*i+5*j+k,2));		dv[0] += (val*Coeff(25*i+5*j+k,3));
//					dv[1] += (val*Coeff(25*i+5*j+k,4));		dv[2] += (val*Coeff(25*i+5*j+k,5));
//				}
//			}
//		}
//
//		// 3. Fill the columns
//		if(col < 3)	
//		{
//			J(0,col) = cos(a1)*dp[0] - sin(a1)*dp[1];		
//			J(1,col) = sin(a1)*dp[0] + cos(a1)*dp[1];		
//			J(2,col) = dp[2];								
//			J(3,col) = nd[0]*(cos(a1)*dv[0] - sin(a1)*dv[1]) + nd[1]*(sin(a1)*dv[0] + cos(a1)*dv[1]) + nd[2]*(dv[2]);	
//		}
//
//		if(col == 3)
//		{
//			J(0,col) = -sin(a1)*dp[0] - cos(a1)*dp[1];		
//			J(1,col) = cos(a1)*dp[0] - sin(a1)*dp[1];		
//			J(2,col) = 0;									
//			J(3,col) = nd[0]*(-sin(a1)*dv[0] - cos(a1)*dv[1]) + nd[1]*(cos(a1)*dv[0] - sin(a1)*dv[1]) + nd[2]*0.0;
//		}
//
//		if(col == 4)
//		{
//			J(0,col) = 0;		
//			J(1,col) = 0;
//			J(2,col) = 1;		
//			J(3,col) = 0;
//		}
//	}
//		
//}


void CTRKin::EvalCurrentKinematicsModel(const double* jAng, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ)
{
	// CKim - Obtain the copy of the model
	Eigen::MatrixXd Coeff(125,6);	GetFAC(Coeff);

	// CKim - Evaluate forward kinematics
	TipFwdKinEx(jAng,Coeff,predTipPosDir);

	// CKim - Evaluate Jacobian if requested
	if(evalJ)	
	{
		EvalAnalyticJacobian(jAng,Coeff,J);			
	}
	
	return;
}


void CTRKin::EvalCurrentKinematicsModelNumeric(const double* jAng, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ)
{
	// CKim - Variables
	Eigen::MatrixXd Coeff(125,6);		double dq = FLT_EPSILON;		double q[5];		double Fq[6];

	// CKim - Obtain the copy of the model
	GetFAC(Coeff);

	// CKim - Evaluate forward kinematics
	TipFwdKinEx(jAng,Coeff,predTipPosDir);

	// CKim - Numerically evalaluate the jacobian dy/dx, if requested
	if(evalJ)	
	{
		// CKim - Perturb each jAng
		for(int col=0; col<5; col++)
		{
			for(int i=0; i<5; i++)
			{
				if(i==col)
				{
					dq = FLT_EPSILON*fabs(jAng[i]);
					if(dq==0.0) {	dq = FLT_EPSILON;	}
				
					// CKim - This code is said to reduce finite-precision error
					q[i] = jAng[i] + dq;
					dq = q[i] - jAng[i];
				}
				else		{	q[i] = jAng[i];		}
			}
		
			TipFwdKinEx(q,Coeff,Fq);

			for(int i=0; i<6; i++)	{	J(i,col) = (Fq[i] - predTipPosDir[i])/dq;	}
		}
	}
	
	return;
}


void CTRKin::EvalCurrentKinematicsModel_NEW(const double* jAng,  const double* tgtPosDir, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ)
{
	Eigen::MatrixXd tmpJ(6,5);
	
	// CKim - Obtain the copy of the model
	Eigen::MatrixXd Coeff(125,6);	GetFAC(Coeff);

	// CKim - Evaluate forward kinematics
	TipFwdKinEx(jAng,Coeff,predTipPosDir);

	// CKim - Evaluate Jacobian if requested
	if(evalJ)	
	{
		EvalAnalyticJacobian(jAng,Coeff,tmpJ);			

		for(int i=0; i<3; i++)	{
			for(int j=0; j<5; j++)	{	J(i,j) = tmpJ(i,j);		J(3,j) = 0;
				for(int k=0; k<3; k++)	{
					J(3,j) -= (tgtPosDir[3+k]*J(3+k,j));	}	}	}
	}
		
	return;
}

void CTRKin::ApplyKinematicControlNullspace(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dq, double* q, ::Eigen::Vector3d& orGoal, ::Eigen::Vector3d& orActual)
{
	//::std::cout << "nullspace" << ::std::endl;
	//::Eigen::VectorXd dotq(5);
	//this->ComputeJointspaceVelocities(J, err, dotq, orGoal, orActual);

	//double limit_margin = 2.0;
	//if ((q[2] >= L31_MAX - limit_margin && dotq[2] > 0) || (q[2] <= L31_MIN && dotq[2] < 0))
	//{
	//	::Eigen::MatrixXd Jlocal = J;		
	//	removeColumn(Jlocal, 2);
	//	this->ComputeJointspaceVelocities(Jlocal, err, dotq, orGoal, orActual);

	//	if(q[2] >= L31_MAX - limit_margin)
	//		dotq[2] = 10*(L31_MAX - limit_margin - q[2]) - 0.1 * dotq[2];
	//	else if(q[2] <= L31_MIN)
	//		dotq[2] = 10*(L31_MIN - q[2]) - 0.1 * dotq[2];
	//}

	//for(int i=0; i<5; i++)
	//	dq[i] = dotq(i,0);	

}

void CTRKin::ApplyKinematicControlNullspace_KHATIB(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dq, double* q, ::Eigen::Vector3d& orGoal, ::Eigen::Vector3d& orActual)
{
	//::std::cout << "KHATIB" << ::std::endl;

	//::Eigen::VectorXd dotq(5);
	//this->ComputeJointspaceVelocities_KHATIB(J, err, dotq, orGoal, orActual);

	//double limit_margin = 2.0;
	//if ((q[2] >= L31_MAX - limit_margin && dotq[2] > 0) || (q[2] <= L31_MIN && dotq[2] < 0))
	//{
	//	::Eigen::MatrixXd Jlocal = J;		
	//	removeColumn(Jlocal, 2);
	//	this->ComputeJointspaceVelocities_KHATIB(Jlocal, err, dotq, orGoal, orActual);

	//	if(q[2] >= L31_MAX - limit_margin)
	//		dotq[2] = 10*(L31_MAX - limit_margin - q[2]) - 0.1 * dotq[2];
	//	else if(q[2] <= L31_MIN)
	//		dotq[2] = 10*(L31_MIN - q[2]) - 0.1 * dotq[2];
	//}

	//for(int i=0; i<5; i++)
	//	dq[i] = dotq(i,0);	

}


void CTRKin::ComputeJointspaceVelocities(const ::Eigen::MatrixXd& J, const ::Eigen::MatrixXd& err, ::Eigen::VectorXd& qdot, ::Eigen::Vector3d& orGoal, ::Eigen::Vector3d& orActual)
{
	int nCol = J.cols();
	double scl_jacobian = M_PI / 180.0 * 3;

	::Eigen::VectorXd qdotTemp(nCol);

	::Eigen::MatrixXd Jtemp = J;

	Jtemp.col(0) *= scl_jacobian;
	Jtemp.col(1) *= scl_jacobian;

	if (nCol == 4)
		Jtemp.col(2) *= scl_jacobian;
	else
		Jtemp.col(3) *= scl_jacobian;

	// position and orientation jacobian
	::Eigen::MatrixXd Jp  = Jtemp.block(0, 0, 3, nCol);
	::Eigen::MatrixXd Jo = Jtemp.block(3, 0, 3, nCol);

	// compute Jo using the new orientation definition
	Jo = -1/sqrt(1 - ::std::pow(orActual.transpose() * orGoal, 2)) * orGoal.transpose() * Jo.eval();

	// condition matrix for pseudo inverse
	::Eigen::Matrix<double, 3, 3> tmpMat (Jp * Jp.transpose());

	double lambda_position = 0.0;
	double epsilon = 0.01;
	double lambda_position_max = 0.01;

	if (nCol == 4)
		lambda_position_max *= 10; //is that necessary?

	::Eigen::JacobiSVD<::Eigen::MatrixXd> svd(tmpMat, ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);
	::Eigen::VectorXd singVal = svd.singularValues();

	if (singVal[singVal.size() - 1] <= epsilon)
		lambda_position = (1.0 - ::std::pow(singVal[singVal.size() - 1]/epsilon, 2)) * lambda_position_max;

	for (int i = 0; i < 3; ++i)
		tmpMat(i, i) += lambda_position;

	// task 1: control position
	::Eigen::MatrixXd task1_pseudo = Jp.transpose() * tmpMat.inverse();
	qdotTemp = task1_pseudo * err.block(0, 0, 3, 1);

	// task 2: control orientation
	::Eigen::MatrixXd IdMat(nCol, nCol);
	IdMat.setIdentity();

	::Eigen::MatrixXd tmpOrient;
	::Eigen::MatrixXd tmpOrientPseudo;

	// do the same thing for orientation here
	tmpOrient = Jo * (IdMat - task1_pseudo * Jp);
	tmpOrientPseudo = tmpOrient * tmpOrient.transpose();

	::Eigen::JacobiSVD<::Eigen::MatrixXd> svdOr(tmpOrientPseudo, ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);
	::Eigen::VectorXd singValOr = svdOr.singularValues();

	double lambda_orientation = 0.0;
	double lambda_orientation_max = 0.0001;

	if (nCol == 4)
		lambda_orientation_max *= 100;


	if (singValOr[singValOr.size() - 1] <= epsilon)
		lambda_orientation = (1.0 - ::std::pow(singValOr[singValOr.size() - 1]/epsilon, 2)) * lambda_orientation_max;

	for (int i = 0; i < 3; ++i)
		tmpOrientPseudo(i, i) += lambda_orientation;

	::Eigen::MatrixXd orientPseudo = tmpOrient.transpose() * tmpOrientPseudo.inverse();
	qdotTemp += orientPseudo * (err.block(3, 0, 1, 1) - Jo * task1_pseudo * err.block(0, 0, 3, 1));


	qdotTemp[0] *= scl_jacobian;
	qdotTemp[1] *= scl_jacobian;


	if (nCol == 4)
	{
		qdotTemp[2] *= scl_jacobian;
		qdot.segment(0, 2) = qdotTemp.segment(0, 2);
		qdot(2) = 0;
		qdot.segment(3, 2) = qdotTemp.segment(2, 2);
	}
	else
	{
		qdotTemp[3] *= scl_jacobian;
		qdot = qdotTemp;
	}

}


void CTRKin::ComputeJointspaceVelocities_KHATIB(const ::Eigen::MatrixXd& J, const ::Eigen::MatrixXd& err, ::Eigen::VectorXd& qdot, ::Eigen::Vector3d& orGoal, ::Eigen::Vector3d& orActual)
{
	int nCol = J.cols();
	double scl_jacobian = M_PI / 180.0 * 3;

	::Eigen::VectorXd qdotTemp(nCol);

	::Eigen::MatrixXd Jtemp = J;

	Jtemp.col(0) *= scl_jacobian;
	Jtemp.col(1) *= scl_jacobian;

	if (nCol == 4)
		Jtemp.col(2) *= scl_jacobian;
	else
		Jtemp.col(3) *= scl_jacobian;

	// position and orientation jacobian
	::Eigen::MatrixXd Jp  = Jtemp.block(0, 0, 3, nCol);
	::Eigen::MatrixXd Jo = Jtemp.block(3, 0, 3, nCol);

	// compute Jo using the new orientation definition
	Jo = -1/sqrt(1 - ::std::pow(orActual.transpose() * orGoal, 2)) * orGoal.transpose() * Jo.eval();

	// condition matrix for pseudo inverse
	::Eigen::Matrix<double, 3, 3> tmpMat (Jp * Jp.transpose());

	double lambda_position = 0.0;
	double epsilon = 0.01;
	double lambda_position_max = 0.01;

	if (nCol == 4)
		lambda_position_max *= 10; //is that necessary?

	::Eigen::JacobiSVD<::Eigen::MatrixXd> svd(tmpMat, ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);
	::Eigen::VectorXd singVal = svd.singularValues();

	if (singVal[singVal.size() - 1] <= epsilon)
		lambda_position = (1.0 - ::std::pow(singVal[singVal.size() - 1]/epsilon, 2)) * lambda_position_max;

	for (int i = 0; i < 3; ++i)
		tmpMat(i, i) += lambda_position;

	// task 1: control position
	::Eigen::MatrixXd task1_pseudo = Jp.transpose() * tmpMat.inverse();
	qdotTemp = task1_pseudo * err.block(0, 0, 3, 1);


	// task 2: control orientation
	::Eigen::Matrix<double, 3, 3> tmpMatOrientation (Jo * Jo.transpose());

	::Eigen::JacobiSVD<::Eigen::MatrixXd> svdOr(tmpMatOrientation, ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);
	::Eigen::VectorXd singValOr = svdOr.singularValues();

	double lambda_orientation = 0.0;
	double lambda_orientation_max = 0.0001;

	if (nCol == 4)
		lambda_orientation_max *= 100;


	if (singValOr[singValOr.size() - 1] <= epsilon)
		lambda_orientation = (1.0 - ::std::pow(singValOr[singValOr.size() - 1]/epsilon, 2)) * lambda_orientation_max;

	for (int i = 0; i < 3; ++i)
		tmpMatOrientation(i, i) += lambda_orientation;

	::Eigen::MatrixXd task2_pseudo = Jo.transpose() * tmpMatOrientation.inverse();

	::Eigen::MatrixXd IdMat(nCol, nCol);
	IdMat.setIdentity();
	
	::Eigen::MatrixXd nullspaceProjection = (IdMat - task1_pseudo * Jp) * task2_pseudo;
	qdotTemp += nullspaceProjection * err(3, 0);

	qdotTemp[0] *= scl_jacobian;
	qdotTemp[1] *= scl_jacobian;

	if (nCol == 4)
	{
		qdotTemp[2] *= scl_jacobian;
		qdot.segment(0, 2) = qdotTemp.segment(0, 2);
		qdot(2) = 0;
		qdot.segment(3, 2) = qdotTemp.segment(2, 2);
	}
	else
	{
		qdotTemp[3] *= scl_jacobian;
		qdot = qdotTemp;
	}

}


void CTRKin::conditionMatrix(::Eigen::Matrix2d& mat_original, double conditionNumberDes, ::Eigen::MatrixXd& mat_conditioned, double& condition_number_new)
{
	::Eigen::JacobiSVD<Eigen::MatrixXd> Jsvd(mat_original, Eigen::ComputeThinU | Eigen::ComputeThinV);
	::Eigen::VectorXd sv = Jsvd.singularValues();	

	double sigma_min = sv(sv.size() - 1, 0);
	double sigma_max = sv(0, 0);
	double conditionNumber = sigma_max/sigma_min;

	double lambda_squared = (conditionNumberDes * sigma_min - sigma_max)/(1 - conditionNumberDes); //assume desired condition number is not 1

	mat_conditioned = mat_original + ::Eigen::MatrixXd::Identity(sv.size(), sv.size()) * lambda_squared;

	Jsvd.compute(mat_conditioned);
	::Eigen::VectorXd sv_new = Jsvd.singularValues();

	condition_number_new = sv(0, 0)/sv(sv.size() - 1, 0);
}


void CTRKin::ApplyHybridPositionForceControl(const ::Eigen::MatrixXd& J, const ::Eigen::MatrixXd& err, const ::Eigen::MatrixXd& desiredForce, const ::Eigen::MatrixXd& actualForce, double* dq, double* q)
{
	::Eigen::VectorXd dotq(5);
	::Eigen::Matrix<double, 6, 5> Jtemp = J;			// this can be avoided - it's just to overcome the const
	Jtemp.col(0) *= M_PI / 180.0;
	Jtemp.col(1) *= M_PI / 180.0;
	Jtemp.col(3) *= M_PI / 180.0;

	::Eigen::Matrix<double, 3, 5> Jp = Jtemp.block(0, 0, 3, 5);

	::Eigen::Matrix<double, 3, 3> selectionMatrixPosition, selectionMatrixForce;
	selectionMatrixPosition.setIdentity();
	selectionMatrixPosition(2,2) = 0.0;
	selectionMatrixForce = ::Eigen::MatrixXd::Identity(3,3) - selectionMatrixPosition;

	double positionGain = 1.0;
	double forceGain = this->m_forceGain;
	::Eigen::MatrixXd generalizedForce = positionGain * selectionMatrixPosition * err.block(0, 0, 3, 1) + forceGain * selectionMatrixForce * (desiredForce - actualForce);
	
	::std::cout << "Actual Force: " << actualForce(2) << ", Desired force:" << desiredForce(2) << ", error:" << desiredForce(2) - actualForce(2) << std::endl;

	
	//::std::cout << "force controller gain: " << forceGain << ::std::endl;
	::Eigen::Matrix<double, 3, 3> tmpMat (Jp * Jp.transpose());
	::Eigen::Matrix<double, 3, 5> Jo = Jtemp.block(3,0, 3, 5);	
	::Eigen::Matrix<double, 5, 5> IdMat;
	IdMat.setIdentity();

	// add small epsilon in the diagonal to avoid singular matrix inversion - damped pseudoinverse
	for (int i = 0; i < 3; ++i)
		tmpMat(i, i) += 0.01;

	//orientation in the nullspace
	double orientationGain = 1000.0;
	dotq += orientationGain*( IdMat - Jp.transpose() * tmpMat.inverse() * Jp) * Jo.transpose() * err.block(3, 0, 3, 1);

	dotq = Jp.transpose() * generalizedForce;

	// overall gain
	dotq *= 0.05;

	// Joint limit avoidance using potential-field method
	double upperSoft = L31_MAX - 10;
	double lowerSoft = L31_MIN + 5;
	double jointLimitGain = 0.002;

	if (q[2] >= upperSoft)
		dotq[2] += max(-1.0, -jointLimitGain/::std::pow(q[2] - L31_MAX, 2) + jointLimitGain/::std::pow(upperSoft - L31_MAX, 2));

	if (q[2] <= lowerSoft )
		dotq[2] += min(1.0, jointLimitGain/::std::pow(q[2] - L31_MIN, 2) - jointLimitGain/::std::pow(lowerSoft - L31_MIN, 2));


	if (q[2] >= L31_MAX && dotq[2] > 0) 
		dotq[2] = 0.0;
	else if (q[2] <= L31_MIN && dotq[2] < 0)
		dotq[2] = 0.0;

	for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}

}

void CTRKin::ApplyKinematicControl(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dq, double* q, ::Eigen::Vector3d& orGoal, ::Eigen::Vector3d& orActual)
{
		//::std::cout << "weighted" << ::std::endl;

	//Eigen::VectorXd localErr = err.col(0);

	//// CKim - This function is called when I use 6 x 5 Jacobian
	//::Eigen::MatrixXd JtJ;	::Eigen::MatrixXd b;		::Eigen::VectorXd dotq;		
	//::Eigen::MatrixXd A;	double lambda = 10;				::Eigen::MatrixXd sv;
	//double eps;						double condNum;

	//// compute Jo using the new orientation definition
	//::Eigen::VectorXd Jo = -1/sqrt(1 - ::std::pow(orActual.transpose() * orGoal, 2)) * orGoal.transpose() * Jo.block(3, 0, 3, 5);

	//::Eigen::Matrix<double, 4, 5> Jtmp;
	//Jtmp.block(0, 0, 3, 5) = J.block(0, 0, 3, 5);
	//Jtmp.row(3) = Jo;

	//// CKim - Invert jacobian, handle singularity and solve
	//double scalarWeight = 50.0;
	//Eigen::Matrix<double, 4,4> weights;
	//weights.setIdentity();
	//for (int i = 3; i < 4; ++i)
	//	weights(i,i) = pow(scalarWeight,2);

	//JtJ = Jtmp.transpose() * weights * Jtmp;			b = Jtmp.transpose()* weights * localErr;

	//Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> Jsvd(JtJ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	//sv = Jsvd.singularValues();	
	//

	//double conditionNumber = sv(0, 0)/sv(4, 0);
	//double conditionThreshold = 2e4;
	//
	//if (conditionNumber >= conditionThreshold)
	//{
	//	::std::cout << "TRANSPOSE" << ::std::endl;

	//	dotq = 0.005*b;
	//	dotq(2) *= 100;
	//	dotq(4) *= 100;

	//	for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}

	//	return;
	//}

	//::std::cout << "INVERSE" << ::std::endl;
	//dotq = Jsvd.solve(b);
	////::std::cout << dotq << ::std::endl;

	//// Joint limit avoidance using potential-field method
	//double upperSoft = L31_MAX - 10;
	//double lowerSoft = L31_MIN + 5;
	//double jointLimitGain = 0.2;

	//if (q[2] >= upperSoft)
	//	dotq[2] += max(-5.0, -jointLimitGain/::std::pow(q[2] - L31_MAX, 2) + jointLimitGain/::std::pow(upperSoft - L31_MAX, 2));

	//if (q[2] <= lowerSoft )
	//	dotq[2] += min(5.0, jointLimitGain/::std::pow(q[2] - L31_MIN, 2) - jointLimitGain/::std::pow(lowerSoft - L31_MIN, 2));


	//if (q[2] >= L31_MAX && dotq[2] > 0) 
	//	dotq[2] = -5.0;
	//else if (q[2] <= L31_MIN && dotq[2] < 0)
	//	dotq[2] = 5.0;

	//for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}
	//for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}

}


void CTRKin::ApplyKinematicControl_NEW(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dq)
{
	// CKim - This function is called when I use 4 x 5 Jacobian
	Eigen::Matrix<double,4,4> JJt;	Eigen::Matrix<double,4,1> b;		Eigen::Matrix<double,5,1> dotq;	

	JJt = J*J.transpose();	
	Eigen::JacobiSVD<Eigen::Matrix<double,4,4>> Jsvd(JJt,Eigen::ComputeThinU | Eigen::ComputeThinV);
	b = Jsvd.solve(err);
	dotq = J.transpose()*b;
		
	for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}

	////	Eigen::Matrix<double,5,5> A;	double lambda = 0.1;				Eigen::Matrix<double,5,1> sv;
	////double eps;							double condNum;//

	//// CKim - Invert jacobian, handle singularity and solve
	//JtJ = J.transpose()*J;			b = J.transpose()*err;
	//Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> Jsvd(JtJ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	//sv = Jsvd.singularValues();	
	//eps = sv(0,0)*Eigen::NumTraits<double>::epsilon();

	//condNum = fabs(sv(4,0));
	////eps = 0.00001;
	//		
	////for(int i=0; i<5; i++)	{	JtJ(i,i) += lambda;		}

	//if(condNum < eps)	
	//{
	//	//localStat.invKinOK = false;
	//	A = JtJ;
	//	for(int i=0; i<5; i++)	{	A(i,i) += (1-sv(4,0)/eps)*lambda;	}
	//			////for(int i=0; i<5; i++)	{	A(i,i) += lambda*JtJ(i,i);		}
	//	Jsvd.compute(A);
	//}
	//else
	//{
	//	//localStat.invKinOK = true;
	//}
	//
	//dotq = Jsvd.solve(b);
	//for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}
}