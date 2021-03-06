#include "StdAfx.h"
#include "CTRKin.h"
#include "ChunTimer.h"
#include "Utilities.h"


CTRKin::CTRKin(const ::std::string& modelStr, int modelInputDim):
	modelInputDim(modelInputDim),
	fName(modelStr)
{

	if (modelStr.size() == 0) return;
	if (readCTR_FAC_file(this->fName, m_Tip_px, m_Tip_py, m_Tip_pz, m_Tip_ox, m_Tip_oy, m_Tip_oz) == false) 
		AfxMessageBox("Error reading Fourier Model Coefficients for the tip");

	this->AllocateCoefficientMatrices();

	// CKim - Initialize local variable used in adaptive update.
	m_tmpMat.resize(this->coeffSize, 6);
	for(int i=0; i<this->coeffSize; i++)	
	{
		m_tmpMat(i,0) = m_Tip_px[i];		m_tmpMat(i,1) = m_Tip_py[i];		m_tmpMat(i,2) = m_Tip_pz[i];
		m_tmpMat(i,3) = m_Tip_ox[i];		m_tmpMat(i,4) = m_Tip_oy[i];		m_tmpMat(i,5) = m_Tip_oz[i];
	}

	// CKim  Coefficients for Balance Pair
	fName = "C:\\01. ConcentricTubeRobots\\CTR\\CTR_BP_FAC.txt";

	if (readCTR_FAC_file(fName, m_BP_px, m_BP_py, m_BP_pz, m_BP_ox, m_BP_oy, m_BP_oz) == false) //file read error
		AfxMessageBox("Error reading Fourier Model Coefficients for the balanced pair");


	// CKim - Initialize matrices for recursive least square
	//for(int i=0; i<6; i++)	{
	//	F[i].resize(this->coeffSize, this->coeffSize);	F[i].setIdentity();		F[i] = 0.1*F[i];		}

	//Fzero.resize(this->coeffSize, this->coeffSize);

	this->ReInitializeEstimator();

	SetInvKinThreshold(0.1,3.0);
	
	m_forgettingFactor = 1.0;

	m_hFACMutex = CreateMutex(NULL,false,"FAC_Mutex");

}

void CTRKin::AllocateCoefficientMatrices()
{
	int basisFunctionLength = 2 * this->modelOrder - 1;
	this->A = new double[basisFunctionLength];
	this->B = new double[basisFunctionLength];
	this->C = new double[basisFunctionLength];
	this->AUpdated = new double[basisFunctionLength];
	this->BUpdated = new double[basisFunctionLength];
	this->CUpdated = new double[basisFunctionLength];

	int BPCoeffZize = 125;
	this->m_BP_px = new double[BPCoeffZize];
	this->m_BP_py = new double[BPCoeffZize];
	this->m_BP_pz = new double[BPCoeffZize];
	this->m_BP_ox = new double[BPCoeffZize];
	this->m_BP_oy = new double[BPCoeffZize];
	this->m_BP_oz = new double[BPCoeffZize];
}

CTRKin::~CTRKin(void)
{
}


void CTRKin::ReInitializeEstimator()
{
	for(int i=0; i<6; i++)	{
		F[i].resize(this->coeffSize, this->coeffSize);	F[i].setIdentity();		F[i] = 0.1*F[i];		}

	Fold.resize(this->coeffSize, this->coeffSize);
	Fold.setZero();

	Fzero.resize(this->coeffSize, this->coeffSize);
}


void CTRKin::ReInitializeModel()
{

	if (readCTR_FAC_file(this->fName, m_Tip_px, m_Tip_py, m_Tip_pz, m_Tip_ox, m_Tip_oy, m_Tip_oz) == false) //file read error
		AfxMessageBox("Sparta!!!!");
	
	// CKim - Initialize local variable used in adaptive update.
	m_tmpMat.resize(this->coeffSize,6);
	for(int i=0; i<this->coeffSize; i++)	
	{
		m_tmpMat(i,0) = m_Tip_px[i];		m_tmpMat(i,1) = m_Tip_py[i];		m_tmpMat(i,2) = m_Tip_pz[i];
		m_tmpMat(i,3) = m_Tip_ox[i];		m_tmpMat(i,4) = m_Tip_oy[i];		m_tmpMat(i,5) = m_Tip_oz[i];
	}

	// CKim  Coefficients for Balance Pair
	fName = "C:\\01. ConcentricTubeRobots\\CTR\\CTR_BP_FAC.txt";

	if (readCTR_FAC_file(fName, m_BP_px, m_BP_py, m_BP_pz, m_BP_ox, m_BP_oy, m_BP_oz) == false) //file read error
		AfxMessageBox("Sparta!!!!");

}


bool CTRKin::readCTR_FAC_file(std::string fileName,  double px[], double py[],  double pz[],  double ox[],  double oy[],  double oz[])
{

	std::string junkS;	
	std::ifstream CTR_FAC_id;	CTR_FAC_id.open(fileName);
	if(CTR_FAC_id.fail())		
		return false;	

	for(int i = 0; i < this->coeffSize; i++)
	{
		CTR_FAC_id >> junkS >> junkS >> junkS >> junkS;
		CTR_FAC_id >> px[i] >>  junkS >> py[i] >> junkS >> pz[i] >> junkS >> ox[i] >> junkS >> oy[i] >> junkS >> oz[i];
		CTR_FAC_id >> junkS;
	}

	CTR_FAC_id.close();
	return true;
}

bool CTRKin::readCTR_FAC_file(std::string fileName,  DVec& px, DVec& py,  DVec& pz,  DVec& ox,  DVec& oy,  DVec& oz)
{
	std::string junkS;	
	std::ifstream CTR_FAC_id;	CTR_FAC_id.open(fileName);
	if(CTR_FAC_id.fail())		
		return false;	

	double tmpX, tmpY, tmpZ, tmpOX,tmpOY,tmpOZ;
	while (!CTR_FAC_id.eof())
	{
		CTR_FAC_id >> junkS >> junkS >> junkS >> junkS;
		CTR_FAC_id >> tmpX >>  junkS >> tmpY >> junkS >> tmpZ >> junkS >> tmpOX >>  junkS >> tmpOY >> junkS >> tmpOZ;
		CTR_FAC_id >> junkS;
		px.push_back(tmpX); py.push_back(tmpY); pz.push_back(tmpZ); ox.push_back(tmpOX); oy.push_back(tmpOY); oz.push_back(tmpOZ);
	}

	CTR_FAC_id.close();

	this->modelOrder = static_cast<int> (0.5 * (::std::pow(px.size(), 1.0/3.0) + 1));
	this->coeffSize = ::std::pow(2 * this->modelOrder - 1, this->modelInputDim);

	return true;

}


void CTRKin::GetFAC(Eigen::MatrixXd& Coeff)
{
	WaitForSingleObject(m_hFACMutex,INFINITE);
	for(int i=0; i<6; i++)
	{
		if(i==0)	
			for(int j = 0; j < this->coeffSize; j++)	
				Coeff(j,i) = m_Tip_px[j];		
		if(i==1)	
			for(int j = 0; j < this->coeffSize; j++)
				Coeff(j,i) = m_Tip_py[j];		
		if(i==2)	
			for(int j = 0; j < this->coeffSize; j++)
				Coeff(j,i) = m_Tip_pz[j];	
		if(i==3)	
			for(int j = 0; j < this->coeffSize; j++)
				Coeff(j,i) = m_Tip_ox[j];	
		if(i==4)	
			for(int j = 0; j < this->coeffSize; j++)
				Coeff(j,i) = m_Tip_oy[j];
		if(i==5)	
			for(int j = 0; j < this->coeffSize; j++)
				Coeff(j,i) = m_Tip_oz[j];
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
	int basisFunctionLength = 2 * this->modelOrder - 1;

	this->A[0] = this->B[0] = this->C[0] = 1.0;

	double L_normalized;		int n,r;	

	for(int i=1; i < basisFunctionLength; i++)	
	{
		n = i/2;	r = i % 2;	L_normalized = L31/L31_MAX*0.5*3.141592;
		if(r==0)	{	this->A[i] = sin(n*a21);		this->B[i] = sin(n*a31);		this->C[i] = sin(n*L_normalized);		}
		else		{	this->A[i] = cos((n+1)*a21);	this->B[i] = cos((n+1)*a31);	this->C[i] = cos((n+1)*L_normalized);	}
	}

	WaitForSingleObject(m_hFACMutex,INFINITE);

	// 2. Multiply coefficients
	double p[3] = { 0, 0, 0 };		double v[3] = { 0, 0, 0 };		double val = 0;
	for(int i=0; i < basisFunctionLength; i++)	{
		for(int j=0; j < basisFunctionLength; j++)	{
			for(int k=0; k < basisFunctionLength; k++)	
			{	
				val = this->A[i] * this->B[j] * this->C[k];
				int ind = ::std::pow(basisFunctionLength, 2)*i + basisFunctionLength*j + k;
				p[0] += (val*m_Tip_px[ind]);		p[1] += (val*m_Tip_py[ind]);		p[2] += (val*m_Tip_pz[ind]);	
				v[0] += (val*m_Tip_ox[ind]);		v[1] += (val*m_Tip_oy[ind]);		v[2] += (val*m_Tip_oz[ind]);
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

	return true;
}


void CTRKin::TipFwdKinEx(const double* jAng, const Eigen::MatrixXd& Coeff, double* posOrt)
{
	// CKim - Evaluate functional approximation at jAng. a21, a31 relative rotation of tube 2 and 3
	// w.r.t. tube 1. L31 is relative protrusion of the tube w.r.t. balanced pair
	double a21 = jAng[0];		double a31 = jAng[1];		double L31 = jAng[2];

	// 1. First evaluate harmonic basis function at joint angle. A = [1, cos(a21), sin(a21), cos(2*a21), sin(2*a21) .... ]
	// The length of protrusion is normalized so that its range falls into [0,pi/2]. L31_max = 80.0;
	int basisFunctionLength = 2 * this->modelOrder - 1;

	this->A[0] = this->B[0] = this->C[0] = 1.0;

	double L_normalized;		int n,r;	

	for(int i=1; i < basisFunctionLength; i++)	
	{
		n = i/2;	r = i % 2;	L_normalized = L31/L31_MAX*0.5*3.141592;
		if(r==0)	{	this->A[i] = sin(n*a21);		this->B[i] = sin(n*a31);		this->C[i] = sin(n*L_normalized);		}
		else		{	this->A[i] = cos((n+1)*a21);	this->B[i] = cos((n+1)*a31);	this->C[i] = cos((n+1)*L_normalized);	}
	}


	// 2. Multiply coefficients
	double p[3] = { 0, 0, 0 };		double v[3] = { 0, 0, 0 };		double val = 0;
	for(int i=0; i < basisFunctionLength; i++)	{
		for(int j=0; j < basisFunctionLength; j++)	{
			for(int k=0; k < basisFunctionLength; k++)	
			{	
				val = this->A[i] * this->B[j] * this->C[k];
				int ind = ::std::pow(basisFunctionLength, 2)*i + basisFunctionLength*j + k;
				p[0] += (val*m_Tip_px[ind]);		p[1] += (val*m_Tip_py[ind]);		p[2] += (val*m_Tip_pz[ind]);	
				v[0] += (val*m_Tip_ox[ind]);		v[1] += (val*m_Tip_oy[ind]);		v[2] += (val*m_Tip_oz[ind]);
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

}


// needs to be changed to work for arbitrary order
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
	Eigen::MatrixXd Coeff(this->coeffSize, 6);		GetFAC(Coeff);
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
	Eigen::MatrixXd Coeff(this->coeffSize, 6);		GetFAC(Coeff);
	for(int i=0; i<5; i++)	{	jAng[i] = init[i];		}
	lambda = 0.001;		exitCond = 0;

	// CKim - Iterate
	for(iter = 0; iter < maxiter; iter++)
	{
		// CKim - Evaluate function at curent joint angle
		EvalF_LSQ(jAng,tgtPosOrt,Coeff,fx);		
		Err[0] = fx.norm(); 
		Err[1] = sqrt( fx(0,0)*fx(0,0) + fx(1,0)*fx(1,0) + fx(2,0)*fx(2,0) );
		Err[2] = acos(1.0 - fx(3,0)*(1-cos(m_MaxOrtErr*3.141592/180.0))/m_MaxPosErr)*180.0/3.141592;

		// CKim - Exit if distance is less than threshold
		if(Err[0] < eps)	{	exitCond = 1;	break;		}

		// CKim - Numerically evalaluate the jacobian dy/dx
		EvalJ_LSQ(jAng,tgtPosOrt,Coeff,J);

		// CKim - Find the update direction. update = - inv ( (JtJ + lambda*diag(JtJ) ) Jt*fx


		b = -1.0 * J.transpose()*fx;
		JtJ = J.transpose()*J;		A = JtJ;
		for(int i=0; i<5; i++)	{	A(i,i) += lambda*JtJ(i,i);	}

		// CKim - Calculate Update step. Use JacobiSVD.solve to get least square solution
		Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> Jsvd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);
		update = Jsvd.solve(b);

		// CKim - If the magnitude of the update direction is small, exit
		if(update.norm() < 0.001)	{	exitCond = 2;	break;		}

		// CKim - Decide how much in the update direction we should move. Full (1) to minimum (0.1)
		// this depends on if |error|^2 is decreasing
		stepSz = 1.0;			
		while(1)
		{
			for(int i=0; i<5; i++)	{	temp[i] = jAng[i] + stepSz*update(i,0);	}

			// CKim - Evalualte distance at new updated point
			EvalF_LSQ(temp,tgtPosOrt,Coeff,fxnew);	

			// CKim - If the norm^2 of the function decreases, accept the step
			if(fxnew.squaredNorm() < fx.squaredNorm())
			{
				if(lambda > 0.001)	{	lambda /= 10.0;		}
				//lambda /= 10.0;
				for(int i=0; i<5; i++)	{	jAng[i] = temp[i];	}
				break;
			}
			else
			{
				stepSz *= 0.5;	
			}

			// CKim - If function is not decreasing, 
			if(stepSz < 0.005)	
			{
				lambda *= 10;	
				break;
			}
		}
	}
	if(iter == maxiter)	{		exitCond = 3;		}
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
	F(3,0) = pmax/(1-cos(thmax))*(1-sum);
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
	double a21 = jAng[0];		double a31 = jAng[1];		double L31 = jAng[2];
	::Eigen::VectorXd x(this->coeffSize);
	x.setZero();

	int basisFunctionLength = 2 * this->modelOrder - 1;

	this->AUpdated[0] = this->BUpdated[0] = this->CUpdated[0] = 1.0;

	double L_normalized;		int n,r;	

	clock_t start, end;
	for(int i=1; i<basisFunctionLength; i++)	
	{
		n = i/2;	r = i%2;	L_normalized = L31/L31_MAX*0.5*3.141592;
		if(r==0)	{	this->AUpdated[i] = sin(n*a21);		this->BUpdated[i] = sin(n*a31);		this->CUpdated[i] = sin(n*L_normalized);		}
		else		{	this->AUpdated[i] = cos((n+1)*a21);	this->BUpdated[i] = cos((n+1)*a31);	this->CUpdated[i] = cos((n+1)*L_normalized);	}
	}

	int cnt = 0;
	for(int i=0; i < basisFunctionLength; i++)	
		for(int j=0; j < basisFunctionLength; j++)
			for(int k=0; k < basisFunctionLength; k++)	
			{	
				x(cnt,0) = this->AUpdated[i] * this->BUpdated[j] * this->CUpdated[k];	
				cnt++;	
			}

			// 0.5. In measuredData, compensate for the rigid rotation/translation about/along z axis
			double a1 = jAng[3];		double L1 = jAng[4];	double meas[6];		double pred[6];		double err;

			meas[0] = cos(a1)*measTipPosDir[0] + sin(a1)*measTipPosDir[1];
			meas[1] = -sin(a1)*measTipPosDir[0] + cos(a1)*measTipPosDir[1];
			meas[2] = measTipPosDir[2] - L1;
			meas[3] = cos(a1)*measTipPosDir[3] + sin(a1)*measTipPosDir[4];
			meas[4] = -sin(a1)*measTipPosDir[3] + cos(a1)*measTipPosDir[4];
			meas[5] = measTipPosDir[5];

			Fold = F[0];
			::Eigen::VectorXd tmpVec = Fold * x;

			double tmp = tmpVec.transpose() * x;
			F[0] = (1/m_forgettingFactor)*(Fold - (1/(m_forgettingFactor+tmp)) * tmpVec*tmpVec.transpose());

			if(doUpdate)		
				for(int i=0; i<6; i++)		
				{
					// 1. Calculate prediction error
					pred[i] = x.dot(m_tmpMat.col(i));
					err = meas[i] - pred[i];

					m_tmpMat.col(i) += err*F[0]*x;

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
				memcpy(m_Tip_px.data(), m_tmpMat.col(0).data(), sizeof(double) * this->coeffSize);
				memcpy(m_Tip_py.data(), m_tmpMat.col(1).data(), sizeof(double) * this->coeffSize);
				memcpy(m_Tip_pz.data(), m_tmpMat.col(2).data(), sizeof(double) * this->coeffSize);
				memcpy(m_Tip_ox.data(), m_tmpMat.col(3).data(), sizeof(double) * this->coeffSize);
				memcpy(m_Tip_oy.data(), m_tmpMat.col(4).data(), sizeof(double) * this->coeffSize);
				memcpy(m_Tip_oz.data(), m_tmpMat.col(5).data(), sizeof(double) * this->coeffSize);

				ReleaseMutex(m_hFACMutex);
			}

	

}


void CTRKin::UpdateInitM(const double jAng[5], bool inv)
{
}


void CTRKin::EvalCurrentKinematicsModelNumeric(const double* jAng, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ)
{
	// CKim - Variables
	Eigen::MatrixXd Coeff(this->coeffSize,6);		double dq = FLT_EPSILON;		double q[5];		double Fq[6];

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


void CTRKin::ApplyKinematicControl(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dq)
{

	// George - ignoring small errors
	Eigen::VectorXd localErr = err.col(0);
	for(int i = 0; i < 3 ; ++i)
		if(fabs(localErr(i)) < 0.1)
			localErr(i) = 0;

	if(localErr.segment(3,3).norm() < 0.1 * M_PI / 180)
		localErr.segment(3,3).setZero();

	// CKim - This function is called when I use 6 x 5 Jacobian
	Eigen::Matrix<double,5,5> JtJ;	Eigen::Matrix<double,5,1> b;		Eigen::Matrix<double,5,1> dotq;		
	Eigen::Matrix<double,5,5> A;	double lambda = 10;				Eigen::Matrix<double,5,1> sv;
	double eps;						double condNum;

	// CKim - Invert jacobian, handle singularity and solve
	double scalarWeight = 50.0;
	Eigen::Matrix<double, 6,6> weights;
	weights.setIdentity();
	for (int i = 3; i < 6; ++i)
		weights(i,i) = pow(scalarWeight,2);

	//JtJ = J.transpose()*J;			b = J.transpose()*err;
	JtJ = J.transpose() * weights * J;			b = J.transpose()* weights * localErr;

	Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> Jsvd(JtJ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	sv = Jsvd.singularValues();	


	double conditionNumber = sv(0, 0)/sv(4, 0);
	double conditionThreshold = 0.1; //2e4;

	if (conditionNumber >= conditionThreshold)
	{
		dotq = 0.0005*b;
		dotq(2) *= 100;
		dotq(4) *= 100;

		for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}

		return;
	}

	dotq = Jsvd.solve(b);

	for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}

}

void CTRKin::ApplyKinematicControlNullspace(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dq, double* q)
{
	::Eigen::VectorXd dotq(5);
	::Eigen::Matrix<double, 6, 5> Jtemp = J;
	Jtemp.col(0) *= M_PI / 180.0;
	Jtemp.col(1) *= M_PI / 180.0;
	Jtemp.col(3) *= M_PI / 180.0;

	::Eigen::Matrix<double, 3, 5> Jp;
	::Eigen::Matrix<double, 3, 5> Jo;

	::Eigen::Matrix<double, 3, 3> tmpMat;
	::Eigen::Matrix<double, 5, 5> IdMat;
	IdMat.setIdentity();

	Jp = Jtemp.block(0,0, 3, 5);
	Jo = Jtemp.block(3,0, 3, 5);

	tmpMat = (Jp * Jp.transpose());
	for (int i = 0; i < 3; ++i)
		tmpMat(i, i) += 0.001;
	//Eigen::Matrix<double,3,1> sv;
	//Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> Jsvd(tmpMat,Eigen::ComputeThinU | Eigen::ComputeThinV);
	//sv = Jsvd.singularValues();
	//::std::cout << "condition number:" << sv[0]/sv[2] << ::std::endl;
	//position control
	dotq = 1.0 * Jp.transpose() * (Jp * Jp.transpose()).inverse() * err.block(0, 0, 3, 1);

	//orientation in the nullspace
	//if (tmpMat.determinant() < 1.7)
	double orientationGain = 1000.0;
	dotq += orientationGain*( IdMat - Jp.transpose() * tmpMat.inverse() * Jp) * Jo.transpose() * err.block(3, 0, 3, 1);
	//::std::cout << "dotq:" << dotq.transpose() << ::std::endl;

	dotq *= 0.1;

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

	//::std::cout << dotq.transpose() << ::std::endl;

	for(int i=0; i<5; i++)	{	dq[i] = dotq(i,0);	}

}
