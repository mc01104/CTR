#include "StdAfx.h"

#include "LWPRKinematics.h"

#include "Utilities.h"
#include "HTransform.h"

#include <iostream>




LWPRKinematics::LWPRKinematics(const ::std::string& pathToForwardModel, const ::std::string& pathToInverseModel):
	CTRKin(),
	forwardModel(pathToForwardModel.c_str()),
	inverseModel(pathToInverseModel.c_str()),
	maxIterations(1000),
	step(1),
	maxPositionError(1.0),
	maxOrientationError(3.0)
{
}

LWPRKinematics::~LWPRKinematics()
{
}

bool
LWPRKinematics::TipFwdKin(const double* jAng, double* posOrt)
{
	::std::vector< double> inputData(jAng, jAng + this->forwardModel.nIn());

	
	//try
	//{
	//WaitForSingleObject(m_hFACMutex,INFINITE);
		// not sure if I need that - until another solution is found it's necessary
		this->checkInputData(inputData);

		::std::vector<double> outputData = this->forwardModel.predict(inputData, 0.0001);

		::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
		//printVector(outputData);
		this->compensateForRigidBodyMotion(jAng, outputData.data(), posOrt);
		
		return true;
	//}
	//catch(LWPR_Exception& e)
	//{
	//	::std::cout << ::std::string(e.getString()) << ::std::endl;
	//	return false;
	//}
		//ReleaseMutex(m_hFACMutex);
}

bool
LWPRKinematics::TipFwdKin2(const double* jAng, double* posOrt)
{
	::std::vector< double> inputData(jAng, jAng + this->forwardModel.nIn());

	//try
	//{

		// not sure if I need that - until another solution is found it's necessary
		this->checkInputData(inputData);

		::std::vector<double> outputData = this->forwardModel.predict(inputData, 0.0001);

		::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
		//printVector(outputData);
		this->compensateForRigidBodyMotion(jAng, outputData.data(), posOrt);
		
		return true;
	//}
	//catch(LWPR_Exception& e)
	//{
	//	::std::cout << ::std::string(e.getString()) << ::std::endl;
	//	return false;
	//}

}

bool
LWPRKinematics::InverseKinematics(const double* posOrt, double* jAng)
{	
	
	double posOrtFinal[6] = {0};
	this->compensateForRigidBodyMotionInverse(jAng, posOrt, posOrtFinal);

	::std::vector< double> inputData(posOrtFinal, posOrtFinal + this->inverseModel.nIn());

	try
	{

		::std::vector<double> outputData = this->inverseModel.predict(inputData);

		jAng = outputData.data();

		return true;

	}
	catch(LWPR_Exception& e)
	{
		::std::cout << ::std::string(e.getString()) << ::std::endl;
		return false;
	}

}

void 
LWPRKinematics::AdaptForwardModel(const double* posOrt, const double* jAng)
{
	::std::vector<double> input_data(jAng, jAng + this->forwardModel.nIn());
		
	double posOrtFinal[6] = {0};
	this->compensateForRigidBodyMotionInverse(jAng, posOrt, posOrtFinal);

	::std::vector<double> output_data(posOrtFinal, posOrtFinal + this->forwardModel.nOut());

	this->adaptModel(this->forwardModel, input_data, output_data);

}

void 
LWPRKinematics::AdaptInverseModel(const double* posOrt, const double* jAng)
{
	double posOrtFinal[6] = {0};
	this->compensateForRigidBodyMotionInverse(jAng, posOrt, posOrtFinal);

	::std::vector<double> input_data(posOrtFinal, posOrtFinal + this->inverseModel.nIn());
	::std::vector<double> output_data(jAng, jAng + this->inverseModel.nOut());

	this->adaptModel(this->inverseModel, input_data, output_data);
}

void
LWPRKinematics::adaptModel(LWPR_Object& model, const ::std::vector< double>& input_data, const ::std::vector< double>& output_data)
{
	try
	{
		model.update(input_data, output_data);

	}
	catch(LWPR_Exception& e)
	{
		::std::cout << ::std::string(e.getString()) << ::std::endl;
	}

}


void
LWPRKinematics::setForgettingFactor(double* ffactor)
{
	this->forwardModel.initLambda(ffactor[0]);
	this->forwardModel.finalLambda(ffactor[1]);

}

void 
LWPRKinematics::compensateForRigidBodyMotion(const double* jAng, const double* posOrt, double* posOrtFinal)
{

	::Eigen::Matrix3d rotation = RotateZ(jAng[3]);
	::Eigen::Vector3d translation;
	
	translation << 0, 0, jAng[4];
	
	HTransform baseTransform(rotation, translation);

	::std::vector<double> finalPosition;
	HTransform::applyHTransform(baseTransform, ::std::vector<double> (posOrt, posOrt + 3), finalPosition);

	memcpy(posOrtFinal, finalPosition.data(), 3 * sizeof(double));

	::std::vector<double> finalOrientation;
	HTransform::applyRotation(baseTransform, ::std::vector<double> (posOrt + 3, posOrt + 6), finalOrientation);

	finalOrientation /= norm2(finalOrientation);
	double a = norm2(finalOrientation);

	memcpy(posOrtFinal + 3, finalOrientation.data(), 3 * sizeof(double));

	// 3. Transform the result by rotation about z axis by a1 and translation along z axis by L1
/*	double a1 = jAng[3];		double L1 = jAng[4];
	double p[3], v[3];
	double val;

	memcpy(p, posOrt, 3 * sizeof(double));
	memcpy(v, posOrt + 3, 3 * sizeof(double));
	
	val = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= val;		v[1] /= val;		v[2] /= val;
	posOrtFinal[0] = cos(a1)*p[0] - sin(a1)*p[1];		posOrtFinal[3] = cos(a1)*v[0] - sin(a1)*v[1];
	posOrtFinal[1] = sin(a1)*p[0] + cos(a1)*p[1];		posOrtFinal[4] = sin(a1)*v[0] + cos(a1)*v[1];
	posOrtFinal[2] = p[2] + L1;							posOrtFinal[5] = v[2];
	*/
}

void LWPRKinematics::compensateForRigidBodyMotionInverse(const double* jAng, const double* posOrt, double* posOrtFinal)
{

	::Eigen::Matrix3d rotation = RotateZ(jAng[3]);
	::Eigen::Vector3d translation;
	
	translation << 0, 0, jAng[4];
	
	HTransform baseTransform(rotation, translation);

	::std::vector<double> finalPosition;
	HTransform::applyHTransform(baseTransform.inverse(), ::std::vector<double> (posOrt, posOrt + 3), finalPosition);

	memcpy(posOrtFinal, finalPosition.data(), 3 * sizeof(double));

	::std::vector<double> finalOrientation;
	HTransform::applyRotation(baseTransform.inverse(), ::std::vector<double> (posOrt + 3, posOrt + 6), finalOrientation);

	memcpy(posOrtFinal + 3, finalOrientation.data(), 3 * sizeof(double));

}

void
LWPRKinematics::SaveModel()
{
	::std::string filename = "../models/" + getDateString() + "_adapted.bin";
	
	try
	{
		this->forwardModel.writeBinary(filename.c_str());
	}
	catch(LWPR_Exception& e)
	{
		::std::cout << ::std::string(e.getString()) << ::std::endl;
	}
}


void LWPRKinematics::evaluateNewtonRaphson(const double* jAng, const double* offset, double* fValue)
{

	double posOrt[6] = {0, 0, 0, 0, 0, 0};
	this->TipFwdKin2(jAng, posOrt);

	double sum = 0;
	for (int i = 0; i < 3; ++i)
	{
		fValue[i] = (posOrt[i] - offset[i]);
		sum +=  posOrt[i + 3] * offset[i + 3];
	}
	fValue[3] =  this->maxPositionError/(1-cos(this->maxOrientationError * M_PI/180.0))*(1 - sum);

	if (abs(fValue[3]) < 0.000001) {fValue[3] = 0.0;}

}


void LWPRKinematics::computeJacobian(const double* jAng, const double* offset, Eigen::Matrix<double,4,5>& J)
{
	// CKim - Numerically evalaluate the jacobian dy/dx
	Eigen::Matrix<double,4,1> Fxo;		Eigen::Matrix<double,4,1> Fx;
	
	double x[5];	double dx = FLT_EPSILON;

	double fOriginal[4];
	double fPerturbed[4];

	this->evaluateNewtonRaphson(jAng, offset, fOriginal);
	Fxo = Eigen::Map<Eigen::Matrix<double,4,1>>(fOriginal);

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
		
		this->evaluateNewtonRaphson(x, offset, fPerturbed);
		Fx = Eigen::Map<Eigen::Matrix<double,4,1>>(fPerturbed);

		for(int i=0; i<4; i++)	{	J(i,col) = (Fx(i,0) - Fxo(i,0))/dx;	}
	}


}

void LWPRKinematics::InverseKinematicsLSQ(const double* tgtPosOrt, const double* init, double* jAng, double* Err, int& exitCond)
{
	//::std::cout << "goerge" << ::std::endl;
	int iter = 0;	int maxiter = 100;	double eps = 0.001;		double lambda = 0.001;		double stepSz = 1.0;
	
	Eigen::Matrix<double,4,5> J;		Eigen::Matrix<double,4,1> fx;		Eigen::Matrix<double,4,1> fxnew;		
	Eigen::Matrix<double,5,1> update;	Eigen::Matrix<double,5,1> b;		Eigen::Matrix<double,5,5> JtJ;
	Eigen::Matrix<double,5,5> A;		double temp[5];						
	
	exitCond = 0;

	double m_MaxOrtErr = this->maxOrientationError;
	double m_MaxPosErr = this->maxPositionError;

	double fValue[4] = {0};
	double fxnewValue[4] = {0};

	for(int i=0; i<5; i++)	{	jAng[i] = init[i];		}
	
	// CKim - Iterate
	for(iter = 0; iter < maxiter; iter++)
	{
		//::std::cout << iter << std::endl;
		//// CKim - Evaluate function at curent joint angle
		this->evaluateNewtonRaphson(jAng, tgtPosOrt, fValue);
		fx = Eigen::Map<Eigen::Matrix<double,4,1>>(fValue);

		Err[0] = fx.norm(); 
		Err[1] = sqrt( fx(0,0)*fx(0,0) + fx(1,0)*fx(1,0) + fx(2,0)*fx(2,0) );
		Err[2] = acos(1.0 - fx(3,0)*(1-cos(m_MaxOrtErr*3.141592/180.0))/m_MaxPosErr)*180.0/3.141592;

		// CKim - Exit if distance is less than threshold
		if(Err[0] < eps)	{	exitCond = 1;	break;		}
	
		// CKim - Numerically evalaluate the jacobian dy/dx
		this->computeJacobian(jAng, tgtPosOrt, J);

		// CKim - Find the update direction. update = - inv ( (JtJ + lambda*diag(JtJ) ) Jt*fx
		b = -1.0 * J.transpose()*fx;
		JtJ = J.transpose()*J;		A = JtJ;
		for(int i=0; i<5; i++)	{	A(i,i) += lambda*JtJ(i,i);	}

		// CKim - Calculate Update step. Use JacobiSVD.solve to get least square solution
		Eigen::JacobiSVD<Eigen::Matrix<double,5,5>> Jsvd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);
		update = Jsvd.solve(b);

//#ifdef _DEBUG
//	::std::cout << "Iteration:" << iter << ::std::endl;
//	::std::cout << "Joint Angle" << ::std::endl;
//	printCArray(jAng, 5);
//	//::std::cout << "Position:" << Err[1] << ::std::endl;
//	//::std::cout << "Orientation:" << Err[2] << ::std::endl;
//	//::std::cout << "update" << ::std::endl;
//	//::std::cout << update;
//	//::std::cout << "jacobian" <<  ::std::endl;
//	//::std::cout << J << ::std::endl;
//	::std::cout << "fValue" << ::std::endl;
//	::std::cout << fx.squaredNorm()   << ::std::endl;
//	//::std::cout << ::std::endl;
//#endif
		// CKim - If the magnitude of the update direction is small, exit
		if(update.norm() < 0.001)	{	exitCond = 2;	break;		}

		// CKim - Decide how much in the update direction we should move. Full (1) to minimum (0.1)
		// this depends on if |error|^2 is decreasing
		stepSz = 1.0;	

		while(1)
		{
			for(int i=0; i<5; i++)	{	temp[i] = jAng[i] + stepSz*update(i,0);	}
		
			// CKim - Evalualte distance at new updated point
			this->evaluateNewtonRaphson(temp, tgtPosOrt, fxnewValue);
			fxnew = Eigen::Map<Eigen::Matrix<double,4,1>>(fxnewValue);
//#ifdef _DEBUG
//	::std::cout << "xNew" << ::std::endl;
//	printCArray(temp, 5);
//	::std::cout << "fNew" << ::std::endl;
//	::std::cout << fxnew.squaredNorm()   << ::std::endl;
//	::std::cout << ::std::endl;
//
//#endif
			// CKim - If the norm^2 of the function decreases, accept the step
			if( fxnew.squaredNorm() <= fx.squaredNorm())
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
