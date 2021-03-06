#include "StdAfx.h"

#include "LWPRKinematics.h"

#include "Utilities.h"
#include "HTransform.h"

#include <iostream>




LWPRKinematics::LWPRKinematics(const ::std::string& pathToForwardModel):
	CTRKin(""), modelPath(pathToForwardModel)
{
	this->forwardModel = new LWPR_Object(pathToForwardModel.c_str());
	this->originalModel = new LWPR_Object(pathToForwardModel.c_str());

	this->m_hLWPRMutex = CreateMutex(NULL,false,"LWPR_Mutex");

	double ffactor[3] = {1.0, 1.0, 0.1};
	this->forwardModel->updateD(true);
	
	this->SetForgettingFactor(ffactor);

	this->directionInJointspace = new double[this->forwardModel->nIn()];
	this->inputArray = new double[this->forwardModel->nIn()];
}


LWPRKinematics::~LWPRKinematics()
{
	delete this->forwardModel;
	delete[] directionInJointspace;
}

bool
LWPRKinematics::TipFwdKin(const double* jAng, double* posOrt)
{
	::std::vector< double> inputData(jAng, jAng + this->forwardModel->nIn());

	this->CheckJointLimits(inputData);

#ifdef _SCALED_
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/L31_MAX ;
#endif
	
	WaitForSingleObject(this->m_hLWPRMutex, INFINITE);
	::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);
	ReleaseMutex(this->m_hLWPRMutex);

	::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
	
	this->CompensateForRigidBodyMotion(jAng, outputData.data(), posOrt);
		
	return true;

}

void 
LWPRKinematics::AdaptForwardModel(const double* posOrt, const double* jAng)
{
	::std::vector<double> inputData(jAng, jAng + this->forwardModel->nIn());

#ifdef _SCALED_	
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/L31_MAX;
#endif

	double posOrtFinal[6] = {0};

	this->CompensateForRigidBodyMotionInverse(jAng, posOrt, posOrtFinal);

	::std::vector<double> outputData(posOrtFinal, posOrtFinal + this->forwardModel->nOut());

	WaitForSingleObject(this->m_hLWPRMutex, INFINITE);
	// This is to deal with data around the boundaries of the periodic function
	for (int i = 0; i < 1; i++)
	{
		this->forwardModel->update(inputData, outputData);
		inputData[0] += 2 * M_PI;
		this->forwardModel->update(inputData, outputData);
		inputData[0] -= 4 * M_PI;
		this->forwardModel->update(inputData, outputData);
	}
	ReleaseMutex(this->m_hLWPRMutex);

}


void
LWPRKinematics::SetForgettingFactor(double* ffactor)
{
	this->forwardModel->initLambda(ffactor[0]);
	this->forwardModel->finalLambda(ffactor[1]);
	this->forwardModel->tauLambda(ffactor[2]);
}

void
LWPRKinematics::ResetModel()
{
	LWPR_Object* temp = this->forwardModel;

	WaitForSingleObject(m_hLWPRMutex, INFINITE);
	this->forwardModel = this->originalModel;
	ReleaseMutex(m_hLWPRMutex);

	this->originalModel = new LWPR_Object(this->modelPath.c_str());

	delete temp;
}

void 
LWPRKinematics::CompensateForRigidBodyMotion(const double* jAng, const double* posOrt, double* posOrtFinal)
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

	finalOrientation /= Norm2(finalOrientation);

	memcpy(posOrtFinal + 3, finalOrientation.data(), 3 * sizeof(double));

}

void LWPRKinematics::CompensateForRigidBodyMotionInverse(const double* jAng, const double* posOrt, double* posOrtFinal)
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
	::std::string filename = "../models/" + GetDateString() + "_adapted.bin";
	
	try
	{
		this->forwardModel->writeBinary(filename.c_str());
	}
	catch(LWPR_Exception& e)
	{
		::std::cout << ::std::string(e.getString()) << ::std::endl;
	}
}


void LWPRKinematics::EvalF_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F)
{
	double posOrt[6];
	double pmax = m_MaxPosErr;		
	double thmax = m_MaxOrtErr*3.141592/180.0;		
	double sum = 0.0;
	
	//TipFwdKinEx(jAng, posOrt);
	TipFwdKin(jAng, posOrt);

	for(int i = 0; i < 3; i++)	
	{	
		F(i,0) = posOrt[i] - tgtPosOrt[i];				
		sum += (posOrt[i+3] * tgtPosOrt[i+3]);
	}

	F(3,0) = pmax/(1 - cos(thmax)) * (1 - sum);
}

bool 
LWPRKinematics::TipFwdKinEx(const double* jAng, double* posOrt)
{
	::std::vector< double> inputData(jAng, jAng + this->forwardModel->nIn());

#ifdef _SCALED_
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/L31_MAX ;
#endif
	
	::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);

	::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
	
	this->CompensateForRigidBodyMotion(jAng, outputData.data(), posOrt);
		
	return true;
}

void
LWPRKinematics::TipFwdKinJac(const double* jAng, double* posOrt, Eigen::MatrixXd& J, bool evalJ)
{
	double dq = FLT_EPSILON;		
	double q[5];		
	double Fq[6];

	WaitForSingleObject(m_hLWPRMutex, INFINITE);
	TipFwdKinEx(jAng, posOrt);

	if(evalJ)	
	{
		for(int col=0; col<5; col++)
		{
			for(int i=0; i<5; i++)
			{
				if(i==col)
				{
					//dq = FLT_EPSILON*fabs(jAng[i]);
					dq = 0.01;

					//if(dq==0.0) {	dq = FLT_EPSILON;	}
				
					q[i] = jAng[i] + dq;
					dq = q[i] - jAng[i];
				}
				else		{	q[i] = jAng[i];		}
			}
		
			TipFwdKinEx(q, Fq);
			//PrintCArray(Fq, 6);
			for(int i=0; i<6; i++)	{	J(i,col) = (Fq[i] - posOrt[i])/dq;}
		}
		//std::cout << "J = " << J << std::endl;
	}
	//::std::cout << "---------------" << ::std::endl;
	ReleaseMutex(this->m_hLWPRMutex);

	return;
}

bool 
LWPRKinematics::TipFwdKinEx(const double* jAng, const double jAngPrev[], double* posOrt)
{
	this->computeDirection(jAng, jAngPrev, this->directionInJointspace);

	memcpy(this->inputArray, jAng, this->forwardModel->nIn() * sizeof(double));	
	memcpy(&this->inputArray[this->forwardModel->nIn()], this->directionInJointspace, this->forwardModel->nIn() * sizeof(double)); 

	::std::vector< double> inputData(this->inputArray, this->inputArray + 2 * this->forwardModel->nIn());

	this->CheckJointLimits(inputData);

#ifdef _SCALED_
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/L31_MAX ;
#endif
		
	try 
	{
		::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);	

		::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());

		this->CompensateForRigidBodyMotion(jAng, outputData.data(), posOrt);

		return true;

	}
	catch (::std::exception& e)
	{
		::std::cout << e.what() << ::std::endl;
		return false;
	}
}

bool 
LWPRKinematics::TipFwdKin(const double* jAng, const double jAngPrev[], double* posOrt)
{
	this->computeDirection(jAng, jAngPrev, this->directionInJointspace);

	memcpy(this->inputArray, jAng, 3 * sizeof(double));	
	memcpy(&this->inputArray[3], this->directionInJointspace, 3 * sizeof(double)); 

	::std::vector< double> inputData(this->inputArray, this->inputArray + 2 * 3);

	this->CheckJointLimits(inputData);

#ifdef _SCALED_
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/L31_MAX ;
#endif

	try 
	{
		WaitForSingleObject(this->m_hLWPRMutex, INFINITE);
		::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);	
		ReleaseMutex(this->m_hLWPRMutex);

		::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());

		this->CompensateForRigidBodyMotion(jAng, outputData.data(), posOrt);

		return true;

	}
	catch (::std::exception& e)
	{
		::std::cout << e.what() << ::std::endl;
		return false;
	}

}

void 
LWPRKinematics::computeDirection(const double jAng[], const double jAngPrev[], double directionInJointspace[])
{
	::Eigen::VectorXd currentAngles = ::Eigen::Map<::Eigen::VectorXd> (const_cast<double*>(jAng), this->forwardModel->nIn());
	::Eigen::VectorXd previousAngles = ::Eigen::Map<::Eigen::VectorXd> (const_cast<double*>(jAngPrev), this->forwardModel->nIn());

	::Eigen::VectorXd direction = (currentAngles.segment(0, 3) - previousAngles.segment(0, 3));
	
	if (direction.norm() >= 0.0001)
		direction.normalize();

	memcpy(directionInJointspace, direction.data(), 3 * sizeof(double));
}

void 
LWPRKinematics::AdaptForwardModel(const double* posOrt, const double* jAng, const double jAngPrev[])
{
	// i may need thread protection
	this->computeDirection(jAng, jAngPrev, this->directionInJointspace);

	memcpy(this->inputArray, jAng, this->forwardModel->nIn() * sizeof(double));	
	memcpy(&this->inputArray[this->forwardModel->nIn()], this->directionInJointspace, this->forwardModel->nIn() * sizeof(double)); 

	::std::vector< double> inputData(this->inputArray, this->inputArray + 2 * this->forwardModel->nIn());
	
#ifdef _SCALED_	
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/L31_MAX;
#endif

	double posOrtFinal[6] = {0};

	this->CompensateForRigidBodyMotionInverse(jAng, posOrt, posOrtFinal);

	::std::vector<double> outputData(posOrtFinal, posOrtFinal + this->forwardModel->nOut());

	WaitForSingleObject(this->m_hLWPRMutex, INFINITE);
	// This is to deal with data around the boundaries of the periodic function
	for (int i = 0; i < 1; i++)
	{
		this->forwardModel->update(inputData, outputData);
		inputData[0] += 2 * M_PI;
		this->forwardModel->update(inputData, outputData);
		inputData[0] -= 4 * M_PI;
		this->forwardModel->update(inputData, outputData);
	}
	ReleaseMutex(this->m_hLWPRMutex);

}