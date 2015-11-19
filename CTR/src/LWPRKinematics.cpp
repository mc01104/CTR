#include "StdAfx.h"

#include "LWPRKinematics.h"

#include "Utilities.h"
#include "HTransform.h"

#include <iostream>




LWPRKinematics::LWPRKinematics(const ::std::string& pathToForwardModel):
	CTRKin()
{
	this->forwardModel = new LWPR_Object(pathToForwardModel.c_str());
	this->forwardModelforInverse = new LWPR_Object(pathToForwardModel.c_str());

	this->m_hLWPRMutex = CreateMutex(NULL,false,"LWPR_Mutex");
	this->m_hLWPRInvMutex = CreateMutex(NULL,false,"LWPRInv_Mutex");

	//::std::ofstream metaData;
	//::std::string name = GetDateString() + "-Adapt_Parameters.txt";
	//metaData.open(name);

	forwardModel->updateD(true);
	forwardModel->useMeta(true);
	forwardModel->metaRate(0.1);
	forwardModel->setInitAlpha(0.001);
	forwardModel->wPrune(1.0);
	forwardModel->initLambda(0.9995);
	forwardModel->finalLambda(0.9995);
	forwardModel->tauLambda(0.1);

	/*double normD[6] = {1, 1, 1, 1, 1, 1};
	forwardModel->normOut(doubleVec(normD, normD + 6));*/

	//metaData << "Update_D:" << forwardModel->updateD() << ::std::endl;
	//metaData << "Use_meta:" << forwardModel->useMeta() << ::std::endl;
	//metaData << "Meta_rate:" << forwardModel->metaRate() << ::std::endl;
	//metaData << "Init_alpha:" << 0.001 << ::std::endl;
	//metaData << "initLambda:" << forwardModel->initLambda() << ::std::endl;
	//metaData << "finalLambda:" << forwardModel->finalLambda() << ::std::endl;
	//metaData << "tauLambda:" << forwardModel->tauLambda() << ::std::endl;

	//metaData.close();
}


LWPRKinematics::~LWPRKinematics()
{
	//this->SaveModel();

	delete this->forwardModel;
}

bool
LWPRKinematics::TipFwdKin(const double* jAng, double* posOrt)
{
	::std::vector< double> inputData(jAng, jAng + this->forwardModel->nIn());

	this->CheckJointLimits(inputData);
	
#ifdef _SCALED_
	inputData[2] = inputData[2]/L31_MAX ;
#endif
	
	WaitForSingleObject(this->m_hLWPRMutex,INFINITE);
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
	inputData[2] = inputData[2]/L31_MAX;
#endif

	double posOrtFinal[6] = {0};
	this->CompensateForRigidBodyMotionInverse(jAng, posOrt, posOrtFinal);

	::std::vector<double> output_data(posOrtFinal, posOrtFinal + this->forwardModel->nOut());

	WaitForSingleObject(this->m_hLWPRMutex,INFINITE);
	for (int i = 0; i < 2; i++)
		this->forwardModel->update(inputData, output_data);
	//Sleep(100);
	ReleaseMutex(this->m_hLWPRMutex);

	//WaitForSingleObject(this->m_hLWPRInvMutex,INFINITE);
	////this->AdaptModel(this->forwardModelforInverse, input_data, output_data);
	//for (int i = 0; i < 10; ++i)
	//	this->forwardModelforInverse.update(input_data, output_data);
	//ReleaseMutex(this->m_hLWPRInvMutex);
}


void
LWPRKinematics::SetForgettingFactor(double* ffactor)
{
	//this->forwardModel.initLambda(ffactor[0]);
	//this->forwardModel.finalLambda(ffactor[1]);
	//this->forwardModel.tauLambda(ffactor[2]);
	//this->forwardModelforInverse.initLambda(ffactor[0]);
	//this->forwardModelforInverse.finalLambda(ffactor[1]);
	//this->forwardModelforInverse.tauLambda(ffactor[2]);
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
	double a = Norm2(finalOrientation);

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
	
	TipFwdKinInv(jAng, posOrt);
	
	for(int i = 0; i < 3; i++)	
	{	
		F(i,0) = posOrt[i] - tgtPosOrt[i];				
		sum += (posOrt[i+3] * tgtPosOrt[i+3]);
	}

	F(3,0) = pmax/(1 - cos(thmax)) * (1 - sum);
}


void
LWPRKinematics::TipFwdKinInv(const double* jAng, double* posOrt)
{
	::std::vector< double> inputData(jAng, jAng + this->forwardModel->nIn());

	this->CheckJointLimits(inputData);

#ifdef _SCALED_
	inputData[2] = inputData[2]/L31_MAX;
#endif

	//WaitForSingleObject(this->m_hLWPRInvMutex,INFINITE);
	::std::vector<double> outputData = this->forwardModelforInverse->predict(inputData, 0.001);
	//ReleaseMutex(this->m_hLWPRInvMutex);

	::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
	
	this->CompensateForRigidBodyMotion(jAng, outputData.data(), posOrt);
		
}

