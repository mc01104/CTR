#include "StdAfx.h"

#include "LWPRKinematics.h"

#include "Utilities.h"
#include "HTransform.h"

#include <iostream>




LWPRKinematics::LWPRKinematics(const ::std::string& pathToForwardModel):
	CTRKin()
{
	this->forwardModel = new LWPR_Object(pathToForwardModel.c_str());

	this->m_hLWPRMutex = CreateMutex(NULL,false,"LWPR_Mutex");

	double ffactor[3] = {1.0, 1.0, 0.1};

}


LWPRKinematics::~LWPRKinematics()
{
	delete this->forwardModel;
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
	
	WaitForSingleObject(this->m_hLWPRMutex,INFINITE);
	::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);
	ReleaseMutex(this->m_hLWPRMutex);

	::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
	
	this->CompensateForRigidBodyMotion(jAng, outputData.data(), posOrt);
		
	return true;

}


//check the scaling
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

	//this is wrong - it doesn't cover all cases
	WaitForSingleObject(this->m_hLWPRMutex,INFINITE);
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



bool 
LWPRKinematics::TipFwdKinEx(const double* jAng, double* posOrt)
{
	::std::vector< double> inputData(jAng, jAng + this->forwardModel->nIn());

	this->CheckJointLimits(inputData);

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

	// TODO: revise the implementation of the Jacobian -> it is very ugly and possible includes reduntant computation
	if(evalJ)	
	{
		for(int col=0; col<5; col++)
		{
			for(int i=0; i<5; i++)
			{
				if(i==col)
				{
					dq = 0.01;

					//if(dq==0.0) {	dq = FLT_EPSILON;	}
				
					q[i] = jAng[i] + dq;
					dq = q[i] - jAng[i];
				}
				else		{	q[i] = jAng[i];		}
			}
		
			TipFwdKinEx(q, Fq);
			for(int i=0; i<6; i++)	{	J(i,col) = (Fq[i] - posOrt[i])/dq;}
		}

	}
	//::std::cout << "---------------" << ::std::endl;
	ReleaseMutex(this->m_hLWPRMutex);

	return;
}

void LWPRKinematics::computeObjectiveFunctionJacobian(const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, ::Eigen::MatrixXd& J)
{
	double epsilon = 0.0001;
	double invEpsilon = 1.0/epsilon;
	
	double f0, fNew, JJ;
	computeObjectiveFunction(targetX, x, t, f0, JJ);

	::Eigen::VectorXd perturbedX = x;

	J.resize(1, x.size());
	for(int i = 0; i < x.size(); ++i)
	{
		perturbedX(i) += epsilon;

		computeObjectiveFunction(targetX, perturbedX, t, fNew, JJ);

		J(0, i) = (fNew - f0) * invEpsilon;

		perturbedX(i) = x(i);
	}
}

void LWPRKinematics::computeObjectiveFunction(const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, double& funVal, double& realError)
{
	::Eigen::VectorXd outX;
	ComputeKinematics(x, outX);

	double phi = -::std::log(1 - (outX.segment(0, 3) - targetX.segment(0, 3)).norm());

	realError = (outX.segment(3, 3) - targetX.segment(3, 3)).norm();

	funVal = t * realError + phi;
}

void LWPRKinematics::solveFirstObjective(const ::Eigen::VectorXd& targetX, ::Eigen::VectorXd& x, double t, double eps, double mu)
{
	double Jcost = 0.0;
	double JcostPrev = 1000.0;
	int iterations = 0;
	int maxIterations = 1000;
	double step = 0.90;

	::Eigen::VectorXd xPrev(x);
	::Eigen::MatrixXd J;
	double realCost = 0;

	computeObjectiveFunction(targetX, x, t, Jcost, realCost);

	while (::std::abs(Jcost - JcostPrev) < 1.e-03  && iterations < maxIterations)
	{
		computeObjectiveFunctionJacobian(targetX, x, t, J);

		xPrev = x;
		x -= step/t * J.transpose() * (J * J.transpose()).inverse() * Jcost;

		this->CheckJointLimits(x);

		JcostPrev = Jcost;
		computeObjectiveFunction(targetX, x, t, Jcost, realCost);

		if (JcostPrev < Jcost)
		{
			step *= 0.8;
			Jcost = JcostPrev;
			x = xPrev;
			continue;
		}

		iterations++;
	}
}


void LWPRKinematics::runOptimizationController(double initialConfiguration[], double goalInTaskSapce[6], double outputConfiguration[])
{
	::Eigen::VectorXd targetX = ::Eigen::Map<::Eigen::VectorXd> (goalInTaskSapce, 6);
	::Eigen::VectorXd configuration = ::Eigen::Map<::Eigen::VectorXd> (initialConfiguration, 5);
	::Eigen::VectorXd currentX;
	::Eigen::MatrixXd J, Jp;

	int iterations = 0;
	int maxIterations = 1000;
	double step = 1.0;

	::Eigen::VectorXd error(6), errorPrev(6);
	double scalingFactors[5] = {M_PI, M_PI, 87, M_PI, 100};
	scaleVector(configuration, scalingFactors);
	ComputeKinematics(configuration, currentX);
	
	error = targetX - currentX;
	::Eigen::VectorXd confPrev = configuration;

	while (error.segment(0, 3).norm() > 1.1 && iterations < maxIterations)
	{
		ComputeJacobian(configuration, J);
		Jp = J.block(0,0,3,5);

		confPrev = configuration;
		configuration += step * Jp.transpose() * (Jp * Jp.transpose()).inverse() * error.segment(0, 3);

		unscaleVector(configuration, scalingFactors);
		CheckJointLimits(configuration);
		scaleVector(configuration, scalingFactors);

		ComputeKinematics(configuration, currentX);

		errorPrev = error;
		error = targetX - currentX;

		if (error.norm() > errorPrev.norm())
		{
			configuration = confPrev;
			step *= 0.8;
			error = errorPrev;
			continue;
		}
		iterations++;
	}

	// check if solution is in the feasible set: if not return --- TODO
	
	double t = 1;
	double mu = 10.0;
	double eps = 0.0001;
	double Jcost = 0.0;


	for (int k = 0; k < maxIterations; ++k)
	{

		solveFirstObjective(targetX, configuration, t, eps, mu);
		
		if (1.0/t < eps) break;

		t *= mu;
	}
	memcpy(outputConfiguration, configuration.data(), configuration.size() * sizeof(double));
}

void LWPRKinematics::unscaleVector(::Eigen::VectorXd& x, double scalingFactors[])
{
	for (int i = 0; i < x.size(); ++i)
		x(i) *= scalingFactors[i];
}

void LWPRKinematics::scaleVector(::Eigen::VectorXd& x, double scalingFactors[])
{
	for (int i = 0; i < x.size(); ++i)
		x(i) /= scalingFactors[i];

}

void LWPRKinematics::scaleVector(double x[], int x_size, double scalingFactors[])
{
	for (int i = 0; i < x_size; ++i)
		x[i] /= scalingFactors[i];
}

bool
LWPRKinematics::ComputeKinematics(const double* jAng, double* posOrt)
{
	double scalingFactors[5] = {M_PI, M_PI, 87, M_PI, 100};

	::std::vector< double> inputData(jAng, jAng + this->forwardModel->nIn());

	for (int i = 0; i < inputData.size(); ++i)
		inputData[i] *= scalingFactors[i];

	this->CheckJointLimits(inputData);

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

bool
LWPRKinematics::ComputeKinematics(const ::Eigen::VectorXd& jAng, ::Eigen::VectorXd& posOrt)
{
	double scalingFactors[5] = {M_PI, M_PI, 87, M_PI, 100};

	::std::vector< double> inputData(jAng.data(), jAng.data() + this->forwardModel->nIn());

	for (int i = 0; i < inputData.size(); ++i)
		inputData[i] *= scalingFactors[i];

	this->CheckJointLimits(inputData);

#ifdef _SCALED_
	inputData[0] /= M_PI;
	inputData[1] /= M_PI;
	inputData[2] = inputData[2]/L31_MAX ;
#endif
	
	::std::vector<double> outputData = this->forwardModel->predict(inputData, 0.001);

	::std::vector<double> orientation = ::std::vector<double> (outputData.begin() + 3, outputData.end());
	
	double posOrtTemp[6] = {0};
	this->CompensateForRigidBodyMotion(jAng.data(), outputData.data(), posOrtTemp);
	
	posOrt.resize(6);
	memcpy(posOrt.data(), posOrtTemp, 6 * sizeof(double));
	return true;
}

bool 
LWPRKinematics::ComputeJacobian(const ::Eigen::VectorXd& configuration, ::Eigen::MatrixXd& J)
{
	J.resize(6, configuration.size());

	double epsilon = 0.00001;
	double invEpsilon = 1.0/epsilon;

	::Eigen::VectorXd currentX, perturbedX;
	this->ComputeKinematics(configuration, currentX);

	::Eigen::VectorXd perturbedConfiguration = configuration;
	for (int i = 0; i < configuration.size(); ++i)
	{
		perturbedConfiguration(i) += epsilon;
		this->ComputeKinematics(perturbedConfiguration, perturbedX);
		J.col(i) = (perturbedX - currentX) * invEpsilon;
		perturbedConfiguration(i) = configuration(i);
	}

	return true;
}


