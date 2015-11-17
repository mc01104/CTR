/**
  *@brief Class for kinematic modeling using LWPR
  *
  * This class is used to implement a kinematic model for the Concentric Tube Robot. The model is based on
  * Locally Weighted Projection Regression (LWPR) as described in Vijaykumar et al (2005)
  * http://homepages.inf.ed.ac.uk/svijayak/publications/vijayakumar-TRUoE2005.pdf
  *
  *@author Georgios Fagogenis
  *@bug no known bugs at the moment
  */

#pragma once

#include "CTRKin.h"
#include <Eigen/Dense>
#include <string>

#include "lwpr.hh"

class LWPRKinematics : public CTRKin
{

	LWPR_Object forwardModel;
	LWPR_Object inverseModel;

	int maxIterations;

	double step;

	double maxPositionError;
	double maxOrientationError;

public:

	/**
	 *@brief - Constructor of LWPR kinematic model for the Contentric Tube Robot
	 */
	LWPRKinematics(const ::std::string& pathToForwardModel = "default_path_here", 
				   const ::std::string& pathToInverseModel = "default_path_here");

	/**
	 *@brief - Destructor of LWPR kinematic model for the Contentric Tube Robot
	 */
	~LWPRKinematics();

	/**
	  *@brief - compute forward kinematics
	  *@param - joint angles
	  *@param - tip position
	  */
	virtual bool TipFwdKin(const double* jAng, double* posOrt);	
	bool TipFwdKin2(const double* jAng, double* posOrt);	

	/**
	  *@brief - compute inverse kinematics using the inverse LWPR model
	  *@param - tip position
	  *@param - joint angles (result)
	  *@param - least square error
	  *@param - exit condition
	  */
	bool InverseKinematics(const double* posOrt, double* jAng);

	/**
	  *@brief adapt LWPR based on incoming information
	  *@param[in] tip position and orientation
	  *@param[in] joint angles
	  */
	void AdaptForwardModel(const double* posOrt, const double* jAng);

	/**
	  *@brief adapt LWPR based on incoming information
	  *@param[in] tip position and orientation
	  *@param[in] joint angles
	  */
	void AdaptInverseModel(const double* posOrt, const double* jAng);

	void setForgettingFactor(double* ffactor);

	const LWPR_Object& GetForwardModel() { return forwardModel;};

	void SaveModel();

	int getWorkspaceDim() {return this->forwardModel.nOut();};

	int getJoinspaceDim() {return this->forwardModel.nIn() + 2;};

	virtual void InverseKinematicsLSQ(const double* tgtPosOrt, const double* init, double* jAng, double* Err, int& exitCond);
private:

	// Copy operation is not allowd at this point - I will enable this feature if necessary
    LWPRKinematics(LWPRKinematics* lwprKinematics);
	LWPRKinematics(LWPRKinematics& lwprKinematics);
	
	// Neither copy through assignment
	LWPRKinematics& operator = (const LWPRKinematics& rhs);
	
	/**
	  *@brief adapt the model based on input-output data samples
	  *@param model to be adapted
	  *@param[in] input data
	  *@param[in] output data
	  */
	void adaptModel(LWPR_Object& model, const ::std::vector< double>& input_data, const ::std::vector< double>& output_data);

	/**
	  *@brief base rotation and translation in forward kinematics computation
	  *@param[in] joint angles
	  *@param[in] relative tip position and orientation
	  *@param[out] tip position and orientation in the robot frame
	  */
	void compensateForRigidBodyMotion(const double* jAng, const double* posOrt, double* posFinal);

	/**
	  *@brief remove the effect of base rotation and translation to compute the relative tip position and orientation
	  *@param[in] joint angles
	  *@param[in] tip position and orientation in the robot frame
	  *@param[out] relative tip position and orientation
	  */
	void compensateForRigidBodyMotionInverse(const double* jAng, const double* posOrt, double* posOrtFinal);


	void evaluateNewtonRaphson(const double* jAng, const double* offset, double* fValue);

	void computeJacobian(const double* x, const double* offset, Eigen::MatrixXd& J);

	void computeJacobian(const double* x, const double* offset, Eigen::Matrix<double,4,5>& J);

	// HACK to make LWPR work --> need to solve it differently
	template<class T>
	void checkInputData(T& inputData)
	{
		inputData[0] = 0.98 * atan2(sin(inputData[0]), cos(inputData[0]));
		inputData[1] = 0.98 * atan2(sin(inputData[1]), cos(inputData[1]));

		if (inputData[2] < 0) {inputData[2] = 1;}
		if (abs(inputData[2]) > 80) {inputData[2] = 80;}
	}

};