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

#include "lwpr.hh"

#define _SCALED_

class LWPRKinematics : public CTRKin
{
	LWPR_Object* forwardModel;
	LWPR_Object* originalModel;
	
	HANDLE m_hLWPRMutex;	
	HANDLE m_hLWPRInvMutex;	
public:
	
	/**
	 *@brief - Constructor of LWPR kinematic model for the Contentric Tube Robot
	 */
	LWPRKinematics(const ::std::string& pathToForwardModel = "default_path_here");

	/**
	 *@brief - Destructor of LWPR kinematic model for the Contentric Tube Robot
	 */
	virtual ~LWPRKinematics();

	/**
	  *@brief - compute forward kinematics using LWPR
	  *@param - joint angles
	  *@param - tip position and orientation
	  */
	virtual bool TipFwdKinEx(const double* jAng, double* posOrt);	
	virtual bool TipFwdKin(const double* jAng, double* posOrt);	
	void TipFwdKinJac(const double* jAng, double* posOrt, Eigen::MatrixXd& J, bool evalJ);

	/**
	  *@brief adapt LWPR based on incoming information
	  *@param[in] tip position and orientation
	  *@param[in] joint angles
	  */
	void AdaptForwardModel(const double* posOrt, const double* jAng);
	
	/**
	  *@brief sets the initial and final forgetting factor for the LWPR adaptation together with the annealing parameter. For more information check LWPR's documentation
	  *@param[in] array containing initial forgetting factor, final forgetting factor and annealing parameter 
	  */
	void SetForgettingFactor(double* ffactor);

	/**
	  *@brief returns a copy of the LWPR model foe the forward kinematics
	  */
	const LWPR_Object* GetForwardModel() { return forwardModel;};

	/**
	  *@brief this is used to save the model after online adaptation fore future use
	  */
	void SaveModel();

	/**
	  *@brief returns the dimension of the tip's workspace
	  */
	int GetWorkspaceDim() {return this->forwardModel->nOut();};

	/**
	  *@brief returns the dimension of the full jointspace (including rigid body transformation)
	  */
	int GetJoinspaceDim() {return this->forwardModel->nIn() + 2;};

	//HACK - currently only used because of thread safety -> CHANGE
	//void TipFwdKinInv(const double* jAng, double* posOrt);

	void ResetModel();

	//void InverseKinematicsLSQ(const double* tgtPosOrt, const double* init, double* jAng, double* Err, int& exitCond);
protected:

	// Copy operation is not allowed at this point
    LWPRKinematics(LWPRKinematics* lwprKinematics);
	LWPRKinematics(LWPRKinematics& lwprKinematics);
	
	// Neither copy through assignment
	//LWPRKinematics& operator = (const LWPRKinematics& rhs);
	

	/**
	  *@brief base rotation and translation in forward kinematics computation
	  *@param[in] joint angles
	  *@param[in] relative tip position and orientation
	  *@param[out] tip position and orientation in the robot frame
	  */
	void CompensateForRigidBodyMotion(const double* jAng, const double* posOrt, double* posFinal);

	/**
	  *@brief remove the effect of base rotation and translation to compute the relative tip position and orientation
	  *@param[in] joint angles
	  *@param[in] tip position and orientation in the robot frame
	  *@param[out] relative tip position and orientation
	  */
	void CompensateForRigidBodyMotionInverse(const double* jAng, const double* posOrt, double* posOrtFinal);

	/**
	  *@brief evaluate objective function for least squares optimization, used in inverse kinematics (root finding of forward model for specific workspace position and orientation)
	  *@param[in] current joint angles
	  *@param[in] target position and orientation in the workspace
	  *@param[in] Parameters of Fourier model - not used for LWPR
	  *@param[out] value of objective function
	  */
	//virtual void EvalF_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F);

	/**
	  *@brief Check if the joint values (input to the forward kinematics) are in limit and if not cap their values accordingly
	  *@param values to be checked and (if necessary) adjusted to fall within the joint limits
	  */
	template<class T>
	void CheckJointLimits(T& inputData)
	{
		inputData[0] = 1 * atan2(sin(inputData[0]), cos(inputData[0]));
		inputData[1] = 1 * atan2(sin(inputData[1]), cos(inputData[1]));

		if (inputData[2] < 0) {inputData[2] = L31_MIN;}
		if (abs(inputData[2]) > L31_MAX) {inputData[2] = L31_MAX;}
	}

};