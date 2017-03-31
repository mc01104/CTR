#pragma once

#include <string>
#include <iostream>
#include <fstream>

// CKim - Eigen Header. Located at "C:\Chun\ChunLib"

#include <Eigen/Dense>

class ChunTimer;

#define L31_MAX 35	// CKim - pi/2*55. Maximum protrusion of third tube from balanced pair
#define L31_MIN 2.00	// CKim - Minimum protrusion of third tube from balanced pair

class CTRKin
{
public:
	CTRKin(void);
	virtual ~CTRKin(void);
	
	// CKim - The tool tip configuration is defined by its position (px, py, pz) and unit vector representing
	// the direction of the needle tip (ox, oy, oz), the robot is 5 dof device and rotation about the
	// needle axis is not controlled

	// CKim - Initialize
	void ReInitializeEstimator();
	void ReInitializeModel();
	::std::ofstream os;

	// CKim - Inverse kinematics parameter setting
	void SetInvKinThreshold(const double MaxPosErr, const double MaxOrtErr); 
	void GetInvKinThreshold(double& MaxPosErr, double& MaxOrtErr); 
	void SetForceGain(double forceGain) {m_forceGain = forceGain;};
	// CKim - Takes current joint angle and returns tool tip position and orientation (= desired tool tip dir) 
	virtual bool TipFwdKin(const double* jAng, double* posOrt);	// CKim - My implementation

	// CKim - Inverse Kinematics by root finding. exitCond 1: lsqErr below threshold, 2: update magnitude too small, 3: Not decreasing, 4: max iteration
	void InverseKinematicsRootFinding(const double* posOrt, const double* init, double* jAng, double& lsqErr, int& exitCond);

	// CKim - Inverse Kinematics by least squares. exitCond 1: lsqErr below threshold, 2: update magnitude too small, 3: Not decreasing, 4: max iteration
	virtual void InverseKinematicsLSQ(const double* posOrt, const double* init, double* jAng, double* Err, int& exitCond);

	// CKim - Takes current joint angle and returns tip position and orientation of Balanced Pair
	bool BalancedPairFwdKin(const double* jAng, double* posOrt);

	// CKim - Update the coefficients of the functional approximation
	void UpdateFAC(const double jAng[5], const double measTipPosDir[6], double predTipPosDir[6], bool doUpdate);

	// CKim - Update Initial matrix used in Adaptive Update
	void UpdateInitM(const double jAng[5], bool invert = false);

	void	conditionMatrix(::Eigen::Matrix2d& J_original, double conditionNumberDes, ::Eigen::MatrixXd& J_conditioned, double& condition_number_new);
	// CKim - Evaluate Forward kinematics and Jacobian using current kinematics model
	void EvalCurrentKinematicsModel(const double* jAng, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ);
	void EvalCurrentKinematicsModel_NEW(const double* jAng, const double* tgtPosDir, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ);
	virtual void EvalCurrentKinematicsModelNumeric(const double* jAng, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ);


	// CKim - Control law for closed loop inverse kinematics control. 'tgtMotorVel'
	void ApplyKinematicControl(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dotq,double* q);
	void ApplyKinematicControl_NEW(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dotq);
	void ApplyKinematicControlNullspace(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dotq, double* q);
	void ApplyHybridPositionForceControl(const ::Eigen::MatrixXd& J, const ::Eigen::MatrixXd& err, const ::Eigen::MatrixXd& desiredForce, const ::Eigen::MatrixXd& actualForce, double* dq, double* q);

	double m_forgettingFactor;

protected:

	// CKim - read the file that contains 'functional approximation coefficients' (FAC)
	// functional approximations are 125 term fourier series for each translation and orientation element px[], py[] ...
	bool readCTR_FAC_file(std::string fileName,  double px[125], double py[125],  double pz[125],  double ox[125],  double oy[125],  double oz[125]);

	void EvalF_RootFinding(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F);
	void EvalJ_RootFinding(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,5>& J);

	virtual void EvalF_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F);
	virtual void EvalJ_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,5>& J);

	virtual void TipFwdKinEx(const double* jAng, const Eigen::MatrixXd& Coeff, double* posOrt);
	void EvalAnalyticJacobian(const double* jAng, const Eigen::MatrixXd& Coeff, Eigen::MatrixXd& J);

	void GetFAC(Eigen::MatrixXd& Coeff); 

	// CKim - Arrays for storing coefficients...
	double	m_Tip_px[125];		double	m_Tip_py[125];		double	m_Tip_pz[125];
	double	m_Tip_ox[125];		double	m_Tip_oy[125];		double	m_Tip_oz[125];

	double  m_BP_px[125];		double  m_BP_py[125];		double  m_BP_pz[125];
	double  m_BP_ox[125];		double  m_BP_oy[125];		double  m_BP_oz[125];

	// CKim - Array used in recursive least square
	Eigen::MatrixXd F[6];		Eigen::MatrixXd m_tmpMat;		Eigen::MatrixXd Fzero;

	// CKim - Mutex for protecting shared variable
	HANDLE m_hFACMutex;		
	double				m_forceGain;
	// CKim - Inverse kinematics threshold
	double m_Thresh;		double m_MaxPosErr;		double m_MaxOrtErr;
};
