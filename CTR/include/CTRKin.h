#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

// CKim - Eigen Header. Located at "C:\Chun\ChunLib"

#include <Eigen/Dense>

class ChunTimer;

#define L31_MAX 86.39	// CKim - pi/2*55. Maximum protrusion of third tube from balanced pair
#define L31_MIN 3.00	// CKim - Minimum protrusion of third tube from balanced pair
//#define L31_MAX 35
typedef ::std::vector<double> DVec;

class CTRKin
{

public:
	::std::ofstream os;
	::std::string fName;

	CTRKin(const ::std::string& modelStr, int modelInputDim = 3);
	virtual ~CTRKin();
	
	void ReInitializeEstimator();
	void ReInitializeModel();

	void SetInvKinThreshold(const double MaxPosErr, const double MaxOrtErr); 
	void GetInvKinThreshold(double& MaxPosErr, double& MaxOrtErr); 

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

	// CKim - Evaluate Forward kinematics and Jacobian using current kinematics model
	void EvalCurrentKinematicsModel(const double* jAng, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ);
	//void EvalCurrentKinematicsModel_NEW(const double* jAng, const double* tgtPosDir, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ);
	virtual void EvalCurrentKinematicsModelNumeric(const double* jAng, double* predTipPosDir, Eigen::MatrixXd& J, bool evalJ);


	// CKim - Control law for closed loop inverse kinematics control. 'tgtMotorVel'
	void ApplyKinematicControl(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dotq);
	void ApplyKinematicControlNullspace(const Eigen::MatrixXd& J, const Eigen::MatrixXd& err, double* dq, double* q);
	double m_forgettingFactor;

protected:

	// CKim - read the file that contains 'functional approximation coefficients' (FAC)
	// functional approximations are 125 term fourier series for each translation and orientation element px[], py[] ...
	bool readCTR_FAC_file(std::string fileName,  double px[], double py[],  double pz[],  double ox[],  double oy[],  double oz[]);
	bool readCTR_FAC_file(std::string fileName,  DVec& px, DVec& py,  DVec& pz,  DVec& ox,  DVec& oy,  DVec& oz);

	void EvalF_RootFinding(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F);
	void EvalJ_RootFinding(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,5>& J);

	virtual void EvalF_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,1>& F);
	virtual void EvalJ_LSQ(const double* jAng, const double* tgtPosOrt, const Eigen::MatrixXd& Coeff, Eigen::Matrix<double,4,5>& J);

	virtual void TipFwdKinEx(const double* jAng, const Eigen::MatrixXd& Coeff, double* posOrt);
	
	void AllocateCoefficientMatrices();

	void GetFAC(Eigen::MatrixXd& Coeff); 

	int modelOrder, coeffSize;
	int modelInputDim;

	// CKim - Arrays for storing coefficients...
	//double *m_Tip_px, *m_Tip_py, *m_Tip_pz;
	//double *m_Tip_ox, *m_Tip_oy, *m_Tip_oz;
	DVec m_Tip_px, m_Tip_py, m_Tip_pz;
	DVec m_Tip_ox, m_Tip_oy, m_Tip_oz;

	double *m_BP_px, *m_BP_py, *m_BP_pz;
	double *m_BP_ox, *m_BP_oy, *m_BP_oz;

	// CKim - Array used in recursive least square
	Eigen::MatrixXd F[6];		Eigen::MatrixXd m_tmpMat;		Eigen::MatrixXd Fzero;
	Eigen::MatrixXd Fold;
	// CKim - Mutex for protecting shared variable
	HANDLE m_hFACMutex;		

	// CKim - Inverse kinematics threshold
	double m_Thresh;		double m_MaxPosErr;		double m_MaxOrtErr;

	double *A, *B, *C;
	double *AUpdated, *BUpdated, *CUpdated;

};
