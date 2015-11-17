// -------------------------------------------------------------- //
// CKim - Class encapsulating the EM tracker
// -------------------------------------------------------------- //

#pragma once

#include <ATC3DG.h>
#include <fstream>
#include <Eigen/Dense>

class ChunTracker
{
public:
	ChunTracker(void);
	~ChunTracker(void);

	bool	InitTracker();
	bool	IsInit()	{	return m_isInit;	};

	void	BeginTracking();
	bool	GetTrackerMatrix(double pMat[4][4], unsigned short sensorNum);
	void	GetMeasuredTipPosDir(double measPosDir[6]);

	// CKim - Perform registration from the list of points saved in a file. 
	// Points are acquired by rotating the EM sensor attached on the calibration rig of known geometry
	void	Registration(char* fName, Eigen::Matrix4d& M);

	char	m_errMsg[1024];

private:

	// CKim - Data Structures from Tracker API
	SYSTEM_CONFIGURATION					m_trackerConfig;
	TRANSMITTER_CONFIGURATION				m_xmtrConfig;		
	SENSOR_CONFIGURATION*					m_pSensorConfig;		
	DOUBLE_POSITION_MATRIX_TIME_Q_RECORD*	m_pRecord;


	Eigen::MatrixXd	m_RegMat;
	Eigen::MatrixXd	m_trackerMat;

	bool	m_isInit;

	void	PrintTrackerError(int errCode);
};

