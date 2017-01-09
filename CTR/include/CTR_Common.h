
// ----------------------------------------------------- //
// CKim - This header file contains common struct 
// definition used in the CTR
// ----------------------------------------------------- //

#pragma once	// CKim - Same as include guards

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduHapticDevice.h>
#include <queue>
#include <Eigen/Dense>

// CKim - Struct for haptic device events
typedef struct HapticEventData
{
	int eventId;
	double refMat[16];

} HapticEventData;

// CKim - Struct for storing haptic device state. 
typedef struct HapticDeviceState
{
	// CKim - Configuration of the stylet, force by the device, any error data
	double		tfMat[16];	
	double		Force[3];
	HDErrorInfo err;

	// George - for camera frame velocity control
	double	position[3];
	double  previousPosition[3];
	// CKim - Storage for the event data
	std::queue<HapticEventData> eventQueue;

	// CKim - Proxy position and other variables for teleoperation
	double slavePos[3];		bool forceFlag;		double forceMag;

	// CKim - Initializer
	HapticDeviceState()
	{
		for(int i=0; i<3; i++)	{	Force[i] = slavePos[i] = 0;		}
		for(int i=0; i<16; i++)	{	tfMat[i] = 0;		}
		forceFlag = false;		forceMag = 0.0;
	}

} HapticDeviceState;


// CKim - Struct containing all robot state
typedef struct CTR_status
{
	// CKim - current / commanded motor position
	double	currMotorCnt[7];		double	tgtMotorCnt[7];		double tgtMotorVel[7];

	// CKim - current / commanded joint angle
	double	currJang[5];			double	tgtJang[5];

	// CKim - current / commanded tip configuration ( = position and direction)
	double	currTipPosDir[6];		double	tgtTipPosDir[6];

	// CKim - current balanced pair tip configuration ( = position and direction)
	double	bpTipPosDir[6];	

	double initJAngLWPR[5];
	double jAngLWPR[5];
	double posOrLWPR[6];
	double currMotorCntLWPR[7];
	double tgtJangLWPR[5];
	double tgtMotorCntLWPR[7];
	double currTipPosDirLWPR[6];
	bool isInLimitLWPR;

	double haptic_velocity[3];
	// CKim - Flags for each amp status
	int	errFlag[7];

	// CKim - Inverse Kinematics parameters
	bool invKinOK;		bool limitOK;	double initJang[5];		
	double condNum;		double invKinErr[2];

	// CKim - coordinate transforms - 4 by 4 matrix stored columnwise
	double M_T0[16];	double	refTipPosDir[6];	
	
	// CKim - Haptic device state
	HapticDeviceState hapticState;

	// CKim - Constructor
	CTR_status()
	{
		// CKim - Initialize state parameters
		for(int i=0; i<5; i++)	{	initJang[i] = tgtJang[i] = currJang[i] = initJAngLWPR[i] = 0;										}
		for(int i=0; i<7; i++)	{	tgtMotorCnt[i] = currMotorCnt[i] = tgtMotorVel[i] = errFlag[i] = 0;				}
		for(int i=0; i<6; i++)	{	tgtTipPosDir[i] = currTipPosDir[i] = bpTipPosDir[i] = refTipPosDir[i] = 0;	}
		for(int i=0; i<16; i++)	{	M_T0[i] = 0;																}
		for(int i=0; i<4; i++)	{	M_T0[5*i] = 1;}
		for(int i = 0; i < 5; ++i) { jAngLWPR[i] = 0;}
		for(int i = 0; i < 6; ++i) { posOrLWPR[i] = 0;}
		invKinOK = limitOK = false;
	}

	// CKim - Mode of operation. 0: Idle, 1: Teleoperating,	2: direct command input
	bool isTeleOpMoving;		

	// CKim - Timer
	long int loopTime;

	// CKim - EM tracker data and registration matrix
	double				emMat[4][4];
	double				sensedTipPosDir[6];

	double				solvedTipPosDir[6];
	int exitCond;

	double gain;


} CTR_status;


// CKim - Structure for storing command
typedef struct CTR_cmd
{
	// CKim - current / commanded motor position
	int cmdType;	double para[10];

} CTR_cmd;


