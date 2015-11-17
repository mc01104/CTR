// ---------------------------------------------------------------- //
// CKim - Wrapper class for Phantom Omin. Based on HDU library's
// wrapper for the haptic device
// Last updated : Oct. 28, 2014 
// ---------------------------------------------------------------- //

#pragma once

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduHapticDevice.h>
#include <queue>
#include <sstream>

#include "CTR_Common.h"

class ChunHaptic
{
public:
	ChunHaptic(void);
	~ChunHaptic(void);

	// CKim - Initialize Haptic device
	static void InitDevice();

	// CKim - Start scheduler and begin the loop that updates state
	void StartLoop();

	// CKim - Synchronize state with the robot. Called from the robot thread.
	// From the passed state, robots state will be copied  to haptic device and 
	// the devic state will be copied to the passed state
	void SynchState(CTR_status& state);


private:
	static HHD m_hHD;
	HDSchedulerHandle hSchedule;

	// CKim - Callback function executed by the scheduler
	static HDCallbackCode HDCALLBACK updateCallback(void *data);
	static HDCallbackCode HDCALLBACK synchCallback(void *data);

	static HapticDeviceState m_currentState;
	static HapticDeviceState m_lastState;

	static hduVector3Dd m_refPt;
	static bool teleOpOn;

    enum EventType
    {
        BUTTON_1_DOWN = 0,	BUTTON_1_UP,	BUTTON_2_DOWN,
        BUTTON_2_UP,		BUTTON_3_DOWN,	BUTTON_3_UP,
        DEVICE_ERROR,		NUM_EVENT_TYPES
    };

	// CKim - Process events....
    void handleEvent(ChunHaptic::EventType event);

};



