#include "StdAfx.h"
#include "ChunHaptic.h"
#include "Utilities.h"

// CKim - Eigen Header. Located at "C:\Chun\ChunLib"
#include <Eigen/Dense>


HHD			ChunHaptic::m_hHD = 0;
HapticDeviceState	ChunHaptic::m_currentState;
HapticDeviceState	ChunHaptic::m_lastState;
bool		ChunHaptic::teleOpOn = false;
hduVector3Dd ChunHaptic::m_refPt;
ChunTimer ChunHaptic::m_timer;
long	ChunHaptic::m_lastTime; 


ChunHaptic::ChunHaptic(void)
{
	m_filters = new ::RecursiveFilter::MovingAverageFilter[6];

	m_lastTime = m_timer.GetTime();
}


ChunHaptic::~ChunHaptic(void)
{
	// CKim - For cleanup, unschedule callback and stop the scheduler and disable the device
    if(m_hHD)
	{
		hdStopScheduler();
		hdUnschedule(hSchedule);
		hdDisableDevice(m_hHD);
	}

	delete[] m_filters;
}

void ChunHaptic::InitDevice()
{
	// Initialize the device, must be done before attempting to call any hd functions.
    if(!m_hHD)	
	{
		m_hHD = hdInitDevice(HD_DEFAULT_DEVICE);

		// CKim - Typical error check
		HDErrorInfo error;
		if (HD_DEVICE_ERROR(error = hdGetError())) 
		{
			std::ostringstream ostr;
			ostr<<"Failed to initialize haptic device : "<<error;
			AfxMessageBox(ostr.str().c_str());
		}
	}
}

void ChunHaptic::StartLoop()
{
	// CKim - 2. Create handle to 'Scheduler'
    // CKim - Schedule the main callback that will render forces to the device.
	// Callback functions are scheduled to be executed inside the scheduler loop. 
	// Callbacks can be scheduled in two varieties -- asynchronous and synchronous. 
	// Synchronous calls only return after they are completed, so the application thread waits for a
	// synchronous call before continuing. Asynchronous calls return immediately after being scheduled.
	// syntax : hdScheduleAsynchronous(pointer to function, pointer to passing data, priority)
	hSchedule = hdScheduleAsynchronous(ChunHaptic::updateCallback, this, HD_MAX_SCHEDULER_PRIORITY);

	// CKim - Enable force
    hdEnable(HD_FORCE_OUTPUT);
    
	// CKim - Start scheduler, Typically call this after all device initialization routines and 
	// after all asynchronous scheduler callbacks have been added
	hdStartScheduler();

	// CKim - Typical error check
	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) 
	{
		std::ostringstream ostr;
		ostr<<"Failed to start scheduler : "<<error;
		AfxMessageBox(ostr.str().c_str());
	}

}

void ChunHaptic::SetForce(double force)
{
	this->m_currentState.Force[0] = force;
	//::std::cout << "in setter" << ::std::endl;
	//::std::cout << force  << ::std::endl;
	//::std::cout << m_currentState.Force[0] <<::std::endl;
}

HDCallbackCode HDCALLBACK ChunHaptic::updateCallback(void *pData)
{
	// CKim - I should update states from here and do some haptic rendering as well if necessary...
	ChunHaptic* mySelf = (ChunHaptic*) pData;

	hdBeginFrame(m_hHD);

	/*Save off the old state as last. */
    mySelf->m_lastState = mySelf->m_currentState;
	
    // CKim - Update the position, transformation, force, and time stamp
    hdGetDoublev(HD_CURRENT_TRANSFORM, mySelf->m_currentState.tfMat);
	
	double velocity[3] = {0};
	hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
	for(int i = 0 ; i < 3; ++i)
		mySelf->m_currentState.velocity[i] = mySelf->m_filters[i].step(velocity[i]);
	
	// numerical angular velocity
	double ang_velocity[3] = {0};
	long currentTime = m_timer.GetTime();
	double dt = (double)(currentTime - m_lastTime)/1000000.0;	// in seconds
	m_lastTime = currentTime;
	ang_velocity[0] = dt;

	::Eigen::MatrixXd lastT = ::Eigen::Map<::Eigen::MatrixXd>(mySelf->m_lastState.tfMat, 4,4);
	::Eigen::MatrixXd currentT = ::Eigen::Map<::Eigen::MatrixXd>(mySelf->m_currentState.tfMat, 4,4);

	::Eigen::Matrix3d Rdot = (currentT - lastT).block(0,0,3,3)/dt;
	::Eigen::Matrix3d R_t = currentT.block(0,0,3,3).transpose();	// R transpose
	::Eigen::Matrix3d angVel_skew = Rdot*R_t;
	ang_velocity[0] = -angVel_skew(1,2);
	ang_velocity[1] = angVel_skew(0,2);
	ang_velocity[2] = -angVel_skew(0,1);

	//hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, ang_velocity);
	//ang_velocity[0] = 1;
	for(int i = 0 ; i < 3; ++i)
		mySelf->m_currentState.ang_velocity[i] = mySelf->m_filters[i + 3].step(ang_velocity[i]);

	//hdGetDoublev(HD_CURRENT_FORCE,mySelf->m_currentState.Force);
	
	// CKim - Render force feedback
	if(m_currentState.forceFlag)
	{
		HDfloat forces[3];
		forces[0] = mySelf->m_currentState.Force[0];
		forces[1] = forces[2] = 0;
		hdSetFloatv(HD_CURRENT_FORCE,forces);
	}

	// --------------------------------------------------------------------------- //
    // CKim - Event handling. 
	// --------------------------------------------------------------------------- //
	
	// CKim - Check for a stylus switch state change.
    HDint nCurrentButtonState, nLastButtonState;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtonState);
    hdGetIntegerv(HD_LAST_BUTTONS, &nLastButtonState);

	HapticEventData ev;

    if ( ((nCurrentButtonState & HD_DEVICE_BUTTON_1) != 0) && ((nLastButtonState & HD_DEVICE_BUTTON_1) == 0) )
    {
		// CKim - Handle the event at device. Button 1 press updates the refence configuration for 
		// Master-Slave coordinate transform

		// CKim - Also add the event information in the queue so that robot thread can react to it
		mySelf->handleEvent(BUTTON_1_DOWN);
    }
    else if ( ((nCurrentButtonState & HD_DEVICE_BUTTON_1) == 0) && ((nLastButtonState & HD_DEVICE_BUTTON_1) != 0) )
    {
		mySelf->handleEvent(BUTTON_1_UP);
    }
    
    if ( ((nCurrentButtonState & HD_DEVICE_BUTTON_2) != 0) && ((nLastButtonState & HD_DEVICE_BUTTON_2) == 0) )
    {
		mySelf->handleEvent(BUTTON_2_DOWN);
    }
    else if ( ((nCurrentButtonState & HD_DEVICE_BUTTON_2) == 0) && ((nLastButtonState & HD_DEVICE_BUTTON_2) != 0) )
    {
		mySelf->handleEvent(BUTTON_2_UP);
    }


	hdEndFrame(m_hHD);
	
	return HD_CALLBACK_CONTINUE;
}


void ChunHaptic::SynchState(CTR_status& state)
{
	hdScheduleSynchronous(synchCallback, &state, HD_DEFAULT_SCHEDULER_PRIORITY);
}


HDCallbackCode HDCALLBACK ChunHaptic::synchCallback(void *pData)
{
	CTR_status* state = (CTR_status*) pData;	HapticEventData ev;

	// CKim - Copy the current haptic device state to the robot state
	for(int i=0; i<3; i++)	{		state->hapticState.Force[i] = m_currentState.Force[i];			}

	// store the current position to previous position variable
	memcpy(state->hapticState.previousPosition, state->hapticState.position, 3 * sizeof(double));

	// update current position
	memcpy(state->hapticState.position, &state->hapticState.tfMat[12], 3 * sizeof(double));
	
	memcpy(state->hapticState.velocity, &m_currentState.velocity, 3 * sizeof(double));
	memcpy(state->hapticState.ang_velocity, &m_currentState.ang_velocity, 3 * sizeof(double));

	for(int i=0; i<16; i++)	{		state->hapticState.tfMat[i] = m_currentState.tfMat[i];			}
	//::std::cout << "Events in queue: " << m_currentState.eventQueue.empty() << ::std::endl;
	while(!m_currentState.eventQueue.empty())
	{
		ev = m_currentState.eventQueue.front();
		state->hapticState.eventQueue.push(ev);
		m_currentState.eventQueue.pop();
	}

	state->hapticState.err = m_currentState.err;


	// CKim - Copy some of the robot state that will be used in haptic rendering loop
	for(int i=0; i<3; i++)	{	m_currentState.slavePos[i] = state->hapticState.slavePos[i];	}
	m_currentState.forceFlag = state->hapticState.forceFlag;
	m_currentState.forceMag = state->hapticState.forceMag;	

	////double velocity[12] = {0};
	//////hdGetFloatv(HD_CURRENT_VELOCITY, velocity);
	////hdGetDoublev(HD_CURRENT_POSITION,velocity);
	//for(int i = 0; i < 3; ++i)
	//	::std::cout << velocity[i] << " ";
	//::std::cout << ::std::endl;
	//// CKim - Error check
 //   if (HD_DEVICE_ERROR(m_currentState.err))
 //   {
	//	AfxMessageBox("Error detected while querying haptic device state");
 //   }


	return HD_CALLBACK_DONE;
}


void ChunHaptic::handleEvent(ChunHaptic::EventType evtId)
{
	// CKim - Record the event type and the configuration of the device at the event
	// and save it on the queue
	HapticEventData ev;
	
	ev.eventId = evtId;
	for(int i=0; i<16; i++)	{	ev.refMat[i] = m_currentState.tfMat[i];		}

	m_currentState.eventQueue.push(ev);
}
	
	//    /* Check if there's a callback to be called. */
//    HapticDeviceCallback *pCallback = (HapticDeviceCallback *) m_aCallbackFunc[event];
//    if (pCallback)
//    {
//        pCallback(event, &m_currentState, m_aCallbackUserData[event]);
//    }
//
//    HapticDeviceEvent *pEvent = new HapticDeviceEvent;
//    pEvent->event = event;
//    memcpy(&pEvent->m_state, &m_currentState, sizeof(HapticDeviceStateCache));
//    m_eventQueue.push(pEvent); 
//}


//
//void ChunHaptic::beginUpdate()
//{
//    hdBeginFrame(m_hHD);
//    
//    /*Save off the old state as last. */
//    m_lastState = m_currentState;
//
//    /* Update the cached position data. */
//    hdGetDoublev(HD_CURRENT_POSITION, m_currentState.getPosition());
//    hdGetDoublev(HD_CURRENT_TRANSFORM, m_currentState.getTransform());
//
//    /* Check for a stylus switch state change. */
//    HDint nCurrentButtonState, nLastButtonState;
//    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtonState);
//    hdGetIntegerv(HD_LAST_BUTTONS, &nLastButtonState);
//
//    if ((nCurrentButtonState & HD_DEVICE_BUTTON_1) != 0 &&
//        (nLastButtonState & HD_DEVICE_BUTTON_1) == 0)
//    {
//        handleEvent(BUTTON_1_DOWN);
//    }
//    else if ((nCurrentButtonState & HD_DEVICE_BUTTON_1) == 0 &&
//             (nLastButtonState & HD_DEVICE_BUTTON_1) != 0)
//    {
//        handleEvent(BUTTON_1_UP);
//    }
//    
//    if ((nCurrentButtonState & HD_DEVICE_BUTTON_2) != 0 &&
//        (nLastButtonState & HD_DEVICE_BUTTON_2) == 0)
//    {
//        handleEvent(BUTTON_2_DOWN);
//    }
//    else if ((nCurrentButtonState & HD_DEVICE_BUTTON_2) == 0 &&
//             (nLastButtonState & HD_DEVICE_BUTTON_2) != 0)
//    {
//        handleEvent(BUTTON_2_UP);
//    }
//    if ((nCurrentButtonState & HD_DEVICE_BUTTON_3) != 0 &&
//        (nLastButtonState & HD_DEVICE_BUTTON_3) == 0)
//    {
//        handleEvent(BUTTON_3_DOWN);
//    }
//    else if ((nCurrentButtonState & HD_DEVICE_BUTTON_3) == 0 &&
//             (nLastButtonState & HD_DEVICE_BUTTON_3) != 0)
//    {
//        handleEvent(BUTTON_3_UP);
//    }
////
////}
//
//
//void ChunHaptic::endUpdate()
//{
//    hdMakeCurrentDevice(m_hHD);
//
//    /* Doing this as part of the endUpdate to give the client a chance to 
//       update his simulation and stuff something into the contact state or 
//       modify the proxy transform. */
//
//    /* Check for change in contact state. */
//    if (m_currentState.isInContact() && !m_lastState.isInContact())
//    {
//        handleEvent(MADE_CONTACT);
//    }
//    else if (!m_currentState.isInContact() && m_lastState.isInContact())
//    {
//        handleEvent(LOST_CONTACT);
//    }
//
//    hdEndFrame(m_hHD);
//
//    /* Check if a device error occurred */
//    m_currentState.setLastError(hdGetError());
//    if (HD_DEVICE_ERROR(m_currentState.getLastError()))
//    {        
//        handleEvent(DEVICE_ERROR);    
//    }
//}
//
//void ChunHaptic::StartLoop()
//{
//
//}
//

//
//
//
//
//
//
//
//
//
//




