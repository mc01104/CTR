#include "stdafx.h"
#include "ChunTracker.h"
#include <iostream>

ChunTracker::ChunTracker(void)
{
	m_pRecord = NULL;
	m_isInit = false;

	m_RegMat.resize(4,4);		m_RegMat.setIdentity();
	m_trackerMat.resize(4,4);	m_trackerMat.setIdentity();
}


ChunTracker::~ChunTracker(void)
{
}


bool ChunTracker::InitTracker()
{
	// ---------------------------------------------------------------- //
	// CKim - Set up the Ascension EM tracker.
	// Related include and library files are 'ATC3DG.h' 
	// ---------------------------------------------------------------- //
	//printf( "Initializing tracker wait please...\n" );
	int errCode;	short xmtrId = 0;		int useMetric = 1;
	
	DATA_FORMAT_TYPE type = DOUBLE_POSITION_MATRIX_TIME_Q;	
	
	
	// CKim - Initialize the tracker and get system configuration. Set measurement unis in mm
	errCode = InitializeBIRDSystem();
	if(errCode!=BIRD_ERROR_SUCCESS)	{		PrintTrackerError(errCode);		return false;	}
	
	errCode = GetBIRDSystemConfiguration(&m_trackerConfig);
	if(errCode!=BIRD_ERROR_SUCCESS)	{		PrintTrackerError(errCode);		return false;	}

	// CKim - Configure the sensors. Get the number of connected sensors. Set the data format for sensor reading
	m_pSensorConfig = new SENSOR_CONFIGURATION[m_trackerConfig.numberSensors];
	
	for(int i=0; i<m_trackerConfig.numberSensors; i++)
	{
		errCode = GetSensorConfiguration(i, &(m_pSensorConfig[i]));
		if(errCode!=BIRD_ERROR_SUCCESS)	{		PrintTrackerError(errCode);			return false;	}

		errCode = SetSensorParameter(i,DATA_FORMAT,&type,sizeof(type));  
		if(errCode!=BIRD_ERROR_SUCCESS)	{		PrintTrackerError(errCode);		return false;	}
	}

	m_pRecord = new DOUBLE_POSITION_MATRIX_TIME_Q_RECORD[m_trackerConfig.numberSensors];
	
	// CKim - Configure the transmitter and turn it on. xmtrId must be type short!!
	errCode = GetTransmitterConfiguration(0,&m_xmtrConfig);
	if(errCode!=BIRD_ERROR_SUCCESS)	{		PrintTrackerError(errCode);		return false;	}

	errCode = SetSystemParameter(SELECT_TRANSMITTER, &xmtrId, sizeof(xmtrId));
	if(errCode!=BIRD_ERROR_SUCCESS)	{		PrintTrackerError(errCode);		return false;	}
	
	errCode = SetSystemParameter(METRIC,&useMetric,sizeof(useMetric));
	if(errCode!=BIRD_ERROR_SUCCESS)	{		PrintTrackerError(errCode);		return false;	}

	m_isInit = true;
	return true;

	//printf("Number of sensors connected : %d\n",sensorNum);
	//printf("Press any key to stat tracking");
	//std::getchar();
}


bool ChunTracker::GetTrackerMatrix(double pMat[4][4], unsigned short sNum)
{
	if(!m_isInit)	
	{	
		::std::cout << "m_isInit" << ::std::endl;
		return false;
	}

	for(int i=0; i<4; i++)	{
		for(int j=0; j<4; j++)	{	pMat[i][j] = ((i==j) ? 1 : 0);	}	}
	
	int errCode;
	
	//errCode = GetSynchronousRecord(sNum, &(m_pRecord[sNum]), sizeof(m_pRecord[sNum]));
	//errCode = GetSynchronousRecord(sNum, m_pRecord+sNum, sizeof(m_pRecord[sNum]));
	errCode = GetSynchronousRecord(ALL_SENSORS, m_pRecord, sizeof(m_pRecord[sNum])*m_trackerConfig.numberSensors);
	
	if(errCode!=BIRD_ERROR_SUCCESS) {	
		PrintTrackerError(errCode);
		::std::cout << "errCode=" << errCode << ::std::endl;	
		return false;	
	}
	
	unsigned int status = GetSensorStatus(sNum);
	//::std::cout << "problem" << ::std::endl;
	// CKim - Each 'row' of s corresponds to the direction of sensor's x,y,z axis
	if( status == VALID_STATUS)
	{
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++)	{	m_trackerMat(i,j) = m_pRecord[sNum].s[j][i];	}	}
		m_trackerMat(0,3) = m_pRecord[sNum].x;		m_trackerMat(1,3) = m_pRecord[sNum].y;		
		m_trackerMat(2,3) = m_pRecord[sNum].z;

		for(int i=0; i<4; i++)	{
			for(int j=0; j<4; j++)	{	pMat[i][j] = m_trackerMat(i,j);	}	}
	}
	else {	::std::cout<< "error!!" << std::endl;
			AfxMessageBox("!!");	
	}

	sprintf(&this->m_errMsg[0] , "patates");
}


void ChunTracker::Registration(char* fName, Eigen::Matrix4d& M)
{
	using namespace Eigen;		using namespace std;
	
	//::std::cout << "Registration" << ::std::endl;
	// CKim - Open file and count number of points
	ifstream ifstr;		string str;			int cnt = 0;
	ifstr.open(fName,std::ifstream::in);
	if(!ifstr.good())	{	AfxMessageBox("File open error");	return;		}

	while(1)
	{
		std::getline(ifstr,str);
		if(ifstr.eof())	{	break;	}
		else			{	cnt++;	}
	}

	//cout<<"Total "<<cnt<<" points\n";
	ifstr.close();

	// CKim - Open file again and read the points
	ifstr.open(fName,std::ifstream::in);		ifstr.seekg(0,ifstr.beg);
	MatrixXd F(3,cnt);		MatrixXd Fn(3,cnt);		MatrixXd Fp(3,cnt);

	for(int i=0; i<cnt; i++)
	{
		ifstr>>F(0,i);	ifstr>>F(1,i);	ifstr>>F(2,i);
	}
	ifstr.close();

	// CKim - Perform registration
	Matrix<double,3,1> cent;	Matrix<double,3,1> n;

	// 1. Find centroid
	cent = F.rowwise().mean();	//	cout<<"Centroid is \n";	cout<<cent<<endl<<endl;

	// 2. Normalize points
	Fn = F.colwise() - cent;

	// 3. Extract normal of the plane. - This is Z axis.
	// Vector corresponding to smallest singular vector of the F*F' is the normal of the plane
	Matrix3d FtF = Fn*Fn.transpose();		//cout<<FtF<<endl<<endl;
	JacobiSVD<Matrix3d> svd(FtF, Eigen::ComputeFullU | Eigen::ComputeFullV);	//cout<<"Singular values are\n";
	//cout<<svd.singularValues()<<endl<<endl;	//	cout<<"Normal vector is\n";
	n = svd.matrixU().col(2);	//	cout<<n<<endl<<endl;	//cout<<svd.matrixV()<<endl;	//cout<<"//----------------------//\n";

	// 4. Define an aribitrary coordinate system (R,t) whose z axis is the normal and located at centroid
	Matrix3d R;		Matrix<double,3,1> xtmp,ytmp,ztmp,t;		
	if(n(0,0) < 0)	{	ztmp = n;	}
	else			{	ztmp = -n;	}
	xtmp = ztmp.cross(Matrix<double,3,1>::Random());
	xtmp.normalize();		ytmp = ztmp.cross(xtmp);
	R.col(0) = xtmp;	R.col(1) = ytmp;	R.col(2) = ztmp; 

	// 5. Transform all points to this coordinate system. All transformed coordinate will be close to xy plane
	for(int i=0; i<cnt; i++)
	{
		Fp.col(i) = R.transpose()*F.col(i) - R.transpose()*cent;
	}
	//cout<<Fp.block(0,40,3,10)<<endl<<endl;

	// 6. Find circle from (x,y) coordinates :  Equation of circle has form of  "x^2 + y^2 + ax + by + c = 0"
	//  = (x+a/2)^2 + (y+b/2)^2 = (a^2 + b^2)/4 - c.  This leads to matrix equation [ x, y, 1 ] * [ a, b, c ]' = -x^2 -y^2
	MatrixXd A(cnt,3);		MatrixXd b(cnt,1);
	for(int i=0; i<cnt; i++)
	{
		A(i,0) = Fp(0,i);		A(i,1) = Fp(1,i);		A(i,2) = 1.0;
		b(i,0) = -(Fp(0,i)*Fp(0,i) + Fp(1,i)*Fp(1,i));
	}
	
	Matrix<double,3,1> sol = A.jacobiSvd(ComputeFullU | ComputeFullV).solve(b);

	Matrix<double,3,1> circ;
	circ(0,0) = -sol(0,0)/2.0;	circ(1,0) = -sol(1,0)/2.0;	circ(2,0) = 0.0;
	double r = sqrt( (sol(0,0)*sol(0,0) + sol(1,0)*sol(1,0))/4.0 - sol(2,0) );

	
	CString ccstr;
	ccstr.Format("Radius : %.2f",r);
	AfxMessageBox(ccstr);
	//cout<<"Radius : "<<r<<endl<<endl;
	//cout<<"Center at "<<R*circ + cent<<endl;
	
	//
	// 7. From the circle, determine the configuration of the robot coordinate system w.r.t EM ttracker 
	Matrix<double,3,1> x,y,z,tmp;		M.setIdentity();	
	// a. Z axis of the robot coordinate system is the normal of the plane. Check dir sign
	z = ztmp;	M.block(0,2,3,1) = z;
	// b. From the structure of the calibration rig, y axis is the direction from circle center to first data point
	//tmp = Fp.col(0);	tmp(2,0) = 0;		y = R*(tmp - circ);		y.normalize();		M.block(0,1,3,1) = y;
	// c. x axis is straight forward
	//x = y.cross(z);		M.block(0,0,3,1) = x;
	
	// b. From the structure of the calibration rig, x axis is the direction from circle center to first data point
	tmp = Fp.col(0);	tmp(2,0) = 0;		x = R*(tmp - circ);		x.normalize();		M.block(0,0,3,1) = x;
	// c. x axis is straight forward
	y = z.cross(x);		M.block(0,1,3,1) = y;


	// d. The origin of the robot coordinate system is the center of the circle moved xx mm in - z direction
	tmp = circ;		tmp(2,0) = -168.0;//	tmp(2,0) = -3.0;
	M.block(0,3,3,1) = R*tmp + cent;

	// CKim - Registration matrix needs to transform EM tracker coordinate to Robot coordinate so...
	m_RegMat.block(0,0,3,3) = M.block(0,0,3,3).transpose();
	m_RegMat.block(0,3,3,1) = -(M.block(0,0,3,3).transpose() * M.block(0,3,3,1));

	//::std::cout << "finished" << ::std::endl;
	//std::cout << m_RegMat << ::std::endl;
	//cout<<M<<endl;
	//cout<<circ<<endl;

}


void ChunTracker::GetMeasuredTipPosDir(double measPosDir[6])
{
	// CKim - Apply registration matrix to the tracker measurement to find measured position in robot coordinate system
	// Tip direction is 'x'axis of the sensor
	//::std::cout << m_trackerMat(0,3) << " " << m_trackerMat(1,3) << " " << m_trackerMat(2,3) << std::endl;
	//::std::cout << m_RegMat << ::std::endl;
	Eigen::Matrix4d M = m_RegMat*m_trackerMat;
	Eigen::Vector3d measPos = M.block(0,3,3,1);		Eigen::Vector3d measDir = M.block(0,0,3,1);
	for(int i=0; i<3; i++)	{
//		measPosDir[i] = measPos(i,0);	measPosDir[i+3] = -measDir(i,0);	}

		// CKim - Account for sensor offset here....
		measPosDir[i] = measPos(i,0);	measPosDir[i+3] = -measDir(i,0);	
		measPosDir[i] -= (9.3*measPosDir[i+3]);		}

}


void ChunTracker::PrintTrackerError(int errCode)
{
	::std::cout << "print tracker error" << ::std::endl;
	while(errCode!=BIRD_ERROR_SUCCESS)
	{
		errCode = GetErrorText(errCode, m_errMsg, sizeof(m_errMsg), SIMPLE_MESSAGE);
		::std::cout << "errorCode:" << m_errMsg << ::std::endl;	
		//numberBytes = strlen(buffer);
		//buffer[numberBytes] = '\n';		// append a newline to buffer
		//printf("%s", buffer);
	}
}

//
//	short xmtrId = 0;
//
//
//	DOUBLE_POSITION_MATRIX_TIME_Q_RECORD record;				
//	DOUBLE_POSITION_MATRIX_TIME_Q_RECORD* pRecord = &record;	
//	COORD coordinate;
//	double pos[7];
//
//	while(!stopFlag)
//	{
//		if(acqFlag)	
//		{
//			for(int i=0; i<2; i++)
//			{
//				int errCode = GetAsynchronousRecord(i, pRecord, sizeof(record));
//				if(errCode!=BIRD_ERROR_SUCCESS) {		errorHandler(errCode);		}
//
//				if(once)	{
//					time0 = record.time;	once = false;
//				}
//
//				// CKim - write tracker data
//				coordinate.Y=6+5*i;		coordinate.X=4;		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coordinate);
//				printf("%8.3f %8.3f %8.3f %8.3f", record.s[0][0], record.s[0][1], record.s[0][2], record.x*25.4);
//				fTrck[i]<<record.s[0][0]<<"\t"<<record.s[0][1]<<"\t"<<record.s[0][2]<<"\t"<<record.x*25.4<<"\n";
//
//				coordinate.Y=7+5*i;		coordinate.X=4;		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coordinate);
//				printf("%8.3f %8.3f %8.3f %8.3f", record.s[1][0], record.s[1][1], record.s[1][2], record.y*25.4);
//				fTrck[i]<<record.s[1][0]<<"\t"<<record.s[1][1]<<"\t"<<record.s[1][2]<<"\t"<<record.y*25.4<<"\n";
//
//				coordinate.Y=8+5*i;		coordinate.X=4;		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coordinate);
//				printf("%8.3f %8.3f %8.3f %8.3f", record.s[2][0], record.s[2][1], record.s[2][2], record.z*25.4);
//				fTrck[i]<<record.s[2][0]<<"\t"<<record.s[2][1]<<"\t"<<record.s[2][2]<<"\t"<<record.z*25.4<<"\n";
//
//				coordinate.Y=9+5*i;		coordinate.X=4;		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coordinate);
//				printf("%8.3f %8.3f %8.3f %8.3f", 0, 0, 0, 1);
//			}
//
//			for(int i=0; i<7; i++)	{	myAmp[i].GetPositionActual(pos[i]);		}
//
//			// CKim - write motor joint angle
//			coordinate.Y=16;		coordinate.X=4;		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coordinate);
//			printf("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);
//			fPos<<(record.time - time0)<<"\t"<<pos[3]<<std::endl;
//		}
//		::Sleep(20);
//	}
//
//
//	
//}


