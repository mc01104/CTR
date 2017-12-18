#pragma once
// Winsock includes for network
#include <winsock2.h>
#include <ws2tcpip.h>
//#include "targetver.h"

#include "SerialCommunication.h"
#include <vector>
#include <fstream>
#include "Utilities.h"

#include "windows.h"

class HeartRateMonitor
{
	SerialPort sPort;

	double heartRate;
	double breathingRate;

	bool logData;
	bool prevLog;
	bool newFile;
	::std::ofstream os;

	int source;

	public:
		HeartRateMonitor();

		~HeartRateMonitor();

		void run();
		bool runNetwork();

		double getHeartRate();

		double getBreathingRate() {return this->breathingRate;};

		void setHRSource(int source) {this->source = source;};
		void toggleLog(bool logFlag) 
		{
			prevLog = logData; 
			logData = logFlag;
			::std::cout << "prev:" << prevLog << ", log:" << logFlag << ::std::endl;
			if (!prevLog && logData)
			{
				if (os.is_open())
					os.close();
				::std::string filename = GetDateString() + ".txt";
				::std::cout <<  filename << ::std::endl;
				os.open(filename);
			}
		};
private:
	::std::vector<bool> parseBitMap(::std::string& flags);

	bool checkForConsistency(::std::string& value);

};