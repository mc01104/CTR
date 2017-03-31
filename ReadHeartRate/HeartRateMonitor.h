#pragma once

#include "SerialCommunication.h"
#include <vector>
#include <fstream>
#include "Utilities.h"

class HeartRateMonitor
{
	SerialPort sPort;

	double heartRate;

	bool logData;
	bool prevLog;
	bool newFile;
	::std::ofstream os;

	public:
		HeartRateMonitor();

		~HeartRateMonitor();

		void run();

		double getHeartRate();

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