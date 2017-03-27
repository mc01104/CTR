#pragma once

#include "SerialCommunication.h"
#include <vector>

class HeartRateMonitor
{
	SerialPort sPort;

	double heartRate;

	public:
		HeartRateMonitor();

		~HeartRateMonitor();

		void run();

		double getHeartRate();
private:
	::std::vector<bool> parseBitMap(::std::string& flags);

	bool checkForConsistency(::std::string& value);

};