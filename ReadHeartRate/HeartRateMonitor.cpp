//#include "stdafx.h"
#include "HeartRateMonitor.h"
#include "Utilities.h"

HeartRateMonitor::HeartRateMonitor():
	heartRate(0)
{
}

HeartRateMonitor::~HeartRateMonitor()
{
}

void HeartRateMonitor::run()
{
	sPort.connect();

	unsigned char m_testRead[200];
	::std::vector<::std::string> strings;
	::std::string bitFlags = "";
	::std::vector<bool> source;
	::std::vector<int> index;
	index.push_back(1);
	//index.push_back(4);
	index.push_back(5);
	index.push_back(8);

	::std::vector< ::std::string> sensorNames;
	sensorNames.push_back("ECG");
	sensorNames.push_back("Oximetry");
	sensorNames.push_back("IBP");

	while (true)
	{
		int bytesRead = sPort.getArray(m_testRead, 200);

		::std::string str(m_testRead, m_testRead + bytesRead);
		strings = splitStr(str, ',');

		// return negative value for unknown source
		source  = this->parseBitMap(strings[0]);

		if (source.size() <= 0)
			continue;
		
		if(!checkForConsistency(strings[index[1]]))
			continue;
		else
		{
			heartRate = atof(strings[index[1]].c_str());
			::std::cout << sensorNames[1] << "-> Heart rate [bpm]:" << heartRate << ", ";
		}

	}

	sPort.clear();
	sPort.disconnect();


}

double HeartRateMonitor::getHeartRate()
{
	return heartRate;
}

bool HeartRateMonitor::checkForConsistency(::std::string& value)
{
	double tmpValue = atof(value.c_str());
	
	return (tmpValue < 0 ? false : true);
}

::std::vector<bool> HeartRateMonitor::parseBitMap(::std::string& flags)
{
	::std::string bitMapFlags = "";
	binary_from_string(flags, bitMapFlags);

	std::string strTmp = bitMapFlags.substr(bitMapFlags.size() - 1 - 9,3);  
	::std::cout << "sensor flag bitmap: "  << strTmp.c_str() << ::std::endl;
	::std::vector<bool> sensorsActivated(strTmp.size());
	int ind = 0;
	for (int i = strTmp.size() - 1; i >= 0 ; --i)
		sensorsActivated[ind++] = strTmp.c_str()[i] == '1' ? true : false; 

	return sensorsActivated;
}