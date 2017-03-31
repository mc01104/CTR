// ReadHeartRate.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "SerialCommunication.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>

::std::vector<::std::string> ReadLinesFromFile(const ::std::string& pathToFile)
{
	::std::vector< ::std::string> linesVector;

	::std::ifstream inputFile(pathToFile.c_str());
	
	::std::string tempLine;
	while(::std::getline(inputFile, tempLine))
		linesVector.push_back(tempLine);

	return linesVector;
}

//::std::vector<unsigned char*> readMeasurementsFromFile(::std::string filename)
//{
//	::std::vector<::std::string> lines = ReadLinesFromFile(filename);	
//	::std::vector<unsigned char*> result;
//	result.resize(lines.size());
//
//	char tmp[200];
//	for (int i = 0; i < lines.size(); ++i)
//		::std::strcpy( (char*) &result[i], lines[i].c_str());
//	
//	return result;
//}
void binary_from_string(::std::string& sHex, ::std::string& sReturn)
{
	for (int i = 0; i < sHex.length (); ++i)
	{
		switch (sHex [i])
		{
			case '0': sReturn.append ("0000"); break;
			case '1': sReturn.append ("0001"); break;
			case '2': sReturn.append ("0010"); break;
			case '3': sReturn.append ("0011"); break;
			case '4': sReturn.append ("0100"); break;
			case '5': sReturn.append ("0101"); break;
			case '6': sReturn.append ("0110"); break;
			case '7': sReturn.append ("0111"); break;
			case '8': sReturn.append ("1000"); break;
			case '9': sReturn.append ("1001"); break;
			case 'a': sReturn.append ("1010"); break;
			case 'b': sReturn.append ("1011"); break;
			case 'c': sReturn.append ("1100"); break;
			case 'd': sReturn.append ("1101"); break;
			case 'e': sReturn.append ("1110"); break;
			case 'f': sReturn.append ("1111"); break;
		}
	}
}

::std::vector<::std::string> splitStr(::std::string inputStr, char delim)
{
	::std::vector<::std::string> result;
	::std::stringstream ss;
	ss.str(inputStr);
	::std::string tmp;
	while (::std::getline(ss, tmp, delim))
	{
		result.push_back(tmp);
	}

	return result;
}

::std::vector<bool> parseBitMap(::std::string& flags)
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

bool checkForConsistency(::std::string& value)
{
	double tmpValue = atof(value.c_str());
	
	return (tmpValue < 0 ? false : true);
}

void testSerial()
{
	SerialPort sPort;
	sPort.connect();

	unsigned char m_testRead[200];
	::std::vector<::std::string> strings;
	::std::string bitFlags = "";
	::std::vector<bool> source;
	::std::vector<int> index;
	index.push_back(1);
	index.push_back(4);
	index.push_back(8);

	double heartRate = 0;
	::std::vector< ::std::string> sensorNames;
	sensorNames.push_back("ECG");
	sensorNames.push_back("Oximetry");
	sensorNames.push_back("IBP");

	while (true)
	{
		int bytesRead = sPort.getArray(m_testRead, 200);

		::std::string str(m_testRead, m_testRead + bytesRead);
		strings = splitStr(str, ',');
		source  = parseBitMap(strings[0]);

		source  = parseBitMap(strings[0]);
		for (int i = 0; i < source.size(); ++i)
			if (source[i])
				::std::cout << sensorNames[i] << "-> active,"; 
		::std::cout << ::std::endl;

		for (int i = 0; i < source.size(); ++i)
		{
			if (source[i])
			{
				if(!checkForConsistency(strings[index[i]]))
				{
					::std::cout << sensorNames[i]<< "-> wrong value -> ignore, "; 
					continue;
				}
				else
				{
					heartRate = atof(strings[index[i]].c_str());
					::std::cout << sensorNames[i] << "-> Heart rate [bpm]:" << heartRate << ", ";
				}

			}

		}
		::std::cout << ::std::endl;
		::std::cout << ::std::endl;

	}

	sPort.clear();
	sPort.disconnect();

}

void testStringParsing()
{
	::std::vector<::std::string> lines = ReadLinesFromFile("working.TXT");	
	::std::vector<::std::string> strings;

	unsigned char* val = new unsigned char[200];
	::std::string tmpStr;

	::std::string bitFlags = "";
	::std::vector<bool> source;
	::std::vector<int> index;
	index.push_back(1);
	index.push_back(4);
	index.push_back(8);

	double heartRate = 0;
	::std::vector< ::std::string> sensorNames;
	sensorNames.push_back("ECG");
	sensorNames.push_back("Oximetry");
	sensorNames.push_back("IBP");

	for (int i = 0; i < lines.size(); ++i)
	{
		::std::copy(lines[i].begin(), lines[i].end(), val);

		::std::string my_std_string(val, val + 200);

		strings = splitStr(my_std_string, ',');

		source  = parseBitMap(strings[0]);
		for (int i = 0; i < source.size(); ++i)
			if (source[i])
				::std::cout << sensorNames[i] << "-> active,"; 
		::std::cout << ::std::endl;

		for (int i = 0; i < source.size(); ++i)
		{
			if (source[i])
			{
				if(!checkForConsistency(strings[index[i]]))
				{
					::std::cout << sensorNames[i]<< "-> wrong value -> ignore, "; 
					continue;
				}
				else
				{
					heartRate = atof(strings[index[i]].c_str());
					::std::cout << sensorNames[i] << "-> Heart rate [bpm]:" << heartRate << ", ";
				}

			}

		}
		::std::cout << ::std::endl;
		::std::cout << ::std::endl;
	}

}

//int _tmain(int argc, _TCHAR* argv[])
//{
//	//testStringParsing();
//	testSerial();
//}

