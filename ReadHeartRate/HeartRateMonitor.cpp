//#include "stdafx.h"
#include "HeartRateMonitor.h"

#include <iostream>
#include <vector>
#include <sstream>

#include "Utilities.h"

#define DEFAULT_BUFLEN 200
#define DEFAULT_PORT "10000"

HeartRateMonitor::HeartRateMonitor():
	heartRate(0),
	logData(false),
	prevLog(false),
	newFile(false),
	source(8)
{
}

HeartRateMonitor::~HeartRateMonitor()
{
}

void HeartRateMonitor::run()
{
	sPort.connect();
	::std::string date;
	unsigned char m_testRead[200];
	::std::vector<::std::string> strings;
	::std::string bitFlags = "";
	::std::vector<bool> source;
	::std::vector<int> index;
	index.push_back(1);
	//index.push_back(4);
	index.push_back(5);
	index.push_back(8);

	//::std::cout << "log:" << logData << "prev:" << prevLog;
	//::std::string filename = GetDateString() + ".txt";
	//::std::ofstream os(filename);
	newFile = false;
	::std::vector< ::std::string> sensorNames;
	sensorNames.push_back("ECG");
	sensorNames.push_back("Oximetry");
	sensorNames.push_back("IBP");

	int counter = 0;
	while (true)
	{
		int bytesRead = sPort.getArray(m_testRead, 200);

		if (bytesRead < 5)
			continue;

		::std::string str(m_testRead, m_testRead + bytesRead);
		strings = splitStr(str, ',');

		// return negative value for unknown source
		source  = this->parseBitMap(strings[0]);

		if (source.size() <= 0)
			continue;
		
		//if(!checkForConsistency(strings[index[1]]))
		//	continue;
		//else
		//{
			heartRate = atof(strings[this->source].c_str());
			breathingRate = atof(strings[strings.size()-1].c_str());
			::std::cout << " Heart rate [bpm]:" << heartRate << ::std::endl;
			//::std::cout << " Breathing rate [bpm]:" << breathingRate << ::std::endl;
		//}


		counter++;
	
		if (logData)
		{
			date = GetDateString();
			os << strings << " " << date <<::std::endl;
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
	//::std::cout << "sensor flag bitmap: "  << strTmp.c_str() << ::std::endl;
	::std::vector<bool> sensorsActivated(strTmp.size());
	int ind = 0;
	for (int i = strTmp.size() - 1; i >= 0 ; --i)
		sensorsActivated[ind++] = strTmp.c_str()[i] == '1' ? true : false; 

	return sensorsActivated;
}

bool HeartRateMonitor::runNetwork()
{
	/**********
	Declare and initialize connection socket
	**********/
	WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;
    char *sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;

	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

	ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    //iResult = getaddrinfo(ipaddress.c_str(), DEFAULT_PORT, &hints, &result);
	iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

	// Attempt to connect to an address until one succeeds
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return 1;
        }

        // Connect to server.
        iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return 1;
    }

	//test
	if (iResult !=0)
	{
		::std::cout << "connection error" << ::std::endl;
		return false;
	}
	else
		::std::cout << "Successfully connected to server" << std::endl;

	::std::vector<double> configuration;
	int counter = 0;
	::std::vector<::std::string> strings;
	::std::string date;

	::std::string bitFlags = "";
	::std::vector<bool> source;
	::std::vector<int> index;
	index.push_back(1);
	index.push_back(4);
	//index.push_back(5);
	index.push_back(8);


	newFile = false;
	::std::vector< ::std::string> sensorNames;
	sensorNames.push_back("ECG");
	sensorNames.push_back("Oximetry");
	sensorNames.push_back("IBP");

	do {

		::std::ostringstream ss;
		ss << counter << ::std::endl;

		counter++;

		iResult = send( ConnectSocket, ss.str().c_str(),  ss.str().size() + 1, 0 );

		Sleep(500);		

		//Receive data through the network
        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
		::std::string conf_str(recvbuf);
		
		strings = splitStr(conf_str, ',');

		source  = this->parseBitMap(strings[0]);

		if (source.size() <= 0)
			continue;
		
		//if(!checkForConsistency(strings[index[4]]))
		//	continue;
		//else
		//{
			//heartRate = atof(strings[this->source].c_str());
			//breathingRate = atof(strings[strings.size()-1].c_str());
			heartRate = atof(strings[2].c_str());
			breathingRate = atof(strings[24].c_str());
		//}


		counter++;
	
		if (logData)
		{
			date = GetDateString();
			os << strings << " " << date <<::std::endl;
		}

    } while(iResult > 0);

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

	return false;
}