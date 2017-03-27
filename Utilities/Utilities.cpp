#include "Utilities.h"
#include <iostream>
#include <fstream>
#include <sstream>

::std::vector<::std::string> ReadLinesFromFile(const ::std::string& pathToFile)
{
	::std::vector< ::std::string> linesVector;

	::std::ifstream inputFile(pathToFile.c_str());
	
	::std::string tempLine;
	while(::std::getline(inputFile, tempLine))
		linesVector.push_back(tempLine);

	return linesVector;
}


::std::vector< double> DoubleVectorFromString(const ::std::string& inputString)
{
	::std::istringstream ss(inputString);

	::std::vector<double> result;
	while(!ss.eof())
	{
		double tmp;
		ss >> tmp;
		result.push_back(tmp);
	}

	return result;
}

double Norm2(const ::std::vector< double>& doubleVector)
{
	double tmp = 0;

	for (::std::vector<double>::const_iterator it = doubleVector.begin(); it != doubleVector.end(); ++ it)
		tmp += ::std::pow(*it, 2);

	return ::std::sqrt(tmp);
}

::Eigen::MatrixXd PseudoInverse(const ::Eigen::MatrixXd& matrix)
{
	::Eigen::MatrixXd quad = matrix * matrix.transpose();
	
	if (quad.determinant() == 0)
		throw("matrix is close to singularity!!");

	return matrix.transpose() * quad.inverse();
}

void PseudoInverse(const ::Eigen::MatrixXd& inputMatrix, ::Eigen::MatrixXd& outputMatrix, const ::Eigen::MatrixXd& weight)
{
	Eigen::MatrixXd tmp;
	if (weight.size() > 0)
		tmp = inputMatrix.transpose() * weight.transpose() * weight * inputMatrix;
	else
		tmp = inputMatrix * inputMatrix.transpose();
	tmp = tmp.inverse();
	outputMatrix =  inputMatrix.transpose() * tmp;

}

double NormSquared(const ::std::vector<double>& input)
{
	double sum = 0;
	for(::std::vector<double>::const_iterator it = input.begin(); it != input.end(); ++it)
		sum += ::std::pow(*it, 2);

	return sum;
}

::std::string GetDateString()
{
  time_t rawtime;
  struct tm * timeinfo;

  char buffer [80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime (buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);

  return ::std::string(buffer);
}


void PrintCArray(const double* toPrint, size_t size, ::std::ostream& os)
{
	for (size_t i = 0; i < size; ++i)
		os << toPrint[i] << " ";

	os << ::std::endl;

}


::Eigen::Vector3d Vec3ToEigen(const Vec3& vec3)
{
	::Eigen::Vector3d vec3Eigen;
	for (int i = 0; i < 3; ++i)
		vec3Eigen(i) = vec3[i];

	return vec3Eigen;
}


void SO3ToEigen(const SO3& rot, ::Eigen::Matrix<double, 3, 3>& rotEigen)
{
	rotEigen.col(0) = Vec3ToEigen(rot.GetX());
	rotEigen.col(1) = Vec3ToEigen(rot.GetY());
	rotEigen.col(2) = Vec3ToEigen(rot.GetZ());
}

template <typename T> 
void circshift(T* data, size_t dataSize, int direction)
{
	T tmp;
	if (direction == 1)
	{
		tmp = data[dataSize - 1];
		memcpy(&data[1], data, sizeof(T) * dataSize - 1);
		data[0] = tmp;
	}
	else
	{		
		tmp = data[0];
		memcpy(data, &data[1], sizeof(T) * dataSize - 1);
		data[dataSize - 1] = tmp;
	}
}

//void 
//circshiftBack(double* data, size_t dataSize)
//{
//	circshift<double>(data, dataSize, -1);
//}

void 
circshiftBack(::std::deque<double>& data)
{
	double tmp = data.front();
	data.pop_front();
	data.push_back(tmp);	
}

void 
binary_from_string(::std::string& sHex, ::std::string& sReturn)
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