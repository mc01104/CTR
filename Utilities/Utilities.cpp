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

::std::vector<::std::string> splitString(const ::std::string& inputStr)
{
	::std::istringstream ss(inputStr);

	::std::vector<::std::string> result;
	while(!ss.eof())
	{
		::std::string tmp;
		ss >> tmp;
		result.push_back(tmp);
	}
	result.pop_back();
	return result;
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
	result.pop_back();
	return result;
}


void fitCircle(::std::vector<::Eigen::Vector3d>& points, ::Eigen::Vector3d& center, double& radius)
{
	::Eigen::MatrixXd A_fit(points.size(), 3);
	::Eigen::VectorXd b_fit(points.size(), 1);
	for (int i = 0; i < points.size(); ++i)
	{
		A_fit(i, 0) = points[i][0];
		A_fit(i, 1) = points[i][1];
		A_fit(i, 2) = 1.0;

		b_fit(i) = - ::std::pow(points[i][0],2) - ::std::pow(points[i][1],2);
	}

	::Eigen::VectorXd x = (A_fit.transpose() * A_fit).inverse() * A_fit.transpose() * b_fit;
	center[0] = -x(0)/2.0;
	center[1] = -x(1)/2.0;
	center[2] = 0;

	radius = ::std::sqrt((::std::pow(x(0), 2) + ::std::pow(x(1), 2))/4.0 - x(2));
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}


::std::map<::std::string, double>  createMapFromKeyValuePairs(const ::std::string& msgToParse)
{
	::std::vector<::std::string> strVector = splitString(msgToParse);

	assert(strVector.size() % 2 == 0);

	::std::map<::std::string, double> result;
	for (int i = 0; i < strVector.size(); ++i)
		result[strVector[i].c_str()] = (double) atof(strVector[++i].c_str());

	return result;
}