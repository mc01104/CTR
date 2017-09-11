#pragma once
#include "time.h"
#include <string>
#include <vector>
#include <map>
#include <deque>
#include <iostream>
#include <Eigen/Dense>
#include "LieGroup.h"

::std::vector< ::std::string> ReadLinesFromFile(const ::std::string& pathToFile);


::std::vector< double> DoubleVectorFromString(const ::std::string& inputString);

::std::vector< double> DoubleVectorFromString(const ::std::string& inputString, char delim);

::std::vector<::std::string> splitString(const ::std::string& inputStr);

template <class T>
::std::vector<T> operator-(const ::std::vector<T>& lhs, const ::std::vector<T>& rhs)
{
		
	if (lhs.size() != rhs.size())
		throw("this operation requires vector of the same size");

	::std::vector<T> result;

	for (size_t i = 0; i < lhs.size(); ++i)
		result.push_back(lhs[i] - rhs[i]);

	return result;
};

std::vector<std::string> splitString(const std::string &s, char delim); 

template <class T>
::std::vector<T>& operator/=(::std::vector< T>& lhs, const ::std::vector< T>& rhs)
{
		
	if (lhs.size() != rhs.size())
		throw("this operation requires vector of the same size");

	for (size_t i = 0; i < lhs.size(); ++i)
	{
		if (rhs[i] == 0.0)
			throw("cannot divide by zero");

		lhs[i] = lhs[i] / rhs[i];
	}

};

template <class T>
::std::vector<T>& operator /= (::std::vector<T>& lhs, const double rhs)
{
	for (size_t i = 0; i < lhs.size(); ++i)
	{
		if (rhs == 0.0)
			throw("cannot divide by zero");

		lhs[i] = lhs[i] / rhs;
	}
}

template <class T>
::std::vector<T>& operator*=(::std::vector< T>& lhs, const ::std::vector< T>& rhs)
{
		
	if (lhs.size() != rhs.size())
		throw("this operation requires vector of the same size");

	for (size_t i = 0; i < lhs.size(); ++i)
		lhs[i] = lhs[i] * rhs[i];

};


template <class T>
::std::vector<T> operator/(const ::std::vector< T>& lhs, const ::std::vector< T>& rhs)
{
		
	if (lhs.size() != rhs.size())
		throw("this operation requires vector of the same size");

	::std::vector<T> result;

	for (size_t i = 0; i < lhs.size(); ++i)
	{
		if (rhs[i] == 0.0)
			throw("cannot divide by zero");

		result.push_back(lhs[i] / rhs[i]);
	}

	return result;
};


template <class T>
void PrintVector(const ::std::vector<T>& vectorToBePrinted)
{
	for(::std::vector<T>::const_iterator it = vectorToBePrinted.begin(); it !=  vectorToBePrinted.end(); ++it)
		::std::cout << *it << " ";

	::std::cout << ::std::endl;
};


double Norm2(const ::std::vector< double>& doubleVector);


::Eigen::MatrixXd PseudoInverse(const ::Eigen::MatrixXd& matrix);


void PseudoInverse(const ::Eigen::MatrixXd& inputMatrix, ::Eigen::MatrixXd& outputMatrix, const ::Eigen::MatrixXd& weight = ::Eigen::MatrixXd());


template <typename Derived>
::std::ostream& operator<<(::std::ostream& os, const ::Eigen::EigenBase<Derived>& toPrint)
{
	::Eigen::IOFormat OctaveFmt(::Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	os << toPrint << ::std::endl;

	return os;
}


double NormSquared(const ::std::vector<double>& input);


template <typename T>
::std::ostream& operator<<(::std::ostream& os, const ::std::vector<T>& toPrint)
{
	for (::std::vector<T>::const_iterator it = toPrint.begin(); it != toPrint.end(); ++it)
		os << *it << " ";
	os << ::std::endl;

	return os;
};


::std::string GetDateString();

void PrintCArray(const double* toPrint, size_t size, ::std::ostream& os = ::std::cout);

void SO3ToEigen(const SO3& rot, ::Eigen::Matrix<double, 3, 3>& rotEigen);

::Eigen::Vector3d Vec3ToEigen(const Vec3& vec3);

template<typename T> void circshift(T* data, int direction = 1);

//void circshiftBack(double* data, size_t dataSize);

void circshiftBack(::std::deque<double>& data);

template <typename T> 
void diff(::std::deque<T> data_in, ::std::vector<T>& data_out)
{
	if(data_out.size() != data_in.size() - 1)
		data_out.resize(data_in.size() - 1);

	for(int i = 0; i < data_in.size(); ++i)
		data_out[i] = data_in[i+1] - data_in[i];
};

template <typename T> 
void diff(::std::vector<T> data_in, ::std::vector<T>& data_out)
{
	if(data_out.size() != data_in.size() - 1)
		data_out.resize(data_in.size() - 1);

	for(int i = 0; i < data_in.size() - 1; ++i)
		data_out[i] = data_in[i+1] - data_in[i];
};


template <typename Iterator, typename T>
void find_all(Iterator it_start, Iterator it_end, T value, ::std::vector<int>& indices)
{
	indices.resize(0);

	int counter = 0;
	for (Iterator it = it_start; it != it_end; ++it)
	{
		if (*it == value)
			indices.push_back(counter);

		counter++;
	}

}

void binary_from_string(::std::string& sHex, ::std::string& sReturn);

::std::vector<::std::string> splitStr(::std::string inputStr, char delim);

void fitCircle(::std::vector<::Eigen::Vector3d>& points, ::Eigen::Vector3d& center, double& radius); 

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);

template <typename K, typename V>
::std::ostream& operator <<(::std::ostream& lhs, const ::std::map<K, V>& rhs)
{
	::std::map<K, V>::const_iterator it = rhs.begin();
	::std::map<K, V>::const_iterator it_end = rhs.end();

	int num_of_elements = rhs.size();
	int i = 0;
	for (it; it != it_end; ++it)
	{
		if (i < num_of_elements - 1)
			lhs << it->first << " " << it->second << " ";
		else
			lhs << it->first << " " << it->second;
		++i;
	}

	return lhs;
};

::std::map<::std::string, double>  createMapFromKeyValuePairs(const ::std::string& msgToParse);

template <typename T>
::std::string num2str(T& inputVariable)
{
	::std::ostringstream convert;   

	convert << inputVariable;   

	return convert.str();
}

template<typename Out>
void splitString(const std::string &s, char delim, Out result) 
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) 
        *(result++) = item;

}