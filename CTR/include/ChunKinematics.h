// ---------------------------------------------------------------- //
// CKim - My own kinematics library based on Eigen matrix library
// and Numerical recipes
// Last updated : Oct. 16, 2014 
// ---------------------------------------------------------------- //

#pragma once

#ifndef CHUN_KIN

// CKim - Eigen Header. Located at "C:\Chun\ChunLib"
#include <Eigen/Dense>

typedef Eigen::Vector3d Vec3;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Matrix<double,6,1,0,6,1> Vec6;
typedef Eigen::Matrix<double,6,6,0,6,6> Mat6;

template<class T>
inline T SQR(const T a) {return a*a;}


namespace ChunKinematics
{
	double	Dot(const Vec4& a, const Vec4& b);
	Vec4	Cross(const Vec4& a, const Vec4& b);
	Mat4	Skew(const Vec4& w);

	Mat4	InvTf(const Mat4& G);	
	
	// CKim - Calculate R = exp(w,th), assumes unit w
	Mat4	Rodrigues(const Vec4& w, const double& th);
	
	// CKim - Calculates (w,th) such that R = exp(w,th). Th is stored in w[3].
	Vec4	invRodrigues(const Mat4& G);

	// CKim - Calculates G = exp(xi,th), xi = [v,w], assumes unit w
	Mat4	TwistExp(const Vec6& xi, const double& th);
	
	
	Mat6	Adjoint(const Mat4& G);

	// CKim - Solve Paden-Kahan 1st subproblem. Find theta that satisfy exp(w,th)*x = y 
	double PadenKahanFirst(const Vec4& w, const Vec4& x, const Vec4& y);
	
	// CKim - Solve Paden-Kahan 2nd subproblem. Find theta 1 and 2 that satisfy exp(w1,th1)*exp(w2,th2)*x = y. 
	// Two pair of solutions are (theta 1,2) = (th[0],th[1]) and (th[2],th[3]) 
	void PadenKahanSecond(const Vec4& w1, const Vec4& w2, const Vec4& x, const Vec4& y, double* th);

}


#define CHUN_KIN
#endif