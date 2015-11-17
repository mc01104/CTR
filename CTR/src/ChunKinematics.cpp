// ---------------------------------------------------------------- //
// CKim - My own kinematics library based on Eigen matrix library
// and Numerical recipes
// Last updated : Oct. 16, 2014 
// ---------------------------------------------------------------- //

#include "stdafx.h"
#include "ChunKinematics.h"	

namespace ChunKinematics
{
	double Dot(const Vec4& a, const Vec4& b)
	{
		double sum = 0;
		for(int i=0; i<4; i++)		{	sum += (a(i)*b(i));	}
		return sum;
	}

	Vec4 Cross(const Vec4& a, const Vec4& b)
	{
		Vec4 x;		// Initialize to zero
		x(0) = a(1)*b(2) - a(2)*b(1);		x(1) = a(2)*b(0)-a(0)*b(2);		x(2) = a(0)*b(1)-a(1)*b(0);
		return x;
	}

	Mat4 Skew(const Vec4& w)
	{
		Mat4 x = Mat4::Identity();		// CKim - Initialize to identity
		x(0,1) = -w(2);		x(1,0) = w(2);
		x(0,2) = w(0);		x(2,0) = -w(0);
		x(1,2) = -w(1);		x(2,1) = w(1);
		return x;
	}

	Mat4 Rodrigues(const Vec4& w, const double& th)
	{
		Mat4 R = Mat4::Identity();	// CKim - initializes to identity

		if(w.norm() == 0)	{	return R;	}
		else
		{
			double c = cos(th);		double cc = 1-cos(th);		double s = sin(th);
			R(0,0) = 1 - (SQR(w(1)) + SQR(w(2)))*cc;		R(0,1) = w(0)*w(1)*cc - w(2)*s;				R(0,2) = w(0)*w(2)*cc + w(1)*s;
			R(1,0) = w(0)*w(1)*cc + w(2)*s;					R(1,1) = 1 - (SQR(w(0))+SQR(w(2)))*cc;		R(1,2) = w(1)*w(2)*cc - w(0)*s;
			R(2,0) = w(0)*w(2)*cc - w(1)*s;					R(2,1) = w(1)*w(2)*cc + w(0)*s;				R(2,2) = 1 - (SQR(w(0))+SQR(w(1)))*cc;
		}
		return R;
	}

	Vec4 invRodrigues(const Mat4& G)
	{
		Vec4 w = Vec4::Zero();		// CKim - Initializes to zero
		double tr = G.trace();	
		
		if(tr==3)	{	}
		else
		{
			double th = acos( (tr-1)/2 );
			w(0) = (G(2,1) - G(1,2))/(2*sin(th));
			w(1) = (G(0,2) - G(2,0))/(2*sin(th));
			w(2) = (G(1,0) - G(0,1))/(2*sin(th));
			w(3) = th;
		}
		return w;
	}


	Mat4 TwistExp(const Vec6& xi, const double& th)
	{
		Mat4 G = Mat4::Identity();		Mat4 I = Mat4::Identity();
		Vec4 v = Vec4::Zero();			Vec4 w = Vec4::Zero();
		for(int i=0; i<3; i++)	{	v(i) = xi(i);		w(i) = xi(i+3);		}

		if(w.norm() == 0)	{	for(int i=0; i<3; i++)	{	G(i,3) = th*v(i);	}	}
		else
		{
			// CKim - R = exp(w,th)
			G = Rodrigues(w,th);

			// CKim - t = (eye(3)-R)*cross(w,v)+ dot(w,v)*th*w;
			Vec4 t = (I-G)*Cross(w,v);
			for(int i=0; i<3; i++)	{	G(i,3) = t(i) + (Dot(w,v)*th)*w(i);	}
		}
		return G;
	}
	
	Mat4 InvTf(const Mat4& G)
	{
		Mat4 X = Mat4::Identity();	
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++)	{	X(i,j) = G(j,i);	X(i,3) -= (G(j,i)*G(j,3));	}	}
		return X;
	}

	Mat6 Adjoint(const Mat4& G)
	{
		Mat6 X;

		// G = [R,t;0,1];	X(1:3,1:3) = X(4:6, 4:6) = G;
		for(int i=0; i<3; i++)	{
			for(int j=0; j<3; j++)	{	X(i,j) = X(i+3,j+3) = G(i,j);	}	}

		// G = [R,t;0,1];	X(1:3,4:6) = hat(t)*R
		for(int i=0; i<3; i++)
		{	
			X(0,3+i) = G(1,3)*G(2,i) - G(2,3)*G(1,i);		
			X(1,3+i) = G(2,3)*G(0,i) - G(0,3)*G(2,i);		
			X(2,3+i) = G(0,3)*G(1,i) - G(1,3)*G(0,i);		
		}

		return X;
	}



//
//	double PadenKahanFirst(const Vec4& w, const Vec4& x, const Vec4& y)
//	{
//		double th;
//
//		if( fabs(Dot(w,x)-Dot(w,y)) > 0.00001 )		{	th = NaN;		}
//		else
//		{
//			Vec4 xp = x - Dot(x,w)*w;		Vec4 yp = y - Dot(y,w)*w;			
//			th = atan2( Dot(w,Cross(xp,yp)), Dot(xp,yp) );
//		}
//		return th;
//	}
//
//	void PadenKahanSecond(const Vec4& w1, const Vec4& w2, const Vec4& x, const Vec4& y, double* th)
//	{
//		Vec4 tmp = Cross(w1,w2);
//		double alp = ( Dot(w1,w2)*Dot(w2,x) - Dot(w1,y) ) / ( SQR(Dot(w1,w2)) - 1 );
//		double bet = ( Dot(w1,w2)*Dot(w1,y) - Dot(w2,x) ) / ( SQR(Dot(w1,w2)) - 1 );
//
//		double gam2 = SQR(x.Norm()) - SQR(alp) - SQR(bet) - 2*alp*bet*Dot(w1,w2);
//		
//		if(gam2 < 0)	{	for(int i=0; i<4; i++)	{	th[i] = NaN;	}		}
//		else
//		{
//			double gam = sqrt(gam2) / tmp.Norm() ;
//	
//			Vec4 z1 = alp*w1 + bet*w2 + gam*tmp;
//			Vec4 z2 = alp*w1 + bet*w2 - gam*tmp;
//
//			th[0] = -PadenKahanFirst(w1,y,z1);		th[1] = PadenKahanFirst(w2,x,z1);
//			th[2] = -PadenKahanFirst(w1,y,z2);		th[3] = PadenKahanFirst(w2,x,z2);
//		}
//	}
//
}