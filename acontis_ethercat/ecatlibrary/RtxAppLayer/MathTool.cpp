#include "MathTool.h"

//====== trigonometric function (in degree)
double sind(const double thetaDeg)
{
	return sin(thetaDeg * DEG2RAD);
}
double cosd(const double thetaDeg)
{
	return cos(thetaDeg * DEG2RAD);
}
double tand(const double thetaDeg)
{
	return tan(thetaDeg * DEG2RAD);
}

double asind(const double value)
{
	if(value > 1)  return 90;
	if(value < -1) return -90;
	return asin(value) * RAD2DEG;
}
double acosd(const double value)
{
	if(value > 1)  return 0;
	if(value < -1) return 180;
	return acos(value) * RAD2DEG;
}
double atand(const double value)
{
	return atan(value) * RAD2DEG;
}

double asin2d(const double numerator, const double denominator)
{
	return asin(numerator/denominator) * RAD2DEG;
}
double acos2d(const double numerator, const double denominator)
{
	return acos(numerator/denominator) * RAD2DEG;
}
double atan2d(const double numerator, const double denominator)
{
	return atan2(numerator , denominator) * RAD2DEG;
}

int sign(double input)
{
	if(input >= 0)	return 1;
	else			return -1;
}

//====== pose (X,Y,Z,A,B,C) ←→ HT (Homogeneous Tranfer) matrix function
void transferT2Pose(VectorXd &out_Pose    , const Matrix4d &T		 )
{
	double	A_Deg, B_Deg, C_Deg;
		
#ifdef EULER
		// EULER Angle Method (Z → Y → X)
		if(fabs(T(0,0)) <= TINY_VALUE && fabs(T(1,0)) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			A_Deg = 0;
			B_Deg = atan2d( -T(2,0) , T(0,0)/cosd(A_Deg) );
			C_Deg = atan2d( -T(1,2) , T(1,1) );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d(T(1,0) , T(0,0));
			sA    = sind(A_Deg);
			cA    = cosd(A_Deg);
			B_Deg = atan2d(-T(2,0) , cA*T(0,0) + sA*T(1,0));
			C_Deg = atan2d( T(2,1) , T(2,2));
		}
#else
		// Fixed Frame Method (use Axis-Angle Method)
		if(fabs(T(1,2)) <= TINY_VALUE && fabs(T(2,2)) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			C_Deg = 0;
			B_Deg = atan2d( T(0,2) , T(2,2)/cosd(C_Deg) );
			A_Deg = atan2d( T(1,0) , T(1,1)/cosd(C_Deg) );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d( -T(0,1) , T(0,0) );
			   sA = sind(A_Deg);
			   cA = cosd(A_Deg);

			B_Deg = atan2d( T(0,2) , cA*T(0,0) - sA*T(0,1) );

			C_Deg = atan2d( -T(1,2) , T(2,2) );
		}

#endif

	out_Pose(0) = T(0,3);
	out_Pose(1) = T(1,3);
	out_Pose(2) = T(2,3);
	out_Pose(3) = A_Deg;
	out_Pose(4) = B_Deg;
	out_Pose(5) = C_Deg;
}	   
void transferT2Pose(double    out_Pose[6] , const Matrix4d &T		 )
{
	double	A_Deg, B_Deg, C_Deg;
		
#ifdef EULER
		// EULER Angle Method (Z → Y → X)
		//========================================================//
		if(fabs(T(0,0)) <= TINY_VALUE && fabs(T(1,0)) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			A_Deg = 0;
			B_Deg = atan2d( -T(2,0) , T(0,0)/cosd(A_Deg) );
			C_Deg = atan2d( -T(1,2) , T(1,1) );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d(T(1,0) , T(0,0));
			sA    = sind(A_Deg);
			cA    = cosd(A_Deg);
			B_Deg = atan2d(-T(2,0) , cA*T(0,0) + sA*T(1,0));
			C_Deg = atan2d( T(2,1) , T(2,2));
		}
		//========================================================//
#else

		// Fixed Frame Method (use Axis-Angle Method)
		//========================================================//
		if(fabs(T(1,2)) <= TINY_VALUE && fabs(T(2,2)) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			C_Deg = 0;
			B_Deg = atan2d( T(0,2) , T(2,2)/cosd(C_Deg) );
			A_Deg = atan2d( T(1,0) , T(1,1)/cosd(C_Deg) );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d( -T(0,1) , T(0,0) );
			   sA = sind(A_Deg);
			   cA = cosd(A_Deg);

			B_Deg = atan2d( T(0,2) , cA*T(0,0) - sA*T(0,1) );

			C_Deg = atan2d( -T(1,2) , T(2,2) );
		}
		//========================================================//
#endif

	out_Pose[0] = T(0,3);
	out_Pose[1] = T(1,3);
	out_Pose[2] = T(2,3);
	out_Pose[3] = A_Deg;
	out_Pose[4] = B_Deg;
	out_Pose[5] = C_Deg;
}		   
void transferT2Pose(double    out_Pose[6] , const double    T[4][4])
{
	double	A_Deg, B_Deg, C_Deg;
		
#ifdef EULER
		// EULER Angle Method (Z → Y → X)
		//==========================================================//
		if(fabs(T[0][0]) <= TINY_VALUE && fabs(T[1][0]) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			A_Deg = 0;
			B_Deg = atan2d( -T[2][0] , T[0][0]/cosd(A_Deg) );
			C_Deg = atan2d( -T[1][2] , T[1][1] );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d(T[1][0] , T[0][0]);
			sA    = sind(A_Deg);
			cA    = cosd(A_Deg);
			B_Deg = atan2d(-T[2][0] , cA*T[0][0] + sA*T[1][0]);
			C_Deg = atan2d( T[2][1] , T[2][2]);
		}
		//==========================================================//
#else

		// Fixed Frame Method (use Axis-Angle Method)
		//==========================================================//
		if(fabs(T[1][2]) <= TINY_VALUE && fabs(T[2][2]) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			C_Deg = 0;
			B_Deg = atan2d( T[0][2] , T[2][2]/cosd(C_Deg) );
			A_Deg = atan2d( T[1][0] , T[1][1]/cosd(C_Deg) );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d( -T[0][1] , T[0][0] );
			   sA = sind(A_Deg);
			   cA = cosd(A_Deg);

			B_Deg = atan2d( T[0][2] , cA*T[0][0] - sA*T[0][1] );

			C_Deg = atan2d( -T[1][2] , T[2][2] );
		}
		//==========================================================//
#endif

	out_Pose[0] = T[0][3];
	out_Pose[1] = T[1][3];
	out_Pose[2] = T[2][3];
	out_Pose[3] = A_Deg;
	out_Pose[4] = B_Deg;
	out_Pose[5] = C_Deg;
}	   
void transferR2Pose(double    out_Pose[6] , const Matrix3d &R		 )
{
	double	A_Deg, B_Deg, C_Deg;
		
#ifdef EULER
		// EULER Angle Method (Z → Y → X)
		//========================================================//
		if(fabs(R(0,0)) <= TINY_VALUE && fabs(R(1,0)) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			A_Deg = 0;
			B_Deg = atan2d( -R(2,0) , R(0,0)/cosd(A_Deg) );
			C_Deg = atan2d( -R(1,2) , R(1,1) );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d(R(1,0) , R(0,0));
			sA    = sind(A_Deg);
			cA    = cosd(A_Deg);
			B_Deg = atan2d(-R(2,0) , cA*R(0,0) + sA*R(1,0));
			C_Deg = atan2d( R(2,1) , R(2,2));
		}
		//========================================================//
#else
		// Fixed Frame Method (use Axis-Angle Method)
		//========================================================//
		if(fabs(R(1,2)) <= TINY_VALUE && fabs(R(2,2)) <= TINY_VALUE) // singular : B = ±90 (Gimbal Lock)
		{	
			C_Deg = 0;
			B_Deg = atan2d( R(0,2) , R(2,2)/cosd(C_Deg) );
			A_Deg = atan2d( R(1,0) , R(1,1)/cosd(C_Deg) );
		}
		else
		{
			double sA,cA;
			A_Deg = atan2d( -R(0,1) , R(0,0) );
			   sA = sind(A_Deg);
			   cA = cosd(A_Deg);

			B_Deg = atan2d( R(0,2) , cA*R(0,0) - sA*R(0,1) );

			C_Deg = atan2d( -R(1,2) , R(2,2) );
		}
#endif

	out_Pose[3] = A_Deg;
	out_Pose[4] = B_Deg;
	out_Pose[5] = C_Deg;
}

void transferPose2T(Matrix4d &out_T		  , const VectorXd &Pose   )
{
	double	sA,sB,sC,cA,cB,cC;
	sA = sind( Pose(3)/* A */ );    sB = sind( Pose(4)/* B*/ );    sC = sind( Pose(5)/* C */ );
	cA = cosd( Pose(3)/* A */ );    cB = cosd( Pose(4)/* B*/ );    cC = cosd( Pose(5)/* C */ );
	
#ifdef EULER
		// EULER Angle Method : Z(A) → Y(B) → X(C)
		//============================================================================================================//
		out_T(0,0) = cA*cB;    out_T(0,1) = cA*sB*sC - sA*cC;    out_T(0,2) = cA*sB*cC + sA*sC;    out_T(0,3) = Pose(0)/* X */;
		out_T(1,0) = sA*cB;    out_T(1,1) = sA*sB*sC + cA*cC;    out_T(1,2) = sA*sB*cC - cA*sC;    out_T(1,3) = Pose(1)/* Y */;
		out_T(2,0) =   -sB;    out_T(2,1) =			   cB*sC;    out_T(2,2) =			 cB*cC;    out_T(2,3) = Pose(2)/* Z */;  
		out_T(3,0) =	 0;	   out_T(3,1) =				   0;	 out_T(3,2) =				 0;	   out_T(3,3) =		 1;
#else
		// Fixed Frame Method (use Axis-Angle Method) : Z(A) → Y(B) → X(C)
		//===============================================================================================================//
		out_T(0,0) =			 cA*cB;    out_T(0,1) =			  - sA*cB;    out_T(0,2) =	   sB;    out_T(0,3) = Pose(0)/* X */;
		out_T(1,0) =  cA*sB*sC + sA*cC;    out_T(1,1) = -sA*sB*sC + cA*cC;    out_T(1,2) = -cB*sC;    out_T(1,3) = Pose(1)/* Y */;
		out_T(2,0) = -cA*sB*cC + sA*sC;    out_T(2,1) =	 sA*sB*cC + cA*sC;    out_T(2,2) =	cB*cC;    out_T(2,3) = Pose(2)/* Z */; 
		out_T(3,0) =				 0;	   out_T(3,1) =					0;	  out_T(3,2) =		0;	  out_T(3,3) =		1;
#endif
}	
void transferPose2T(Matrix4d &out_T		  , const double	Pose[6])
{
	if(Pose == 0) { out_T = Matrix4d::Identity(); return; }

	double	sA,sB,sC,cA,cB,cC;
	sA = sind( Pose[3]/* A */ );    sB = sind( Pose[4]/* B */ );    sC = sind( Pose[5]/* C */ );
	cA = cosd( Pose[3]/* A */ );    cB = cosd( Pose[4]/* B */ );    cC = cosd( Pose[5]/* C */ );

#ifdef EULER
		// EULER Angle Method : Z(A) -> Y(B) -> X(C)
		//===========================================================================================================//
		out_T(0,0) = cA*cB;    out_T(0,1) = cA*sB*sC - sA*cC;    out_T(0,2) = cA*sB*cC + sA*sC;    out_T(0,3) = Pose[0]/* X */;
		out_T(1,0) = sA*cB;    out_T(1,1) = sA*sB*sC + cA*cC;    out_T(1,2) = sA*sB*cC - cA*sC;    out_T(1,3) = Pose[1]/* Y */;
		out_T(2,0) =   -sB;    out_T(2,1) =			   cB*sC;    out_T(2,2) =			 cB*cC;    out_T(2,3) = Pose[2]/* Z */;  
		out_T(3,0) =	 0;	   out_T(3,1) =				   0;	 out_T(3,2) =				 0;	   out_T(3,3) =		 1;
#else
		// Fixed Frame Method (use Axis-Angle Method)
		//==============================================================================================================//
		out_T(0,0) =			 cA*cB;    out_T(0,1) =			  - sA*cB;    out_T(0,2) =	   sB;    out_T(0,3) = Pose[0]/* X */;
		out_T(1,0) =  cA*sB*sC + sA*cC;    out_T(1,1) = -sA*sB*sC + cA*cC;    out_T(1,2) = -cB*sC;    out_T(1,3) = Pose[1]/* Y */;
		out_T(2,0) = -cA*sB*cC + sA*sC;    out_T(2,1) =	 sA*sB*cC + cA*sC;    out_T(2,2) =	cB*cC;    out_T(2,3) = Pose[2]/* Z */; 
		out_T(3,0) =				 0;	   out_T(3,1) =					0;	  out_T(3,2) =		0;	  out_T(3,3) =		1;
#endif

}		
void transferPose2T(double	  out_T[4][4] , const double	Pose[6])
{
	double	sA,sB,sC,cA,cB,cC;
	sA = sind( Pose[3]/* A */ );    sB = sind( Pose[4]/* B */ );    sC = sind( Pose[5]/* C */ );
	cA = cosd( Pose[3]/* A */ );    cB = cosd( Pose[4]/* B */ );    cC = cosd( Pose[5]/* C */ );

#ifdef EULER
		// EULER Angle Method : Z(A) -> Y(B) -> X(C)
		//===============================================================================================================//
		out_T[0][0] = cA*cB;    out_T[0][1] = cA*sB*sC - sA*cC;    out_T[0][2] = cA*sB*cC + sA*sC;    out_T[0][3] = Pose[0]/* X */;
		out_T[1][0] = sA*cB;    out_T[1][1] = sA*sB*sC + cA*cC;    out_T[1][2] = sA*sB*cC - cA*sC;    out_T[1][3] = Pose[1]/* Y */;
		out_T[2][0] =   -sB;    out_T[2][1] =			 cB*sC;    out_T[2][2] =			cB*cC;    out_T[2][3] = Pose[2]/* Z */;  
		out_T[3][0] =	  0;    out_T[3][1] =				 0;    out_T[3][2] =				0;    out_T[3][3] =		 1;
#else
		// Fixed Frame Method (use Axis-Angle Method)
		//==================================================================================================================//
		out_T[0][0] =			  cA*cB;    out_T[0][1] =		    - sA*cB;    out_T[0][2] =	  sB;    out_T[0][3] = Pose[0]/* X */;
		out_T[1][0] =  cA*sB*sC + sA*cC;    out_T[1][1] = -sA*sB*sC + cA*cC;    out_T[1][2] = -cB*sC;    out_T[1][3] = Pose[1]/* Y */;
		out_T[2][0] = -cA*sB*cC + sA*sC;    out_T[2][1] =  sA*sB*cC + cA*sC;    out_T[2][2] =  cB*cC;    out_T[2][3] = Pose[2]/* Z */; 
		out_T[3][0] =				  0;    out_T[3][1] =				  0;	out_T[3][2] =	   0;    out_T[3][3] =		1;
#endif
}		
void transferPose2R(Matrix3d &out_R		  , const double	Pose[6])
{
	double	sA,sB,sC,cA,cB,cC;
	sA = sind( Pose[3]/* A */ );    sB = sind( Pose[4]/* B */ );    sC = sind( Pose[5]/* C */ );
	cA = cosd( Pose[3]/* A */ );    cB = cosd( Pose[4]/* B */ );    cC = cosd( Pose[5]/* C */ );

#ifdef EULER
		// EULER Angle Method : Z(A) -> Y(B) -> X(C)
		//===================================================================================//
		out_R(0,0) = cA*cB;    out_R(0,1) = cA*sB*sC - sA*cC;    out_R(0,2) = cA*sB*cC + sA*sC;
		out_R(1,0) = sA*cB;    out_R(1,1) = sA*sB*sC + cA*cC;    out_R(1,2) = sA*sB*cC - cA*sC;
		out_R(2,0) =   -sB;    out_R(2,1) =			   cB*sC;    out_R(2,2) =			 cB*cC;
#else
		// Fixed Frame Method (use Axis-Angle Method)
		//======================================================================================//
		out_R(0,0) =			 cA*cB;    out_R(0,1) =			  - sA*cB;    out_R(0,2) =	   sB;
		out_R(1,0) =  cA*sB*sC + sA*cC;    out_R(1,1) = -sA*sB*sC + cA*cC;    out_R(1,2) = -cB*sC;
		out_R(2,0) = -cA*sB*cC + sA*sC;    out_R(2,1) =	 sA*sB*cC + cA*sC;    out_R(2,2) =	cB*cC;
#endif
}

//====== axis-angle method  旋轉向量表示法 -> R矩陣
void AxisAngleMethod_R(Matrix3d &out_R, const double    axis_vector[3], const double angle_thetaDeg)
{
	double ux, uy, uz, ss, cs;
	double theta;
	theta = angle_thetaDeg;

	ux = axis_vector[0];
	uy = axis_vector[1];
	uz = axis_vector[2];

	if(ux==0 && uy==0 && uz==0) theta = 0;

	ss = sind(theta);
	cs = cosd(theta);

	
	out_R(0,0) = (ux*ux)*(1 - cs) +    cs;    out_R(0,1) = (ux*uy)*(1 - cs) - uz*ss;    out_R(0,2) = (ux*uz)*(1 - cs) + uy*ss;
	out_R(1,0) = (uy*ux)*(1 - cs) + uz*ss;    out_R(1,1) = (uy*uy)*(1 - cs) +    cs;    out_R(1,2) = (uy*uz)*(1 - cs) - ux*ss;
	out_R(2,0) = (uz*ux)*(1 - cs) - uy*ss;    out_R(2,1) = (uz*uy)*(1 - cs) + ux*ss;    out_R(2,2) = (uz*uz)*(1 - cs) +    cs;
}
void AxisAngleMethod_R(Matrix3d &out_R, const Vector3d &axis_vector   , const double angle_thetaDeg)
{
	double ux, uy, uz, ss, cs;
	double theta;
	theta = angle_thetaDeg;

	ux = axis_vector[0];
	uy = axis_vector[1];
	uz = axis_vector[2];

	if(ux==0 && uy==0 && uz==0) theta = 0;

	ss = sind(theta);
	cs = cosd(theta);

	out_R(0,0) = (ux*ux)*(1 - cs) +    cs;    out_R(0,1) = (ux*uy)*(1 - cs) - uz*ss;    out_R(0,2) = (ux*uz)*(1 - cs) + uy*ss;
	out_R(1,0) = (uy*ux)*(1 - cs) + uz*ss;    out_R(1,1) = (uy*uy)*(1 - cs) +    cs;    out_R(1,2) = (uy*uz)*(1 - cs) - ux*ss;
	out_R(2,0) = (uz*ux)*(1 - cs) - uy*ss;    out_R(2,1) = (uz*uy)*(1 - cs) + ux*ss;    out_R(2,2) = (uz*uz)*(1 - cs) +    cs;
}
void AxisAngleMethod_T(Matrix4d &out_T, const double    axis_vector[3], const double angle_thetaDeg)
{
	double ux, uy, uz, ss, cs;
	double theta;
	theta = angle_thetaDeg;

	ux = axis_vector[0];
	uy = axis_vector[1];
	uz = axis_vector[2];

	if(ux==0 && uy==0 && uz==0) theta = 0;

	ss = sind(theta);
	cs = cosd(theta);

	out_T(0,0) = (ux*ux)*(1 - cs) +    cs;    out_T(0,1) = (ux*uy)*(1 - cs) - uz*ss;    out_T(0,2) = (ux*uz)*(1 - cs) + uy*ss;    out_T(0,3) = 0;
	out_T(1,0) = (uy*ux)*(1 - cs) + uz*ss;    out_T(1,1) = (uy*uy)*(1 - cs) +    cs;    out_T(1,2) = (uy*uz)*(1 - cs) - ux*ss;    out_T(1,3) = 0;
	out_T(2,0) = (uz*ux)*(1 - cs) - uy*ss;    out_T(2,1) = (uz*uy)*(1 - cs) + ux*ss;    out_T(2,2) = (uz*uz)*(1 - cs) +    cs;    out_T(2,3) = 0;
	out_T(3,0) =						0;	  out_T(3,1) =						  0;	out_T(3,2) =						0;    out_T(3,3) = 1;
}

bool getAxisAndAngle_R(double    out_axis_vector[3], double &out_angle_thetaDeg, const Matrix3d &R)
{
	out_angle_thetaDeg = acosd( (R(0,0) + R(1,1) + R(2,2) - 1)/2 );

	if(fabs(out_angle_thetaDeg) < TINY_VALUE || fabs(out_angle_thetaDeg - 180) < TINY_VALUE)
	{
		out_axis_vector[0] = 0;
		out_axis_vector[1] = 0;
		out_axis_vector[2] = 0;
		out_angle_thetaDeg = 0;
		return false;
	}
	else
	{
		out_axis_vector[0] = ( R(2,1) - R(1,2) ) / (2*sind(out_angle_thetaDeg));
		out_axis_vector[1] = ( R(0,2) - R(2,0) ) / (2*sind(out_angle_thetaDeg));
		out_axis_vector[2] = ( R(1,0) - R(0,1) ) / (2*sind(out_angle_thetaDeg));

		return true;
	}
}
bool getAxisAndAngle_R(Vector3d &out_axis_vector   , double &out_angle_thetaDeg, const Matrix3d &R)
{
	out_angle_thetaDeg = acosd( (R(0,0) + R(1,1) + R(2,2) - 1)/2 );

	if(fabs(out_angle_thetaDeg) < TINY_VALUE || fabs(out_angle_thetaDeg - 180) < TINY_VALUE)
	{
		out_axis_vector[0] = 0;
		out_axis_vector[1] = 0;
		out_axis_vector[2] = 0;
		out_angle_thetaDeg = 0;
		return false;
	}
	else
	{
		out_axis_vector(0) = ( R(2,1) - R(1,2) ) / (2*sind(out_angle_thetaDeg));
		out_axis_vector(1) = ( R(0,2) - R(2,0) ) / (2*sind(out_angle_thetaDeg));
		out_axis_vector(2) = ( R(1,0) - R(0,1) ) / (2*sind(out_angle_thetaDeg));

		return true;
	}
}
bool getAxisAndAngle_T(double    out_axis_vector[3], double &out_angle_thetaDeg, const Matrix4d &T)
{
	out_angle_thetaDeg = acosd( (T(0,0) + T(1,1) + T(2,2) - 1)/2 );

	if(fabs(out_angle_thetaDeg) < TINY_VALUE || fabs(out_angle_thetaDeg - 180) < TINY_VALUE)
	{
		out_axis_vector[0] = 0;
		out_axis_vector[1] = 0;
		out_axis_vector[2] = 0;
		out_angle_thetaDeg = 0;
		return false;
	}
	else
	{
		out_axis_vector[0] = ( T(2,1) - T(1,2) ) / (2*sind(out_angle_thetaDeg));
		out_axis_vector[1] = ( T(0,2) - T(2,0) ) / (2*sind(out_angle_thetaDeg));
		out_axis_vector[2] = ( T(1,0) - T(0,1) ) / (2*sind(out_angle_thetaDeg));

		return true;
	}
}


void copyHT(double out_T[4][4], const double T[4][4])
{
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			out_T[i][j] = T[i][j];
}
void transferArray2Eigen(Matrix4d &out_Matrix, const double T[4][4])
{
	for(int i = 0; i<4; i++)
		for(int j = 0; j<4; j++)
			out_Matrix(i,j) = T[i][j];
}

//====== matrix (4x4) and vector tool functuin
void inverseHT(double out_invT[4][4], const double T[4][4]) // only for homogeneous transformation matrix using (much fast than general inverse)
{
    out_invT[0][0] = T[0][0];    out_invT[0][1] = T[1][0];    out_invT[0][2] = T[2][0];    out_invT[0][3] = - T[0][3]*T[0][0] - T[1][3]*T[1][0] - T[2][3]*T[2][0];
	out_invT[1][0] = T[0][1];    out_invT[1][1] = T[1][1];    out_invT[1][2] = T[2][1];    out_invT[1][3] = - T[0][3]*T[0][1] - T[1][3]*T[1][1] - T[2][3]*T[2][1];
	out_invT[2][0] = T[0][2];    out_invT[2][1] = T[1][2];    out_invT[2][2] = T[2][2];    out_invT[2][3] = - T[0][3]*T[0][2] - T[1][3]*T[1][2] - T[2][3]*T[2][2];
	out_invT[3][0] =	   0;    out_invT[3][1] =		0;    out_invT[3][2] =	     0;    out_invT[3][3] =														1;
}
void multiplyHT(double out_C[4][4], const double A[4][4], const double B[4][4])
{
	int i, j, k;
    
	// Do 3x3 R matrix
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 4; j++)
		{
			out_C[i][j] = 0.0; // initialize the element
			for(k = 0; k < 3; k++)
			{
				out_C[i][j] += A[i][k] * B[k][j];
            }
		}
    }
	
	// last column
	for(i = 0; i < 3; i++)
	{
		out_C[i][3] += A[i][3];
	}

	// last row
	out_C[3][0] = 0;
	out_C[3][1] = 0;
	out_C[3][2] = 0;
	out_C[3][3] = 1;
}
void multiplyMatrix_4x4(double out_C[4][4], const double A[4][4], const double B[4][4])
{
    int i, j, k;
    
	for(i = 0; i < 4; i++)
	{
		for(j = 0; j < 4; j++)
		{
			out_C[i][j] = 0.0; // initialize the element
			for(k = 0; k < 4; k++)
			{
				out_C[i][j] += A[i][k] * B[k][j];
            }
		}
    }
}
void normalizeVector(double out_unitVector[3], const double vector[3])
{
	int i;
	double norm = sqrt( pow(vector[0],2) + pow(vector[1],2) + pow(vector[2],2) );

	if(norm < TINY_VALUE)
	{
		for(i = 0; i < 3; i++)
			out_unitVector[i] = 0;
	}
	else
	{
		for(i = 0; i < 3; i++)
			out_unitVector[i] = vector[i] / norm;
	}
}
void normalizeVector(double out_unitVector[3], double &out_Norm, const double vector[3])
{	
	out_Norm = sqrt( pow(vector[0],2) + pow(vector[1],2) + pow(vector[2],2) );

	if(out_Norm < TINY_VALUE)
	{
		for(int i = 0; i < 3; i++)
			out_unitVector[i] = 0;		
	}
	else
	{
		for(int i = 0; i < 3; i++)
			out_unitVector[i] = vector[i] / out_Norm;
	}
}
void crossProduct(double out_c[3], const double a[3], const double b[3])
{
	out_c[0] = a[1]*b[2] - a[2]*b[1];
	out_c[1] = a[2]*b[0] - a[0]*b[2];
	out_c[2] = a[0]*b[1] - a[1]*b[0];
}
double dotProduct(double a[3], const double b[3])
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

//Note: this function use double[4][4] instead of Eigen Marix. 
//      would save 0.02ms on inverse().
void getBindedPose(double out_S_targetPose[6], 
	const double M_pre_Pose[6], const double M_target_Pose[6], 
	const double S_pre_Pose[6], const double S_target_Pose[6], 
	const double M_World2Root_T[4][4], const double S_World2Root_T[4][4])
{
	// 此部分的變數全部改成 class 的 member 變數
	//-----------------------------//
	double    S_pre_Root2Flange_T[4][4];
	double    M_pre_Root2Flange_T[4][4];
	double S_target_Root2Flange_T[4][4];
	double M_target_Root2Flange_T[4][4];

	double S_delta_T[4][4];//delta = pre to target
	double M_delta_T[4][4];

	double S_pre_World2Flange_T[4][4];
	double M_pre_World2Flange_T[4][4];

	double CoupleRelation_T[4][4];

	double out_S_target_T[4][4];
	double temp_T1[4][4];
	double temp_T2[4][4];
	double temp_T3[4][4];
	//-----------------------------//

	// transfer input Pose to T
	transferPose2T(S_pre_Root2Flange_T , S_pre_Pose);
	transferPose2T(M_pre_Root2Flange_T , M_pre_Pose);

	transferPose2T(S_target_Root2Flange_T , S_target_Pose);
	transferPose2T(M_target_Root2Flange_T , M_target_Pose);


	// -------------- calculate the T for between pre & target T ---------

	//Note: S_delta_T = S_pre_Root2Flange_T.inverse() * S_target_Root2Flange_T;
	inverseHT(temp_T1, S_pre_Root2Flange_T);
	multiplyHT(S_delta_T, temp_T1, S_target_Root2Flange_T);

	//Note: M_delta_T = M_pre_Root2Flange_T.inverse() * M_target_Root2Flange_T;
	inverseHT(temp_T1, M_pre_Root2Flange_T);
	multiplyHT(M_delta_T, temp_T1, M_target_Root2Flange_T);
	


	// --------------- transfer root T tool -> world T tool --------------

	//Note: S_pre_World2Flange_T = S_World2Root_T * S_pre_Root2Flange_T;
	//Note: M_pre_World2Flange_T = M_World2Root_T * M_pre_Root2Flange_T;
	multiplyHT(S_pre_World2Flange_T, S_World2Root_T, S_pre_Root2Flange_T);
	multiplyHT(M_pre_World2Flange_T, M_World2Root_T, M_pre_Root2Flange_T);

	// ---------------- calculate the relationship between S_tool & M_tool --------
	//Note: CoupleRelation_T = S_pre_World2Flange_T.inverse() * M_pre_World2Flange_T; // S_flange T M_flange (pre)
	inverseHT(temp_T1, S_pre_World2Flange_T);
	multiplyHT(CoupleRelation_T, temp_T1, M_pre_World2Flange_T);
	

	// ---------------- calculate the modified S_target Pose -------------------

	/*  Note: 
	[Master-Slave]
	out_S_target_T = S_pre_Root2Flange_T * CoupleRelation_T * M_delta_T * CoupleRelation_T.inverse() * S_delta_T;
	*/

	inverseHT(temp_T1, CoupleRelation_T);
	multiplyHT(temp_T2, temp_T1, S_delta_T);
	multiplyHT(temp_T1, M_delta_T, temp_T2);
	multiplyHT(temp_T2, CoupleRelation_T, temp_T1);
	multiplyHT(out_S_target_T, S_pre_Root2Flange_T, temp_T2);
	
	// --------------- transfer back to Pose. -------------------
	transferT2Pose(out_S_targetPose , out_S_target_T);
}




