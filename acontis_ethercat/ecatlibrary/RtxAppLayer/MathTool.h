#pragma once

#include "Eigen/Dense" //Matrix Library API found online. must to declare first, otherwise compile error.
#include <math.h>

#define PI		 3.141592653589793238462643383279	/* π */
#define DEG2RAD	 0.017453292519943295769236907684	/* π/180 */
#define RAD2DEG 57.295779513082320876798154814105	/* 180/π */
#define TINY_VALUE 1e-5

using namespace Eigen;

// trigonometric function (in degree)
//=============================================================//
double sind(const double thetaDeg);
double cosd(const double thetaDeg);
double tand(const double thetaDeg);

double asind(const double value);
double acosd(const double value);
double atand(const double value);

double asin2d(const double numerator , const double denominator);
double acos2d(const double numerator , const double denominator);
double atan2d(const double numerator , const double denominator);

int sign(double input);
//=============================================================//


// orientation represent & inverse HT matrix function
//==================================================================//
void transferT2Pose(VectorXd &out_Pose     , const Matrix4d &T      );
void transferT2Pose(double    out_Pose[6]  , const Matrix4d &T      );
void transferT2Pose(double    out_Pose[6]  , const double    T[4][4]);
void transferR2Pose(double    out_Pose[6]  , const Matrix3d &R      );

void transferPose2T(Matrix4d &out_T       , const VectorXd &Pose   );
void transferPose2T(Matrix4d &out_T       , const double    Pose[6]);
void transferPose2T(double    out_T[4][4] , const double    Pose[6]);
void transferPose2R(Matrix3d &out_R       , const double    Pose[6]);
//==================================================================//


// axis-angle method 
//=================================================================================================//
void AxisAngleMethod_R(Matrix3d &out_R, const double    axis_vector[3], const double angle_thetaDeg);
void AxisAngleMethod_R(Matrix3d &out_R, const Vector3d &axis_vector   , const double angle_thetaDeg);
void AxisAngleMethod_T(Matrix4d &out_T, const double    axis_vector[3], const double angle_thetaDeg);

bool getAxisAndAngle_R(double    out_axis_vector[3], double &out_angle_thetaDeg, const Matrix3d &R);
bool getAxisAndAngle_R(Vector3d &out_axis_vector   , double &out_angle_thetaDeg, const Matrix3d &R);
bool getAxisAndAngle_T(double    out_axis_vector[3], double &out_angle_thetaDeg, const Matrix4d &T);
//=================================================================================================//

void inverseHT(double out_invT[4][4], const double T[4][4]); // only for Homogeneous Transform Matrix
void multiplyMatrix_4x4(double out_C[4][4], const double A[4][4], const double B[4][4]); // C = A*B
void normalizeVector(double out_unitVector[3], const double vector[3]);
void normalizeVector(double out_unitVector[3], double &out_Norm, const double vector[3]);
void crossProduct(double out_c[3], double a[3], const double b[3]); // c = a X b (cross product)
double dotProduct(double a[3], const double b[3]); // return = a ． b (dot product)


void copyHT(double out_T[4][4], const double T[4][4]);
void transferArray2Eigen(Matrix4d &out_Matrix, const double T[4][4]);


void getBindedPose(
	double out_S_target_Pose[6], 
	const double M_pre_Pose[6], 
	const double M_target_Pose[6], 
	const double S_pre_Pose[6], 
	const double S_target_Pose[6], 
	const double M_World2Root_Array[4][4], 
	const double S_World2Root_Array[4][4]);



