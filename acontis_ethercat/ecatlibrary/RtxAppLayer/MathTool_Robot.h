#pragma once

#include "MathTool.h"
#include "Def.h"


// Base & Tool function
//===========================================================================//
void removeBase(double out_Pose[6], const double Pose[6], const int baseIndex);
void attachBase(double out_Pose[6], const double Pose[6], const int baseIndex);
void removeTool(double out_Pose[6], const double Pose[6], const int toolIndex);
void attachTool(double out_Pose[6], const double Pose[6], const int toolIndex);
//===========================================================================//


void moveABC(double out_Pose[MAX_REDUNDANCY], const double PoseAtBase[MAX_REDUNDANCY], double deltaTheta, int ind, eFrame::e frame );
void moveXYZ(double out_Pose[MAX_REDUNDANCY], const double PoseAtBase[MAX_REDUNDANCY], double deltaDist, int ind, eFrame::e frame);


//======= Circle Class for Circle-Path-Planning. ===========
class Circle
{
public:

	Circle();
	Circle(const CircleData& cData);

	int theta2XYZ(double out_Pose[6], const double deltaTheta);
	
	// 三點求 圓心, 半徑, 角度, 圓平面法向量, 起始方向
	void calculateBasicParameter(double startPose[6], double midPose[6], double endPose[6], double thetaUser);
	void getCData(CircleData& out_cData);

private:
	CircleData _cData;

	Matrix3d _startR, _midR, _endR, _deltaR, _tempR, Identity_3x3;
	//Vector3d _uNormalR;
	Vector3d _uNow, Zero_3;

};