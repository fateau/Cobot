#pragma once

#include "Def.h"

using namespace Eigen;
using namespace std;

namespace eKin
{
	enum e{
		COMPLETE,				
		OUT_OF_WORKSPACE,
		SPACE_LIMIT,
		ANGLE_LIMIT,
		OVERHEAD_SINGULAR,
		EXTENDED_SINGULAR, //add by yunyu 20250502
		WRIST_SINGULAR
	};
};

struct RobPose
{
	double Thetas[MAX_MOTOR_PER_ROBOT];
	double Pose[MAX_REDUNDANCY];
};
struct ThetaSolution
{
	// 8 solutions + 1 default solution 
	double	totalThetas[9][MAX_MOTOR_PER_ROBOT];
	eKin::e ret[9];	//解IK的成功與否

};
struct WristPosition
{
	double x;
	double y;
	double z;
};

class Kinematics
{
public:
	Kinematics(int rInd);
	~Kinematics(){}
	
	eKin::e FK(double  out_Pose[], double thetas[], int toolIndex);		// 正向 //檢查角度極限->去掉thetaShift->核心正向運動學->加上 Tool
	eKin::e IK(double* out_Thetas, const double* pose, int toolIndex);	// 逆向 //移除 Tool->核心逆向運動學->加上thetaShift->檢查角度極限

	bool isReachAngleLimit(const double angleDeg , int axisInd);
	bool isReachAngleLimitAll(const double angleDeg[] , int length);

	virtual eKin::e FK(double out_Pose[], const double thetas[]) = 0;	// 核心正向運動學，由子輩實現
	virtual eKin::e IK(double* out_Thetas, const double* Pose) = 0;		// 核心逆向運動學，由子輩實現
	virtual void setRefThetas(const double* refThetas, bool isNeedThetaShift=true);
	
	static const char* errorMsg(eKin::e eKin);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	//to allow fixed array as class member.
	
protected:

	void initKin();

	int rInd;
	int _axisNum;
	int _gestureNum; // = xyzabc + re + rx ....
	double _refThetas[7];	//參考角度.可透過 1.FK更新  2. setRefTheta函式
	eRobotType::e _type;
	DHtable* DH;

	Matrix4d _world2RootT;	//  world T root
	   
	VectorXd FK_outPose;
	VectorXd FK_inThetas;	
	VectorXd IK_inPose;
	VectorXd IK_outThetas;
	Matrix4d IK_inMat;		//note: Mat and Pose is basically the same. But Pose has redundent axis.
};

class KinSerial : public Kinematics {
public:
	KinSerial(int rInd);
	~KinSerial();

	void setRefThetas(const double* refThetas, bool isNeedThetaShift=true);

	eKin::e FK(double out_Pose[], const double thetas[]);
	eKin::e IK(double* out_Thetas, const double* Pose);

protected:
	Matrix4d endT, tempT2, baseT;

	Matrix4d Identity_4x4;
	VectorXd Zero_2;
	VectorXd Zero_7;

	WristPosition _wrist;
	ThetaSolution _thetaSolution;
	int			  _solutionNum;
	
	// initialize function
	void initThetaSolution(ThetaSolution &thetaSolution);
	virtual eKin::e IK() = 0;				//must be override.
	
	// tool function
	double	changeAxisAngle(double angleDeg, const int axisInd);	
	eKin::e	chooseSolution(double totalThetas[9][MAX_MOTOR_PER_ROBOT]);
};

//add by yunyu 20250401
class Kin_Cobot : public KinSerial {

public:
	Kin_Cobot(int rInd);

private:
	Matrix4d T01 , T16 , T45 , T56 , T14 , T12 , T23 , T34;
	Vector2d Theta1_Deg_Array,Theta2_Deg_Array ,Theta3_Deg_Array, Theta4_Deg_Array, Theta5_Deg_Array, Theta6_Deg_Array;
	Vector2d temp_Theta1_Deg, temp_Theta3_Deg, temp_Theta4_Deg, temp_Theta5_Deg, temp_Theta6_Deg;

	eKin::e IK(); // 實際運算
		bool isWristSingular(); //11406 PMC Add

	// --- kinematics tool function	----
	bool getTheta1Deg(Vector2d &out_Theta1_Deg); 
	bool getTheta56Deg(Vector2d &out_Theta5_Deg , Vector2d &out_Theta6_Deg , const double Theta1Deg);
	bool getTheta234Deg(Vector2d &out_Theta2_Deg , Vector2d &out_Theta3_Deg , Vector2d &out_Theta4_Deg , double theta1 , double theta5 , double theta6);
};

//add by yunyu 20250502
class Kin_Ext : public KinSerial {

public:
	Kin_Ext(int rInd, int axisNum);

private:
	eKin::e IK();
	eKin::e FK();
};