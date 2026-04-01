#pragma once

#include "Def.h"


class Robot;
class MySystem;
class Jog
{
public:

	Jog(Robot* robot);
	~Jog(void);
	
	// main function
	int  processCommand();
	void reset();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	//to allow fixed array as class member.

private:
	Matrix4d tempT, tempInvT, temp_out_T, delta_T, Identity_4x4;
	MySystem* sys;
	Robot*	robot;
	JogCmd*	jCmd;
	int		gestureNum;
	int		ind;			//要jog的索引編號

	double	dist;
	double	deltaDist;		//要增加的進給量(這一輪要計算的)
	double	nowV;
	double	maxV;
	double	maxA;
	double  targetAxis[MAX_MOTOR_PER_ROBOT];
	double* poseAtBase;

	bool	isRunning;				//功能相當於 start. 但值剛好相反。
	bool	isNeedDecV;
	double	leftoverDist;

	// ------ functions ------------
	bool computeDeltaDist();

	void computeNextVelocityNoLimit(); 
	void computeNextVelocityLimitDist();
	double calculateDist(double startV, double endV, double maxA);

	// for change base. 
	void attachBase(double out_PoseAtBase[MAX_REDUNDANCY], double PoseAtRoot[MAX_REDUNDANCY]);
	void removeBase(double out_PoseAtRoot[MAX_REDUNDANCY], double PoseAtBase[MAX_REDUNDANCY]);

	void jogABC(double out_Pose[MAX_REDUNDANCY], const double Pose[MAX_REDUNDANCY], const double ThetaDeg, const char type);
	void jogXYZ(double out_Pose[MAX_REDUNDANCY], const double Pose[MAX_REDUNDANCY], const double dist, const char type);
};

