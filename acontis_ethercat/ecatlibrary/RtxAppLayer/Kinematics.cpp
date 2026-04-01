#include "Kinematics.h"
#include "MathTool_Robot.h"
#include "Shm.h"
#include "Debug.h"

extern SHMData* shm;

Kinematics::Kinematics(int rInd)
{ 
	this->rInd = rInd; 
	initKin();
}

void Kinematics::initKin()  //1080617: move to kinematics.cpp
{
	IK_inMat = _world2RootT = Matrix4d::Identity();

	DH = & (shm->robots[rInd].spec.DH); 
	_type = shm->robotDeclareTable[rInd].robotType;
}

// 正向運動學，然後加上 Tool
eKin::e Kinematics::FK(double  out_Pose[], double thetas[], int toolIndex)
{
	// check angle limit //check before subtract thetaShift
	if(isReachAngleLimitAll(thetas , _axisNum))  return eKin::ANGLE_LIMIT;

	// input θ shift
	double shiftedThetas[MAX_MOTOR_PER_ROBOT];
	for(int i = 0; i < _axisNum; i++)
		shiftedThetas[i] = thetas[i] - DH->thetaShiftDeg[i];

	eKin::e solCheck = FK(out_Pose, shiftedThetas);
	attachTool(out_Pose, out_Pose, toolIndex);
	return solCheck;
}

// 移除 Tool，然後逆向運動學
eKin::e Kinematics::IK(double* out_Thetas, const double* pose, int toolIndex)
{	
	double poseNoTool[MAX_MOTOR_PER_ROBOT]; 
	memcpy(poseNoTool, pose, sizeof(double) * MAX_MOTOR_PER_ROBOT);	

	removeTool(poseNoTool, poseNoTool, toolIndex);

	eKin::e solCheck = IK(out_Thetas, poseNoTool);

	if (solCheck != eKin::COMPLETE) return solCheck;

	// output θ shift
	for(int i = 0; i < _axisNum; i++)
		out_Thetas[i] =  out_Thetas[i] + DH->thetaShiftDeg[i]; 

	// check angle limit
	if(isReachAngleLimitAll(out_Thetas, _axisNum))  
		return eKin::ANGLE_LIMIT;	

	setRefThetas(IK_outThetas.data(), false);


	return solCheck;
}


void Kinematics::setRefThetas(const double* refThetas, bool isNeedThetaShift)
{
	// do nothing.

}

bool Kinematics::isReachAngleLimit(const double angle_Deg, const int axis_NO)
{
	if( angle_Deg > DH->axisPositiveLimit[ axis_NO ]) return true;
	
	if(	angle_Deg < DH->axisNegativeLimit[ axis_NO ]) return true;
	
	
    return false;
}
bool Kinematics::isReachAngleLimitAll(const double angle_Deg[], const int length)
{
	for(int i = 0; i< length; i++) 
		if(isReachAngleLimit(angle_Deg[i], i) == true)
			return true;

	return false;
}

//static function
const char* Kinematics::errorMsg(eKin::e eKin)
{
	static char msg[20];
	switch(eKin)
	{
		case eKin::ANGLE_LIMIT:  
			sprintf(msg, "ANGLE_LIMIT"); 
			break;
		case eKin::WRIST_SINGULAR:
			sprintf(msg, "WRIST_SINGULAR");
			break;
		case eKin::OUT_OF_WORKSPACE:
			sprintf(msg, "OUT_OF_WORKSPACE");
			break;
		case eKin::OVERHEAD_SINGULAR:
			sprintf(msg, "OVERHEAD_SINGULAR");
			break;
		default:
			sprintf(msg, "");
			break;
	}
	return msg;
}