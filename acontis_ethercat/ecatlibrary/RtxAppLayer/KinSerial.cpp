#include "Kinematics.h"
#include "Shm.h"
#include "Debug.h"

extern SHMData* shm;

KinSerial::KinSerial(int rInd) : Kinematics(rInd)
{
	Identity_4x4 = Matrix4d::Identity();
	Zero_2 = VectorXd::Zero(2);
	Zero_7 = VectorXd::Zero(7);

	FK_inThetas = Zero_7;
	FK_outPose = Zero_7;
	IK_inPose = Zero_7;
	IK_outThetas = Zero_7;

	initThetaSolution(_thetaSolution);
}
KinSerial::~KinSerial(void) {}

// ==================== public function ======================
void KinSerial::setRefThetas(const double* refThetas, bool isNeedThetaShift)
{
	if (isNeedThetaShift) {
		for (int j = 0; j < MAX_MOTOR_PER_ROBOT; j++)
			_refThetas[j] = refThetas[j] - DH->thetaShiftDeg[j];
	}
	else {
		for (int j = 0; j < MAX_MOTOR_PER_ROBOT; j++)
			_refThetas[j] = refThetas[j];
	}

}
// ==================== initialize function ======================
void KinSerial::initThetaSolution(ThetaSolution &thetaSolution)
{
	for (int i = 1; i < 9; i++)
		thetaSolution.ret[i] = eKin::COMPLETE;
}

eKin::e KinSerial::FK(double out_Pose[], const double thetas[])
{
	// input θ shift
	for (int i = 0; i < _axisNum; i++)
		FK_inThetas(i) = thetas[i];

	// forward kinematics. (result: endT, a 4x4 Matrix)
	endT = Identity_4x4;
	for (int i = 0; i < _axisNum; i++)
	{
		DH->getT(tempT2, FK_inThetas(i), i + 1);
		endT *= tempT2;			// endT = T01 ; T02 ; T03 ; ... = I*T01 ; T01*T12 ; T02*T23 ; ... 
	}

	// manipulate endT.
	transferT2Pose(FK_outPose, endT);
	setRefThetas(FK_inThetas.data(), false);  
	
	for (int i = 0; i < 6; i++)
		out_Pose[i] = FK_outPose(i);

	return eKin::COMPLETE;
}
eKin::e KinSerial::IK(double* out_Thetas, const double* pose)
{
	// init
	initThetaSolution(_thetaSolution);
	transferPose2T(IK_inMat, pose);
	for (int i = 0; i < _gestureNum; i++)
		IK_inPose(i) = pose[i];


	// 實際運算的 IK funciotn. 結果將放入 tempThetas內
	eKin::e ret = IK();
	if (ret != eKin::COMPLETE) return ret;


	for (int i = 0; i < _axisNum; i++)
		out_Thetas[i] = IK_outThetas(i);

	return eKin::COMPLETE;
}


// 將最佳解寫入IK_outThetas
eKin::e KinSerial::chooseSolution(double totalThetas[9][MAX_MOTOR_PER_ROBOT])
{
	int fitSolutionNO = 0;
	double tempSum = 0.0;
	double minSum = 1.0e+10;
	double totalSum = 0.0;

	// ------- choose the nearest solution using LSM ----------
	for (int i = 1; i <= 8; i++)
	{
		tempSum = 0.0; totalSum = 0.0;
		for (int j = 0; j < _axisNum; j++)
		{
			tempSum += (totalThetas[i][j] - _refThetas[j]) * (totalThetas[i][j] - _refThetas[j]);
			totalSum += totalThetas[i][j]; 
		}

		if (tempSum < minSum && totalSum != 0)
		{
			minSum = tempSum;
			fitSolutionNO = i;
		}
	}

	
	// 輸出最佳解
	for (int i = 0; i < _axisNum; i++)
		IK_outThetas(i) = _thetaSolution.totalThetas[fitSolutionNO][i];

	return _thetaSolution.ret[fitSolutionNO]; 
}


double KinSerial::changeAxisAngle(double angle_Deg, const int axis_NO)
{
	//處理參考角度超過+-180時. 
	double upper = _refThetas[axis_NO - 1] + 180;
	int blocks = (upper - angle_Deg) / 360;

	if (upper < angle_Deg)
		blocks -= 1;
	double suitable_AngleDeg = angle_Deg + blocks * 360;

	return suitable_AngleDeg;
}