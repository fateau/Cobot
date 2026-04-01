#include "Jog.h"
#include "Robot.h"
#include "Shm.h" 
#include "Kinematics.h"
#include "ErrorHandler.h"
#include "Debug.h"
extern SHMData* shm;

Jog::Jog(Robot* robot)
{
	Identity_4x4 = Matrix4d::Identity();

	this->robot = robot;
	this->sys = robot->sys;
	jCmd = &shm->jogCmd;

	gestureNum = robot->gestureNum;
	poseAtBase = new double[gestureNum];
	isNeedDecV = false;
	isRunning = false;
}

Jog::~Jog(void)
{
	delete[]poseAtBase;
}

void Jog::attachBase(double out_PoseAtBase[6], double PoseAtRoot[6])
{
	double world2Pose_T[4][4];
	double  root2Pose_T[4][4];
	double  base2Pose_T[4][4];

	// 先複製一份完整的 gesture data 到 out_PoseAtBase
	// 這樣 out_PoseAtBase 才能帶有 redundancy 的資訊
	for (int i = 0; i < gestureNum; i++)
	{
		out_PoseAtBase[i] = PoseAtRoot[i];
	}

	transferPose2T(root2Pose_T, PoseAtRoot); // get root T pose
	multiplyMatrix_4x4(world2Pose_T, robot->world2Root_T, root2Pose_T); // world T pose = world T root * root T pose
	multiplyMatrix_4x4(base2Pose_T, robot->base2World_T, world2Pose_T); // base T pose = base T world * world T pose

	transferT2Pose(out_PoseAtBase, base2Pose_T);
}
void Jog::removeBase(double out_PoseAtRoot[MAX_REDUNDANCY], double PoseAtBase[MAX_REDUNDANCY])
{
	double world2Pose_T[4][4];
	double  root2Pose_T[4][4];
	double  base2Pose_T[4][4];

	// 先複製一份完整的 gesture data 到 out_PoseAtRoot
	// 這樣 out_PoseAtRoot 才能帶有 redundancy 的資訊
	for (int i = 0; i < gestureNum; i++)
	{
		out_PoseAtRoot[i] = PoseAtBase[i];
	}

	transferPose2T(base2Pose_T, PoseAtBase);
	multiplyMatrix_4x4(world2Pose_T, robot->world2Base_T, base2Pose_T);
	multiplyMatrix_4x4(root2Pose_T, robot->root2World_T, world2Pose_T);
	transferT2Pose(out_PoseAtRoot, root2Pose_T);
}
bool Jog::computeDeltaDist()
{
	//從人機更新速度參數  (人機按鈕放開時速度歸零)
	maxV = jCmd->vel;

	//計算進給量
	if (jCmd->isLimitDist)
		computeNextVelocityLimitDist();
	else
		computeNextVelocityNoLimit();


	if (abs(deltaDist) < 0.00000000001) //距離過短，不需jog.
	{
		isRunning = false;
		return false;
	}

	dist -= abs(deltaDist);
	jCmd->dist = dist;

	return true;
}
int Jog::processCommand() {

	// 若不是這隻robot，則返回
	if (jCmd->r != robot->rId) return -1;

	// 若上一輪判斷Jog已停止，下列情況時返回 =>不作事。
	if (!isRunning) {
		if (!jCmd->isUserPressJOG) return -1; //人機沒有下命令

		if (jCmd->isLimitDist && jCmd->dist < 0.00000000001) return -1; //有限距離且剩餘距離為0
	}

	// 第一次進入JOG模式(人機按下的第一次)，初始化參數
	if (!isRunning)
	{
		//從人機拿參數
		ind = jCmd->ind;
		dist = jCmd->dist;
		maxA = jCmd->acc;
		nowV = 0;
		isNeedDecV = false;
		isRunning = true;

		if (shm->cmdFormat == eCmdFormat::POSE)
		{
			if (ind >= 3 && ind < 6)  maxV *= 0.1; //slower when jog ABC.
			attachBase(poseAtBase, robot->poseAtRoot);
		}
		else
		{
			for (int i = 0; i < robot->motorNum; i++)
				targetAxis[i] = robot->axisDegNow[i];
		}
	}

	//計算進給量。若計算失敗(例如:進給量太小) 則回傳false
	if (computeDeltaDist() == false) return false;

	if (shm->cmdFormat == eCmdFormat::POSE) {

		//由進給量更新位置
		if (ind == 6)		poseAtBase[6] += deltaDist;
		else if (ind >= 3)	jogABC(poseAtBase, poseAtBase, deltaDist, 'a' + ind - 3); // 用 ASCII 碼判別 char A , B , C
		else			jogXYZ(poseAtBase, poseAtBase, deltaDist, 'x' + ind);	  // 用 ASCII 碼判別 char X , Y , Z

		removeBase(robot->nextPose, poseAtBase);

		//逆解成axis	
		//eKin::e ikRes = robot->kinMC->IK(robot->axisDegCmd, robot->nextPose, shm->jogToolIds[robot->rId]);
		eKin::e ikRes = robot->kinMC->IK(targetAxis, robot->nextPose, shm->jogToolIds[robot->rId]);
		if (ikRes != eKin::COMPLETE) {
			reset(); //停止此次Jog.
			Debug::writeln(0,"Jog: IK error. %s\n", Kinematics::errorMsg(ikRes));		
			ErrorHandler::Set(eError::IK_FAIL, "Jog: IK error. %s", Kinematics::errorMsg(ikRes));  
			return false;
		}
	}
	else if (shm->cmdFormat == eCmdFormat::AXIS)
	{
		targetAxis[ind] += deltaDist;

		//檢查角度極限
		if (robot->kinMC->isReachAngleLimit(targetAxis[ind], ind)) {
			reset(); //停止此次Jog.
			ErrorHandler::Set(eError::ANGLE_LIMIT, "Jog: axis %d reach Angle limit", ind + 1); 
			return false;
		}
	}

	for (int i = 0; i < robot->motorNum; i++)
		robot->axisDegCmd[i] = targetAxis[i]; //由進給量更新位置

	return true;
}

void Jog::reset()
{
	isRunning = false;
	jCmd->isUserPressJOG = false;
}
// ====================== Private Functions ==========================
void Jog::computeNextVelocityNoLimit()
{
	//目標:		計算出 deltaDist, 更新 nowV

	if (maxV == 0 && nowV == 0) {
		deltaDist = 0;
		return;
	}

	//減速	 
	if (nowV > maxV) {
		nowV -= maxA;
		if (nowV <= maxV)
			nowV = maxV;
	}

	//加速 
	else if (nowV < maxV) {
		nowV += maxA;
		if (nowV >= maxV)
			nowV = maxV;
	}

	//等速 則nowV不變
	deltaDist = nowV;

	return;
}
void Jog::computeNextVelocityLimitDist()
{
	//目標:		計算出 deltaDist, 並更新 nowV
	double	decLength = 0;		//從速度V 降速到0所需的距離。
	double	nextV = 0;

	if (dist == 0) {
		deltaDist = 0;
		isNeedDecV = false;
		return;
	}

	if (maxV == 0 && nowV == 0) {
		deltaDist = 0;
		isNeedDecV = false;
		return;
	}

	if (isNeedDecV == false)	// 距離尚可加減速 
	{
		//  假設速度改變後，計算"減速到0所需距離"				 
		if (nowV < maxV) {		//  加速

			nextV = nowV + maxA;
			decLength = calculateDist(nextV, 0, maxA);
			nextV = nowV + 0.5 * maxA;
		}
		else if (nowV > maxV) {	// 減速 
			nextV = nowV - maxA;
			decLength = calculateDist(nextV, 0, maxA);
			nextV = nowV - 0.5 * maxA;
		}
		else {					// 等速 
			nextV = nowV;
			decLength = calculateDist(nextV, 0, maxA);
		}

		// 若剩餘距離足夠，則計算deltaDist, 並更新nowV
		if (dist >= (decLength + fabs(nextV)))
		{
			if (nowV > maxV)					// 減速 
			{
				if (nowV - maxA <= maxV) {
					deltaDist = maxV;
					nowV = maxV;
				}
				else {
					deltaDist = nowV - maxA * 0.5;
					nowV -= maxA;
				}
			}
			else if (nowV < maxV) 			//加速 
			{
				if (nowV + maxA >= maxV) {
					deltaDist = maxV;
					nowV = maxV;
				}
				else {
					deltaDist = nowV + maxA * 0.5;
					nowV += maxA;
				}
			}
			else 							//等速
			{
				deltaDist = maxV;
			}
		}
		// 若剩餘距離不夠，則嚐試 1.等速，若還是不夠則 2.降速
		else
		{
			isNeedDecV = true;

			//假設等速，計算"減速到0所需距離"
			decLength = calculateDist(nowV, 0, maxA);

			//若剩餘距離足夠
			if (dist >= (decLength + fabs(nowV)))
			{
				deltaDist = nowV;
				leftoverDist = dist - decLength - fabs(nowV);
			}
			//若剩餘距離還是不夠，則降速
			else
			{
				if (nowV > 0)
				{
					deltaDist = nowV - maxA * 0.5;
					nowV -= maxA;

					leftoverDist = dist - decLength;
					if (leftoverDist >= deltaDist)
					{
						deltaDist = leftoverDist;
						nowV += maxA;
						leftoverDist = -1;
					}
				}
				else
				{
					deltaDist = nowV + maxA * 0.5;
					nowV += maxA;

					leftoverDist = dist - decLength;

					if (leftoverDist >= fabs(deltaDist))
					{
						deltaDist = -leftoverDist;
						nowV -= maxA;
						leftoverDist = -1;
					}
				}
			}
		}// 剩餘距離不夠end 
	}
	else  //isNeedDecV == true 的情況，則以maxA的斜率降速。同時找機會補足leftoverDist
	{
		if (nowV > 0)
		{
			if (leftoverDist > fabs(nowV - maxA * 0.5)) {		//找到地方插入leftoverDist
				deltaDist = leftoverDist;
				leftoverDist = -1;
			}
			else {
				deltaDist = nowV - maxA * 0.5;
				nowV -= maxA;
			}
		}
		else if (nowV < 0)
		{
			if (leftoverDist > fabs(nowV + maxA * 0.5)) {	//找到地方插入leftoverDist
				deltaDist = -leftoverDist;
				leftoverDist = -1;
			}
			else {
				deltaDist = nowV + maxA * 0.5;
				nowV += maxA;
			}
		}
		else
		{
			int direct = deltaDist > 0 ? 1 : -1;

			if (dist <= maxA * 0.5)
				deltaDist = dist * direct;
			else
				deltaDist = 0;
		}
	}

	// 最後檢查
	if ((dist - fabs(deltaDist)) <= 0)
	{
		int direct = deltaDist > 0 ? 1 : -1;
		deltaDist = dist * direct;
	}

	return;
}
double Jog::calculateDist(double startV, double endV, double maxA)
{
	//	算路徑長(距離)
	if (maxA == 0) return -1;

	int	sign;
	if (startV * endV >= 0)  sign = -1; // start_v & end_v 同號
	else					sign = 1;  // start_v & end_v 異號

	double dis = fabs(pow(startV, 2) + sign * pow(endV, 2)) / (2 * fabs(maxA));

	return dis;
}

// 依照輸入 Pose 的座標系統(C.S.) , 繞其 X , Y , Z 軸旋轉
void Jog::jogABC(double out_Pose[MAX_REDUNDANCY], const double Pose[MAX_REDUNDANCY], const double deltaThetaDeg, const char type)
{
	int i;
	double axis_vector[3] = { 0 };

	// 先複製一份完整的 gesture data (Pose) 到 out_Pose
	// 這樣 out_Pose 才帶有 redundancy 資訊
	for (i = 0; i < gestureNum; i++)
		out_Pose[i] = Pose[i];

	tempT = tempInvT = temp_out_T = delta_T = Identity_4x4;

	transferPose2T(tempT, Pose);

	tempInvT = tempT.inverse();

	switch (type)
	{
		// 繞 Z 軸
	case 'A':
	case 'a':
	{
		if (shm->jogFrame == eFrame::BASE) {	// base frame
			axis_vector[0] = tempInvT(0, 2);
			axis_vector[1] = tempInvT(1, 2);
			axis_vector[2] = tempInvT(2, 2);
		}
		else {										// tool frame
			axis_vector[0] = 0;
			axis_vector[1] = 0;
			axis_vector[2] = 1;
		}
	}
	break;

	// 繞 Y 軸
	case 'B':
	case 'b':
	{
		if (shm->jogFrame == eFrame::BASE) {	// base frame
			axis_vector[0] = tempInvT(0, 1);
			axis_vector[1] = tempInvT(1, 1);
			axis_vector[2] = tempInvT(2, 1);
		}
		else {										// tool frame
			axis_vector[0] = 0;
			axis_vector[1] = 1;
			axis_vector[2] = 0;
		}
	}
	break;

	// 繞 X 軸
	case 'C':
	case 'c':
	{
		if (shm->jogFrame == eFrame::BASE) {	// base frame
			axis_vector[0] = tempInvT(0, 0);
			axis_vector[1] = tempInvT(1, 0);
			axis_vector[2] = tempInvT(2, 0);
		}
		else {										// tool frame
			axis_vector[0] = 1;
			axis_vector[1] = 0;
			axis_vector[2] = 0;
		}
	}
	break;
	}

	AxisAngleMethod_T(delta_T, axis_vector, deltaThetaDeg); // get ΔT

	temp_out_T = tempT * delta_T; // T' = T x ΔT

	transferT2Pose(out_Pose, temp_out_T);
}
void Jog::jogXYZ(double out_Pose[MAX_REDUNDANCY], const double PoseAtBase[MAX_REDUNDANCY], const double dist, const char type)
{
	tempT = temp_out_T = delta_T = Identity_4x4;

	transferPose2T(tempT, PoseAtBase); // tempT : base T tool

	switch (type)
	{
		// X
	case 'X':
	case 'x':
	{
		if (shm->jogFrame == eFrame::BASE) {	// base frame
			tempT(0, 3) += dist;
		}
		else {									// tool frame
			delta_T(0, 3) = dist;
		}
	}
	break;

	// Y
	case 'Y':
	case 'y':
	{
		if (shm->jogFrame == eFrame::BASE) {	// base frame
			tempT(1, 3) += dist;
		}
		else {									// tool frame
			delta_T(1, 3) = dist;
		}
	}
	break;

	// Z
	case 'Z':
	case 'z':
	{
		if (shm->jogFrame == eFrame::BASE) {	// base frame
			tempT(2, 3) += dist;
		}
		else {									// tool frame
			delta_T(2, 3) = dist;
		}
	}
	break;
	}

	temp_out_T = tempT * delta_T; // T' = T x ΔT

	transferT2Pose(out_Pose, temp_out_T);
}