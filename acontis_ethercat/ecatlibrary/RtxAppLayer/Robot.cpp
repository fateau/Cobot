#include "Robot.h"

#include "MySystem.h"
#include "Script.h"
#include "Jog.h"
#include "Motor.h"
#include "HelpFunctions.h"
#include "Debug.h" 
#include "ErrorHandler.h"
#include "Kinematics.h"
#include "HG6Cobot.h" //add by yunyu 20250423

#include "Shm.h"

extern SHMData* shm;


/** @brief Robot建構式
* 
* 1.初始化參數
* 2.建立Jog, script 等物件
* 3.建立馬達物件陣列
* 4.建立運動學物件
*
* @param rId 此Robot的編號
*		 motorNum 馬達數量
*		 robotType Robot類型。
* @return 無
*/
Robot::Robot(MySystem* sys, int rId, RobotDeclareData* data)
{
	int m;
	isNeedMove		= false;
	isAtSyncId		= -1;
	kinMC			= NULL;
	kinIntp			= NULL;
	kinPath			= NULL;
	this->sys		= sys;
	this->rId		= rId;
	this->robotType	= data->robotType;
	this->motorNum	= data->motorNum;
	this->gestureNum= data->gestureNum;

	switch(robotType) {
		//add by yunyu 20250502
    case eRobotType::Ext: 
			kinMC	= new Kin_Ext(rId, motorNum);
			kinIntp = new Kin_Ext(rId, motorNum);
			kinPath = new Kin_Ext(rId, motorNum);
			break;
        //add by yunyu 20250401
		case eRobotType::Cobot:
			kinMC	= new Kin_Cobot(rId);
			kinIntp = new Kin_Cobot(rId);
			kinPath = new Kin_Cobot(rId);
			break;
	}

	jog				= new Jog(this);
	script			= new Script(this);
	HGCobot         = new HG6Cobot(this); //add by yunyu 20250423
	RobotSpec* spec = &(shm->robots[rId].spec);
	masteringData	= spec->masteringData;
	
	for(m = 0; m<motorNum; m++) {
		motors[m]			= new Motor(rId,m);
		motors[m]->setGearRatio(spec->reductionRatio[m]);
		axisDegNow[m]		= spec->DH.thetasInit[m];
		axisDegCmd[m]		= spec->DH.thetasInit[m];
		axisDegCmdPrev[m]	= axisDegCmd[m];
		axisTorqNow[m]		= 0;
		axisVelCmd[m] = 0.0;  //PMC Modified 11411 //1114
		axisVelCmdPrev[m] = 0.0;
		axisAccNow[m] = 0.0;
	}

		// set refRoot;
	for(int i = 0; i < 6; i++)	 this->refRoot[i] = data->refRoot[i];

	transferPose2T(script->ref2Root_T, this->refRoot);
	transferPose2T(ref2Root_T, refRoot); 
	script->world2Ref_T = 0;
	copyHT(script->world2Root_T, script->ref2Root_T);	// set world T root as constant.
	copyHT(world2Root_T, ref2Root_T);
	inverseHT( root2World_T, world2Root_T);

	// init world2Base
	this->nowBaseId = 0;
	SetArrayValue(this->nowBase, 0.0, 6);
	transferPose2T(world2Base_T, shm->bases[0].Pose);
	inverseHT(base2World_T, world2Base_T);

	printf("create Robot obj: Type = %d, motorNum = %d\n", robotType, motorNum);
}
Robot::~Robot(void)
{
	for(int i = 0; i<motorNum; i++) 
		delete motors[i];
	
	if(kinMC	!= NULL) delete kinMC;
	if(kinIntp	!= NULL) delete kinIntp;
	if(kinPath	!= NULL) delete kinPath;
	if(jog		!= NULL) delete jog;
	if(script	!= NULL) delete script;
	if(HGCobot  != NULL) delete HGCobot; //add by yunyu 20250423
}

int Robot::updateData()
{
	int m,i;
	
	int baseId = shm->jogBaseIds[rId];
	if(!IsArrayEqual(nowBase, shm->bases[baseId].Pose, 6)) {
		nowBaseId = baseId;
		CopyArray(nowBase, shm->bases[baseId].Pose, 6);

		transferPose2T(world2Base_T, shm->bases[baseId].Pose);
		inverseHT(base2World_T, world2Base_T);
	}

	if(shm->isApplyToRealMotor == true) {	//從馬達拉值。

		double rawDeg[MAX_MOTOR_PER_ROBOT];

		//update Motors
		bool allPass = true;
		for(m = 0; m < motorNum; m++) {
			if(motors[m]->updateData() < 0) allPass = false;
		}
		if(!allPass) return -1;

		for(m = 0; m < motorNum; m++) {
			rawDeg[m]		= motors[m]->getActualDeg();
			axisVelNow[m]	= motors[m]->getActualDegVel();
			axisTorqNow[m]= motors[m]->getActualTorque();
		}

		//歸零補償
		masteringDataAdd(axisDegNow, rawDeg);
	}
	else {
		setSimulationData();
	}

	//計算Pose (poseAtRoot)
	kinMC->FK(poseAtRoot, axisDegNow, shm->jogToolIds[rId]); 

	
	// 人機顯示的 pose (poseAtWorld)
	double world2Pose_T[4][4];
	double  root2Pose_T[4][4];
	
	// 將 poseAtRoot 資料複製到 poseAtBase & poseAtWorld 裡 ,
	// 是為了讓 poseAtBase & poseAtWorld 帶有 redundancy (RX,RY,...,RE) 的資訊 ,
	// 因為 transferT2Pose 只能計算 XYZABC 
	for(i = 6; i < gestureNum; i++)
	{
		poseAtBase [i] = poseAtRoot[i];
		poseAtWorld[i] = poseAtRoot[i];
	}

	transferPose2T(root2Pose_T, poseAtRoot);
	multiplyMatrix_4x4(world2Pose_T, world2Root_T, root2Pose_T);
	transferT2Pose(poseAtWorld, world2Pose_T);


	// 人機顯示的 pose (poseAtBase)
	double base2Pose_T[4][4];
	multiplyMatrix_4x4(base2Pose_T, base2World_T, world2Pose_T);
	transferT2Pose(poseAtBase, base2Pose_T);
	calculateTorq(); //add by yunyu 20250505
	updateCollisionState(); //add by chilung 20250616

	//將值寫到 shm. //設定下次要送出的預設值
	RobotData* rData = &(shm->robots[rId]);

	for(m = 0; m< motorNum; m++) {
		rData->axisDegNow[m]	= axisDegNow[m];
		rData->velocity[m]		= axisVelNow[m]; 
		rData->torque[m]		= axisTorqNow[m];

		axisDegCmd[m] = axisDegNow[m];
	}
	
	for(i = 0; i< 7; i++) {
		rData->poseAtRoot[i] = poseAtRoot[i];
		rData->poseAtBase[i] = poseAtBase[i];
		rData->poseAtWorld[i] = poseAtWorld[i];
		nextPose[i] = poseAtRoot[i];
	}

	return 1;
}

void Robot::resetCmd()
{
	// 當 isAxisJump 跳點檢測觸發時。應令命令 = 實際值。
	CopyArray(axisDegCmd, axisDegNow, motorNum);
}
void Robot::resetCmdPrev()
{
	// 用在 isAxisJump 跳點檢測中。沒動作時，應令此值 = 實際值。
	CopyArray(axisDegCmdPrev, axisDegNow, motorNum);
}

void Robot::writeNextAxisForAllMotors()
{
	if (isNeedMove == false) {
		//resetCmdPrev();  //11412 PMC modified //1211
		return;	// 此robot不需要移動	
	}
	if (isAxisJump()){			// 若有跳點，不送命令到馬達, 且急停
		resetCmd();
		return;
	}	

	getaxisAccNow(); //PMC Modified 11411 //1114

	if(shm->isApplyToRealMotor == false) 
		return;	
		
	double rawDeg[MAX_MOTOR_PER_ROBOT];

	//逆歸零補償
	masteringDataRemove(rawDeg, axisDegCmd);

	//寫入馬達 (axis)
	for (int m = 0; m < motorNum; m++) {
		motors[m]->setTorqueOffset(gravityTorq[m]);  //PMC Modified 11412 //1229
		motors[m]->setTargetDeg(rawDeg[m]);
	}
	CopyArray(axisDegCmdPrevAcc, axisDegCmd, motorNum);  //11501 PMC modified //0121

}
void Robot::writeNextTorqForAllMotors()
{
	//寫入馬達 (axis)
	for (int m = 0; m < motorNum; m++) {
		motors[m]->setTorqueOffset(0);  //PMC Modified 11412 //1229
		motors[m]->setTargetTorque(estimateTorq[m]);
	}
	return;
}

void Robot::calculateTorq() //在不同模式(CSP.CST)下進入不同計算函式 //add by yunyu 20250505
{
	if (shm->RTXMode == eRTXMode::CST)
		HGCobot->calTorq_HandGuide(estimateTorq);
	else {
		HGCobot->calTorq_Collision(estimateTorq);
		HGCobot->calTorq_Gravity(gravityTorq);
	}
	/*
	static int count = 0;
	if (count++ > 500)count = 0;
	if (count == 0)
		Debug::writeln(0, "estimateTorq:%d, %d, %d, %d, %d, %d  Residual: %d, %d, %d, %d, %d, %d \n",estimateTorq[0], estimateTorq[1], estimateTorq[2], estimateTorq[3], estimateTorq[4], estimateTorq[5], 
			axisTorqNow[0] - estimateTorq[0],
			axisTorqNow[1] - estimateTorq[1],
			axisTorqNow[2] - estimateTorq[2],
			axisTorqNow[3] - estimateTorq[3],
			axisTorqNow[4] - estimateTorq[4],
			axisTorqNow[5] - estimateTorq[5]);
	*/
	static int print_count[MAX_ROBOT_NUM] = {0};
	if (shm->isScriptRunning) {
		if (print_count[rId]++ > 0) {
			print_count[rId] = 0;
			Debug::writeln(2, "Robot[%d], pos：,%f, %f, %f, %f, %f, %f, axisDeg：,%f, %f, %f, %f, %f, %f, axisVel：,%f, %f, %f, %f, %f, %f, axisTorq：,%d, %d, %d, %d, %d, %d, estimateTorq：,%d, %d, %d, %d, %d, %d, Residual：, %d, %d, %d, %d, %d, %d",
				rId,
				poseAtRoot[0], poseAtRoot[1], poseAtRoot[2], poseAtRoot[3], poseAtRoot[4], poseAtRoot[5],
				axisDegNow[0], axisDegNow[1], axisDegNow[2], axisDegNow[3], axisDegNow[4], axisDegNow[5],
				axisVelNow[0], axisVelNow[1], axisVelNow[2], axisVelNow[3], axisVelNow[4], axisVelNow[5],
				axisTorqNow[0], axisTorqNow[1], axisTorqNow[2], axisTorqNow[3], axisTorqNow[4], axisTorqNow[5],
				estimateTorq[0], estimateTorq[1], estimateTorq[2], estimateTorq[3], estimateTorq[4], estimateTorq[5],
				axisTorqNow[0] - estimateTorq[0],
				axisTorqNow[1] - estimateTorq[1],
				axisTorqNow[2] - estimateTorq[2],
				axisTorqNow[3] - estimateTorq[3],
				axisTorqNow[4] - estimateTorq[4],
				axisTorqNow[5] - estimateTorq[5]
			);
		}
	}
}

void Robot::updateCollisionState() //PMC Modified 11410
{	
	int* errorValue = shm->hgParams[rId].errorValue;
	int  MaxTolerateTime = shm->hgParams[rId].MaxTolerateTime;
	if(shm->CollisionMode[rId]){ 
		if (shm->isScriptRunning){
			for(int m=0;m<motorNum; m++){  // all axis
				if (abs(axisVelNow[m]) > 0.5){ //速度大於0.5再取誤差值，因小於0.5的扭力值在CSP時不穩
					// 計算各軸馬達實際扭矩與估計值計算的絕對值是否大於設定的誤差值，若大於則誤差值計數+1
					// 直到誤差總數量大於設定數值，由於程式是每1ms判斷一次，因此設定100意義為，100ms內持續有異常狀態發生
					if (abs(axisTorqNow[m] - estimateTorq[m]) > errorValue[m])	errorCount[m]++;
					else														errorCount[m] = 0;
					//先暫時拿掉 可看errorCount 數值
					if( errorCount[m] > MaxTolerateTime ){
						shm->CollisionState[rId]=1;
						//shm->isSlowStop = false; //11408 PMC modified
						shm->stopScript = true; //11408 PMC modified
					}
				}
			}
		}
	}
}

void Robot::StartServoOnOff() //11412 PMC modified//1211
{
	bool anyFault = false;
	//檢查全部馬達是否錯誤
	for (int m = 0; m < motorNum; m++) {
		if (motors[m]->isLinkedToEcat) {
			if (motors[m]->getStatusWord() & 0x0008) {
				anyFault = true;
				break;
			}
		}
	}
	for (int m = 0; m < motorNum; m++) {
		if (anyFault) {
			if (!(motors[m]->getStatusWord() & 0x0008)) {
				shm->robots[rId].motors[m].isNeedServoOn = false;
			}
		}
		motors[m]->StartServoOnOff();
	}
}

void Robot::isServoOn()
{
	for(int m = 0; m < motorNum; m++) 
		shm->robots[rId].motors[m].isServoOn = motors[m]->isServoOn();
}
bool Robot::isReachAngleLimit_next()
{
	for(int m = 0; m< motorNum; m++) {
		if(kinMC->isReachAngleLimit(axisDegCmd[m], m)){
			// note: do not print here. will print a lot and watchdog.
			return true;
		}
	}
	
	return false;
}

bool Robot::isAxisJump()    //11412 PMC modified //1211
{
	const double Max_Velocity = 200.0;
	for (int i = 0; i < motorNum; i++) {

		// 200度/s 視為跳點。 若無跳點則繼續；若跳點則回傳false;
		if (abs(axisVelNow[i]) < Max_Velocity) continue;
		ScriptData* shmScrData = &(shm->scriptData[rId]);
		shm->stopScript = true;
		ErrorHandler::Set(eError::AXIS_JUMP,
			"Robot/isAxisJump: r[%d], axis[%d], func[%d], line[%d], axisVelNow[%lf]",
			rId + 1,
			i + 1,
			shmScrData->nowFuncId + 1,
			shmScrData->nowLineId,
			axisVelNow[i]);
		return true;
	}
	return false;
}

void Robot::masteringDataAdd(double* outCompDeg, double* inRawDeg)
{
	for(int i = 0; i < motorNum; i++)
		outCompDeg[i] = inRawDeg[i] + masteringData[i];
	return;
}
void Robot::masteringDataRemove(double* outRawDeg, double* inCompDeg)
{
	for(int i = 0; i < motorNum; i++)
		outRawDeg[i] = inCompDeg[i] - masteringData[i];
	return;
}
void Robot::setSimulationData()
{
	//不從馬達拉值。直接將上一毫秒的指令當成現在的值
	CopyArray(axisDegNow, axisDegCmd, motorNum);

	//將角度相減得到速度
	for(int m = 0; m< motorNum; m++) {
		axisVelNow[m] = (axisDegCmd[m] - axisDegCmdPrev[m]) * SAMPLING_T_INV ;
		axisAccNow[m] = (axisVelNow[m] - axisVelPrev[m]) * SAMPLING_T_INV ;
		axisDegCmdPrev[m] = axisDegCmd[m];
		axisVelPrev[m] = axisVelNow[m];
		axisTorqNow[m] = estimateTorq[m]; //PMC Modified 11411 //1127
	}
}

//PMC Modified 11410
double* Robot::getThetaInit()
{
	return shm->robots[rId].spec.DH.thetasInit;
}

//PMC Modified 11410
double* Robot::getRefRoot()
{
	return this->refRoot;
}

void Robot::getaxisAccNow() //11501 PMC modified //0121
{
	for (int m = 0; m < motorNum; m++) {
		axisVelCmd[m] = (axisDegCmd[m] - axisDegCmdPrevAcc[m]) * SAMPLING_T_INV;
		axisAccNow[m] = (axisVelCmd[m] - axisVelCmdPrev[m]) * SAMPLING_T_INV;
		axisVelCmdPrev[m] = axisVelCmd[m];
	}
	return;
}
