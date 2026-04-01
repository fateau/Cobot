#include "CmdStrategy.h"
#include "MathTool_Robot.h"
#include "HelpFunctions.h"
#include "IOModule.h"

#include "IOGroup.h"
#include "Script.h"
#include "Robot.h"
#include "Motor.h"
#include "PathStartSyncer.h" //see if can remove.
#include "Kinematics.h"

#include "MathTooL.h"
#include "Debug.h"
#include "SHM.h"
#include "ErrorHandler.h" 

extern SHMData* shm;
CmdStrategy::CmdStrategy(Robot* robot)
{
	this->robot = robot;
	this->script = robot->script;
}
CmdStrategy::~CmdStrategy(void){}

int CmdStrategy::raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd)
{
	pCmd.info	= rCmd.info;
	return 1;
}

//========= Move ============
void CmdStrategyMove::reset()
{
	//reset parameters
	currentBaseId = -1;

	// set start point
	CopyArray(startAxis, robot->axisDegNow, robot->motorNum);
	CopyArray(startPoseAtRoot, robot->poseAtRoot, robot->gestureNum);
}
void CmdStrategyMove::updateBase_T(int baseId)
{
	if(currentBaseId == baseId) return;

	currentBaseId = baseId;
	transferPose2T(world2Base_T, shm->bases[baseId].Pose);
	inverseHT(base2World_T, world2Base_T);
}
void CmdStrategyMove::attachBase(double out_PoseAtBase[MAX_REDUNDANCY], double PoseAtRoot[MAX_REDUNDANCY])
{	
	double world2Pose_T[4][4];
	double  root2Pose_T[4][4];
	double  base2Pose_T[4][4];

	// 先複製一份完整的 gesture data 到 out_PoseAtBase
	// 這樣 out_PoseAtBase 才能帶有 redundancy 的資訊
	CopyArray(out_PoseAtBase, PoseAtRoot, robot->gestureNum);

	//PMC Modified 11411
	if (script->isNeedUpdateAttachRef)
	{
			script->isNeedUpdateAttachRef = false;
			for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 4; j++)
							script->refPose[i][j] = robot->world2Root_T[i][j];
			}
	}

	transferPose2T(root2Pose_T, PoseAtRoot); // get root T pose
	//multiplyMatrix_4x4(base2Pose_T, base2World_T, root2Pose_T); // base T pose = base T world * root T pose //PMC Modified 11411
	multiplyMatrix_4x4(world2Pose_T, script->refPose, root2Pose_T); // world T pose = world T root * root T pose //PMC Modified 11411
	multiplyMatrix_4x4(base2Pose_T, base2World_T, world2Pose_T); // base T pose = base T world * world T pose //PMC Modified 11411
	transferT2Pose(out_PoseAtBase, base2Pose_T);
}
void CmdStrategyMove::removeBase(double out_PoseAtRoot[MAX_REDUNDANCY], double PoseAtBase[MAX_REDUNDANCY])
{
	// 移除目前當行的 base, 以傳往下一層  (form "base T pose" --> "root T pose")
	// base -> world -> root -> pose

	double  base2Pose_T[4][4];
	double world2Pose_T[4][4];
	double  root2Pose_T[4][4];
	double root2World_T[4][4];

	// 先複製一份完整的 gesture data 到 out_PoseAtRoot
	// 這樣 out_PoseAtRoot 才能帶有 redundancy 的資訊
	CopyArray(out_PoseAtRoot, PoseAtBase, robot->gestureNum);

	transferPose2T(base2Pose_T, PoseAtBase);
	multiplyMatrix_4x4(world2Pose_T, world2Base_T, base2Pose_T);

	inverseHT(root2World_T, script->refPose);
	multiplyMatrix_4x4(root2Pose_T, root2World_T, world2Pose_T);
	transferT2Pose(out_PoseAtRoot, world2Pose_T);
	transferT2Pose(out_PoseAtRoot, root2Pose_T); //PMC Modified 11411
}
void CmdStrategyMove::setStartPoint(CmdMove* cmPath)
{
	// 特殊情況-同步:  
	if(script->isAfterSync) {
		script->isAfterSync = false;

		//以最後一次命令做起點 (因馬達可能沒到位，因此不採用目前位置)
		CopyArray(startAxis, robot->axisDegCmd, robot->motorNum);
		robot->kinPath->FK(startPoseAtRoot, startAxis, cmPath->nowToolId);
	}

	//一般情況->以上一行結尾做為起點
	CopyArray(cmPath->startAxis, startAxis, robot->motorNum);
	CopyArray(cmPath->startPose, startPoseAtRoot, robot->gestureNum); // without base
}
double CmdStrategyMove::calEndPointByMask(double userInput, double startPoint, BOOL isMasked, BOOL isAbs)
{
	if(isMasked)	return calEndPointByRelative(userInput, startPoint, isAbs);	// user has input.			
	else			return startPoint;											// user no input
}
double CmdStrategyMove::calEndPointByRelative(double userInput, double startPoint, BOOL isAbs)
{
	if(isAbs)	return userInput;				// absolute			
	else		return userInput + startPoint;	// relative
}
void CmdStrategyMove::moveRelativeToToolFrame(double* endPoseAtBase, double* startPoseAtBase, const CmdMoveRaw* cmRaw)
{
	memcpy(endPoseAtBase, startPoseAtBase, sizeof(double)*robot->gestureNum);

	for(int m = 0; m < robot->gestureNum; m++) {

		if(cmRaw->PoseMask[m]) {
			if(m<=2)
				moveXYZ(endPoseAtBase, endPoseAtBase, cmRaw->Pose[m], m, cmRaw->frame);
			else
				moveABC(endPoseAtBase, endPoseAtBase, cmRaw->Pose[m], m, cmRaw->frame);
		}
	}

}
void CmdStrategyMove::calEndPoseAtBase(double* out_endPoseAtBase, double* startPoseAtBase, const CmdMoveRaw* cmRaw)
{
	// 相對BASE座標移動
	if(cmRaw->frame == eFrame::BASE) {
		for(int m = 0; m < robot->gestureNum; m++) 
			out_endPoseAtBase[m] = calEndPointByMask(cmRaw->Pose[m], startPoseAtBase[m], cmRaw->PoseMask[m], cmRaw->isAbs);
	}
	// 相對TOOL末端座標移動
	else	
		moveRelativeToToolFrame(out_endPoseAtBase, startPoseAtBase, cmRaw);
	
}
int CmdStrategyMove::raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd)
{
	using namespace eCmdFormat;
	using namespace eMovePathType;

	CmdStrategy::raw2Path(pCmd, rCmd);

	int m;
	double startPoseAtBase[MAX_REDUNDANCY], midPoseAtBase[MAX_REDUNDANCY], endPoseAtBase[MAX_REDUNDANCY];
	const CmdMoveRaw* cmRaw = &rCmd.cmdMoveRaw;
	CmdMove* cmPath = &pCmd.cmdMove;
	eKin::e res = eKin::COMPLETE;

	cmPath->mvData	 = cmRaw->mvData;
	cmPath->pathType = cmRaw->pathType;
	cmPath->format   = cmRaw->format;

	cmPath->preToolId = cmPath->nowToolId; //save previous tool id.
	cmPath->nowToolId = cmRaw->scriptToolIndex;

	for(m = 0; m < robot->motorNum; m++) {
		cmPath->startVs[m] = 0;
		cmPath->  endVs[m] = 0;
	}

	setStartPoint(cmPath);	//設定 path 的起點 (root T pose)

	// 『起點』穿上目前當行的 base, 以便跟使用者輸入對應  (form "root T pose" --> "base T pose")
	// base -> world -> root -> pose
	updateBase_T(cmRaw->scriptBaseIndex);
	attachBase(startPoseAtBase, startPoseAtRoot);

	/*
	求終點 -- 三種情況：
	1. 移動各軸 
	2. 移動末端 (圓)
	3. 移動末端 (非圓)
	
	＠此階段考慮Tool
	＠除了情況1，皆考慮base

	*/
	if(	cmRaw->format == AXIS || cmRaw->format == PT_AXIS) {

		// 1. 根據 user input 得到 endAxis
		if(	cmRaw->format == AXIS) {
			for(m = 0; m < robot->motorNum; m++) 
				cmPath->endAxis[m] = calEndPointByMask(cmRaw->axis[m], startAxis[m], cmRaw->axisMask[m], cmRaw->isAbs);
		}
		else if (cmRaw->format == PT_AXIS)
			CopyArray(cmPath->endAxis, cmRaw->axis,  robot->motorNum);
		
		// 2. 順解求得 endPose
		res = robot->kinPath->FK(cmPath->endPose, cmPath->endAxis, cmPath->nowToolId);
	}
	else if(cmRaw->pathType == CIRCLE) 
	{
		// 2. 移動末端 (圓)
		//		-> user input 得到 endPose, midPose
		//		-> 移除 base
		//		-> 三點求圓弧參數
		//		-> 逆解求得 endAxis
		CopyArray(midPoseAtBase, cmRaw->Pose,  robot->gestureNum);
		CopyArray(endPoseAtBase, cmRaw->Pose2, robot->gestureNum);
		cmPath->th_User = cmRaw->theta;

		// 移除 base
		removeBase(cmPath->endPose,	endPoseAtBase);
		removeBase(cmPath->midPose, midPoseAtBase);
		
		// 三點求 圓心, 半徑, 角度, 圓平面法向量, 起始方向， 
		Circle circleObj;
		circleObj.calculateBasicParameter(cmPath->startPose, cmPath->midPose, cmPath->endPose, cmPath->th_User);
		
		// 向下傳到 cmPath->circle
		circleObj.getCData(cmPath->circle);

		// 若使用者指定角度，則更新 endPose。 //note:只更新xyz. 其餘(ΔR)維持和endPose相同
		if (cmPath->th_User >= 0) 
			circleObj.theta2XYZ(cmPath->endPose, cmPath->circle.th_Final);	

		// 逆解
		res = robot->kinPath->IK(cmPath->endAxis, cmPath->endPose, cmPath->nowToolId);	
	}	
	else {
		// 3. 移動末端 (非圓)
		//		-> user input 得到 endPose
		//		-> 移除 base
		//		-> 逆解求得 endAxis
		if(cmRaw->format == POSE) 	
			calEndPoseAtBase(endPoseAtBase, startPoseAtBase, cmRaw);
		
		else if(cmRaw->format == PT_TCP)	
			CopyArray(endPoseAtBase, cmRaw->Pose, robot->gestureNum);
		
		removeBase(cmPath->endPose,	endPoseAtBase); // 移除 base

		// 逆解
		if(cmRaw->format == POSE) 
			robot->kinPath->setRefThetas(startAxis);	//POSE. 參考路徑起點姿態 
		if(cmRaw->pathType == CIRCLE)
  		   robot->kinPath->setRefThetas(cmRaw->axis);	//PT. 參考記點姿態

		res = robot->kinPath->IK(cmPath->endAxis, cmPath->endPose, cmPath->nowToolId);

		// 若「點位」為奇異點，又想用 P2P 到達，則使用各軸移動。  條件: 1.不能換tool/base 2.不為Master/Slave
		if( res == eKin::WRIST_SINGULAR	&& 
			cmRaw->pathType == P2P	&&
			cmRaw->format == PT_TCP	&&
			cmRaw->scriptBaseIndex == cmRaw->pointBaseIndex &&
			cmRaw->scriptToolIndex == cmRaw->pointToolIndex &&
			pCmd.info.myMasterId == -1 ) 
		{
			// 直接用「點位」裡記錄的 axis 值
			CopyArray(cmPath->endAxis, cmRaw->axis, robot->motorNum);			
			res = robot->kinPath->FK(cmPath->endPose, cmPath->endAxis, cmPath->nowToolId); 
		}
	}	

	// 若運動學不通過 且 不是Master-Slave，則回報錯誤
	if(res != eKin::COMPLETE && pCmd.info.myMasterId == -1 ) {
		eError::e errType;
		if(res == eKin::ANGLE_LIMIT)
			errType = eError::ANGLE_LIMIT;
		else									
			errType = eError::IK_FAIL;

		ErrorHandler::Set(errType, "CmdStrategyMove/raw2path: Robot%d, func=%d, line=%d,Error=%s", 
			robot->rId+1, pCmd.info.funcId+1, pCmd.info.lineId, Kinematics::errorMsg(res));
		return -1;	
	}

	// 更新下一行命令的起點
	CopyArray(startAxis,		cmPath->endAxis, MAX_MOTOR_PER_ROBOT);
	CopyArray(startPoseAtRoot,	cmPath->endPose, robot->gestureNum); // without base

	return 1;
}
int CmdStrategyMove::processIntpCmd(const IntpCmd& iCmd)  //11412 PMC modified //1211
{
	const double Max_Velocity = 200.0;
	for (int i = 0; i < robot->motorNum; i++) {
		double axisVelCmd = (robot->axisDegCmdPrev[i] - iCmd.axisDeg[i]) * SAMPLING_T_INV;

		// 200度/s 視為跳點。 若無跳點則繼續；若跳點則回傳false;
		if (abs(axisVelCmd) < Max_Velocity) continue;
		ScriptData* shmScrData = &(shm->scriptData[robot->rId]);
		shm->stopScript = true;
		ErrorHandler::Set(eError::AXIS_JUMP,
			"Robot/isAxisJump: r[%d], axis[%d], func[%d], line[%d],  PrevDeg[%lf], nextDeg[%lf],axisVelCmd[%lf]",
			robot->rId + 1,
			i + 1,
			shmScrData->nowFuncId + 1,
			shmScrData->nowLineId,
			robot->axisDegCmdPrev[i],iCmd.axisDeg[i],
			axisVelCmd);
		return -1;
	}
	CopyArray(robot->axisDegCmdPrev, iCmd.axisDeg, robot->motorNum);
	CopyArray(robot->axisDegCmd, iCmd.axisDeg, robot->motorNum);

	robot->isNeedMove = true;	
	
	// ----- 這筆插補已經完成。把它丟掉。
	script->intpQueue.dequeue();
	
	return 1;
}

//========= IO ============
int CmdStrategyIO::raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd)
{
	CmdStrategy::raw2Path(pCmd, rCmd);
	pCmd.cmdIo = rCmd.cmdIo;
	return 1;
}
int CmdStrategyIO::processIntpCmd(const IntpCmd& iCmd)
{
	const CmdIo* cmd = &iCmd.cmdIo;

	for(int i = 0; i< cmd->ioCmdNum; i++) {
		int  m	= cmd->io[i].moduleInd;
		int  b	= cmd->io[i].bitInd;
		bool s	= cmd->io[i].status;

		ioGroup->ioModules[m]->setOutput(b, s);
		
	}
	script->intpQueue.dequeue();
	return 1;
}

//========= Delay ============
int CmdStrategyDelay::raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd)
{
	CmdStrategy::raw2Path(pCmd, rCmd);
	pCmd.cmdDelay = rCmd.cmdDelay;
	return 1;
}
int CmdStrategyDelay::processIntpCmd(const IntpCmd& iCmd)
{
	// 新的delay命令。拿delay Time。
	if (remainTime <= 0)
		remainTime = iCmd.cmdDelay.time;

	// 檢查急停/緩停
	if (shm->stopScript == true) //11408 PMC Modified
	{
		remainTime = 0;
		if (shm->isSlowStop == true)
			shm->isSlowStop = false;
	}
	if (shm->vGain == 0)  return 1; //PMC Modified 11412
	remainTime--;
	
	if(remainTime <= 0 ) 
		script->intpQueue.dequeue();

	return 1;
}
void CmdStrategyDelay::reset()
{
	remainTime = 0;
}

//========= Sync ============
int CmdStrategySync::raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd)
{
	CmdStrategy::raw2Path(pCmd, rCmd);
	pCmd.cmdSync = rCmd.cmdSync;
	return 1;
}
int CmdStrategySync::processIntpCmd(const IntpCmd& iCmd)
{
	int rInd = robot->rId;
	int syncId = iCmd.cmdSync.syncId;
	bool isReadyToContinue = false;

	//temp: for debug. if no bug then delete it.
	if(syncId<0) { 
		Debug::writeln(0,"error in CmdStrategySync::processIntpCmd!!  r=%d, sync=%\n"); 
		syncId=0;
	} 

	// 將自己燈號改為 "READY".
	if(shm->syncTable[syncId][rInd] != eSyncState::READY) {
		shm->syncTable[syncId][rInd] = eSyncState::READY;
		robot->isAtSyncId = syncId;	
	}

	// 檢查 'Gate'行 是否為READY。
	isReadyToContinue = shm->syncTable[syncId][MAX_ROBOT_NUM] == eSyncState::READY? true: false;

	// 檢查急停/緩停
	if(shm->stopScript == true) 
		isReadyToContinue = true;
	
	// 若Sync通過，則將自己燈號改回NOT_YET，並前進至下一行命令。
	if(isReadyToContinue == true ) {

		script->setEvtHMISync(); 
		shm->syncTable[syncId][rInd] = eSyncState::NOT_YET;
		script->intpQueue.dequeue();
	}
	return 1;
}

//========= HMISync ============
int CmdStrategyHMISync::raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd)
{
	CmdStrategy::raw2Path(pCmd, rCmd);
	return 1;
}
int CmdStrategyHMISync::processIntpCmd(const IntpCmd& iCmd)
{
	script->setEvtHMISync(); 
	script->intpQueue.dequeue();
	return 1;
}

//========= MS (Master-Slave) ============
int CmdStrategyMS::raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd)
{
	CmdStrategy::raw2Path(pCmd, rCmd);

	script->isNeedUpdateAttachRef = true;//PMC Modified 11411
	
	if(pCmd.info.myMasterId != -1) //區段開始時
		script->isInMSBlock = true;
	else {//區段結束時
		script->psSyncer->setIsNeedSync(true);
		script->isInMSBlock = false;
	}
	return 1;
}
int CmdStrategyMS::processIntpCmd(const IntpCmd& iCmd)
{
	// 遇到 MS 區段結尾，需和人機作同步。為了下一行的raw2Path在這之後才執行，才能鋪對起點。
	if(iCmd.info.myMasterId == -1)
		script->psSyncer->setEvt();

	script->intpQueue.dequeue();
	return 1;
}

