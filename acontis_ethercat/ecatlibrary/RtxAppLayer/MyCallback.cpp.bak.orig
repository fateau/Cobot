#include "MyCallback.h"
#include "Shm.h"
#include "Debug.h"
#include "MySystem.h"
#include "Robot.h"
#include "Motor.h"
#include "Kinematics.h"
#include "Script.h"
#include "CmdContext.h"
#include "PathStartSyncer.h"
#include "Jog.h"
#include "HelpFunctions.h"
#include "ErrorHandler.h" 
#include "MathTool_Robot.h" 

extern MySystem*	sys;
extern SHMData*		shm;

MSPose		msPoses[MAX_ROBOT_NUM];
IntpCmd		intpCmd[MAX_ROBOT_NUM];
bool		isIntpCmdOK[MAX_ROBOT_NUM];
int			mId = -1; //Master-Slave      
bool        isStartMasterSlave = true; 

void scriptStopRunning()
{
	Debug::write(0, "* scriptStopRunning!\n");
	shm->isScriptRunning = false;
	EventHandler::Set(sys->evtScriptEnd); // tell HMI script is finished.
	EventHandler::Reset(sys->evtScriptStart); // to prevent scriptStartThread running w/o HMI press btn.
	

	for (int r = 0; r < sys->robotNum; r++) {
		sys->robots[r]->script->prepareStop(); // stop script.
		EventHandler::Set(sys->evtScriptSet[r]);
	}

	return;
}
void mcLinkMasterSlave(int mId, int sId)
{
	for (int i = 0; i < 6; i++) {
		msPoses[sId].M_target_pose[i] = sys->robots[mId]->poseAtRoot[i];
		msPoses[sId].S_target_pose[i] = sys->robots[sId]->poseAtRoot[i];
		msPoses[sId].M_pre_pose[i] = sys->robots[mId]->poseAtRoot[i];
		msPoses[sId].S_pre_pose[i] = sys->robots[sId]->poseAtRoot[i];
	}
}
bool mcDoMasterSlave(int mId, int sId, IntpCmd intpCmd[], bool isIntpCmdOK[])
{
	if ((isIntpCmdOK[mId] == false || isIntpCmdOK[sId] == false) && shm->isSlowStop == false)  
		return false;

	sys->robots[sId]->isNeedMove = true;		//就算Slave本身沒命令，還是有可能移動。

	double* M_pre_pose = msPoses[sId].M_pre_pose;
	double* S_pre_pose = msPoses[sId].S_pre_pose;
	double* M_target_pose = msPoses[sId].M_target_pose;
	double* S_target_pose = msPoses[sId].S_target_pose;

	double newSlavePose[MAX_MOTOR_PER_ROBOT];
	double newSlaveAxis[MAX_MOTOR_PER_ROBOT];

	// ========== calculate new value ========== // note: Pose is "Pose At Root".

	if (intpCmd[mId].info.type != eScriptCmdType::MOVE) {}	//Master本身沒命令，直接改 nextPose	 
	else {
		for (int i = 0; i < 6; i++)
			M_target_pose[i] = intpCmd[mId].Pose[i];		//Master本身有命令，改intpCmd的值。
	}


	if (intpCmd[sId].info.type != eScriptCmdType::MOVE) {}	//Slave本身沒命令，就是follow。
	else {
		for (int i = 0; i < 6; i++)
			S_target_pose[i] = intpCmd[sId].Pose[i];		//Slave本身有命令，改intpCmd的值
		attachTool(S_target_pose,S_target_pose,intpCmd[sId].toolIndex); 
	}

	getBindedPose(newSlavePose,
		M_pre_pose, M_target_pose,
		S_pre_pose, S_target_pose,
		sys->robots[mId]->world2Root_T, sys->robots[sId]->world2Root_T);

	eKin::e res = sys->robots[sId]->kinMC->IK(newSlaveAxis, newSlavePose, intpCmd[sId].toolIndex);
	
	//IK Fail 選擇返回直接跟隨
	if(res != eKin::COMPLETE ) {
		eError::e errType;
		if(res == eKin::ANGLE_LIMIT)
			errType = eError::ANGLE_LIMIT;
		else									
			errType = eError::IK_FAIL;

		ErrorHandler::Set(errType, "mcDoMasterSlave: r[%d], func[%d], line[%d],Error=%s", 
			sId+1, intpCmd[sId].info.funcId+1,  intpCmd[sId].info.lineId, Kinematics::errorMsg(res));
	    return false;
	}

	//遇到跳點偵測
	if(!isStartMasterSlave)
	{
	       for (int i = 0; i < MAX_MOTOR_PER_ROBOT; i++)
	       {
	       	     if(abs(newSlaveAxis[i] - sys->robots[sId]->axisDegCmd[i]) > 1) 
	       	     {
	       	     	ErrorHandler::Set(eError::AXIS_JUMP, 
	       	     	"Robot/isAxisJump: r[%d], axis[%d], func[%d], line[%d],  prevDeg[%lf], nextDeg[%lf]", 
	       	     	sId+1, 
	       	     	i+1, 
	       	     	intpCmd[sId].info.funcId+1,
	       	     	intpCmd[sId].info.lineId, 
	       	     	sys->robots[sId]->axisDegCmd[i],
	       	     	newSlaveAxis[i]);	
	       	     	return false;
	       	     }
	       }
	}
	isStartMasterSlave = false; //11310 IFang

	// ========== assign value =================
	for (int i = 0; i < MAX_MOTOR_PER_ROBOT; i++)
	{
		if (intpCmd[sId].info.type == eScriptCmdType::MOVE) //note: might duplicate with mcProcessScriptCmd
			intpCmd[sId].axisDeg[i] = newSlaveAxis[i];
		else
			sys->robots[sId]->axisDegCmd[i] = newSlaveAxis[i];
	}

	return true;
}
void mcProcessScriptCmd(int r, IntpCmd& intpCmd)
{
	Script* script = sys->robots[r]->script;
	ScriptData* shmScrData = &(shm->scriptData[r]);

	//換行-- 若這一筆插補為新一行、或切到副程式，則更新行數
	int lineId = intpCmd.info.lineId;
	int funcId = intpCmd.info.funcId;

	if (lineId != shmScrData->nowLineId ||
		funcId != shmScrData->nowFuncId)
	{
		shmScrData->nowLineId = lineId;
		shmScrData->nowFuncId = funcId;
	}

	//處理這一筆插補。如果失敗則急停劇本。
	int ret = script->cmdContext->processIntpCmd(intpCmd);
	if (ret < 0) scriptStopRunning();
}
void mcUpdateSyncTable(int r)
{
	int syncId = sys->robots[r]->isAtSyncId;
	if (syncId < 0) return;

	// 將 Gate行 預設為 'READY', 若該列中有任何一個為 NOT_YET, 則設為NOT_YET. 
	shm->syncTable[syncId][MAX_ROBOT_NUM] = eSyncState::READY;

	for (int r2 = 0; r2 < sys->robotNum; r2++) {
		if (shm->syncTable[syncId][r2] == eSyncState::NOT_YET) {
			shm->syncTable[syncId][MAX_ROBOT_NUM] = eSyncState::NOT_YET;
			break;
		}
	}
}

void getCommandForEachScript(int r)
{
	Script* script = sys->robots[r]->script;
	ScriptData* shmScrData = &(shm->scriptData[r]);

	if (script->isIntpQueueTerminate == true) return;

	//拿取 intpQ 的一筆命令。 用while loop 來處理平行命令(ex.運動中途IO)，平行命令不占用 1ms。當遇到不是平行命令時跳出迴圈
	while (true) {
		int ret = script->intpQueue.getNodeAtFront(&intpCmd[r]);//只看不拿		

		if (ret < 0) {
			isIntpCmdOK[r] = false;
			if (script->isNoMoreInterpolate() == true) {
				script->isIntpQueueTerminate = true;
				sys->scrRobotNum--;
			}

			break;
		}
		else {

			isIntpCmdOK[r] = true;
			mId = intpCmd[r].info.myMasterId;

			//檢查是否為 MS命令。
			if (intpCmd[r].info.type == eScriptCmdType::MS && mId != -1)
				mcLinkMasterSlave(mId, r); //建立 Master-Slave 連結關係。若 mId=-1則表示解除

			//若為平行命令。則執行此命令並繼續迴圈。 否:跳出迴圈 
			if (intpCmd[r].isParallel)
				script->cmdContext->processIntpCmd(intpCmd[r]);
			else
				break;
		}
	}

	//處理Master-Slave的移動指令變更 
	if (mId != -1)
	{
		//如果逆解失敗則直接結束後續插補，避免跳點 
		bool isFailIntp = mcDoMasterSlave(mId, r, intpCmd, isIntpCmdOK) ;
		if(!isFailIntp)  
		{
			for (int i = 0; i < sys->robotNum; i++) isIntpCmdOK[i] = false;
			Debug::writeln(0,"Fail to master slave");
		}
	}
}

//===== Core function =======
void mcChangeRTXMode()
{
	static eRTXMode::e currentRTXMode = eRTXMode::CSP; // 預設為CSP

	if (currentRTXMode == shm->RTXMode) return;

	Debug::writeln(0, "Mycallback:  currentMode:%d, shm->RTXMode:%d\n", currentRTXMode, shm->RTXMode);
	currentRTXMode = shm->RTXMode;

	for (int r = 0; r < sys->robotNum; r++) {

		Robot* robot = sys->robots[r];
		robot->isNeedMove = true;

		for (int m = 0; m < robot->motorNum; m++) {
			robot->axisDegCmd[m] = robot->axisDegNow[m]; //為了切回CSP時不跳點。
			robot->axisDegCmdPrev[m] = robot->axisDegNow[m];
			robot->motors[m]->setMode(shm->RTXMode);
		}
	}

}
void mcDoScript()
{
	if (shm->stopScript == true && shm->isSlowStop == false) {
		scriptStopRunning();	//急停
		return;
	}

	for (int r = 0; r < sys->robotNum; r++)
		getCommandForEachScript(r);

	// 對每個機器人，執行一段插補指令
	for (int r = 0; r < sys->robotNum; r++) {
		if (isIntpCmdOK[r] == false) continue;
		//if (shm->vGain == 0) return; //PMC Modified 11412
		mcProcessScriptCmd(r, intpCmd[r]);
	}

	// update syncTable 
	for (int r = 0; r < sys->robotNum; r++)
		mcUpdateSyncTable(r);

	// 若所有 robot 皆結束劇本，宣告劇本執行結束。
	if (sys->scrRobotNum == 0)
		scriptStopRunning();

	if (shm->stopScript == true && shm->isSlowStop == true)
		for (int r = 0; r < sys->robotNum; r++)
			sys->robots[r]->script->slowStop();		//緩停	
}
void mcDoJog()
{
	int ret;
	for (int r = 0; r < sys->robotNum; r++) {
		ret = sys->robots[r]->jog->processCommand();
		if (ret > 0) sys->robots[r]->isNeedMove = true;
	}

}
//===========================