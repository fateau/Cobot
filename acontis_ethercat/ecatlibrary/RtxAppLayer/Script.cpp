#include "Script.h"
#include "Robot.h"
#include "Motor.h"
#include "TrajectoryPlanner.h"
#include "PathStartSyncer.h"
#include "CmdContext.h"
#include "PathLoader.h"
#include "Debug.h"
#include "Kinematics.h"
#include "Shm.h"
#include "ErrorHandler.h"

extern SHMData* shm;
void copyIOData(CmdIo& target, CmdIo& source);

Script::Script(Robot* robot)
	:rawQueue(RAWQ_SIZE), ppQueue(PPQ_SIZE), intpQueue(INTPQ_SIZE)
{
	this->robot			= robot;
	rId					= robot->rId;
	trajPlanner			= new TrajectoryPlanner(robot);
	psSyncer			= new PathStartSyncer(rId);
	isHMITerminate		= &(shm->scriptData[rId].isHMICmdTerminate);
	
	setIsTerminate(true);

	pLoader = new PathLoader();
	pLoader->init(this);
	
	// 開啟 Event
	char str[20];
	sprintf(str, "evtScriptHMISync%d", robot->rId);
	evtHMISync	= EventHandler::Open(str);

}
Script::~Script(void)
{
	delete trajPlanner;
	delete psSyncer;
	delete pLoader;
	delete cmdContext;	// cmdContext is newed in MySystem::initRobots();
}

void Script::prepareStart()
{
	// 每次執行劇本前呼叫。重置參數、清空 Queue等
	clearAllQ();
	setIsTerminate(false);	
	setDefaultScriptToolIndex(shm->jogToolIds[rId]);
	pLoader->reset();
	cmdContext->reset();
	trajPlanner->reset();
	psSyncer->reset();
	isAfterSync = false;
	isIntpErrorOccur = false;
	isInMSBlock = false;
	isNeedUpdateAttachRef = true; //PMC Modified 11411
	
	updateTransferMatrix(robot->poseAtRoot);

	_currentVGain = shm->vGain;

	for(int i = 0; i < MAX_MOTOR_PER_ROBOT; i++)
	{
		_preICmd.axisDeg[i] = 0;
		_preICmd.axisVel[i] = 0;
		_preICmd.axisAcc[i] = 0;

		_nowICmd.axisDeg[i] = 0;
		_nowICmd.axisVel[i] = 0;
		_nowICmd.axisAcc[i] = 0;
		robot->axisDegCmdPrev[i] = robot->axisDegNow[i];
	}
	_nowICmd.isParallel = false;

	shm->scriptData[rId].nowLineId = 0;
	shm->scriptData[rId].nowFuncId = -1;
	shm->scriptData[rId].scriptRawCmd.info.myMasterId = -1;

	robot->kinPath->setRefThetas(robot->axisDegNow);
	robot->kinIntp->setRefThetas(robot->axisDegNow);

	//重置
	EventHandler::Set(evtHMISync);
	EventHandler::Reset(evtHMISync);
}
void Script::prepareStop()		
{
	// 每次停止劇本前呼叫。設定flag、釋放event等
	clearAllQ();	
	setIsTerminate(true);
	EventHandler::Reset(evtHMISync);
	psSyncer->release();

	for(int m = 0; m<robot->motorNum;  m++)
		robot->motors[m]->setMode(eRTXMode::CSP);  

	Debug::writeln(0, "Script::prepareStop()	\n");
}

int Script::raw2Path(ScriptPathCmd& pCmd, ScriptRawCmd& rCmd)
{	
	//在raw2Path之前,先等待同步 (參閱CmdStrategySync)  
	if(psSyncer->getIsNeedSync() == true){		
		psSyncer->waitEvt();
		isAfterSync = true;
	}

	return cmdContext->raw2Path(pCmd, rCmd);;
}

void Script::updateTransferMatrix(double poseAtRoot[6])
{
	// update World2Pose according to the pose in latest pCmd. In order to let other robot take reference.
	double root2Pose_T[4][4];
	transferPose2T(root2Pose_T, poseAtRoot);
	multiplyMatrix_4x4(world2Pose_T, world2Root_T, root2Pose_T);
}

bool Script::isNoMoreInterpolate()
{
	if(isPPQueueTerminate == true &&
	   trajPlanner->emptyPathSetterNum() == 2)
	   return true;

	return false;
}

//設定interpQ的目標路徑
int  Script::tryToSetNewPath()
{
	int emptyPSNum = trajPlanner->emptyPathSetterNum();

	if(emptyPSNum == 0)			return 2;  // no need New Path.
	if(pLoader->load() < 0)		return -2; // load new path fail.	
	if(!isReadyToSetNextPath()) return 3;
	if(shm->vGain == 0)			return -3; //don't set new path while pause.(因為vGain=0時, acc等初始參數的計算會有問題,尤其是S-Curve)

	ScriptPathCmd tmpPCmd = pLoader->takePCmd();
	return setNewPath(tmpPCmd);
}
bool Script::isReadyToSetNextPath()
{
	if(trajPlanner->emptyPathSetterNum() == 2)		return true;
	if(pLoader->pCmdType() == eScriptCmdType::MOVE)	return true;
	return false;
}
int  Script::setNewPath(ScriptPathCmd& pCmd)
{
	_nowICmd.info = pCmd.info;

	switch (pCmd.info.type) {
		case eScriptCmdType::MOVE:
			updateVelAndAcc(pCmd.cmdMove);
			trajPlanner->setNewPath(pCmd);//設定軌跡規劃器
			return 4;
			break;

		case eScriptCmdType::IO_OUT:
			copyIOData(_nowICmd.cmdIo, pCmd.cmdIo);
			break;

		case eScriptCmdType::DELAY:
			_nowICmd.cmdDelay = pCmd.cmdDelay;
			break;

		case eScriptCmdType::SYNC:
			_nowICmd.cmdSync = pCmd.cmdSync;
			break;
		
		case eScriptCmdType::HMI_SYNC:
		case eScriptCmdType::MS:
			break;

		default:
			return -1;
			break;
	}

	
	intpQueue.enqueue(_nowICmd);

	return 5;
}

int  Script::interpolate()
{
	if(trajPlanner->emptyPathSetterNum() == 2) return 1; //不需插補	
	if(shm->stopScript == false) changeSpeed(shm->vGain); //變速

	eIntpResult::e	intpRes = trajPlanner->getIntpCmd(_nowICmd);	

	if(intpRes != eIntpResult::SUCCESS)	
		return intpRes;
	
	// 計算命令端的各軸速度
	intpQueue.getNodeAtRear(&_preICmd);
	updateIntpAxisVelAndAcc(_nowICmd , _preICmd);
		
	//檢查插補結果是否 angle limit 或 sigular	
	if(checkIntpCmd(_nowICmd) != eIntpResult::SUCCESS)
		return eIntpResult::FAIL;

	intpQueue.enqueue(_nowICmd);
	return 1;	
}
void Script::changeSpeed(double newVGain)
{
	trajPlanner->setVGain(newVGain);
	return;
}
void Script::clearAllQ()
{
	rawQueue.clearAll();
	ppQueue.clearAll();
	intpQueue.clearAll();
}
void Script::slowStop()
{
	// Note: 此函式可由兩種途徑觸發
	//		1.人機更改shm->stopScript 參數；MyCallBack 檢查此參數後觸發。
	//		2.底層運算發現錯誤時觸發。
	
	changeSpeed(0);

	rawQueue.clearAll();
	ppQueue.clearAll();

	isRawQueueTerminate = true;
	isPPQueueTerminate = true;

	// 若緩停時手臂不需要移動(ex SYNC, DELAY)則清空intpQ，以免之後繼續移動。Note. 不可將isIntpQTerminate=true.否則mcDoScript會卡住
	if( !robot->isNeedMove)
		intpQueue.clearAll();

	// 由底層觸發的情況下，要一併改shm參數。此參數可讓interpolator分辨中止/暫停。
	shm->stopScript = true;
	shm->isSlowStop = true;
}

void Script::setEvtHMISync()
{
	EventHandler::Set(evtHMISync);
}

void Script::setDefaultScriptToolIndex(int defaultScriptToolIndex)
{
	trajPlanner->setDefaultScriptToolIndex(defaultScriptToolIndex);
}
void Script::setIsTerminate(bool isTerminate)
{
	*isHMITerminate			= isTerminate;
	isRawQueueTerminate		= isTerminate;
	isPPQueueTerminate		= isTerminate;
	isIntpQueueTerminate	= isTerminate;
}

// ================= private function ========================
int Script::updateVelAndAcc(CmdMove& path)
{
	// 令各軸最大速度=劇本指定速度 (如果spec檔有另外指定最大速度，則取兩者中較小值)
	RobotSpec* spec = &(shm->robots[rId].spec);
	
	if( path.pathType <= eMovePathType::P2P){	//P2P or JOINT
		for( int i = 0; i < robot->motorNum ; i++)
			path.maxVs[i] = min(spec->maxAxisVel[i], path.mvData.maxV);
	}
	else {										//LINE or CIRCLE
		for( int i = 0; i < 2 ; i++)
			path.maxVs[i] = min(spec->maxTCPVel, path.mvData.maxV);
	}
	
	return 1;
}

//檢查插補結果是否 1.angle limit  2.singular
eIntpResult::e Script::checkIntpCmd(IntpCmd& intpCmd)
{
	if(isIntpErrorOccur) return eIntpResult::FAIL;

	if(intpCmd.format == eTargetFormat::AXIS) {
		for(int m = 0; m < robot->motorNum; m++) {

			// 若超過角度極限 && 沒有跟隨 Master，則回報錯誤。
			if(robot->kinIntp->isReachAngleLimit(intpCmd.axisDeg[m], m) == true &&
			   intpCmd.info.myMasterId == -1) 
			{
				ErrorHandler::Set(eError::ANGLE_LIMIT, 
					"Script/isIntpCmdOk: r[%d], func[%d], line[%d] axis[%d], deg[%lf]", 
					rId+1, 
					intpCmd.info.funcId+1,
					intpCmd.info.lineId, 
					m+1, 
					intpCmd.axisDeg[m]
				);
				isIntpErrorOccur = true;
				return eIntpResult::FAIL;
			}
		}

		robot->kinIntp->FK(intpCmd.Pose, intpCmd.axisDeg);
	}
	else if(intpCmd.format == eTargetFormat::POSE) {
		eKin::e eKin = robot->kinIntp->IK(intpCmd.axisDeg, intpCmd.Pose, -1);//check: no tool here?

		// 若逆解失敗 && 沒有跟隨 Master，則回報錯誤。
		if(eKin != eKin::COMPLETE && intpCmd.info.myMasterId == -1) {
			ErrorHandler::Set(eError::IK_FAIL, 
					"Script/isIntpCmdOk: r[%d], func[%d], line[%d] pose[%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf],Error=%s\n", 
					rId+1,
					intpCmd.info.funcId,
					intpCmd.info.lineId, 
					intpCmd.Pose[0],
					intpCmd.Pose[1],
					intpCmd.Pose[2],
					intpCmd.Pose[3],
					intpCmd.Pose[4],
					intpCmd.Pose[5],
					Kinematics::errorMsg(eKin)
				);
			isIntpErrorOccur = true;
			return eIntpResult::FAIL;
		}
	}

	return eIntpResult::SUCCESS;
}

void Script::updateIntpAxisVelAndAcc(IntpCmd& out_nowIntpCmd , IntpCmd& preIntpCmd)
{
	// 利用差分法來計算估測的各軸速度
	for(int i = 0; i < MAX_MOTOR_PER_ROBOT; i++)
	{
		out_nowIntpCmd.axisVel[i] = ( out_nowIntpCmd.axisDeg[i] - preIntpCmd.axisDeg[i] ) / SAMPLING_T;
		out_nowIntpCmd.axisAcc[i] = ( out_nowIntpCmd.axisVel[i] - preIntpCmd.axisVel[i] ) / SAMPLING_T;
	}
}

void copyIOData(CmdIo& target, CmdIo& source)
{
	target.ioCmdNum = source.ioCmdNum;
			
	for(int i = 0; i< source.ioCmdNum; i++)
		target.io[i] = source.io[i];
}