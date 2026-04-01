#pragma once
#include "Def.h"
#include "MyQueue.h"
#include "MyQueueNoLock.h"

class Interpolator;
class CmdContext;
class Robot;
class PathLoader;
class TrajectoryPlanner;
class PathStartSyncer;

class Script
{
public:

	//-------- member --------
	CmdContext* cmdContext;					//Note: is newd in MySystem::initRobots();

	MyQueue<ScriptRawCmd>	rawQueue;
	MyQueue<ScriptPathCmd>	ppQueue;		//path-planning queue.
	MyQueueNoLock<IntpCmd>	intpQueue;		//分解後的cmd放在這個Queue	

	
	bool* isHMITerminate;					//指到shm的isHMIScriptTerminate
	bool isRawQueueTerminate;
	bool isPPQueueTerminate;
	bool isIntpQueueTerminate;
	
	int	rId;

	//pathStart sync (sync before do raw2Path)
	PathStartSyncer* psSyncer;
	bool isAfterSync;

	// MasterSlave
	bool isInMSBlock;
	bool isNeedUpdateAttachRef; //PMC Modified 11411
	
	Robot* robot; //temp move to public. (used in Planning Thread-sync)

	//-------- function --------
	Script(Robot* robot);
	~Script(void);

	void prepareStart();
	void prepareStop();

	void setStartPoint();
	void setEvtHMISync();
	void setDefaultScriptToolIndex(int defaultScriptToolIndex);
	void setIsTerminate(bool isTerminate);

	int  raw2Path(ScriptPathCmd& pCmd, ScriptRawCmd& rCmd);
	bool isNoMoreInterpolate();
	int  tryToSetNewPath();			//設定interpQ的目標路徑
	int  interpolate();				//呼叫插補器進行插補，並將結果interpCmd塞入interpQ	
	void changeSpeed(double vGain);
	void clearAllQ();
	void slowStop();	//劇本緩停	

	double	world2Pose_T[4][4];
	double	world2Root_T[4][4];	// = world2Ref_T * ref2Root_T	
	double	ref2Root_T[4][4];	// = constant.
	double	(*world2Ref_T)[4];	// = world2Pose_T of another robot; or world(Identical)

	double refPose[4][4];
	
	void updateTransferMatrix(double poseAtRoot[6]);

private:
	
	TrajectoryPlanner*	trajPlanner;	// 此類別使用 Eigen，必須由new 產生物件。不然會爆炸
	HANDLE				evtHMISync;

	PathLoader*		pLoader;
	// for pathPlanning.
	double			_currentVGain;	//用來判斷是否需要resume
	// for Interpolator to count which ppQueue Cmd it is using.
	IntpCmd			_nowICmd;		//單筆插補命令
	IntpCmd			_preICmd;		//單筆插補命令

	bool isIntpErrorOccur; //to let intp stop immediatly after error occur.

	// ============== support function ========================
	int updateVelAndAcc(CmdMove& path);
	eIntpResult::e checkIntpCmd(IntpCmd& intpCmd);

	void updateIntpAxisVelAndAcc(IntpCmd& out_nowIntpCmd , IntpCmd& preIntpCmd);
	bool isReadyToSetNextPath();
	int  setNewPath(ScriptPathCmd& pCmd);

};

