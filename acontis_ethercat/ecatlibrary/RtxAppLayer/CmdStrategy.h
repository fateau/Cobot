#pragma once

#include "Def.h"

class Robot;
class Script;
class IOGroup;
class CmdStrategyMS;
class CmdStrategySync;


class CmdStrategy
{
public:
	CmdStrategy(Robot* robot);
	~CmdStrategy(void);
	
	virtual int raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd);
	virtual int processIntpCmd(const IntpCmd& iCmd)=0;

	Script*	script;
	Robot*	robot;
};

class CmdStrategyMove:public CmdStrategy
{
public:
	CmdStrategyMove(Robot* robot, IOGroup* ioGroup):CmdStrategy(robot){ 
		currentBaseId=-1;
		this->ioGroup = ioGroup; //temp.
	};
	~CmdStrategyMove(){};
	int raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd);
	int processIntpCmd(const IntpCmd& iCmd);

	void reset();

	CmdStrategyMS* csMS; // in order to get variable "isEndOfSlave"
	CmdStrategySync* csSync;

private:
	// starting point for next move-cmd.
	double startPoseAtRoot[MAX_REDUNDANCY];
	double startAxis[MAX_MOTOR_PER_ROBOT];

	// for base changing.
	int		currentBaseId;
	double	world2Base_T[4][4];	
	double	base2World_T[4][4];
	void	updateBase_T(int baseId);
	void	attachBase(double out_PoseAtBase[MAX_REDUNDANCY], double PoseAtRoot[MAX_REDUNDANCY]);
	void	removeBase(double out_PoseAtRoot[MAX_REDUNDANCY], double PoseAtBase[MAX_REDUNDANCY]);

	void	setStartPoint(CmdMove* cmPath);
	double	calEndPointByRelative(double userInput, double startPoint, BOOL isRelative);
	double	calEndPointByMask(double userInput, double startPoint, BOOL isMasked, BOOL isRelative);
	void	moveRelativeToToolFrame(double* endPoseAtBase, double* startPoseAtBase, const CmdMoveRaw* cmRaw);
	void	calEndPoseAtBase(double* out_endPoseAtBase, double* startPoseAtBase, const CmdMoveRaw* cmRaw);
	IOGroup* ioGroup; //temp. should use a "Condition" class to handle condition judgement.
};
class CmdStrategyIO:public CmdStrategy
{
public:
	CmdStrategyIO(Robot* robot, IOGroup* ioGroup):CmdStrategy(robot){
		this->ioGroup = ioGroup;
	};
	~CmdStrategyIO(){};
	int raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd);
	int processIntpCmd(const IntpCmd& iCmd);
private:
	IOGroup* ioGroup;
};
class CmdStrategyDelay:public CmdStrategy
{
public:
	CmdStrategyDelay(Robot* robot):CmdStrategy(robot){};
	~CmdStrategyDelay(){};
	int raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd);
	int processIntpCmd(const IntpCmd& iCmd);
	void reset();

	int remainTime;
};
class CmdStrategySync:public CmdStrategy
{
public:
	CmdStrategySync(Robot* robot):CmdStrategy(robot){};
	~CmdStrategySync(){};
	int raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd);
	int processIntpCmd(const IntpCmd& iCmd);
};
class CmdStrategyHMISync:public CmdStrategy
{
	public:
	CmdStrategyHMISync(Robot* robot):CmdStrategy(robot){};
	~CmdStrategyHMISync(){};
	int raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd);
	int processIntpCmd(const IntpCmd& iCmd);
};
class CmdStrategyMS:public CmdStrategy
{
	public:
	CmdStrategyMS(Robot* robot):CmdStrategy(robot){};
	~CmdStrategyMS(){};
	int raw2Path(ScriptPathCmd& pCmd, const ScriptRawCmd& rCmd);
	int processIntpCmd(const IntpCmd& iCmd);

};