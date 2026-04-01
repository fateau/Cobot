#pragma once

#include "Def.h"
#include "Interpolator.h"



class Interpolator;
class PathSetter;
class PathSetter_P2P;
class PathSetter_Lin;
class PathSetter_Cir;
class Circle;

namespace ePathSetterStatus{ //todo: change to bool isEmpty?
	enum e {
		EMPTY,
		FULL,
		MIDDLE
	};
};
using namespace ePathSetterStatus;

class PathSetterContext
{
public:
	friend class PathSetter;
	PathSetterContext(int motorNum, int gestureNum);
	~PathSetterContext();

	PathSetter* selectStrategy(eMovePathType::e type);

	bool isEmpty();
	void reset();

private:
	PathSetter*		pathSetter;

	PathSetter_P2P* pathSetterP2P;
	PathSetter_Lin* pathSetterLin;
	PathSetter_Cir* pathSetterCir;

	ePathSetterStatus::e _status; //-1:empty 0:full  1:middle
};

class PathSetter
{
public:
	friend class PathSetterContext;

	PathSetter (int motorNum, int gestureNum);
	~PathSetter(void){};

	int				set(ScriptPathCmd& pCmd);
	void			setVGain(double vGain);
	void			stop();
	bool			isNeedOverlap();

	virtual int		getComponent(IntpComponent& out_cpn, IntpSlice& sourceSlice) = 0;
	eSliceResult::e get(IntpComponent& out_cpn, IntpComponent& out_cpnTool);
	CmdBasicInfo	getInfo(){return _info;}
	CmdIo			getCmdIO(){return _cmdIO;}
	//int				getInsertType();
	MoveData		getMoveData(){return _mvData;}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

protected:

	ePathSetterStatus::e *_status;
	eMovePathType::e _type;

	Interpolator*	_intp;
	Interpolator*	_intp_T; 
	Interpolator*	_intp_S;
	CmdMove*		_path;
	CmdBasicInfo	_info;
	int				_preToolId, _nowToolId;
	

	int		_intpNum;
	int		_motorNum;
	int		_gestureNum;
	//double	_overlapD;

	double _startAxis[MAX_REDUNDANCY];	// 起點位置。可代表 length, A, B, C, theta...
	double _startPose[MAX_REDUNDANCY];
	//double _maxVs	 [MAX_REDUNDANCY];	// 壓速度的結果(新的MaxVel)//deprecated
	//double _maxAccs  [MAX_REDUNDANCY];	// 壓速度的結果(新的MaxAcc)
	//double _maxDecs  [MAX_REDUNDANCY];	// 壓速度的結果(新的MaxDec)
	double _dists	 [MAX_REDUNDANCY];	// 起點到終點距離。	可代表 length, A, B, C, theta...
	double _unitVec	 [MAX_REDUNDANCY];	// 起點到終點的單位向量。(對axis來說,就是單純的正負號)

	// for single interpolator.
	double _distRatio[MAX_REDUNDANCY];  // 每個_dists[i]占 _totalDist 的比例
	double _totalDist;					// 所有_dists[i]的總合
	double _totalTime;					// 要花的時間
	double _maxV;  
	double _maxAcc;
	double _maxDec; 
	
	// for 中途輸出命令
	MoveData _mvData;
	/*double			 _insertRatio;
	int				 _insertType;
	eInsertSource::e _insertSrce;*/
	CmdIo			 _cmdIO;

	Matrix3d _startR;
	Vector3d _uNormalR;
	
	//---- tool
	double* _startToolPose;
	double* _endToolPose;
	double _unitToolVec[3];
	Matrix3d _startToolR, _endToolR, _deltaToolR;
	Vector3d _uNormalToolR;

	// ==== function ======
	virtual int setDistAndVec() = 0;
	
	int modifyPathParameter		(const double* maxV, double maxAcc, double maxDec);
	int setPathToInterpolator	();

	int calculateIntpTime		(double& out_time, double& out_maxV, double dist, double maxV, double maxAcc, double maxDec);
	//int calculateNewParameter	(double& out_newMaxV, double& out_newMaxAcc, double& out_newMaxDec, double dist, double maxAccTime, double maxDecTime, double maxTotalTime);
	int calculateDist			(double& out_dist, double maxV, double maxAcc); // 計算速度從 0 到 maxV 所需的距離

	int calculateDeltaR			(Matrix3d &out_DeltaR, const double startPose[6], const double endPose[6]); // 計算旋轉矩陣的變化量 ΔR
	int calculateDeltaR			(Matrix3d &out_DeltaR, const Matrix3d &startR,    const double endPose[6]); // 計算旋轉矩陣的變化量 ΔR

	int getToolComponent		(IntpComponent& out_cpnTool, IntpSlice& sourceSlice);
	void setToolIntp();
};
class PathSetter_P2P : public PathSetter
{
public:
	PathSetter_P2P(int motorNum, int gestureNum) : PathSetter(motorNum, 0) { 
		_intpNum = motorNum;
		_type    = eMovePathType::P2P;
	};
	~PathSetter_P2P(){};

private:
	
	double _startVs	[MAX_REDUNDANCY];	// 起點速度
	double _endVs	[MAX_REDUNDANCY];	// 終點速度

	int setDistAndVec();
	int getComponent(IntpComponent& out_cpn,  IntpSlice& sourceSlice);

};
class PathSetter_Lin : public PathSetter
{
public:
	PathSetter_Lin(int motorNum, int gestureNum) : PathSetter(motorNum, gestureNum) {
		_intpNum = 2 + 2 + (gestureNum-6); // =endPose + tool + redundant
		_type    = eMovePathType::LINE;
	};
	~PathSetter_Lin(){};

private:
	Matrix3d _endR, _deltaR, _tempR;

	double _startVs	[MAX_REDUNDANCY];	// 起點速度
	double _endVs	[MAX_REDUNDANCY];	// 終點速度

	int setDistAndVec();
	int getComponent(IntpComponent& out_cpn, IntpSlice& sourceSlice);
	
};
class PathSetter_Cir : public PathSetter
{
public:
	PathSetter_Cir(int motorNum, int gestureNum) : PathSetter(motorNum, gestureNum) { 
		_intpNum	= 2 + 2+ (gestureNum-6); // =endPose + tool + redundant
		circleObj	= NULL;
		_type       = eMovePathType::CIRCLE;
	}
	~PathSetter_Cir() {if(circleObj != NULL) delete circleObj;};

private:
	Matrix3d _midR, _endR, _deltaR, _tempR;
	Circle* circleObj;

	int setDistAndVec();
	int getComponent(IntpComponent& out_cpn, IntpSlice& sourceSlice);

	int calUNormalAndTheta(Vector3d& out_uNormalR, double& out_theta, const double* startPose, const double* endPose);
	int decideUNormalR(Vector3d& out_uNormalR, Vector3d& uNormalR1, Vector3d& uNormalR2);
};
