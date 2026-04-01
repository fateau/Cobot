#pragma once
#include <stdio.h>//for printf.
#include <math.h>
#include "Def.h"

namespace eSliceResult
{
	enum e{
		FAIL = -1,	//插補切片失敗
		SUCCESS,	//插補切片成功
		FINISH,		//此區段切片結束
		CONTINUE	//尚未完成,繼續計算
	};
};
namespace eIntpRegion
{
	enum e{
		BEGIN,		//加速階段
		MIDDLE,		//等速階段
		FINAL		//降速階段 (末端)
	};
};
struct IntpSlice //回傳給user的資料。可用於回溯時的插補資料復元。
{
	double	deltaD;
	double	nowD, nowV, nowA;// D:已經走的距離.(絕對值)
	double	maxDec;			 // 在進入"線段末端速度調整區間"時，maxDec可能會改變
	eIntpRegion::e	region;	 // 是否進入"線段末端速度調整區間"
};
struct IntpComponent 
{
	eMovePathType::e type;
	int		 toolId;
	int		 intpNum;
	bool	 isToolChange;
	double	 nowD		[MAX_MOTOR_PER_ROBOT];
	double	 startAxis	[MAX_MOTOR_PER_ROBOT];
	double	 startPose	[MAX_REDUNDANCY];
	Matrix3d nowR;
	Matrix3d startR;
};


class Interpolator
{
public:
	Interpolator(void);
	~Interpolator(void);

	virtual eSliceResult::e getStep(IntpSlice& out_step);

	virtual void setPath(double totalTime, double totalDist, double targetV, double maxAcc, double maxDec, double vGain);
	virtual void setStep(const IntpSlice& step);
	virtual void setVGain(double vGain)=0;
	bool isOverDistPercent(const double percent);
	bool isOverTimePercent(const double percent);
	void stop();

protected:
	double _nowD, _nowRD, _totalD, _totalT;		// nowD:已經走的距離 ; RD: 剩餘距離.  D + RD = total dist.
	double _nowV, _targetV, _vGain;				// targetV(單位: mm/s) = maxV(劇本) * VGain (Bruce)
	double _nowA, _maxAcc, _maxDec, _Jerk;		// 單位: mm/s^2
	double _targetVOri, _maxAccOri, _maxDecOri, _JerkOri; // 沒有乘過 vGain的原始值
	int	   _nowT;								// 經過的時間(ms)
	eIntpRegion::e _region;
};

class Intp_T:public Interpolator
{
public:
	Intp_T(){};
	~Intp_T(){};

	void setPath(double totalTime, double totalDist, double targetV, double maxAcc, double maxDec, double vGain);
	void setStep(const IntpSlice& step);
	void setVGain(double vGain);
	eSliceResult::e getStep(IntpSlice& out_step);

	eIntpRegion::e getRegion(){return _region;}
	void setMaxDec(double maxDec) {_maxDec = maxDec;}

private:
	//double calStepDist(double startV, double endV);		//從startV 到 endV (經過1單位時間)走的距離 
	//double calFinalNeedD(double startV, double dec);	//從startV 降速到 0 所需的距離 (已知降速度)
	//double calFinalDec(double startV, double dist);		//從startV 降速到 0 所需的降速度 (已知距離)
};
class Intp_S:public Interpolator
{
public:
	Intp_S(){};
	~Intp_S(){};
	Intp_T intp_t_acc;
	IntpSlice slice_t_acc;
    Intp_T intp_t_dec;
	IntpSlice slice_t_dec;
	bool _isFinalRegion;

	void setPath(double totalTime, double totalDist, double targetV, double maxAcc, double maxDec, double vGain);
	void setStep(const IntpSlice& step);
	void setVGain(double vGain);
	eSliceResult::e getStep(IntpSlice& out_step);

private:
	double calculateDist(double startV, double endV, double maxAcc,bool mode);
	void   checkMiddleRegion(double nextV);
	void   checkFinalRegion(double nextV, double deltaD);
	bool   isPrint;//temp. for debugg.
};