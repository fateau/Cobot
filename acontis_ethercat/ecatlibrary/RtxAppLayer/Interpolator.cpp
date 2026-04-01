#include "Interpolator.h"
#include "Shm.h" 
extern SHMData* shm; 

Interpolator::Interpolator(void){}
Interpolator::~Interpolator(void){}

void Interpolator::setPath(double totalTime, double totalDist, double targetV, double maxAcc, double maxDec, double vGain)
{
	this->_totalT= totalTime*1000; //sec -> ms
	this->_totalD= totalDist;
	this->_nowRD = totalDist;
	this->_nowD  = 0;
	this->_nowV  = 0;	
	this->_nowT	 = 0;
	this->_targetVOri = targetV;
	this->_maxAccOri = maxAcc;
	this->_maxDecOri = maxDec;
	this->_JerkOri   = shm->Jerk;
	
}

eSliceResult::e Interpolator::getStep(IntpSlice& out_step)
{
	if(_nowRD < TINY_VALUE && _vGain != 0) // 剩餘插補距離太短, 不做插補
	{
		_nowRD = 0;
		_nowV  = 0;
		_nowA  = 0;
		out_step.nowD	= _totalD;
		out_step.nowV	= _nowV;
		out_step.nowA   = _nowA;
		out_step.region = eIntpRegion::FINAL;
		return eSliceResult::FINISH;
	}

	if(_nowV == 0 && _targetV == 0) 
	{
		out_step.nowD	= _nowD;
		out_step.nowV	= _nowV;
		out_step.nowA   = _nowA;
		out_step.maxDec	= _maxDec;
		out_step.region = _region;


		//case 1: 區段正常結束
		if(_vGain != 0 || _nowRD==0)
			return eSliceResult::FINISH;		

		//case 2: 劇本緩停
		if(shm->stopScript == true)		
			return eSliceResult::FINISH;

		//case 3: 劇本暫停 (當user將速度拉霸設為0，將一直回傳同一個位置)
		return eSliceResult::SUCCESS;
		
	}
	_nowT ++;
	return eSliceResult::CONTINUE;
}
bool Interpolator::isOverDistPercent(const double percent)
{
	if(percent <= 0) return false;

	if(_totalD <= 0) return true;
	if(_nowRD/_totalD <= percent/100) // percent: 剩餘距離的百分比
		return true;
	else
		return false;
}
bool Interpolator::isOverTimePercent(const double percent)
{
	if(percent <= 0) return false;

	if(_totalT <= 0) return true;
	if(_nowT/_totalT >= 1-percent/100) // percent: 剩餘時間的百分比
		return true;
	else
		return false;
}
void Interpolator::stop()
{
	//直接結束插補
	_nowRD = _nowV = _targetV = 0;
}
// 回溯時使用
void Interpolator::setStep(const IntpSlice& step)
{
	_nowD   = step.nowD;
	_nowV   = step.nowV;
	_nowA   = step.nowA; // S 型速度規劃使用
	_nowRD -= _nowD; 
	_region  = step.region;

	if(step.region == eIntpRegion::FINAL)
	{ _maxDec = step.maxDec; }
}
