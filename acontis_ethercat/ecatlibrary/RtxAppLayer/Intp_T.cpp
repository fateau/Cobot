#include "Interpolator.h"
#include "Def.h"

inline double calStepDist(double startV, double endV)
{
	return (startV + endV) * SAMPLING_T / 2; //梯形公式
}
inline double calFinalNeedD(double startV, double dec)
{ 
	return startV * startV / dec / 2 ; 
}

inline double calFinalDec(double startV, double dist )
{
	return  startV * startV / dist / 2 ;
}

void Intp_T::setPath(double totalTime, double totalDist, double targetV, double maxAcc, double maxDec, double vGain)
{	
	_region = eIntpRegion::MIDDLE;
	Interpolator::setPath(totalTime, totalDist, targetV, maxAcc, maxDec, vGain);

	_vGain   = vGain; //note: vGain can't be 0.
	_targetV = _targetVOri * _vGain;
	_maxAcc  = _maxAccOri  * _vGain;
	_maxDec  = _maxDecOri  * _vGain;
}
void Intp_T::setStep(const IntpSlice& step)
{
	Interpolator::setStep(step);
}
void Intp_T::setVGain(double vGain)
{
	if(_region == eIntpRegion::FINAL) //降速區不可改速度
		return;
	
	_vGain		= vGain;
	_targetV	= _targetVOri * _vGain;

	if(_vGain == 0){
		_maxAcc  = _maxAccOri;
		_maxDec  = _maxDecOri;
	}
	else {
		_maxAcc  = _maxAccOri * _vGain;
		_maxDec  = _maxDecOri * _vGain;
	}
}
/* note:
1. 梯形的startV, endV 必為0
2. dist, targetV, acc, dec 必為正
3. 進入降速區後，targetV=0, 不能改vGain
*/
eSliceResult::e Intp_T::getStep(IntpSlice& out_step)
{
	eSliceResult::e res = Interpolator::getStep(out_step);
	if(res != eSliceResult::CONTINUE)
		return res;

	double nextV;	
	double deltaD; // 下一步要增加的距離。 nextD = nowD + deltaD;

	if(_region == eIntpRegion::MIDDLE) // 距離尚可進行加減速調變 
	{
		//double tempD;	
		
		// 判斷加速度方向signA： >0→加速; =0→等速; <0→減速
		if(_nowV != _targetV) 
		{
			int signA = _nowV < _targetV? 1:-1; 
			nextV = _nowV + signA*_maxAcc*SAMPLING_T;

			// 若V超過上限, 令V等於上限
			if(signA*nextV > signA*_targetV) 
				nextV = _targetV; 
			deltaD = calStepDist(_nowV, nextV);
		}
		else 
		{
			nextV  = _nowV;
			deltaD = _nowV * SAMPLING_T;
		}
		
		if(_nowRD < calFinalNeedD( nextV, _maxDec) ) // 若剩餘距離不夠，則進入降速區
		{
			_targetV = 0;
			_region = eIntpRegion::FINAL;			
			_maxDec = calFinalDec(nextV, _nowRD-deltaD);// 依照剩餘距離重新計算降速度
		}
	} 
	
	else if( _region == eIntpRegion::FINAL) // 進入「線段末端速度調整區間」
	{
		nextV = _nowV - _maxDec*SAMPLING_T;
			
		if(nextV < 0 ) // 當 nextV < 0, 直接把剩餘的距離(屑屑)送出
		{ 
			nextV = 0; 
			deltaD = _nowRD;		
			//_region = eIntpRegion::MIDDLE; //no need this line?
		}
		else
			deltaD = calStepDist(_nowV, nextV);	
	}

	_nowD  += deltaD;
	_nowRD -= deltaD;
	_nowV   = nextV;

	out_step.nowD	= _nowD;
	out_step.nowV	= _nowV;
	out_step.maxDec	= _maxDec;
	out_step.region = _region;

	return eSliceResult::SUCCESS;
}

