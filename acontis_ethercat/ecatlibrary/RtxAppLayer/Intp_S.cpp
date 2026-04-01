#include "Interpolator.h"
#include "Def.h"
#include "Debug.h"//1231

inline double calStepDist(double startV, double endV)
{
	return (startV + endV) * SAMPLING_T / 2; //梯形公式
}

void Intp_S::setPath(double totalTime, double totalDist, double targetV, double maxAcc, double maxDec, double vGain)
{	
	_region = eIntpRegion::BEGIN;
	
	Interpolator::setPath(totalTime, totalDist, targetV, maxAcc, maxAcc, vGain);

	_vGain   = vGain; //note: vGain can't be 0.
	_targetV = _targetVOri * _vGain;
	_maxAcc  = _maxAccOri  * _vGain;
	_maxDec  = _maxDecOri  * _vGain;
	//_Jerk	 = _JerkOri    * _vGain; //PMC Modified 11412 //1231
	_Jerk = 1000 * _vGain; //PMC Modified 11412 //1231

	// modify _targetV & _maxAcc
	double a2   = _maxAcc*_maxAcc;
	double a2_j = a2/_Jerk;
	double j2d  = _Jerk * _Jerk * _totalD;

	if(_targetV >= a2_j)
	{
		double condition1 = _targetV * _targetV / _maxAcc + _targetV * _maxAcc / _Jerk;
		double condition2 = 2*_maxAcc*a2_j/_Jerk;

		if(_totalD<condition1 && _totalD>=condition2)
		{
			_targetV=( sqrt( a2*a2 + 4*_maxAcc*j2d) - a2) / (2*_Jerk);
		}
		else if( _totalD<condition2)
		{
			_maxAcc=pow(j2d/2, 1.0/3);
			_targetV=_maxAcc*_maxAcc/_Jerk; //note: since _maxAcc changed, != a2_j
		}
	}
	else
    {
		double condition3 = sqrt( pow(_targetV,3)/_Jerk)*2;
		if(_totalD>=condition3)
		{
			_maxAcc=sqrt(_targetV*_Jerk);
		}
		else
		{
			_maxAcc =pow(j2d/2, 1.0/3);
			_targetV=_maxAcc*_maxAcc/_Jerk; //note: since _maxAcc changed, != a2_j
		}
	}
	
	_targetVOri = _targetV / _vGain;
	_maxAccOri  = _maxAcc  / _vGain;
	_maxDecOri  = _maxDec  / _vGain;

	intp_t_acc.setPath(-1, _targetV, _maxAcc, _Jerk, _Jerk, 1);
	intp_t_dec.setPath(-1, _targetV, _maxAcc, _Jerk, _Jerk, 1);
}
void Intp_S::setStep(const IntpSlice& step)
{
	Interpolator::setStep(step);
}
void Intp_S::setVGain(double newVGain)
{
	if (_region == eIntpRegion::FINAL) //降速區不可改速度
		return;
	
	if(_vGain == newVGain) //若已經在當前速度，則返回。
		return;

	_vGain		= newVGain;
	_targetV	= _targetVOri * _vGain;
	if (_vGain == 0) {
		_maxAcc = _maxAccOri;
		_maxDec = _maxDecOri;
		_Jerk = 1000;
		_targetV = 0;
		_region = eIntpRegion::MIDDLE;
	}
	else {
		_maxAcc = _maxAccOri * _vGain;
		_maxDec = _maxDecOri * _vGain;
		_Jerk = 1000 * _vGain;
	}
}

eSliceResult::e Intp_S::getStep(IntpSlice& out_step)
{
	eSliceResult::e res = Interpolator::getStep(out_step);
	if(res != eSliceResult::CONTINUE)
		return res;

	double nextV, nextA, deltaD;
	if( _region == eIntpRegion::BEGIN)
	{
		isPrint = true;
		intp_t_acc.getStep(slice_t_acc);//得到下一筆 _nowV=nowD _nowA=nowV
		nextV = slice_t_acc.nowD;
		nextA = slice_t_acc.nowV;		
		deltaD = calculateDist(_nowV, nextV, _Jerk, slice_t_acc.region==eIntpRegion::FINAL);
	
		checkMiddleRegion(nextV);
		checkFinalRegion(nextV, deltaD);
	}
	else if(_region == eIntpRegion::MIDDLE) 
	{
		// 判斷加速度方向signA： >0→加速; =0→等速; <0→減速
		if(_nowV != _targetV) 
		{
			int signA = _nowV < _targetV? 1:-1; 
			nextV = _nowV + signA*_maxAcc*SAMPLING_T;

			// 若V超過上限, 令V等於上限
			if(signA*nextV > signA*_targetV) 
				nextV = _targetV; 
			deltaD = calStepDist(_nowV, nextV);
			_nowV = nextV;
		}
		else 
		{
			nextV  = _nowV;
			deltaD = _nowV * SAMPLING_T;
		}
		nextA = 0;
		checkFinalRegion(nextV, deltaD);
	} 	
	else if(_region == eIntpRegion::FINAL) 
	{				
		intp_t_acc.getStep(slice_t_acc);
		nextV = _targetV - slice_t_acc.nowD;
		nextA = -1*slice_t_acc.nowV;

		if(nextV < TINY_VALUE && nextA == 0) {
			deltaD = 1e-3;//給一個很小的值，讓他走到終點
		}
		else
			deltaD = calculateDist(_nowV, nextV, _Jerk, slice_t_acc.region==eIntpRegion::FINAL);

		if((_nowD + deltaD) >= _totalD || _nowRD <= 1e-4)
		{
			nextV = 0;
			deltaD =_nowRD;
		}
		
	}

	_nowD  += deltaD;
	_nowRD -= deltaD;
	_nowV   = nextV;
	_nowA   = nextA;

	out_step.nowD	= _nowD;
	out_step.nowV	= _nowV;
	out_step.nowA   = _nowA;
	out_step.region = _region;

	return eSliceResult::SUCCESS;
}

double Intp_S::calculateDist(double startV, double endV, double maxJ,bool mode)
{
	if(maxJ == 0) 
		return (startV + endV) * SAMPLING_T / 2; 

	if(startV == endV) // 等速運動
		return (startV + endV) * SAMPLING_T / 2;

	double	sign1,sign2;
	double	Amax, J;
	double	delta_v, delta_x1, delta_x2, t, v_t;

	delta_v = endV - startV;
	sign1   = delta_v / fabs(delta_v); // 加速度的 ± 號
	
	sign2 = mode !=true ? 1:-1 ;
	Amax  = mode !=true ? 0:_maxAcc;

	Amax = sign1 * Amax;
	J    = sign2 * _Jerk;

	t        = ( 2*(delta_v - Amax*SAMPLING_T)/J - pow(SAMPLING_T,2) ) / (2*SAMPLING_T);
	delta_x1 = 0.5*Amax*( 2*t*SAMPLING_T + pow(SAMPLING_T,2) ) + J/6*( 3*pow(t,2)*SAMPLING_T + 3*t*pow(SAMPLING_T,2) + pow(SAMPLING_T,3) );
	v_t      = Amax*t + 0.5*J*pow(t,2);
	delta_x2 = delta_x1 - v_t*SAMPLING_T;
	return startV*SAMPLING_T + delta_x2;
}
void Intp_S::checkMiddleRegion(double nextV)
{
	if(_nowV == nextV) // 若變成等速，則進入中間區域
		_region = eIntpRegion::MIDDLE;
}
void Intp_S::checkFinalRegion(double nextV, double deltaD)
{
	// 計算降速所需的距離 finalNeedD
	double finalNeedD = (nextV*nextV/2/_maxAcc)+(nextV*_maxAcc/2/_Jerk);
	if(_nowRD > finalNeedD) return;

	// 若剩餘距離不夠，則進入降速區域
	_region = eIntpRegion::FINAL;		
	double D = _nowRD - deltaD;

	_Jerk = _maxAcc * _maxAcc * nextV / (2*_maxAcc * D - nextV*nextV); //依照剩餘距離重新計算Jerk	
	intp_t_acc.setPath(-1, nextV, _maxAcc, _Jerk, _Jerk, 1);	
}
