#include "PathSetter.h"
#include "HelpFunctions.h"
#include "MathTool_Robot.h"
#include "Shm.h"

extern SHMData* shm;
PathSetterContext::PathSetterContext(int motorNum, int gestureNum)
{
	pathSetterP2P  = new PathSetter_P2P(motorNum, gestureNum); 
	pathSetterLin  = new PathSetter_Lin(motorNum, gestureNum);
	pathSetterCir  = new PathSetter_Cir(motorNum, gestureNum);

	_status = EMPTY;

	pathSetterP2P->_status = &_status;
	pathSetterLin->_status = &_status;
	pathSetterCir->_status = &_status;
}
PathSetterContext::~PathSetterContext() 
{
	delete pathSetterP2P;
	delete pathSetterLin;
	delete pathSetterCir;
}
PathSetter* PathSetterContext::selectStrategy(eMovePathType::e type)
{
	using namespace eMovePathType;

	if(type == P2P)		return pathSetterP2P;
	if(type == JOINT)	return pathSetterP2P;
	if(type == LINE)	return pathSetterLin;
	if(type == CIRCLE)	return pathSetterCir;

	return NULL;
}
bool PathSetterContext::isEmpty() 
{
	return _status == EMPTY? true:false;
}
void PathSetterContext::reset()
{
	_status = EMPTY;	
}

//==================================================
PathSetter::PathSetter(int motorNum, int gestureNum)
{
	_intp_T       = new Intp_T();
	_intp_S       =	new Intp_S();
	_intp         =	_intp_S; //

	_motorNum	  = motorNum;
	_gestureNum	  = gestureNum;
	//_overlapD	  = -1;
}


int PathSetter::set(ScriptPathCmd& pCmd)
{
	*_status	= FULL;
    //_intp		= shm->onScurve==1? _intp_S:_intp_T; //PMC Modified 11412 //1231
	_intp = _intp_S; //PMC Modified 11412 //1231
	_path		= &(pCmd.cmdMove); 

	//注意: _path的值到下一輪會改變。以後要用的值(如下)要另外存。
	_info		= pCmd.info;
	_cmdIO		= pCmd.cmdIo;
	_preToolId  = _path->preToolId;
	_nowToolId  = _path->nowToolId;
	_mvData		= _path->mvData;


	setDistAndVec();
	modifyPathParameter(_path->maxVs, _path->mvData.maxA, _path->mvData.maxA);	// 壓速度
	setPathToInterpolator();

	return 1;
}
int PathSetter::modifyPathParameter(const double* maxVs, double maxAcc, double maxDec)
{
	double totalTimes[MAX_MOTOR_PER_ROBOT] = {0};
	double   accTimes[MAX_MOTOR_PER_ROBOT] = {0};
	double   decTimes[MAX_MOTOR_PER_ROBOT] = {0};
	double  tempMaxVs[MAX_MOTOR_PER_ROBOT] = {0};
	double maxAccTime   = -1;
	double maxDecTime   = -1;
	_totalTime = -1;

	// 計算個別所需的時間，並找出最長時間(maxTotalTime & maxAccTime & maxDecTime)。
	for(int i = 0; i < _intpNum; i++) 
	{
		// 計算各個插補的 t_total 及可達的 maxV
		// TO DO : calculateIntpTime(.) return 接到 -1 時 , 劇本砍掉
		calculateIntpTime(totalTimes[i] , tempMaxVs[i] , _dists[i] , maxVs[i] , maxAcc , maxDec);		
		
		accTimes[i] = tempMaxVs[i] / maxAcc;	// 依照可達的 maxV 計算各別的 t_acc
		decTimes[i] = tempMaxVs[i] / maxDec;	// 依照可達的 maxV 計算各別的 t_dec

		if(_totalTime < totalTimes[i])	    _totalTime = totalTimes[i];	// 找出時間最長的 t_total
		if(maxAccTime < accTimes[i])		maxAccTime = accTimes[i];		// 找出時間最長的 t_acc
		if(maxDecTime < decTimes[i])		maxDecTime = decTimes[i];		// 找出時間最長的 t_dec
	}

	_totalDist = 0;
	if(_totalTime <= TINY_VALUE || maxAccTime <= TINY_VALUE || maxDecTime <= TINY_VALUE)  
		return 1; //會設定一個 _distTotal為 0 的 插補器


	// deprecated
	// 根據最長時間(maxTotalTime & maxAccTime & maxDecTime)，重新計算各個插補的 maxV , maxAcc , maxDec (壓速度 & 最大速度)
	//for(int i = 0; i < _intpNum; i++) 
	//	calculateNewParameter(out_maxVs[i] , out_maxAccs[i] , out_maxDecs[i], _dists[i] , maxAccTime , maxDecTime , _totalTime);

	// 計算各個_dist[i] 的總和 和各自所占比例 	
	for(int i = 0; i < _intpNum; i++)
		_totalDist += _dists[i];
	
	for(int i = 0; i < _intpNum; i++)
		_distRatio[i] = _dists[i] / _totalDist;

	_maxV   = _totalDist / (_totalTime - maxAccTime/2 - maxDecTime/2);
	_maxAcc = _maxV / maxAccTime;
	_maxDec = _maxV / maxDecTime;

	return 1;
}
int PathSetter::setPathToInterpolator()
{
	_intp->setPath(_totalTime, _totalDist, _maxV, _maxAcc, _maxDec, shm->vGain); 
	return 1;
}
void PathSetter::setVGain(double vGain)
{
	_intp->setVGain(vGain);
}

eSliceResult::e PathSetter::get(IntpComponent& out_cpn, IntpComponent& out_cpnTool)
{
	eSliceResult::e res;
	IntpSlice slice;

	res = _intp->getStep(slice);
	out_cpn.type = _type;
	out_cpn.intpNum = _intpNum;

	getComponent(out_cpn, slice);
	
	// Tool 插補
	if( _type != eMovePathType::P2P)
		getToolComponent(out_cpnTool, slice);

	out_cpn.toolId = _nowToolId; 

	return res;
}
bool PathSetter::isNeedOverlap()
{
	return _intp->isOverDistPercent(_mvData.contD);
}

void PathSetter::stop()
{
	_intp->stop();
}
int PathSetter_P2P::setDistAndVec()
{
	CopyArray(_startAxis, _path->startAxis, _motorNum);

	for(int i = 0; i < _motorNum; i++) {
		_dists[i]	= fabs(_path->endAxis[i] - _path->startAxis[i]);
		_unitVec[i] = sign(_path->endAxis[i] - _path->startAxis[i]);
		if(_dists[i] < TINY_VALUE) _unitVec[i] = 0; //temp. 為了讓getIntpCmd 的intpCmd.nowDs[i] * _unitVec[i] = 0;//有空時，測試拿掉這行是否ok
	}
	return 1;
}
int PathSetter_Lin::setDistAndVec()
{	
	CopyArray(_startAxis, _path->startAxis, _motorNum);
	CopyArray(_startPose, _path->startPose, _gestureNum);

	// 計算起點的旋轉矩陣 (_startR)
	transferPose2R(_startR, _path->startPose);

	// 1. XYZ 
	double fullPathVec[3];
	for(int i = 0; i < 3; i++) fullPathVec[i] = _path->endPose[i] - _path->startPose[i];
	normalizeVector(_unitVec, _dists[0], fullPathVec); //計算 dists[0] & vecXYZ

	// 2. ABC
	calculateDeltaR(_deltaR, _startR, _path->endPose); // 計算ΔR
	getAxisAndAngle_R(_uNormalR, _dists[1], _deltaR);  // 依照 ΔR 計算對應的等效轉軸(_uNormalR)及轉角(_dists[1])

	// 3. Tool XYZ & ABC
	setToolIntp();

	return 1;
}
int PathSetter_Cir::setDistAndVec()
{
	if(circleObj != NULL) delete circleObj;	
	circleObj = new Circle(_path->circle);

	CopyArray(_startAxis, _path->startAxis, _motorNum);
	CopyArray(_startPose, _path->startPose, _gestureNum);

	// 計算起點的旋轉矩陣 (_startR)
	transferPose2R(_startR, _path->startPose);


	// 1. xyz (圓弧半徑 theta)
	_dists[0] = _path->circle.th_Final;
	if(_dists[0] < TINY_VALUE) return -1;


	// 2. ABC (計算 start-end 的 ΔR 的等校轉軸及轉角。不考慮 mid）
	calUNormalAndTheta(_uNormalR, _dists[1], _path->startPose, _path->endPose); 
	if(_dists[1] < 0.1 ) _dists[1] = 0;

	// 4. Tool XYZ & ABC
	setToolIntp();

	return 1;
}

int PathSetter_P2P::getComponent(IntpComponent& out_cpn, IntpSlice& sourceSlice)
{
	for(int i = 0; i < _motorNum; i++) {
		out_cpn.nowD[i]	  = sourceSlice.nowD * _distRatio[i] * _unitVec[i];
		out_cpn.startAxis[i] = _startAxis[i];
	}
	return 1;
}
int PathSetter_Lin::getComponent(IntpComponent& out_cpn, IntpSlice& sourceSlice)
{

	CopyArray(out_cpn.startAxis, _startAxis, _motorNum);	

	// 1.compute XYZ.
	double sliceL;
	sliceL	  = sourceSlice.nowD * _distRatio[0];
	for(int i = 0; i < 3; i++) {
		out_cpn.nowD[i] = sliceL * _unitVec[i];
		out_cpn.startPose[i] = _startPose[i];
	}

	// 2.compute R.
	double sliceR;
	sliceR	  = sourceSlice.nowD * _distRatio[1];
	AxisAngleMethod_R(out_cpn.nowR, _uNormalR, sliceR);
	out_cpn.startR = _startR;

	return 1;
}
int PathSetter_Cir::getComponent(IntpComponent& out_cpn, IntpSlice& sourceSlice)
{
	CopyArray(out_cpn.startAxis, _startAxis, _motorNum);

	// 1.compute XYZ.
	double sliceTh;
	sliceTh	  = sourceSlice.nowD * _distRatio[0];
	circleObj->theta2XYZ(out_cpn.nowD, sliceTh);

	for(int i = 0; i < 3; i++) {
		out_cpn.nowD[i]		-= _startPose[i];
		out_cpn.startPose[i] = _startPose[i];
	}

	// 2.compute R.
	double sliceR;
	sliceR	  = sourceSlice.nowD * _distRatio[1];
	AxisAngleMethod_R(out_cpn.nowR, _uNormalR, sliceR);
	out_cpn.startR = _startR;

	return 1;
}

void PathSetter::setToolIntp()
{
	_startToolPose = shm->tools[_preToolId].Pose;
	transferPose2R(_startToolR, _startToolPose);
	
	if(_preToolId == _nowToolId) 
		_dists[2] = _dists[3] = 0; //不用插補Tool
	else 
	{
		//要插補Tool
		_endToolPose   = shm->tools[_nowToolId].Pose;
		double fullPathVec[3];
		for(int i = 0; i < 3; i++) 
			fullPathVec[i] = _endToolPose[i] - _startToolPose[i];
		normalizeVector(_unitToolVec, _dists[2], fullPathVec);

		transferPose2R(_startToolR, _startToolPose);
		calculateDeltaR(_deltaToolR, _startToolR, _endToolPose);
		getAxisAndAngle_R(_uNormalToolR, _dists[3], _deltaToolR);
	}
}
int PathSetter::getToolComponent(IntpComponent& out_cpnTool, IntpSlice& sourceSlice)
{
	// compute Tool XYZ & R
	out_cpnTool.isToolChange = (_preToolId != _nowToolId);

	out_cpnTool.startR = _startToolR;
	CopyArray(out_cpnTool.startPose, _startToolPose, 3);

	if(out_cpnTool.isToolChange) //要插補Tool
	{
		double sliceL	  = sourceSlice.nowD * _distRatio[2];
		for(int i = 0; i < 3; i++) {
			out_cpnTool.nowD[i] = sliceL * _unitToolVec[i];
			out_cpnTool.startPose[i] = _startToolPose[i];
		}
		double sliceR	  = sourceSlice.nowD * _distRatio[3];
		AxisAngleMethod_R(out_cpnTool.nowR, _uNormalToolR, sliceR);		
	}
	return 1;
}

int PathSetter::calculateIntpTime(double& out_time, double& out_maxV, double dist, double maxV, double maxAcc, double maxDec)
{
	if(maxV <= 0 || maxAcc <= 0 || maxDec <= 0)
	{
		out_time = 0;
		out_maxV = 0;
		return -1; // stop script
	}

	// s = vo*t + 0.5 * a * t^2
	// vo(初速)為零, 所以 s = 0.5 * a * t^2
	// 因此, 當 dist (插補距離) < ( 0.5 * acc * sampling_T^2 + 0.5 * dec * sampling_T^2 ) 時
	// 即代表距離小到無法進行插補規劃
	double tiny_dist = 0.5 * ( maxAcc + maxDec ) * pow(SAMPLING_T , 2);
	if( dist < tiny_dist ) 
	{
		out_time = 0;
		out_maxV = 0;
		return 1;
	}


	double d_acc, d_v, d_dec;

	calculateDist(d_acc, maxV, maxAcc); // 計算速度從    0 到 maxV 所需的距離 (d_acc)
	calculateDist(d_dec, maxV, maxDec); // 計算速度從 maxV 到    0 所需的距離 (d_dec)

	if(dist >= (d_acc + d_dec) ) // 速度可達到 maxV, 計算等速運動區間的行走距離
		d_v  = dist - d_acc - d_dec; 
	else // 速度無法達到 maxV → 重新計算可達到的 maxV' (以 default 的 maxAcc & maxDec 進行計算)
	{
		d_v = 0;
		maxV = sqrt( (2*dist*maxAcc*maxDec) / (maxAcc + maxDec) ); // 計算可達到的 maxV'
	}

	out_maxV = maxV;
	out_time = maxV/maxAcc + d_v/maxV + maxV/maxDec; // = 加速+等速+減速

	return 1;
}
//int PathSetter::calculateNewParameter(double& out_newMaxV, double& out_newMaxAcc, double& out_newMaxDec, double dist, double maxAccTime, double maxDecTime, double maxTotalTime)
//{
//	if(maxTotalTime <= 0 || maxAccTime <= 0 || maxDecTime <= 0 || dist <= 0)
//	{
//		out_newMaxV   = 0;
//		out_newMaxAcc = 0;
//		out_newMaxDec = 0;
//		return -1;
//	}
//
//	double t_acc	= maxAccTime;
//	double t_dec	= maxDecTime;
//	double t_total	= maxTotalTime;
//	double t_v		= t_total - t_acc - t_dec;
//
//	out_newMaxV		= dist / ( t_acc/2 + t_v + t_dec/2 );
//	out_newMaxAcc	= out_newMaxV / t_acc;
//	out_newMaxDec	= out_newMaxV / t_dec;
//
//	return 1;
//}
int PathSetter::calculateDist(double& out_dist, double maxV, double maxAcc)
{
	if(maxAcc == 0) return -1;

	if(maxV == 0)	out_dist = 0; 
	else			out_dist = pow(maxV,2) / (2 * maxAcc);

	return 1;
}

void printfR(Matrix3d &R)
{
	printf("\n");
	for(int i = 0; i<3; i++){
		for(int j = 0; j<3; j++)
			printf("%lf\t",R(i,j));

		printf("\n");
	}
	printf("\n");
}
int PathSetter::calculateDeltaR(Matrix3d &out_DeltaR, const double startPose[6], const double endPose[6])
{
	Matrix3d startR, endR;
	transferPose2R( startR, startPose);
	transferPose2R( endR,   endPose);
	out_DeltaR = startR.inverse() * endR;
	return 1;
}
int PathSetter::calculateDeltaR(Matrix3d &out_DeltaR, const Matrix3d &startR,    const double endPose[6])
{
	Matrix3d endR;
	transferPose2R( endR,   endPose);
	out_DeltaR = startR.inverse() * endR;
	return 1;
}

int PathSetter_Cir::calUNormalAndTheta(Vector3d& out_uNormalR, double& out_theta, const double* startPose, const double* endPose)
{
	// 計算 startPose 及 endPose 間的 等效轉軸 out_uNormalR 及轉角 theta
	calculateDeltaR(_deltaR, startPose, endPose);
	getAxisAndAngle_R(out_uNormalR, out_theta, _deltaR); // 依照 ΔR 計算對應的等效轉軸(temp_uNormalR1)及轉角(theta1)
	return 1;
}
int PathSetter_Cir::decideUNormalR(Vector3d& out_uNormalR, Vector3d& uNormalR1, Vector3d& uNormalR2)
{
	// TODO
	double norm = (uNormalR1 + uNormalR2).norm();

	if ( norm == 0 || norm > TINY_VALUE) // ΔR 沒轉。uNormalR={0,0,0}  或  兩段不為反向
	{
		out_uNormalR = uNormalR1;  
		return 1;
	}
	
	if ( norm < TINY_VALUE ) // 兩段為反向(尚未處理)
	{
		printf("the vectors of rotate axis are opposing!!"); 
		return -1;
	}
	return 1;
}