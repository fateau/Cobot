#include "TrajectoryPlanner.h"
#include "Robot.h"
#include "ComponentMerger.h"
#include "PathSetter.h"
#include "Interpolator.h"
using namespace eMovePathType;

TrajectoryPlanner::TrajectoryPlanner(Robot* robot)
{
	_motorNum   = robot->motorNum;
	_gestureNum = robot->gestureNum;
	
	compMerger		= new ComponentMerger(robot->kinIntp);
	comp1			= new IntpComponent();
	comp2			= new IntpComponent();
	compMerged		= new IntpComponent();
	compMergedTool	= new IntpComponent();
	psCtxt1			= new PathSetterContext(_motorNum, _gestureNum); 
	psCtxt2			= new PathSetterContext(_motorNum, _gestureNum); 

	// 事先設定ps1 ps2, 避免它們在正式設定前就被call，造成exception.
	ps1 = psCtxt1->selectStrategy(P2P);  
	ps2 = psCtxt2->selectStrategy(P2P);

	_endT = _toolT = Matrix4d::Identity();
}
TrajectoryPlanner::~TrajectoryPlanner(void)
{
	delete compMerger		;
	delete comp1			;
	delete comp2			;
	delete compMerged		;
	delete compMergedTool	;

	delete psCtxt1;
	delete psCtxt2;
}

void TrajectoryPlanner::reset()
{
	psCtxt1->reset();
	psCtxt2->reset();
	ps1->stop();
	ps2->stop();
}
int  TrajectoryPlanner::emptyPathSetterNum()
{
	int num = 0;
	if(psCtxt1->isEmpty()) num++;
	if(psCtxt2->isEmpty()) num++;

	return num;
}
void TrajectoryPlanner::swap()
{
	PathSetterContext *tempCtxt;
	tempCtxt = psCtxt1;
	psCtxt1 = psCtxt2;
	psCtxt2 = tempCtxt;

	PathSetter *tempPs;
	tempPs = ps1;
	ps1 = ps2;
	ps2 = tempPs;

}
void TrajectoryPlanner::setDefaultScriptToolIndex(int defaultScriptToolIndex)
{
	_scriptToolIndex = defaultScriptToolIndex;
}

int  TrajectoryPlanner::setNewPath(ScriptPathCmd& pCmd)
{
	_scriptToolIndex = pCmd.cmdMove.nowToolId;

	if(psCtxt1->isEmpty()) {
		ps1 = psCtxt1->selectStrategy(pCmd.cmdMove.pathType);	
		return ps1->set(pCmd);
	}


	if(psCtxt2->isEmpty()) {
		ps2 = psCtxt2->selectStrategy(pCmd.cmdMove.pathType);
		return ps2->set(pCmd);
	}

	return 1;
}
void TrajectoryPlanner::setVGain(double vGain)
{
	ps1->setVGain(vGain);
	ps2->setVGain(vGain);
}
eIntpResult::e TrajectoryPlanner::getIntpCmd(IntpCmd& intpCmd)
{

	if(psCtxt1->isEmpty())		return eIntpResult::NOTSET;

	eSliceResult::e res;

	if(ps1->isNeedOverlap() && !psCtxt2->isEmpty() )  // 在兩條連續指令的重疊區
	{
		res = ps1->get(*comp1, *compMergedTool);

		if(res == eSliceResult::FINISH) 
		{
			swap();
			psCtxt2->reset();
			return eIntpResult::NOTSET;
		}
		else
		{
			ps2->get(*comp2, *compMergedTool);
			compMerger->merge(*compMerged, *comp1, *comp2);
		}
		intpCmd.info = ps2->getInfo();
		intpCmd.isContOverlap = true;
	}
	else//不在重疊區
	{
		res = ps1->get(*compMerged, *compMergedTool);

		if(res == eSliceResult::FINISH) 
		{			
			swap();				//交換 路徑容器1 和 路徑容器2 (把新的、尚未執行的路徑放到 容器1)
			psCtxt2->reset();	//清空 路徑容器2，讓容器2可以接受新的路徑。

			return eIntpResult::NOTSET; //根據是否檢查到位，
		}

		intpCmd.info = ps1->getInfo();
		intpCmd.isContOverlap = false;
	}
	
	intpCmd.toolIndex = compMerged->toolId;	

	switch(compMerged->type)
	{
		case eMovePathType::P2P:
			intpCmd.format = eTargetFormat::AXIS;
			for(int i = 0; i < _motorNum; i++) 
				intpCmd.axisDeg[i] = compMerged->nowD[i] + compMerged->startAxis[i];		
			break;

		case eMovePathType::LINE:
		case eMovePathType::CIRCLE:
			intpCmd.format = eTargetFormat::POSE; //此時的pose會把tool去掉
						
		   	for(int i = 0; i < 3; i++) {
				_endT(i,3)  = compMerged->startPose[i] + compMerged->nowD[i]; 
				_toolT(i,3) = compMergedTool->startPose[i];
			}
			_endT.block<3,3>(0,0)  = compMerged->startR * compMerged->nowR;		
			_toolT.block<3,3>(0,0) = compMergedTool->startR;

			// if not constant tool
			if(compMergedTool->isToolChange) { 
				for(int i = 0; i < 3; i++)
					_toolT(i,3) += compMergedTool->nowD[i];
				_toolT.block<3,3>(0,0) *= compMergedTool->nowR;
			}

			// remove tool
			transferT2Pose(intpCmd.Pose , _endT*_toolT.inverse() );

			// update Redundency  note:Pose[]: 0~5:XYZABC;  nowD[]: 0,1,2:XYZ
			for(int i = 6; i < _gestureNum; i++)
				intpCmd.Pose[i] = compMerged->nowD[i-3] + compMerged->startPose[i]; 	

			break;
	}

	return eIntpResult::SUCCESS;
}

CmdIo TrajectoryPlanner::getCmdIo()
{
	return ps1->getCmdIO();
}


