#pragma once
#include "Def.h"
#include <math.h>

namespace eCalTimeResult
{
	enum e{
		FAIL = -1,
		SUCCESS = 1,
		TINT_DIST,
		VARY_ENDV, // only use for startV & endV ≠ 0
	};
};

struct IntpComponent;
class PathSetterContext;
class PathSetter;
class ComponentMerger;
class Robot;

class TrajectoryPlanner
{

public:
	TrajectoryPlanner(Robot* robot);
	~TrajectoryPlanner(void);

	// ====== function ======
	void setDefaultScriptToolIndex(int defaultScriptToolIndex);
	int  setNewPath(ScriptPathCmd& pCmd);
	void setVGain(double vGain);

	eIntpResult::e getIntpCmd(IntpCmd& intpCmd);
	CmdIo		getCmdIo();

	void reset();
	int emptyPathSetterNum();

	

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;		// to allow fixed array as class member.

private:
	PathSetterContext *psCtxt1, *psCtxt2;
	PathSetter		  *ps1, *ps2;
	ComponentMerger	  *compMerger;

	IntpComponent *compMerged, *compMergedTool;
	IntpComponent *comp1;
	IntpComponent *comp2;

	int _motorNum;
	int _gestureNum;
	int _scriptToolIndex;					
									   
	Matrix3d _deltaR, _tempR;
	Matrix4d _endT, _toolT;

	void swap();
};