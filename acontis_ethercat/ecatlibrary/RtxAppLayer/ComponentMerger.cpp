#include "ComponentMerger.h"
#include "Kinematics.h"
#include "Interpolator.h"
using namespace eMovePathType;

ComponentMerger::ComponentMerger(Kinematics* kin)
{
	this->_kin = kin;
}

int ComponentMerger::merge(IntpComponent& out_comp, IntpComponent& comp1, IntpComponent& comp2)
{
	out_comp.toolId = comp1.toolId; //假設 comp1 & 2 有相同tool. (不考慮不相同的情況) 

	if(comp1.type == comp2.type)
		mergeSameType(out_comp, comp1, comp2);
	else
		mergeDiffType(out_comp, comp1, comp2);
	return 1;
}

int ComponentMerger::mergeSameType(IntpComponent& out_comp, IntpComponent& comp1, IntpComponent& comp2)
{
	out_comp.type	 = comp1.type;
	out_comp.intpNum = comp1.intpNum;

	if(comp1.type == P2P)  
	{
		for(int i = 0; i< out_comp.intpNum; i++) {
			out_comp.nowD[i]		= comp1.nowD[i]   + comp2.nowD[i];
			out_comp.startAxis[i]	= comp1.startAxis[i];
		}
	}
	else // for Line & Circle
	{
		// XYZ
		for(int i = 0; i< 3; i++) { 
			out_comp.nowD[i]   = comp1.nowD[i]   + comp2.nowD[i];
			out_comp.startPose[i] = comp1.startPose[i];
		}

		// ABC
		out_comp.nowR = comp1.nowR * comp2.nowR;
		out_comp.startR = comp1.startR;
	}
	return 1;
}
int ComponentMerger::mergeDiffType(IntpComponent& out_comp, IntpComponent& comp1, IntpComponent& comp2)
{
	out_comp.type	 = P2P;
	out_comp.intpNum = 7; //?

	for(int i = 0; i< out_comp.intpNum; i++)
			out_comp.startAxis[i] = comp1.startAxis[i];


	if(comp1.type == P2P)
		mergeP2P_LinCir(out_comp, comp1, comp2); // P2P -> Lin or Cir

	else if(comp2.type == P2P)
		mergeP2P_LinCir(out_comp, comp2, comp1); // Lin or Cir -> P2P
	
	else if(comp1.type == LINE)
		mergeLin_Cir(out_comp, comp1, comp2);	 // Lin -> Cir
	
	else
		mergeLin_Cir(out_comp, comp2, comp1);    // Cir -> Lin
	
	return 1;
}
int calDeltaAxisOfLinCir(double out_DeltaAxis[7], IntpComponent& comp, Kinematics* kin)
{
	// get the pose of LIN/CIR component.
	double tempPose[6];
	double tempAxis[7];

	//xyz
	for(int i = 0; i < 3; i++)
		tempPose[i] = comp.nowD[i] + comp.startPose[i]; 	
	
	//abc
	transferR2Pose(tempPose , comp.startR * comp.nowR);

	// get the axis of LIN/CIR component. 
	kin->IK(tempAxis, tempPose, comp.toolId);

	for(int i = 0; i< 7; i++)
		out_DeltaAxis[i] = tempAxis[i] - comp.startAxis[i];

	return 1;
}
int ComponentMerger::mergeP2P_LinCir(IntpComponent& out_comp, IntpComponent& compP2P, IntpComponent& compLin)
{
	double deltaAxis[7];
	calDeltaAxisOfLinCir(deltaAxis, compLin, _kin);

	// merge P2P & axis of LINE
	for(int i = 0; i< out_comp.intpNum; i++) {
		out_comp.nowD[i]   = compP2P.nowD[i]   + deltaAxis[i]; //tempAxis[i] - compLin.startAxis[i];
	}
	return 1;
}

int ComponentMerger::mergeLin_Cir(IntpComponent& out_comp, IntpComponent& compLin, IntpComponent& compCir)
{
	double deltaAxisLin[7];
	double deltaAxisCir[7];
	calDeltaAxisOfLinCir(deltaAxisLin, compLin, _kin);
	calDeltaAxisOfLinCir(deltaAxisCir, compCir, _kin);

	// merge axis of Lin & axis of Cir
	for(int i = 0; i< out_comp.intpNum; i++) {
		out_comp.nowD[i]   =  deltaAxisLin[i] + deltaAxisCir[i]; //tempAxisLin[i] - compLin.startAxis[i]  + tempAxisCir[i] - compCir.startAxis[i];
	}
	return 1;
}