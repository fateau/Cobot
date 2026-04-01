#pragma once

#include "Def.h"
//#include "Interpolator.h"
//#include "Kinematics.h"
class Kinematics;
class IntpComponent;

class ComponentMerger
{
public:
	ComponentMerger(Kinematics* kin);
	~ComponentMerger(void){};

	int merge(IntpComponent& out_comp, IntpComponent& comp1, IntpComponent& comp2);

private:
	Kinematics* _kin;

	int mergeSameType(IntpComponent& out_comp, IntpComponent& comp1, IntpComponent& comp2);
	int mergeDiffType(IntpComponent& out_comp, IntpComponent& comp1, IntpComponent& comp2);

	int mergeP2P_LinCir(IntpComponent& out_comp, IntpComponent& compP2P, IntpComponent& compLin);
	int mergeLin_Cir(IntpComponent& out_comp, IntpComponent& compLin, IntpComponent& compCir);
};

