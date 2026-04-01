#include "Kinematics.h"

Kin_Ext::Kin_Ext(int rInd, int axisNum) : KinSerial(rInd)
{
	_axisNum = axisNum;
	_gestureNum = 6;
}

// ==================== kinematics kernel function ======================

eKin::e Kin_Ext::IK()
{
	// DO nothing.
	return eKin::EXTENDED_SINGULAR;
}

