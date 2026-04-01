#include "IOGroup.h"
#include "IOModule.h"
#include <stdio.h>


IOGroup::IOGroup()
{
	ioModuleNum = 0;
}


IOGroup::~IOGroup(void)
{
	for(int i = 0; i<ioModuleNum; i++) {
		delete ioModules[i];
	}
}

int IOGroup::addModule(U16_T masterID, U16_T slaveAddr, int outNum, int inNum)
{
	if(ioModuleNum >= MAX_IO_NUM) return -1;
	ioModules[ioModuleNum] = new IOModule(ioModuleNum, masterID, slaveAddr, outNum, inNum);
	ioModuleNum++;	
	return 1;
}

void IOGroup::updateData()
{
	for(int m = 0; m<ioModuleNum; m++) {

		ioModules[m]->getEcatInput();
		ioModules[m]->getEcatOutput();
	}
	
	return;
}
void IOGroup::setEcatOutput()
{
	// 輸出至Ecat
	for(int m = 0; m<ioModuleNum; m++) {	
		ioModules[m]->setEcatOutput();
	}
	return;
}
