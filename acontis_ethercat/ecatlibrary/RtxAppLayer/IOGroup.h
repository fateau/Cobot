#pragma once
#include "Def.h"


class IOModule;
class IOGroup
{
public:
	IOGroup();
	~IOGroup(void);

	IOModule* ioModules[MAX_IO_NUM];
	int	ioModuleNum;

	int addModule(U16_T masterID, U16_T slaveAddr, int OutNum, int InNum);
	void updateData();
	void setEcatOutput();
};

