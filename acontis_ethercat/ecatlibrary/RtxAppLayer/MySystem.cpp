#include "MySystem.h"

#include "HelpFunctions.h"
#include "Robot.h"
#include "Script.h"
#include "Motor.h"
#include "IOGroup.h"
#include "IOModule.h"
#include "CmdContext.h"
#include "Kinematics.h"
#include "Shm.h"
#include "EventHandler.h"
#include "ErrorHandler.h"

extern SHMData* shm;

MySystem::MySystem()
{
	slaveNum = 0;
	slaveMotorNum = 0;
	slaveIONum = 0;	
	this->currentBaseId = -1;

	robotNum = 0;
	scrRobotNum = 0;
	linkedMotorNumTotal = 0;
	pbyPDIn  = nullptr;
	pbyPDOut = nullptr;

	for(int i = 0; i < MAX_MOTOR_NUM; i++)
		memset(&axisEcat[i], 0, sizeof(AXIS_ECAT));
	for(int i = 0; i < MAX_IO_NUM; i++)
		memset(&ioEcat[i], 0, sizeof(IO_ECAT));

	for(int i = 0; i< MAX_ROBOT_NUM; i++) 
		robots[i] = NULL;
	ioGroup = new IOGroup();

	initEvents();
	ErrorHandler::Init();
}

MySystem::~MySystem(void)
{
	for(int i = 0; i< robotNum; i++) 
		if(robots[i] != NULL) delete robots[i];
	
	delete ioGroup;
}

/** 偵測 EtherCAT Slave (acontis 版本)
*
* 使用 acontis API 列舉所有配置的 slave，
* 區分 motor slave (有 DS402 PDO) 和 IO slave (無 DS402 PDO)。
*
* param: slaveNums[3] 偵測到的Slave數量。0:總數, 1:非Driver Slave數量 2:Driver Slave數量
* return: 成功為1, 失敗為-1
*/
int MySystem::detectEcatSlave(int slaveNums[3])
{
	slaveNum      = slaveNums[0];
	slaveIONum    = slaveNums[1];
	slaveMotorNum = slaveNums[2];

	printf("slaveNum = %d, ioNum = %d, motorNum = %d\n", slaveNum, slaveIONum, slaveMotorNum);

	// Get process image pointers
	pbyPDIn  = ecatGetProcessImageInputPtr();
	pbyPDOut = ecatGetProcessImageOutputPtr();
	if (!pbyPDIn || !pbyPDOut) {
		printf("ecatGetProcessImageInputPtr/OutputPtr error.\n");
		return -1;
	}

	shm->slaveMotorNum = slaveMotorNum;
	shm->slaveIONum    = slaveIONum;

	return 1;
}

/**  動態配置IO物件。
*
* 檢查所有Ecat偵測到的 IO Slave (=非Driver Slave)，
* 對每個Slave 的 Input/Output Byte數，建立一相應的ioModule物件，加入ioGroup中。
*/
int MySystem::initIO()
{
	for(int i = 0; i < slaveIONum; i++) {
		U32_T inpByteNum = ioEcat[i].dwInpBitLength / 8;
		U32_T outByteNum = ioEcat[i].dwOutBitLength / 8;
		shm->ioModules[i].inpByteNum = inpByteNum;
		shm->ioModules[i].outByteNum = outByteNum;

		ioGroup->addModule(masterId, ioEcat[i].wStationAddress, outByteNum, inpByteNum);
		ioGroup->ioModules[i]->kingIo = &ioEcat[i];

		printf("IO slave[%d], [in= %d, out= %d].\n", i, inpByteNum, outByteNum);
	}
	return 1;
}

/**  動態配置robot物件。*/
int MySystem::initRobots()	
{
	// delete previous data.
	robotNum = 0;
	for(int r = 0; r< MAX_ROBOT_NUM; r++) {
		if(robots[r] != NULL) {
			printf("delete robot[%d]\n", r);
			printf("robot[%d].motorNum = %d\n", r, robots[r]->motorNum);
			delete robots[r];
			robots[r] = NULL;
		}
	}
	
	// create new Robot obj
	RobotDeclareData* rData = shm->robotDeclareTable;

	for(int r = 0; r< MAX_ROBOT_NUM; r++) {
		if(rData[r].motorNum < 0) continue;		
		robots[robotNum] = new Robot(this, r, &rData[r]);
		robots[robotNum]->script->cmdContext = new CmdContext(robots[robotNum], ioGroup);
		robotNum ++;
	}

	// Eigen 第一次運行會比較慢。先跑一次運動學 IK & FK 讓Eigen初始化
	for(int r = 0; r < robotNum; r++) {
		double tempPose[7] = {0, 0, 0, 0, 0, 0, 0};
		double tempAxis[7] = {0, 0, 0, 0, 0, 0, 0};
		int toolId = 0;

		robots[r]->kinMC->FK(tempPose, tempAxis, toolId);
		robots[r]->kinMC->IK(tempAxis, tempPose, toolId);

		robots[r]->kinIntp->FK(tempPose, tempAxis, toolId);
		robots[r]->kinIntp->IK(tempAxis, tempPose, toolId);	
												   
		robots[r]->kinPath->FK(tempPose, tempAxis, toolId);
		robots[r]->kinPath->IK(tempAxis, tempPose, toolId);	
	}

	printf("robotNum = %d \n", robotNum);
	for(int r = 0; r< robotNum; r++) {
		printf("robot[%d] = [%d, %d] \n", r, robots[r]->motorNum, robots[r]->robotType);
	}
	return 1;
}

/**  將Motor物件配置給實體驅動器 (acontis版本)
*
* 根據shm->ecatMappingTable 和事先透過 setupPDO() 建立好的 axisEcat[]，
* 對每個Motor物件指派對應的 AXIS_ECAT 指標。
*/
int MySystem::linkToEcat()	
{
	int r, m;
	
	Motor* motorPtr;
	printf("\n********** slaveMotorNum = %d\n", slaveMotorNum);
	for(int i = 0; i < slaveMotorNum; i++) {
		printf("[%d] robotId = %d, axisId = %d\n", i, shm->ecatMappingTable[i].robotInd, shm->ecatMappingTable[i].axisInd);
		r = shm->ecatMappingTable[i].robotInd;	if(r < 0 || r >= robotNum) continue;
		m = shm->ecatMappingTable[i].axisInd;	if(m < 0 || m >= robots[r]->motorNum) continue;
		
		motorPtr = robots[r]->motors[m];
		motorPtr->kingAxis = &axisEcat[i];
		motorPtr->setEncoderResolution(motorPtr->kingAxis->dwVendorId);

		motorPtr->isLinkedToEcat = true;
		motorPtr->setMode(eRTXMode::CSP); 

		linkedMotorsTotal[linkedMotorNumTotal] = motorPtr;
		linkedMotorNumTotal++;

		printf("robots[%d]->motors[%d] is mapping to slaveAddr %d (vendorId=%u)\n", 
			r, m, i, motorPtr->kingAxis->dwVendorId);
	}

	return 1;
}

void MySystem::processMotorStatus()
{
	for(int m = 0; m < linkedMotorNumTotal; m++) {
		linkedMotorsTotal[m]->processStatus();		
	}
}

void MySystem::StartServoOnOff()
{
	for (int r = 0; r < robotNum; r++)
		robots[r]->StartServoOnOff();
}

void MySystem::isServoOn()
{
	for(int r = 0; r < robotNum; r++) {
		robots[r]->isServoOn();
	}
	return;
}

int MySystem::updateData()
{
	for(int r = 0; r < robotNum; r++) {
		robots[r]->isNeedMove = false;
		if(robots[r]->updateData() < 0) return -1;
	}
	return 1;
}

//-------- Private Function -----------
void MySystem::initEvents()
{
	evtScriptStart = EventHandler::Open("evtScriptStart");
	evtScriptEnd = EventHandler::Open("evtScriptEnd");
	evtGotSlaveInfo = EventHandler::Open("evtGotSlaveInfo");
	evtHMICommand = EventHandler::Open("evtHMICommand");
	evtHomingEnd = EventHandler::Open("evtHomingEnd");

	EventHandler::Reset(evtScriptStart);
	EventHandler::Reset(evtScriptEnd);
	EventHandler::Reset(evtGotSlaveInfo);
	EventHandler::Reset(evtHMICommand);
	EventHandler::Reset(evtHomingEnd);

	char str[20];
	for (int r = 0; r < MAX_ROBOT_NUM; r++) {
		snprintf(str, sizeof(str), "evtScriptSet%d", r);
		evtScriptSet[r] = EventHandler::Open(str);
		EventHandler::Reset(evtScriptSet[r]);

		snprintf(str, sizeof(str), "evtScriptNeed%d", r);
		evtScriptNeed[r] = EventHandler::Open(str);
		EventHandler::Reset(evtScriptNeed[r]);
	}
}
