#include "MySystem.h"

#include "HelpFunctions.h"
#include "Robot.h"
#include "Script.h"
#include "Motor.h"
#include "IOGroup.h"
#include "IOModule.h"
#include "CmdContext.h"
#include "Kinematics.h" //see if can remove.
#include "SHM.h"
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

/** 連線至Ecat - KingStarIO版本
* 記錄偵測到的Slave數量，並寫至shm。
* 開啟與Ecat交換資訊用的 Shared Memory, "ecatDataMemory"
*
* param: slaveNums[3] 偵測到的Slave數量。0:總數, 1:非Driver Slave數量 2:Driver Slave數量
*                     此參數由 RtEcatStartMaster()取得。
* return: 成功為1, 失敗為-1
*/

int MySystem::detectEcatSlave_KingStarIO(int slaveNums[3])
{
#if defined KING_STAR_IO
	// this function will get "slaveNum, slaveIONum, and slaveMotorNum".
	// Note: get slaveNums[3] from RtEcatStartMaster();
	slaveNum		= slaveNums[0];
	slaveIONum		= slaveNums[1];
	slaveMotorNum	= slaveNums[2];

	printf("slaveNum = %d, ioNum = %d, motorNum = %d\n", slaveNum, slaveIONum, slaveMotorNum);

	// open shm to store Ecat info.
	hDatamem = RtOpenSharedMemory(SHM_MAP_WRITE, FALSE, "RtxEcatDataSpace", (void **) &ecatDataMemory);
	if(hDatamem == NULL) { printf("RtOpenSharedMemory error.\n"); return -1;}
	
	shm->slaveMotorNum	= slaveMotorNum;
	shm->slaveIONum		= slaveIONum;
#endif
	return 1;

}


/**  動態配置IO物件。
*
* 檢查所有Ecat偵測到的 IO Slave (=非Driver Slave)，
* 對每個Slave 的 Input/Output Byte數，建立一相應的ioModule物件，加入ioGroup中。
* 此資訊同時寫入Shm。
* param: 無
* return: 成功為1, 失敗為-1
*/
int MySystem::initIO()
{
	//note: 雖然不使用 in/out Ptr, 但若不放此參數,結果會錯.
	U8_T* inpPtr;
	U8_T* outPtr;
	U32_T inpByteNum;
	U32_T outByteNum;

	#if defined KING_STAR_IO
	// 注意：所有非Driver都會被算成 IO。例如adaptor (0 out, 0 in).
	IO_ECAT*	kingIo;	
	for(int i = 0; i<slaveIONum; i++) {
		kingIo = (IO_ECAT*)((char*) ecatDataMemory + i*sizeof(IO_ECAT));

		inpByteNum = kingIo->dwInpLength/8;
		outByteNum = kingIo->dwOutLength/8;
		shm->ioModules[i].inpByteNum = inpByteNum;
		shm->ioModules[i].outByteNum = outByteNum;
		ioGroup->addModule(masterId, kingIo->dwSlaveID, outByteNum, inpByteNum); 
		ioGroup->ioModules[i]->kingIo = kingIo;

		printf("slave[%d], [in= %d, out= %d].\n", i, inpByteNum, outByteNum);
	}

	#elif defined WIN32_SIMULATE   

		shm->slaveIONum = 2;

		ioGroup->addModule(masterId, 0, 2, 0);
		ioGroup->addModule(masterId, 0, 0, 2);

		for(int i = 0; i<shm->slaveIONum; i++) {
			shm->ioModules[i].inpByteNum = ioGroup->ioModules[i]->inpByteNum;
			shm->ioModules[i].outByteNum = ioGroup->ioModules[i]->outByteNum;
			printf("IO[%d] In:%d bytes, Out:%d bytes\n", 
				i, 
				shm->ioModules[i].inpByteNum, 
				shm->ioModules[i].outByteNum);
		}
	#endif
	return 1;
}

/**  動態配置robot物件。
*
* 根據 shm->robotDeclareTable (使用者指定的Robot配置)，
* 動態建立並初始化Robot物件，指定其使用的馬達數量、運動學種類等。
* param: 無
* return: 成功為1, 失敗為-1
*/
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

	// Eigen 第一次運行會比較慢。
	// 因此先跑一次運動學 IK & FK 讓Eigen初始化，以免占用到 MyCallBack 時間。	
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

/**  將Motor物件配置給實體驅動器
*
* 根據shm->ecatMappingTable， (使用者指定的馬達-Ecat配置)，
* 對每個Ecat Slave，也就是實體驅動器，連結到相應的Motor物件。
* param: 無
* return: 成功為1, 失敗為-1
*/
int MySystem::linkToEcat()	
{
	#if defined UNDER_RTSS
	int r,m;
	
	//將每個ECAT馬達 mapping 到 EcatMaster的記憶體
	Motor* motorPtr;
	printf("\n********** slaveMotorNum = %d\n", slaveMotorNum);
	for(int i = 0; i< slaveMotorNum; i++) {
		printf("[%d] robotId = %d, axisId = %d\n", i, shm->ecatMappingTable[i].robotInd, shm->ecatMappingTable[i].axisInd);
		r = shm->ecatMappingTable[i].robotInd;	if(r < 0 || r >= robotNum) continue;			//invalid data.
		m = shm->ecatMappingTable[i].axisInd;	if(m < 0 || m >= robots[r]->motorNum) continue;	//invalid data.
		
		motorPtr = robots[r]->motors[m];

		#if defined KING_STAR_IO		
		motorPtr->kingAxis = (AXIS_ECAT*)((char*) ecatDataMemory + slaveIONum*sizeof(IO_ECAT) + i*sizeof(AXIS_ECAT));	
		motorPtr->setEncoderResolution(motorPtr->kingAxis->dwVendorId);
		#endif

		//initialize this motor.
		motorPtr->isLinkedToEcat = true;
		motorPtr->setMode(eRTXMode::CSP); 

		//將每個Robot的每個馬達的指標 assign 給 motorsAll。方便以後集體存取。
		//Note: 只計算"有link到Ecat的馬達"
		linkedMotorsTotal[linkedMotorNumTotal] = motorPtr;		//兩邊存的是同一個 "指向motor的指標".
		linkedMotorNumTotal++;

		printf("robots[%d]->motors[%d] is mapping to slaveAddr %d\n", r, m, i);
	}
	#endif

	return 1;
}

/**  令馬達轉至servoOn/Off狀態。
*
* 對每個Motor物件，呼叫一次Process()函式。
* Process()會馬達的狀態(servo On或Off) 轉成使用者指定的狀態。
* 此函式在MyCallBack被呼叫。
* see Motor::servoOn()
* see Motor::servoOff()
* param: 無
* return: 無
*/
void MySystem::processMotorStatus()
{
	#if defined KING_STAR_IO
	int m;
	for(m = 0; m< linkedMotorNumTotal; m++) {
		linkedMotorsTotal[m]->processStatus();		
	}
	#endif
}
void MySystem::StartServoOnOff()
{
	for (int r = 0; r < robotNum; r++)
		robots[r]->StartServoOnOff();
}

/**  將所有馬達的servo On/Off狀態寫至shm
*
* 對每一個Robot，呼叫其 isServoOn()函式。
* see Robot::isServoOn()
* param: 無
* return: 無
*/
void MySystem::isServoOn()
{
	for(int r = 0; r < robotNum; r++) {
		robots[r]->isServoOn();
	}
	return;
}

/**  更新Robot位姿
*
* 令所有Robot計算位姿，並寫至shm。
* 此函式在MyCallBack被呼叫。
* param: 無
* return: 無
*/
int MySystem::updateData()
{
#if defined WIN32_SIMULATE
	// if User press "Mastering Data" button in HMI: reset all degs to initial value.
	if(shm->isNeedResetDegree == true) {
		shm->isNeedResetDegree = false;

		int rId = shm->homingData.rId;
		int mId = shm->homingData.mId;
		
		double* th = shm->robots[rId].spec.DH.thetasInit;
		Robot* rob = robots[rId];
		int num = rob->motorNum;
		if(mId==-1) 
		{
			CopyArray(rob->axisDegNow, th, num);
			CopyArray(rob->axisDegCmd, th, num);
			CopyArray(rob->axisDegCmdPrev, th, num);
		}
		else
		{
			rob->axisDegNow[mId] = th[mId];
			rob->axisDegCmd[mId] = th[mId];
			rob->axisDegCmdPrev[mId] = th[mId];
		}
	}
#endif

	//update each robot.
	for(int r = 0; r < robotNum; r++) {
		robots[r]->isNeedMove = false;	//重置參數。預設不移動，除非有劇本/Jog命令。
		if(robots[r]->updateData() < 0) return -1;
		
	}
	return 1;
}

//-------- Private Function -----------
void MySystem::initEvents() //事件觸發物件 用來跟人機交握
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
		sprintf(str, "evtScriptSet%d", r);
		evtScriptSet[r] = EventHandler::Open(str);
		EventHandler::Reset(evtScriptSet[r]);

		sprintf(str, "evtScriptNeed%d", r);
		evtScriptNeed[r] = EventHandler::Open(str);
		EventHandler::Reset(evtScriptNeed[r]);
	}
}
