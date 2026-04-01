#pragma once

#include "Def.h"


/** @brief 系統類別。包含Robot, IOGroup等重要物件；負責Ecat連線及配置。
*
*  包含數個Robot物件，及一個公用IOGroup物件。
*  此類別處理:
*  1.宣告、初始化以上物件，連線至Ecat。
*  2.單一Robot無法處理的功能，例如:同動。
*  3.統一操作，例如:更新所有馬達狀態
*/
class Robot;
class Motor;
class IOGroup;
class Script;

class MySystem
{
public:
	 
	MySystem(void);
	~MySystem(void);

	//主要物件
	IOGroup*		ioGroup;				//!<IO物件
	Robot*		robots[MAX_ROBOT_NUM];	//!<Robot物件的陣列
										  
	int			robotNum;				//!<Robot數量
	int			scrRobotNum;			//!<執行劇本的Robot數量

	//跟人機交握用的event物件
	HANDLE		evtScriptStart;
	HANDLE		evtScriptEnd;
	HANDLE		evtGotSlaveInfo;
	HANDLE		evtHMICommand;
	HANDLE		evtHomingEnd;

	HANDLE		evtScriptSet[MAX_ROBOT_NUM];
	HANDLE		evtScriptNeed[MAX_ROBOT_NUM];

	//ECAT 相關變數
	int		masterId;
	U16_T	slaveNum;
	int		slaveMotorNum;
	int		slaveIONum;

	#if defined KING_STAR_IO
	HANDLE	hDatamem;
	void*	ecatDataMemory;
	#endif

	//--------- function ----------//	
	int detectEcatSlave_KingStarIO(int slaveNums[3]);
	int initIO();
	int initRobots();
	int linkToEcat();

	void processMotorStatus();
	void StartServoOnOff();
	void isServoOn();
	
	int  updateData();				
private:
	Motor*	linkedMotorsTotal[MAX_MOTOR_NUM];
	int		linkedMotorNumTotal;

	void 	initEvents(); 

	// Base related.
	int		currentBaseId;
};

