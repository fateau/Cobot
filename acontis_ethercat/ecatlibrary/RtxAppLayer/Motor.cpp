#include "Motor.h"
#include <stdio.h>
#include "SHM.h"
#include "ErrorHandler.h"
#include "Debug.h"

extern SHMData* shm;
Motor::Motor(int rId,int mId)
{
	this->rId           = rId;
	this->mId			= mId;

	encoderResolution	= 0;
	angleToEncoderRatio = 0;
	encoderToAngleRatio = 0;

	isLinkedToEcat		= false;
	hasPrintError		= false;
	_statusWord			= 0;

	#if defined KING_STAR_IO
	status		= eMotorStatus::NOT_READY;
	command		= eMotorCommand::NONE;
	#endif
}

void Motor::setGearRatio(double gearRatio)
{
	this->gearRatio = gearRatio;
}
double Motor::getGearRatio()
{
	return this->gearRatio;
}
void Motor::setEncoderResolution(U32_T vendorID)
{
	this->vendorID = vendorID;
	switch (vendorID) {
	case SYNAPTICON:
		encoderResolution = 524288;
	default:
		encoderResolution = 524288;	//modify by yunyu 20250509
	}
	if (mId == 0 || mId == 1 || mId == 2) encoderResolution = 1048576; //PMC Modified 11501 //0114

	angleToEncoderRatio = gearRatio *encoderResolution / 360; 
	encoderToAngleRatio = 360 / gearRatio / encoderResolution;

	return;
}


int  Motor::updateData()
{
	if (!isLinkedToEcat) return 2;
#if defined KING_STAR_IO
	unsigned long error = 0;
	unsigned long errorReport = 0;  //PMC 11411 //1127

	//detect if Driver has error.-> block myCallback.  
	ReadStatusWord(kingAxis, &_statusWord);

	if (status == eMotorStatus::OPERATION_ENABLED)  //Status : Brake OFF 
	{
		if ((_statusWord & 0x4F) == 0x00 || (_statusWord & 0x4F) == 0x40 || (_statusWord & 0x4F) == 0x60)
		{
			status = eMotorStatus::NOT_READY;
			return -1;
		}
	}

	//錯誤碼讀取
	if (vendorID == SYNAPTICON) {   //PMC 11411 //1127
		ReadCustomInput1(kingAxis, &error);
		ReadCustomInput2(kingAxis, &errorReport);
	}

	/*if (error == 20992) {
		Debug::writeln(0, "Error = 5200, r[%d], axis[%d] \n", rId + 1, mId + 1);
		return 1;
	}*/

	if (_statusWord & 0x0008) { //bit 3 should be 0  //PMC 11411 //1127
		if (!hasPrintError) {
			hasPrintError = true;
			ErrorHandler::Set(eError::DRIVER_ERROR,
				"r[%d], axis[%d]. statusWord=%x, error(603F)=%x, report(203F)=%x\n",
				rId + 1,
				mId + 1,
				_statusWord,
				error,
				errorReport);
		}
		return -1;
	}
	else if(_statusWord & 0x0080) {   //PMC 11411 //1127
		Debug::writeln(4, "Warning(603F) = %x, report(203F)=%x, r[%d], axis[%d]", error, errorReport, rId + 1, mId + 1);
		return 1;
	}

	hasPrintError = false;

	unsigned long acPos;
	unsigned long acVel;

	#ifdef _AMD64_
		unsigned short acTorq;
	#endif
	ReadActualPosition(kingAxis, &acPos);
	ReadActualVelocity(kingAxis, &acVel);
	ReadActualTorque(kingAxis, &acTorq);
	_actualPosition = (int)acPos;
	_actualVelocity = (int)acVel;
	_actualTorque	= (I16_T)acTorq;
	#endif	

	//PMC Modified 11411 //1118
	_actualDegVel	= _actualVelocity * gearRatio * 6.0 /1000; //encoderToAngleRatio; //gearRatio = +-1 //  360(一圈度數) / (60(秒數) * 1000(0.001rpm))
	_actualDeg		= _actualPosition * encoderToAngleRatio;

	ProcessPIDCommands(); //PMC Modified 11412 //1229

	return 1;
}
void Motor::servoOn()
{
	if(!isLinkedToEcat) return;

	#if defined KING_STAR_IO	
	//let TargetPos = ActualPos, to prevent suddenly jump at ServoOn.
	unsigned long acPos;
	ReadActualPosition(kingAxis, &acPos);
	WriteTargetPosition(kingAxis, acPos);
	_actualPosition = (int)acPos;
	command = eMotorCommand::SERVO_ON;
	//add by yunyu 20250509 //ServoOn時，先初始化為常數值
	if(vendorID == SYNAPTICON) WriteCustomOutput2(kingAxis,1000);
	#endif
}
void Motor::servoOff()
{
	if(!isLinkedToEcat) return;

	#if defined KING_STAR_IO
	command = eMotorCommand::SERVO_OFF;
	#endif
}

bool Motor::isServoOn() {

	#if defined KING_STAR_IO	
	if( status == eMotorStatus::OPERATION_ENABLED)
		return true;
	else
		return false;
	#elif defined WIN32_SIMULATE
		return true;
	#endif
	
}

void Motor::StartServoOnOff() //伺服開關   //11412 PMC modified//1211
{
	MotorData* motorData = &(shm->robots[rId].motors[mId]);
	if(motorData->isNeedServoOn){
#if defined KING_STAR_IO
		if (_statusWord & 0x0008) {
			setControlWord(0x80);
			motorData->isNeedServoOn = false;
			hasPrintError = false;
			return;
			
		}
#endif
		motorData->isNeedServoOn = false;
		servoOn();
	}
	else if (motorData->isNeedServoOff) {
		motorData->isNeedServoOff = false;
		servoOff();
	}
}

void Motor::setMode(eRTXMode::e mode)
{
	_mode = mode;
	#if defined KING_STAR_IO
	WriteModeOP(kingAxis, mode); // Note: 需在初始化時將MOP設為true
	#endif
}
void Motor::setControlWord(U16_T controlWord)
{
	if(!isLinkedToEcat) return;

	#if defined KING_STAR_IO
	WriteControlWord(kingAxis, controlWord);
	#endif
}
void Motor::setTargetPosition(int targetPosition)
{
	if (!isServoOn()) return;
	if (!isLinkedToEcat) return;

#if defined KING_STAR_IO
	WriteTargetPosition(kingAxis, targetPosition);
	#endif
}

int ToInt(double d)
{	
	if(d > INT_MAX) 			
		d = d - INT_MAX - INT_MAX; //note: can't write as  d -= 2*INT_MAX. 

	else if(d < INT_MIN) 
		d = d + INT_MAX + INT_MAX;

	int i = (int)d;
	return i;
}
void Motor::setTargetDeg(double targetDeg)
{
	int i = ToInt(targetDeg * angleToEncoderRatio);
	setTargetPosition(i);
}
void Motor::setTargetTorque(int targetTorque)
{
	#if defined KING_STAR_IO

		#ifdef _AMD64_
			WriteTargetTorque(kingAxis, (unsigned short)targetTorque);;
		#endif

	#endif
}

//////////////////////// //PMC Modified 11412//1229
void Motor::setTorqueOffset(int TorqueOffset)
{
	if (!isLinkedToEcat) return;
#if defined KING_STAR_IO
	if (this->_mode == 10 && TorqueOffset != 0) {
		WriteTorqueOffset(kingAxis, 0);
		return;
	}
	WriteTorqueOffset(kingAxis, TorqueOffset);
#endif
}
float Motor::readPositionKp()
{
	float valueRead = 0.0f;

	if (!isLinkedToEcat) return valueRead;

#if defined KING_STAR_IO

	unsigned short index = 0x2012;
	unsigned char subIndex = 0x01;
	unsigned long outSize = 0;

	int ret = CoeSdoUpload(
		kingAxis->dwSlaveID,
		index,
		subIndex,
		(unsigned char*)&valueRead,
		sizeof(float),
		&outSize,
		100,
		0
	);

#endif
	return valueRead;
}
float Motor::readVelocityKp()
{
	float valueRead = 0.0f;

	if (!isLinkedToEcat) return valueRead;

#if defined KING_STAR_IO

	unsigned short index = 0x2012;
	unsigned char subIndex = 0x05;
	unsigned long outSize = 0;

	int ret = CoeSdoUpload(
		kingAxis->dwSlaveID,
		index,
		subIndex,
		(unsigned char*)&valueRead,
		sizeof(float),
		&outSize,
		100,
		0
	);

#endif
	return valueRead;
}
float Motor::readVelocityKi()
{
	float valueRead = 0.0f;

	if (!isLinkedToEcat) return valueRead;

#if defined KING_STAR_IO

	unsigned short index = 0x2012;
	unsigned char subIndex = 0x06;
	unsigned long outSize = 0;

	int ret = CoeSdoUpload(
		kingAxis->dwSlaveID,
		index,
		subIndex,
		(unsigned char*)&valueRead,
		sizeof(float),
		&outSize,
		100,
		0
	);

#endif
	return valueRead;
}
void Motor::writePositionKp(float PositionKp)
{
	if (!isLinkedToEcat) return;

#if defined KING_STAR_IO

	unsigned short index = 0x2012;
	unsigned char subIndex = 0x01;

	float valueToWrite = PositionKp;

	int ret = CoeSdoDownload(
		kingAxis->dwSlaveID,
		index,
		subIndex,
		(unsigned char*)&valueToWrite,
		sizeof(float),
		100,
		0
	);
#endif
}
void Motor::writeVelocityKp(float VelocityKp)
{
	if (!isLinkedToEcat) return;

#if defined KING_STAR_IO

	unsigned short index = 0x2012;
	unsigned char subIndex = 0x05;

	float valueToWrite = VelocityKp;

	int ret = CoeSdoDownload(
		kingAxis->dwSlaveID,
		index,
		subIndex,
		(unsigned char*)&valueToWrite,
		sizeof(float),
		100,
		0
	);
#endif
}
void Motor::writeVelocityKi(float VelocityKi)
{
	if (!isLinkedToEcat) return;

#if defined KING_STAR_IO

	unsigned short index = 0x2012;
	unsigned char subIndex = 0x06;

	float valueToWrite = VelocityKi;

	int ret = CoeSdoDownload(
		kingAxis->dwSlaveID,
		index,
		subIndex,
		(unsigned char*)&valueToWrite,
		sizeof(float),
		100,
		0
	);
#endif
}
void Motor::saveParametersToFlash()
{
	if (!isLinkedToEcat) return;

#if defined KING_STAR_IO

	unsigned short index = 0x1010;
	unsigned char subIndex = 0x01;

	// 寫入值: "save" (0x65766173)
	unsigned long saveSignature = 0x65766173;

	int ret = CoeSdoDownload(
		kingAxis->dwSlaveID,
		index,
		subIndex,
		(unsigned char*)&saveSignature,
		sizeof(unsigned long),
		1000,
		0
	);
#endif
}

void Motor::ProcessPIDCommands()
{
	if (!isLinkedToEcat) return;

	MotorData* mData = &(shm->robots[rId].motors[mId]);

	if (mData->isNeedSetPID)
	{
		writePositionKp((float)mData->cmdKpp);
		writeVelocityKp((float)mData->cmdKvp);
		writeVelocityKi((float)mData->cmdKvi);

		mData->Kpp = mData->cmdKpp;
		mData->Kvp = mData->cmdKvp;
		mData->Kvi = mData->cmdKvi;

		mData->isNeedSetPID = false;
	}

	if (mData->isNeedReadPID)
	{
		float kpp = readPositionKp();
		float kvp = readVelocityKp();
		float kvi = readVelocityKi();

		mData->Kpp = (double)kpp;
		mData->Kvp = (double)kvp;
		mData->Kvi = (double)kvi;

		mData->isNeedReadPID = false;
	}

	if (mData->isNeedSaveParams)
	{
		if (isServoOn() == false)
		{
			saveParametersToFlash(); // 執行存檔
		}
		else
		{
#if defined KING_STAR_IO
			RtPrintf("Warning: Cannot Save to Flash while Servo is ON!\n");
#endif
		}

		mData->isNeedSaveParams = false;
	}
}
///////////////////////////////////////////////////

U16_T Motor::getStatusWord()
{
	return _statusWord;
}
int Motor::getActualPosition()
{
	return _actualPosition;
}
double Motor::getActualDeg()
{	
	return _actualDeg;
}
double Motor::getActualDegVel() 
{	
	return _actualDegVel;
}
U16_T Motor::getActualTorque()
{	
	return _actualTorque;
}
unsigned long Motor::getErrorCode()
{

	#if defined KING_STAR_IO
	unsigned long errorCode = 0;
	
		#ifdef _AMD64_
			ReadDWord((unsigned char*)kingAxis->pPdoOut, 0x603F, &errorCode);
		#else
			ReadDWord((unsigned long*)kingAxis->pPdoOut, 0x603F, &errorCode);
		#endif
	
	return errorCode;
	#elif defined WIN32_SIMULATE
	return 0;
	#endif


	
}
#if defined KING_STAR_IO
void Motor::processStatus()
{	 
	if(command == eMotorCommand::SERVO_ON) {
		ReadStatusWord(kingAxis, &_statusWord);
		switch (status) {
			case eMotorStatus::NOT_READY:
				if( (_statusWord & 0x4F) == 0x00 ||  
					(_statusWord & 0x4F) == 0x40 ||  
					(_statusWord & 0x4F) == 0x60 )
				{
					status = eMotorStatus::SWITCH_ON_DISABLED;
				} 
				else if (_statusWord & 0x08) {
					status = eMotorStatus::FAULT;
					setControlWord(0x80);			//fault reset
				} 
				break;
			case eMotorStatus::SWITCH_ON_DISABLED:    
				if( (_statusWord & 0x4F) == 0x40 ||
					(_statusWord & 0x4F) == 0x60 )
				{
					setControlWord(0x06);			//shut down
					status = eMotorStatus::READY_TO_SWITCH_ON;
				} 
				break;
			case eMotorStatus::READY_TO_SWITCH_ON:   
				if((_statusWord & 0x6F) == 0x21) {
					setControlWord(0x07);			//switch on
					status = eMotorStatus::SWITCHED_ON;
				} break;
			case eMotorStatus::SWITCHED_ON:
				if((_statusWord & 0x6F) == 0x23) {	
					setControlWord(0x0F);			//enable operation
					status = eMotorStatus::OPERATION_ENABLED;
					command = eMotorCommand::NONE;	//重置命令
				} break;
			case eMotorStatus::FAULT:               
					setControlWord(0x80);			//fault reset
					status = eMotorStatus::NOT_READY;
				break;
		}
	}
	else if(command == eMotorCommand::SERVO_OFF){
		setControlWord(0x00);						//disable voltage.
		command		= eMotorCommand::NONE;			//重置命令
		status		= eMotorStatus::NOT_READY;
	}
	
	return ;
}

#endif



