#pragma once
#include "Def.h"


namespace eMotorCommand {
	enum e {
		SERVO_ON,
		SERVO_OFF,
		NONE
	};
}
namespace eMotorStatus {	//difined according to CiA402 Profile
	enum e	
	{
		NOT_READY,
		SWITCH_ON_DISABLED,	//Servo Off
		READY_TO_SWITCH_ON,
		SWITCHED_ON,
		OPERATION_ENABLED,	//Servo On
		QUICK_STOP,
		FAULT
	};
}

class Motor
{
public:

	//-------- member --------
	
	#if defined KING_STAR_IO
	AXIS_ECAT*	kingAxis;
	void	processStatus();		//function to turn servoOn
	#endif

	int		rId,mId;					//在robot上的配置 //to get gearRatio from shm.
	int       index;                 
	U32_T	vendorID;

	bool	isLinkedToEcat;

	//-------- function --------
	Motor(int rId,int mId);

	void	setEncoderResolution(U32_T vendorID);
	void	setGearRatio(double gearRatio);
	double  getGearRatio();
	int		updateData();

	bool	isServoOn();

	void	StartServoOnOff(); 

	void	servoOn();
	void	servoOff();
	//========= Homing =============
	eMotorCommand::e	command;			//人機下達指令。
	eMotorStatus::e		status;				//馬達狀態。相當於statusWord的文字敘述

	void	setMode(eRTXMode::e mode);
	void	setControlWord(U16_T controlWord);
	void	setTargetPosition( int targetPosition);
	void	setTargetDeg(double targetDeg);
	void	setTargetTorque(int targetTorque);
	//PMC Modified 11412 //1229
	void setTorqueOffset(int TorqueOffset); 
	float readPositionKp(); 
	float readVelocityKp(); 
	float readVelocityKi(); 
	void writePositionKp(float PositionKp); 
	void writeVelocityKp(float VelocityKp);
	void writeVelocityKi(float VelocityKi); 
	void saveParametersToFlash(); 
	void	ProcessPIDCommands(); 

	U16_T	getStatusWord();
	int		getActualPosition();
	double	getActualDeg();
	double	getActualDegVel(); 
	U16_T	getActualTorque();
	unsigned long getErrorCode();

private:
	
	eRTXMode::e _mode;

	U16_T	_statusWord;
	int		_actualPosition;
	int		_actualVelocity;		// unit: encoder/s
	I16_T	_actualTorque;
	double	_actualDeg;
	double	_actualDegVel;			// unit: deg/s

	int     errorCounter;
	int		encoderResolution;
	double	gearRatio;
	double	angleToEncoderRatio;	// = encoderResolution * gearRatio / 360
	double	encoderToAngleRatio;	// = 360 / encoderResolution / gearRatio
	bool	hasPrintError;	//temp
};

