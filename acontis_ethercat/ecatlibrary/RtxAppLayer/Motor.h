#pragma once

/*---------------------------------------------------------------------------
 * Motor.h - Motor control for acontis EC-Master (Linux)
 *           Uses direct pointer-based PDO access
 *---------------------------------------------------------------------------*/

#include "Def.h"

namespace eMotorCommand {
    enum e {
        SERVO_ON,
        SERVO_OFF,
        NONE
    };
}
namespace eMotorStatus {    // CiA402 Profile
    enum e
    {
        NOT_READY,
        SWITCH_ON_DISABLED,
        READY_TO_SWITCH_ON,
        SWITCHED_ON,
        OPERATION_ENABLED,  // Servo On
        QUICK_STOP,
        FAULT
    };
}

class Motor
{
public:
    //-------- member --------
    AXIS_ECAT*  kingAxis;           // acontis axis pointer (keeping name for minimal code changes)
    void        processStatus();    // CiA402 state machine

    int         rId, mId;
    int         index;
    U32_T       vendorID;

    bool        isLinkedToEcat;

    eMotorCommand::e    command;
    eMotorStatus::e     status;

    //-------- function --------
    Motor(int rId, int mId);

    void    setEncoderResolution(U32_T vendorID);
    void    setGearRatio(double gearRatio);
    double  getGearRatio();
    int     updateData();

    bool    isServoOn();
    void    StartServoOnOff();
    void    servoOn();
    void    servoOff();

    void    setMode(eRTXMode::e mode);
    void    setControlWord(U16_T controlWord);
    void    setTargetPosition(int targetPosition);
    void    setTargetDeg(double targetDeg);
    void    setTargetTorque(int targetTorque);

    void    setTorqueOffset(int TorqueOffset);
    float   readPositionKp();
    float   readVelocityKp();
    float   readVelocityKi();
    void    writePositionKp(float PositionKp);
    void    writeVelocityKp(float VelocityKp);
    void    writeVelocityKi(float VelocityKi);
    void    saveParametersToFlash();
    void    ProcessPIDCommands();

    U16_T   getStatusWord();
    int     getActualPosition();
    double  getActualDeg();
    double  getActualDegVel();
    U16_T   getActualTorque();
    unsigned long getErrorCode();

private:
    eRTXMode::e _mode;

    U16_T   _statusWord;
    int     _actualPosition;
    int     _actualVelocity;
    I16_T   _actualTorque;
    double  _actualDeg;
    double  _actualDegVel;

    int     errorCounter;
    int     encoderResolution;
    double  gearRatio;
    double  angleToEncoderRatio;
    double  encoderToAngleRatio;
    bool    hasPrintError;
};
