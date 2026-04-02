/*---------------------------------------------------------------------------
 * Motor.cpp - Motor control for acontis EC-Master (Linux)
 *             Uses direct pointer-based PDO access (EC_SETWORD/EC_GETWORD)
 *---------------------------------------------------------------------------*/
#include "Motor.h"
#include <cstdio>
#include "Shm.h"
#include "ErrorHandler.h"
#include "Debug.h"

extern SHMData* shm;

Motor::Motor(int rId, int mId)
{
    this->rId           = rId;
    this->mId           = mId;

    encoderResolution   = 0;
    angleToEncoderRatio = 0;
    encoderToAngleRatio = 0;

    isLinkedToEcat      = false;
    hasPrintError       = false;
    _statusWord         = 0;

    status      = eMotorStatus::NOT_READY;
    command     = eMotorCommand::NONE;
    kingAxis    = nullptr;
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
        break;
    default:
        encoderResolution = 524288;
    }
    if (mId == 0 || mId == 1 || mId == 2) encoderResolution = 1048576;

    angleToEncoderRatio = gearRatio * encoderResolution / 360;
    encoderToAngleRatio = 360.0 / gearRatio / encoderResolution;
}


int Motor::updateData()
{
    if (!isLinkedToEcat || !kingAxis) return 2;

    unsigned long error = 0;
    unsigned long errorReport = 0;

    // Read status word from process data
    if (kingAxis->pwStatusWord)
        _statusWord = EC_GETWORD(kingAxis->pwStatusWord);

    if (status == eMotorStatus::OPERATION_ENABLED)
    {
        if ((_statusWord & 0x4F) == 0x00 || (_statusWord & 0x4F) == 0x40 || (_statusWord & 0x4F) == 0x60)
        {
            status = eMotorStatus::NOT_READY;
            return -1;
        }
    }

    // Read error codes (Synapticon)
    if (vendorID == SYNAPTICON) {
        if (kingAxis->pwCustomInput1)
            error = EC_GETWORD(kingAxis->pwCustomInput1);
        if (kingAxis->pwCustomInput2)
            errorReport = EC_GETWORD(kingAxis->pwCustomInput2);
    }

    if (_statusWord & 0x0008) {
        if (!hasPrintError) {
            hasPrintError = true;
            ErrorHandler::Set(eError::DRIVER_ERROR,
                "r[%d], axis[%d]. statusWord=%x, error(603F)=%lx, report(203F)=%lx\n",
                rId + 1, mId + 1, _statusWord, error, errorReport);
        }
        return -1;
    }
    else if (_statusWord & 0x0080) {
        Debug::writeln(4, "Warning(603F) = %lx, report(203F)=%lx, r[%d], axis[%d]", error, errorReport, rId + 1, mId + 1);
        return 1;
    }

    hasPrintError = false;

    // Read actual position, velocity, torque from process data
    if (kingAxis->pnActPosition)
        _actualPosition = (int)EC_GETDWORD(kingAxis->pnActPosition);
    if (kingAxis->pnActVelocity)
        _actualVelocity = (int)EC_GETDWORD(kingAxis->pnActVelocity);
    if (kingAxis->pwActTorque)
        _actualTorque = (I16_T)EC_GETWORD(kingAxis->pwActTorque);

    _actualDegVel = _actualVelocity * gearRatio * 6.0 / 1000;
    _actualDeg    = _actualPosition * encoderToAngleRatio;

    ProcessPIDCommands();

    return 1;
}

void Motor::servoOn()
{
    if (!isLinkedToEcat || !kingAxis) return;

    // Let TargetPos = ActualPos, to prevent suddenly jump at ServoOn
    if (kingAxis->pnActPosition && kingAxis->pnTargetPosition) {
        int acPos = (int)EC_GETDWORD(kingAxis->pnActPosition);
        EC_SETDWORD(kingAxis->pnTargetPosition, acPos);
        _actualPosition = acPos;
    }
    command = eMotorCommand::SERVO_ON;

    // Synapticon: initialize custom output
    if (vendorID == SYNAPTICON && kingAxis->pwCustomOutput2) {
        EC_SETWORD(kingAxis->pwCustomOutput2, 1000);
    }
}

void Motor::servoOff()
{
    if (!isLinkedToEcat || !kingAxis) return;
    command = eMotorCommand::SERVO_OFF;
}

bool Motor::isServoOn()
{
    if (status == eMotorStatus::OPERATION_ENABLED)
        return true;
    return false;
}

void Motor::StartServoOnOff()
{
    MotorData* motorData = &(shm->robots[rId].motors[mId]);
    if (motorData->isNeedServoOn) {
        if (_statusWord & 0x0008) {
            setControlWord(0x80);
            motorData->isNeedServoOn = false;
            hasPrintError = false;
            return;
        }
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
    if (kingAxis && kingAxis->pbyModeOfOperation) {
        EC_T_BYTE byMode = (EC_T_BYTE)mode;
        EC_SETBITS(kingAxis->pbyModeOfOperation, &byMode, 0, 8);
    }
}

void Motor::setControlWord(U16_T controlWord)
{
    if (!isLinkedToEcat || !kingAxis) return;
    if (kingAxis->pwControlWord)
        EC_SETWORD(kingAxis->pwControlWord, controlWord);
}

void Motor::setTargetPosition(int targetPosition)
{
    if (!isServoOn()) return;
    if (!isLinkedToEcat || !kingAxis) return;
    if (kingAxis->pnTargetPosition)
        EC_SETDWORD(kingAxis->pnTargetPosition, (EC_T_DWORD)targetPosition);
}

static int ToInt(double d)
{
    if (d > INT_MAX)
        d = d - (double)INT_MAX - (double)INT_MAX;
    else if (d < INT_MIN)
        d = d + (double)INT_MAX + (double)INT_MAX;
    return (int)d;
}

void Motor::setTargetDeg(double targetDeg)
{
    int i = ToInt(targetDeg * angleToEncoderRatio);
    setTargetPosition(i);
}

void Motor::setTargetTorque(int targetTorque)
{
    if (!isLinkedToEcat || !kingAxis) return;
    if (kingAxis->pwTargetTorque)
        EC_SETWORD(kingAxis->pwTargetTorque, (unsigned short)targetTorque);
}

void Motor::setTorqueOffset(int TorqueOffset)
{
    if (!isLinkedToEcat || !kingAxis) return;
    if (this->_mode == 10 && TorqueOffset != 0) {
        if (kingAxis->pwTorqueOffset)
            EC_SETWORD(kingAxis->pwTorqueOffset, 0);
        return;
    }
    if (kingAxis->pwTorqueOffset)
        EC_SETWORD(kingAxis->pwTorqueOffset, (unsigned short)TorqueOffset);
}

// SDO functions use acontis emCoeSdoUpload / emCoeSdoDownload (ecatCoeSdoUpload/ecatCoeSdoDownload wrappers)
float Motor::readPositionKp()
{
    float valueRead = 0.0f;
    if (!isLinkedToEcat || !kingAxis) return valueRead;

    EC_T_DWORD outSize = 0;
    ecatCoeSdoUpload(kingAxis->dwSlaveID, 0x2012, 0x01,
        (EC_T_BYTE*)&valueRead, sizeof(float), &outSize, 500, 0);
    return valueRead;
}

float Motor::readVelocityKp()
{
    float valueRead = 0.0f;
    if (!isLinkedToEcat || !kingAxis) return valueRead;

    EC_T_DWORD outSize = 0;
    ecatCoeSdoUpload(kingAxis->dwSlaveID, 0x2012, 0x05,
        (EC_T_BYTE*)&valueRead, sizeof(float), &outSize, 500, 0);
    return valueRead;
}

float Motor::readVelocityKi()
{
    float valueRead = 0.0f;
    if (!isLinkedToEcat || !kingAxis) return valueRead;

    EC_T_DWORD outSize = 0;
    ecatCoeSdoUpload(kingAxis->dwSlaveID, 0x2012, 0x06,
        (EC_T_BYTE*)&valueRead, sizeof(float), &outSize, 500, 0);
    return valueRead;
}

void Motor::writePositionKp(float PositionKp)
{
    if (!isLinkedToEcat || !kingAxis) return;
    ecatCoeSdoDownload(kingAxis->dwSlaveID, 0x2012, 0x01,
        (EC_T_BYTE*)&PositionKp, sizeof(float), 500, 0);
}

void Motor::writeVelocityKp(float VelocityKp)
{
    if (!isLinkedToEcat || !kingAxis) return;
    ecatCoeSdoDownload(kingAxis->dwSlaveID, 0x2012, 0x05,
        (EC_T_BYTE*)&VelocityKp, sizeof(float), 500, 0);
}

void Motor::writeVelocityKi(float VelocityKi)
{
    if (!isLinkedToEcat || !kingAxis) return;
    ecatCoeSdoDownload(kingAxis->dwSlaveID, 0x2012, 0x06,
        (EC_T_BYTE*)&VelocityKi, sizeof(float), 500, 0);
}

void Motor::saveParametersToFlash()
{
    if (!isLinkedToEcat || !kingAxis) return;

    unsigned long saveSignature = 0x65766173; // "save"
    ecatCoeSdoDownload(kingAxis->dwSlaveID, 0x1010, 0x01,
        (EC_T_BYTE*)&saveSignature, sizeof(unsigned long), 1000, 0);
}

void Motor::ProcessPIDCommands()
{
    if (!isLinkedToEcat) return;

    MotorData* mData = &(shm->robots[rId].motors[mId]);

    if (mData->isNeedSetPID) {
        writePositionKp((float)mData->cmdKpp);
        writeVelocityKp((float)mData->cmdKvp);
        writeVelocityKi((float)mData->cmdKvi);
        mData->Kpp = mData->cmdKpp;
        mData->Kvp = mData->cmdKvp;
        mData->Kvi = mData->cmdKvi;
        mData->isNeedSetPID = false;
    }

    if (mData->isNeedReadPID) {
        mData->Kpp = (double)readPositionKp();
        mData->Kvp = (double)readVelocityKp();
        mData->Kvi = (double)readVelocityKi();
        mData->isNeedReadPID = false;
    }

    if (mData->isNeedSaveParams) {
        if (!isServoOn()) {
            saveParametersToFlash();
        } else {
            printf("Warning: Cannot Save to Flash while Servo is ON!\n");
        }
        mData->isNeedSaveParams = false;
    }
}

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
    if (kingAxis && kingAxis->pwErrorCode)
        return (unsigned long)EC_GETWORD(kingAxis->pwErrorCode);
    return 0;
}

void Motor::processStatus()
{
    if (!kingAxis || !kingAxis->pwStatusWord) return;

    if (command == eMotorCommand::SERVO_ON) {
        _statusWord = EC_GETWORD(kingAxis->pwStatusWord);
        switch (status) {
            case eMotorStatus::NOT_READY:
                if ((_statusWord & 0x4F) == 0x00 ||
                    (_statusWord & 0x4F) == 0x40 ||
                    (_statusWord & 0x4F) == 0x60)
                {
                    status = eMotorStatus::SWITCH_ON_DISABLED;
                }
                else if (_statusWord & 0x08) {
                    status = eMotorStatus::FAULT;
                    setControlWord(0x80);   // fault reset
                }
                break;
            case eMotorStatus::SWITCH_ON_DISABLED:
                if ((_statusWord & 0x4F) == 0x40 ||
                    (_statusWord & 0x4F) == 0x60)
                {
                    setControlWord(0x06);   // shutdown
                    status = eMotorStatus::READY_TO_SWITCH_ON;
                }
                break;
            case eMotorStatus::READY_TO_SWITCH_ON:
                if ((_statusWord & 0x6F) == 0x21) {
                    setControlWord(0x07);   // switch on
                    status = eMotorStatus::SWITCHED_ON;
                }
                break;
            case eMotorStatus::SWITCHED_ON:
                if ((_statusWord & 0x6F) == 0x23) {
                    setControlWord(0x0F);   // enable operation
                    status = eMotorStatus::OPERATION_ENABLED;
                    command = eMotorCommand::NONE;
                }
                break;
            case eMotorStatus::FAULT:
                setControlWord(0x80);       // fault reset
                status = eMotorStatus::NOT_READY;
                break;
            default:
                break;
        }
    }
    else if (command == eMotorCommand::SERVO_OFF) {
        setControlWord(0x00);               // disable voltage
        command = eMotorCommand::NONE;
        status  = eMotorStatus::NOT_READY;
    }
}
