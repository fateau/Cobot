/*-----------------------------------------------------------------------------
 * ecatLayer.h              header file
 * Description              EtherCAT abstraction layer for acontis EC-Master (Linux)
 *                          Replaces KingStar API with acontis EC-Master API
 *---------------------------------------------------------------------------*/
#pragma once

#include "EcMaster.h"

#define NO_ERRORS       0
#define ERROR_OCCURED  -1

/*-TYPEDEFS------------------------------------------------------------------*/
#ifndef ECAT_LAYER
#define ECAT_LAYER

/* DS402 object identifiers */
#define DRV_OBJ_ERROR_CODE                  0x603F
#define DRV_OBJ_CONTROL_WORD                0x6040
#define DRV_OBJ_STATUS_WORD                 0x6041
#define DRV_OBJ_MODES_OF_OPERATION          0x6060
#define DRV_OBJ_MODES_OF_OPERATION_DISPLAY  0x6061
#define DRV_OBJ_POSITION_ACTUAL_VALUE       0x6064
#define DRV_OBJ_VELOCITY_ACTUAL_VALUE       0x606C
#define DRV_OBJ_TARGET_TORQUE               0x6071
#define DRV_OBJ_TORQUE_ACTUAL_VALUE         0x6077
#define DRV_OBJ_TARGET_POSITION             0x607A
#define DRV_OBJ_TORQUE_OFFSET               0x60B2
#define DRV_OBJ_FOLLOWING_ERROR             0x60F4
#define DRV_OBJ_TARGET_VELOCITY             0x60FF

#define OBJOFFSET                           0x800

/* Slave axis structure - direct pointer-based PDO access (acontis style) */
typedef struct
{
    unsigned int        dwVendorId;
    unsigned int        dwProductCode;
    unsigned int        dwSlaveID;
    unsigned short      wStationAddress;
    unsigned short      wSlaveState;

    /* PDO Output pointers (write to slave) */
    EC_T_WORD*          pwControlWord;        // 0x6040
    EC_T_INT*           pnTargetPosition;     // 0x607A
    EC_T_INT*           pnTargetVelocity;     // 0x60FF
    EC_T_WORD*          pwTargetTorque;       // 0x6071
    EC_T_BYTE*          pbyModeOfOperation;   // 0x6060
    EC_T_WORD*          pwTorqueOffset;       // 0x60B2
    EC_T_WORD*          pwCustomOutput2;      // custom output (Synapticon init)

    /* PDO Input pointers (read from slave) */
    EC_T_WORD*          pwErrorCode;          // 0x603F
    EC_T_WORD*          pwStatusWord;         // 0x6041
    EC_T_INT*           pnActPosition;        // 0x6064
    EC_T_INT*           pnActVelocity;        // 0x606C
    EC_T_WORD*          pwActTorque;          // 0x6077
    EC_T_DWORD*         pdwActFollowErr;      // 0x60F4
    EC_T_WORD*          pwCustomInput1;       // custom input1
    EC_T_WORD*          pwCustomInput2;       // custom input2

    unsigned int        dwEncoderRes;
} AXIS_ECAT;

/* IO module structure */
typedef struct
{
    unsigned int        dwVendorId;
    unsigned int        dwProductCode;
    unsigned int        dwSlaveID;
    unsigned short      wStationAddress;
    unsigned short      wSlaveState;

    unsigned int        dwInpBitLength;
    unsigned int        dwInpByteOff;
    unsigned int        dwOutBitLength;
    unsigned int        dwOutByteOff;

    EC_T_BYTE*          pInp;   // pointer into process image input
    EC_T_BYTE*          pOut;   // pointer into process image output
} IO_ECAT;

/* Callback type for cyclic task */
typedef void (*AppCallback)(void* context, int masterState);

#endif /* ECAT_LAYER */
