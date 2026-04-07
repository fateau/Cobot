/*---------------------------------------------------------------------------
 * AppLayer.cpp - Main application entry point for acontis EC-Master (Linux)
 *                Replaces KingStar/RTX64 based initialization
 *---------------------------------------------------------------------------*/
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dlfcn.h>
#include <time.h>
#include <errno.h>

#include "EcMaster.h"

#include "MySystem.h"
#include "Script.h"
#include "Shm.h"
#include "ErrorHandler.h"

#include "Jog.h"
#include "Motor.h"
#include "Robot.h"
#include "IOGroup.h"
#include "IOModule.h"
#include "CmdContext.h"
#include "EventHandler.h"
#include "Kinematics.h"
#include "Debug.h"
#include "MyCallback.h"
#include "HelpFunctions.h"

#include "Eigen/src/Core/products/Parallelizer.h"

/* ======================== Global Variables ======================== */
SHMData*  shm = NULL;
MySystem* sys = NULL;

static void sigintHandler(int sig)
{
    (void)sig;
    printf("\nCaught SIGINT, shutting down...\n");
    if (shm) shm->stopRTX = true;
    /* Wake up any blocking WaitFor (e.g. evtHMICommand in DetectSlave) */
    if (sys) EventHandler::Set(sys->evtHMICommand);
}


// EventRegistry statics
LinuxEvent EventRegistry::events[MAX_EVENTS];
int        EventRegistry::count = 0;
pthread_mutex_t EventRegistry::registryMutex = PTHREAD_MUTEX_INITIALIZER;

/* Thread handles */
HANDLE	hScriptStartThread;
HANDLE	hScriptHMIThread       [MAX_ROBOT_NUM];
HANDLE	hScriptPlanningThread  [MAX_ROBOT_NUM];
HANDLE	hScriptInterpThread    [MAX_ROBOT_NUM];
HANDLE	hLogThread;
static bool g_bThreadsStarted = false;

/* acontis specific */
static volatile bool g_bJobTaskRunning  = false;
static volatile bool g_bJobTaskShutdown = false;
static pthread_t     g_jobTaskThread;
static const char*   g_szENIFilename    = NULL;
static const char*   g_szLicenseKey     = "";
static EC_T_DWORD    g_dwBusCycleTimeUsec = 1000;  /* 1ms default */
static EC_T_DWORD    g_dwInstanceId     = 0;
static EC_T_DWORD    g_dwCpuIndex       = 0;
static EC_T_DCM_MODE g_eDcmMode         = eDcmMode_Off;
static bool          g_bDcmConfigure    = false;
static bool          g_bDcmLogEnabled   = true;  /* always enable DCM log for sync diagnosis */
static bool          g_bDcmSyncToCycleStart = false;
static bool          g_bDcmCsvLog       = false; /* save DCM log to CSV file */
static FILE*         g_pDcmCsvFile      = NULL;
static bool          g_bNoHmi           = false; /* no-HMI test mode */
static char          g_szLogDir[256]    = "log"; /* log output directory */

/* DCM MasterShift: cycle time adjustment for clock_nanosleep */
#define NSEC_PER_SEC 1000000000
static volatile EC_T_INT g_nCycleTimeNsec         = 1000000; /* 1ms default */
static volatile EC_T_INT g_nOriginalCycleTimeNsec = 1000000;

static EC_T_DWORD DcmAdjustCycleTime(EC_T_VOID* pvContext, EC_T_INT nAdjustPermil)
{
    (void)pvContext;
    EC_T_INT nAdjustment = (g_nOriginalCycleTimeNsec * nAdjustPermil) / 1000;
    g_nCycleTimeNsec = g_nOriginalCycleTimeNsec + nAdjustment;
    return EC_E_NOERROR;
}

static EC_T_DWORD DcmGetHostTime(EC_T_VOID* pvContext, EC_T_UINT64* pnActualHostTimeInNsec)
{
    (void)pvContext;
    return OsSystemTimeGet(pnActualHostTimeInNsec);
}

/* Vendor / product for Synapticon */
#define ECVENDOR_SYNAPTICON   0x000022D2
#define ECPRODUCT_SYNAPTICON  0x00000301

/* Timing constants */
#define ETHERCAT_STATE_CHANGE_TIMEOUT   15000
#define ETHERCAT_DC_TIMEOUT             12000
#define ETHERCAT_DC_DEV_LIMIT              13
#define ETHERCAT_DC_SETTLE_TIME          1500
#define ETHERCAT_DC_ARMW_BURSTCYCLES    10000
#define ETHERCAT_DC_ARMW_BURSTSPP          12

#define MASTER_CFG_ECAT_MAX_BUS_SLAVES    256
#define MASTER_CFG_MAX_ACYC_FRAMES_QUEUED  32
#define MASTER_CFG_MAX_ACYC_BYTES_PER_CYC 4096
#define MASTER_CFG_MAX_ACYC_CMD_RETRIES     3

#define MY_CONFIG "_acontis_linux"

/* =========================== Forward Decl =========================== */
bool OpenScriptThread();
bool OpenLogThread();
void closeThread();
void logThread(void* Context);

/* ===================== Slave Discovery ========================= */
/* Find slaves by vendor/product code on the bus, return station address */
static EC_T_BOOL FindSlaveGetFixedAddr(
    EC_T_DWORD dwSlaveInstance,
    EC_T_DWORD dwVendorId,
    EC_T_DWORD dwProductCode,
    EC_T_WORD* pwStationAddr)
{
    EC_T_DWORD dwSlaveIdx = 0;
    EC_T_DWORD dwSlaveInstanceCnt = 0;
    EC_T_DWORD dwNumSlaves = ecatGetNumConnectedSlaves();

    for (dwSlaveIdx = 0; dwSlaveIdx < dwNumSlaves; dwSlaveIdx++)
    {
        EC_T_WORD wAutoIncAddr = (EC_T_WORD)(0 - dwSlaveIdx);
        EC_T_BUS_SLAVE_INFO oBusSlaveInfo;
        OsMemset(&oBusSlaveInfo, 0, sizeof(EC_T_BUS_SLAVE_INFO));

        EC_T_DWORD dwRes = ecatGetBusSlaveInfo(EC_FALSE, wAutoIncAddr, &oBusSlaveInfo);
        if (EC_E_NOERROR != dwRes) continue;

        if ((oBusSlaveInfo.dwVendorId == dwVendorId) &&
            (oBusSlaveInfo.dwProductCode == dwProductCode))
        {
            if (dwSlaveInstanceCnt == dwSlaveInstance) {
                *pwStationAddr = oBusSlaveInfo.wStationAddress;
                return EC_TRUE;
            }
            dwSlaveInstanceCnt++;
        }
    }
    return EC_FALSE;
}

/* ===================== PDO Setup (MT_Setup pattern) ===================== */
/* Map PDO variables to direct pointers in AXIS_ECAT structs.
   This follows the motrotech.cpp MT_Setup pattern. */
static EC_T_DWORD SetupPDO()
{
    EC_T_BYTE* pbyPDIn  = ecatGetProcessImageInputPtr();
    EC_T_BYTE* pbyPDOut = ecatGetProcessImageOutputPtr();
    if (!pbyPDIn || !pbyPDOut) {
        printf("Error: Process image pointers are NULL\n");
        return EC_E_ERROR;
    }

    int motorIdx = 0;
    int ioIdx    = 0;

    /* Enumerate all configured slaves */
    EC_T_DWORD dwNumCfgSlaves = ecatGetNumConfiguredSlaves();
    printf("Configured slaves: %u\n", dwNumCfgSlaves);

    for (EC_T_DWORD dwSlaveIdx = 0; dwSlaveIdx < dwNumCfgSlaves; dwSlaveIdx++)
    {
        EC_T_CFG_SLAVE_INFO oCfgSlaveInfo;
        OsMemset(&oCfgSlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));

        EC_T_DWORD dwRes = ecatGetCfgSlaveInfo(EC_FALSE, (EC_T_WORD)(0 - dwSlaveIdx), &oCfgSlaveInfo);
        if (EC_E_NOERROR != dwRes) continue;

        EC_T_WORD wStationAddr = oCfgSlaveInfo.wStationAddress;
        EC_T_DWORD dwVendorId  = oCfgSlaveInfo.dwVendorId;
        EC_T_DWORD dwProductCode = oCfgSlaveInfo.dwProductCode;
        EC_T_WORD wNumOutpVars = oCfgSlaveInfo.wNumProcessVarsOutp;
        EC_T_WORD wNumInpVars  = oCfgSlaveInfo.wNumProcessVarsInp;

        bool isMotorSlave = false;

        /* Check if this is a motor slave (has DS402 PDOs like ControlWord 0x6040) */
        if (wNumOutpVars > 0) {
            EC_T_PROCESS_VAR_INFO_EX* pOutVars = (EC_T_PROCESS_VAR_INFO_EX*)
                OsMalloc(sizeof(EC_T_PROCESS_VAR_INFO_EX) * wNumOutpVars);
            if (pOutVars) {
                EC_T_WORD wReadEntries = 0;
                dwRes = ecatGetSlaveOutpVarInfoEx(EC_TRUE, wStationAddr, wNumOutpVars, pOutVars, &wReadEntries);
                if (EC_E_NOERROR == dwRes) {
                    printf("  Slave addr=%u  OutVars=%u:", wStationAddr, wReadEntries);
                    for (EC_T_WORD i = 0; i < wReadEntries; i++) {
                        printf(" 0x%04X(%ubits@%u)", pOutVars[i].wIndex, pOutVars[i].nBitSize, pOutVars[i].nBitOffs);
                        if (pOutVars[i].wIndex == DRV_OBJ_CONTROL_WORD) {
                            isMotorSlave = true;
                        }
                    }
                    printf("%s\n", isMotorSlave ? " [MOTOR]" : "");
                }

                if (isMotorSlave && motorIdx < MAX_MOTOR_NUM) {
                    AXIS_ECAT* pAxis = &sys->axisEcat[motorIdx];
                    pAxis->dwVendorId     = dwVendorId;
                    pAxis->dwProductCode  = dwProductCode;
                    pAxis->dwSlaveID      = ecatGetSlaveId(wStationAddr);
                    pAxis->wStationAddress = wStationAddr;

                    /* Map output PDO pointers */
                    for (EC_T_WORD i = 0; i < wReadEntries; i++) {
                        EC_T_DWORD dwBitOff = pOutVars[i].nBitOffs;
                        EC_T_WORD  wIndex   = pOutVars[i].wIndex;

                        if (wIndex == DRV_OBJ_CONTROL_WORD)
                            pAxis->pwControlWord = (EC_T_WORD*)&pbyPDOut[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_TARGET_POSITION)
                            pAxis->pnTargetPosition = (EC_T_INT*)&pbyPDOut[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_TARGET_VELOCITY)
                            pAxis->pnTargetVelocity = (EC_T_INT*)&pbyPDOut[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_TARGET_TORQUE)
                            pAxis->pwTargetTorque = (EC_T_WORD*)&pbyPDOut[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_MODES_OF_OPERATION)
                            pAxis->pbyModeOfOperation = (EC_T_BYTE*)&pbyPDOut[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_TORQUE_OFFSET)
                            pAxis->pwTorqueOffset = (EC_T_WORD*)&pbyPDOut[dwBitOff / 8];
                        else if (wIndex == 0x7010) {
                            /* Custom output - Synapticon specific */
                            if (pAxis->pwCustomOutput2 == nullptr)
                                pAxis->pwCustomOutput2 = (EC_T_WORD*)&pbyPDOut[dwBitOff / 8];
                        }
                    }
                }
                OsFree(pOutVars);
            }
        }

        /* Map input PDO pointers */
        if (isMotorSlave && wNumInpVars > 0 && motorIdx < MAX_MOTOR_NUM) {
            EC_T_PROCESS_VAR_INFO_EX* pInpVars = (EC_T_PROCESS_VAR_INFO_EX*)
                OsMalloc(sizeof(EC_T_PROCESS_VAR_INFO_EX) * wNumInpVars);
            if (pInpVars) {
                EC_T_WORD wReadEntries = 0;
                dwRes = ecatGetSlaveInpVarInfoEx(EC_TRUE, wStationAddr, wNumInpVars, pInpVars, &wReadEntries);
                if (EC_E_NOERROR == dwRes) {
                    printf("  Slave addr=%u  InpVars=%u:", wStationAddr, wReadEntries);
                    for (EC_T_WORD i = 0; i < wReadEntries; i++)
                        printf(" 0x%04X(%ubits@%u)", pInpVars[i].wIndex, pInpVars[i].nBitSize, pInpVars[i].nBitOffs);
                    printf("\n");
                    AXIS_ECAT* pAxis = &sys->axisEcat[motorIdx];
                    for (EC_T_WORD i = 0; i < wReadEntries; i++) {
                        EC_T_DWORD dwBitOff = pInpVars[i].nBitOffs;
                        EC_T_WORD  wIndex   = pInpVars[i].wIndex;

                        if (wIndex == DRV_OBJ_ERROR_CODE)
                            pAxis->pwErrorCode = (EC_T_WORD*)&pbyPDIn[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_STATUS_WORD)
                            pAxis->pwStatusWord = (EC_T_WORD*)&pbyPDIn[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_POSITION_ACTUAL_VALUE)
                            pAxis->pnActPosition = (EC_T_INT*)&pbyPDIn[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_VELOCITY_ACTUAL_VALUE)
                            pAxis->pnActVelocity = (EC_T_INT*)&pbyPDIn[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_TORQUE_ACTUAL_VALUE)
                            pAxis->pwActTorque = (EC_T_WORD*)&pbyPDIn[dwBitOff / 8];
                        else if (wIndex == DRV_OBJ_FOLLOWING_ERROR)
                            pAxis->pdwActFollowErr = (EC_T_DWORD*)&pbyPDIn[dwBitOff / 8];
                        else if (wIndex == 0x6000) {
                            /* Custom input - Synapticon specific */
                            if (pAxis->pwCustomInput1 == nullptr)
                                pAxis->pwCustomInput1 = (EC_T_WORD*)&pbyPDIn[dwBitOff / 8];
                            else if (pAxis->pwCustomInput2 == nullptr)
                                pAxis->pwCustomInput2 = (EC_T_WORD*)&pbyPDIn[dwBitOff / 8];
                        }
                    }
                }
                OsFree(pInpVars);
            }
            motorIdx++;
        }
        else if (!isMotorSlave && ioIdx < MAX_IO_NUM) {
            /* IO Slave (including bus couplers like EK1100 with no process data) */
            IO_ECAT* pIo = &sys->ioEcat[ioIdx];
            pIo->dwVendorId     = dwVendorId;
            pIo->dwProductCode  = dwProductCode;
            pIo->dwSlaveID      = ecatGetSlaveId(wStationAddr);
            pIo->wStationAddress = wStationAddr;

            /* Get IO bit lengths and offsets */
            if (wNumInpVars > 0) {
                EC_T_PROCESS_VAR_INFO_EX* pInpVars = (EC_T_PROCESS_VAR_INFO_EX*)
                    OsMalloc(sizeof(EC_T_PROCESS_VAR_INFO_EX) * wNumInpVars);
                if (pInpVars) {
                    EC_T_WORD wReadEntries = 0;
                    ecatGetSlaveInpVarInfoEx(EC_TRUE, wStationAddr, wNumInpVars, pInpVars, &wReadEntries);
                    if (wReadEntries > 0) {
                        pIo->dwInpByteOff    = pInpVars[0].nBitOffs / 8;
                        pIo->dwInpBitLength  = 0;
                        for (EC_T_WORD i = 0; i < wReadEntries; i++)
                            pIo->dwInpBitLength += pInpVars[i].nBitSize;
                        pIo->pInp = pbyPDIn;
                    }
                    OsFree(pInpVars);
                }
            }
            if (wNumOutpVars > 0) {
                EC_T_PROCESS_VAR_INFO_EX* pOutVars = (EC_T_PROCESS_VAR_INFO_EX*)
                    OsMalloc(sizeof(EC_T_PROCESS_VAR_INFO_EX) * wNumOutpVars);
                if (pOutVars) {
                    EC_T_WORD wReadEntries = 0;
                    ecatGetSlaveOutpVarInfoEx(EC_TRUE, wStationAddr, wNumOutpVars, pOutVars, &wReadEntries);
                    if (wReadEntries > 0) {
                        pIo->dwOutByteOff    = pOutVars[0].nBitOffs / 8;
                        pIo->dwOutBitLength  = 0;
                        for (EC_T_WORD i = 0; i < wReadEntries; i++)
                            pIo->dwOutBitLength += pOutVars[i].nBitSize;
                        pIo->pOut = pbyPDOut;
                    }
                    OsFree(pOutVars);
                }
            }
            ioIdx++;
        }
    }

    printf("PDO Setup complete: %d motor slaves, %d IO slaves\n", motorIdx, ioIdx);
    return EC_E_NOERROR;
}

/* =================== Notification Callback ========================= */
static EC_T_DWORD EcMasterNotifyCallback(
    EC_T_DWORD dwCode,
    EC_T_NOTIFYPARMS* pParms)
{
    /* Handle critical notifications */
    switch (dwCode) {
    case EC_NOTIFY_STATECHANGED:
        break;
    case EC_NOTIFY_ETH_LINK_CONNECTED:
        printf("EtherCAT link connected.\n");
        break;
    case EC_NOTIFY_SB_STATUS:
        break;
    default:
        break;
    }
    return EC_E_NOERROR;
}

/* =================== Cyclic Job Task ========================= */
static void* EcMasterJobTaskWrapper(void* pvArg)
{
    (void)pvArg;
    EC_T_USER_JOB_PARMS oJobParms;
    OsMemset(&oJobParms, 0, sizeof(EC_T_USER_JOB_PARMS));

    /* Set CPU affinity for real-time thread */
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(g_dwCpuIndex, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    }

    /* Use absolute clock_nanosleep for precise cycle timing (DCM compatible) */
    struct timespec tNext;
    clock_gettime(CLOCK_MONOTONIC, &tNext);

    g_bJobTaskRunning = true;

    while (!g_bJobTaskShutdown)
    {
        /* Advance to next cycle target time */
        tNext.tv_nsec += g_nCycleTimeNsec;
        while (tNext.tv_nsec >= NSEC_PER_SEC) {
            tNext.tv_nsec -= NSEC_PER_SEC;
            tNext.tv_sec++;
        }

        /* Wait for cycle (absolute timer - no drift accumulation) */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tNext, NULL);

        /* Process all received frames (read inputs) */
        ecatExecJob(eUsrJob_ProcessAllRxFrames, &oJobParms);

        /* Application work on process data */
        EC_T_STATE eMasterState = ecatGetMasterState();
        if (eMasterState == eEcatState_SAFEOP || eMasterState == eEcatState_OP)
        {
            /* This is the MyCallback equivalent */
            if (shm && shm->RTXState == eRTXState::RUNNING)
            {
                int masterState = (int)eMasterState;
                if (shm->stopRTX == (int)true) goto skip_workpd;
                if (shm->ecatState != 8) goto skip_workpd;  /* 8 = OP */

                shm->ecatState = masterState;

                sys->processMotorStatus();
                sys->ioGroup->updateData();
                sys->isServoOn();

                if (sys->updateData() < 0) {
                    shm->isSlowStop = false;
                    shm->stopScript = true;
                }

                mcChangeRTXMode();

                /* Process commands based on mode */
                static eRTXMode::e currentRTXMode = eRTXMode::CSP;
                currentRTXMode = shm->RTXMode;

                if (currentRTXMode == eRTXMode::CSP)
                {
                    if (shm->isScriptRunning) mcDoScript();
                    else                      mcDoJog();

                    for (int r = 0; r < sys->robotNum; r++)
                        sys->robots[r]->writeNextAxisForAllMotors();
                }
                else if (currentRTXMode == eRTXMode::CST)
                {
                    for (int r = 0; r < sys->robotNum; r++)
                        sys->robots[r]->writeNextTorqForAllMotors();
                }

                sys->ioGroup->setEcatOutput();
            }
        }
skip_workpd:
        /* Collect DCM log data (every cycle, if enabled) */
        if (g_bDcmCsvLog && g_pDcmCsvFile != NULL) {
            EC_T_CHAR* pszDcmLog = EC_NULL;
            ecatDcmGetLog(&pszDcmLog);
            if (EC_NULL != pszDcmLog) {
                fprintf(g_pDcmCsvFile, "%s\n", pszDcmLog);
            }
        }

        /* Send all cyclic frames (write outputs) */
        ecatExecJob(eUsrJob_SendAllCycFrames, &oJobParms);

        /* Master timer */
        ecatExecJob(eUsrJob_MasterTimer, EC_NULL);

        /* Send acyclic frames */
        ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL);
    }

    g_bJobTaskRunning = false;
    return nullptr;
}

/* ============== Shared Memory (POSIX) =================== */
bool OpenShareMemory()
{
    int shm_size = sizeof(SHMData);
    int fd = shm_open("/DataSpace", O_RDWR, 0666);
    if (fd < 0) {
        /* If not existing, create it */
        fd = shm_open("/DataSpace", O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            printf("shm_open error\n");
            return false;
        }
        if (ftruncate(fd, shm_size) < 0) {
            printf("ftruncate error\n");
            close(fd);
            return false;
        }
    }

    void* ptr = mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) {
        printf("mmap error\n");
        return false;
    }
    shm = (SHMData*)ptr;
    shm->stopRTX = false;
    shm->RTXState = eRTXState::OPEN;
    printf("Open SharedMemory. SHMData size = %lu\n", (unsigned long)sizeof(SHMData));
    return true;
}

/* ============== EtherCAT Init  =================== */
bool OpenEthercat(const char* szENIFilename, EC_T_LINK_PARMS* pLinkParms)
{
    EC_T_DWORD dwRes;

    /* 1. Init Master */
    EC_T_INIT_MASTER_PARMS oInitParms;
    OsMemset(&oInitParms, 0, sizeof(EC_T_INIT_MASTER_PARMS));
    oInitParms.dwSignature              = ATECAT_SIGNATURE;
    oInitParms.dwSize                   = sizeof(EC_T_INIT_MASTER_PARMS);
    oInitParms.pLinkParms               = pLinkParms;
    oInitParms.pLinkParmsRed            = EC_NULL;
    oInitParms.dwBusCycleTimeUsec       = g_dwBusCycleTimeUsec;
    oInitParms.dwMaxBusSlaves           = MASTER_CFG_ECAT_MAX_BUS_SLAVES;
    oInitParms.dwMaxAcycFramesQueued    = MASTER_CFG_MAX_ACYC_FRAMES_QUEUED;
    if (g_dwBusCycleTimeUsec >= 1000) {
        oInitParms.dwMaxAcycBytesPerCycle = MASTER_CFG_MAX_ACYC_BYTES_PER_CYC;
    } else {
        oInitParms.dwMaxAcycBytesPerCycle = 1500;
        oInitParms.dwMaxAcycFramesPerCycle = 1;
        oInitParms.dwMaxAcycCmdsPerCycle   = 20;
    }
    oInitParms.dwEcatCmdMaxRetries      = MASTER_CFG_MAX_ACYC_CMD_RETRIES;

    dwRes = ecatInitMaster(&oInitParms);
    if (EC_E_NOERROR != dwRes) {
        printf("ecatInitMaster error: 0x%lx\n", (unsigned long)dwRes);
        return false;
    }

    /* 2. License key */
    if (strlen(g_szLicenseKey) > 0) {
        dwRes = ecatSetLicenseKey(g_szLicenseKey);
        if (EC_E_NOERROR != dwRes)
            printf("ecatSetLicenseKey warning: 0x%lx (eval mode)\n", (unsigned long)dwRes);
    }

    /* 3. Configure network (load ENI) */
    dwRes = ecatConfigureNetwork(eCnfType_Filename, (EC_T_BYTE*)szENIFilename, (EC_T_DWORD)strlen(szENIFilename));
    if (EC_E_NOERROR != dwRes) {
        printf("ecatConfigureNetwork error: 0x%lx (ENI: %s)\n", (unsigned long)dwRes, szENIFilename);
        return false;
    }
    printf("ENI file loaded: %s\n", szENIFilename);

    /* 4. Register client */
    EC_T_REGISTERRESULTS oRegResults;
    OsMemset(&oRegResults, 0, sizeof(EC_T_REGISTERRESULTS));
    dwRes = ecatRegisterClient(EcMasterNotifyCallback, EC_NULL, &oRegResults);
    if (EC_E_NOERROR != dwRes) {
        printf("ecatRegisterClient error: 0x%lx\n", (unsigned long)dwRes);
        return false;
    }

    /* 5. DC configuration */
    EC_T_DC_CONFIGURE oDcCfg;
    OsMemset(&oDcCfg, 0, sizeof(EC_T_DC_CONFIGURE));
    oDcCfg.dwTimeout          = ETHERCAT_DC_TIMEOUT;
    oDcCfg.dwDevLimit         = ETHERCAT_DC_DEV_LIMIT;
    oDcCfg.dwSettleTime       = ETHERCAT_DC_SETTLE_TIME;
    oDcCfg.dwTotalBurstLength = ETHERCAT_DC_ARMW_BURSTCYCLES;
    if (g_dwBusCycleTimeUsec < 1000)
        oDcCfg.dwBurstBulk    = ETHERCAT_DC_ARMW_BURSTSPP / 2;
    else
        oDcCfg.dwBurstBulk    = ETHERCAT_DC_ARMW_BURSTSPP;
    oDcCfg.bAcycDistributionDisabled = EC_TRUE;

    dwRes = ecatDcConfigure(&oDcCfg);
    if (EC_E_NOERROR != dwRes)
        printf("ecatDcConfigure warning: 0x%lx\n", (unsigned long)dwRes);

    /* 6. DCM configuration */
    if (g_bDcmLogEnabled && !g_bDcmConfigure) {
        /* Auto-detect BusShift from ENI */
        EC_T_BOOL bBusShiftConfiguredByEni = EC_FALSE;
        dwRes = ecatDcmGetBusShiftConfigured(&bBusShiftConfiguredByEni);
        if (EC_E_NOERROR == dwRes && bBusShiftConfiguredByEni) {
            g_bDcmConfigure = true;
            g_eDcmMode = eDcmMode_BusShift;
            printf("DCM: BusShift detected from ENI, auto-configuring\n");
        }
    }
    if (g_bDcmConfigure) {
        EC_T_DWORD dwCycleTimeNsec   = g_dwBusCycleTimeUsec * 1000;
        EC_T_INT   nCtlSetValNsec    = (EC_T_INT)(dwCycleTimeNsec * 2 / 3);  /* 66% */
        EC_T_DWORD dwInSyncLimitNsec = dwCycleTimeNsec / 5;                  /* 20% */

        EC_T_DCM_CONFIG oDcmConfig;
        OsMemset(&oDcmConfig, 0, sizeof(EC_T_DCM_CONFIG));

        switch (g_eDcmMode) {
        case eDcmMode_Off:
            oDcmConfig.eMode = eDcmMode_Off;
            break;
        case eDcmMode_BusShift:
            oDcmConfig.eMode = eDcmMode_BusShift;
            oDcmConfig.u.BusShift.nCtlSetVal    = nCtlSetValNsec;
            oDcmConfig.u.BusShift.dwInSyncLimit  = dwInSyncLimitNsec;
            oDcmConfig.u.BusShift.bLogEnabled    = g_bDcmLogEnabled ? EC_TRUE : EC_FALSE;
            if (g_bDcmSyncToCycleStart)
                oDcmConfig.u.BusShift.pfnGetTimeElapsedSinceCycleStart = EC_NULL;
            break;
        case eDcmMode_MasterShift:
            oDcmConfig.eMode = eDcmMode_MasterShift;
            oDcmConfig.u.MasterShift.nCtlSetVal    = nCtlSetValNsec;
            oDcmConfig.u.MasterShift.dwInSyncLimit  = dwInSyncLimitNsec;
            oDcmConfig.u.MasterShift.bLogEnabled    = g_bDcmLogEnabled ? EC_TRUE : EC_FALSE;
            if (g_bDcmSyncToCycleStart)
                oDcmConfig.u.MasterShift.pfnGetTimeElapsedSinceCycleStart = EC_NULL;
            oDcmConfig.u.MasterShift.pAdjustCycleTimeContext = EC_NULL;
            oDcmConfig.u.MasterShift.pfnAdjustCycleTime = DcmAdjustCycleTime;
            break;
        case eDcmMode_MasterRefClock:
            oDcmConfig.eMode = eDcmMode_MasterRefClock;
            oDcmConfig.u.MasterRefClock.nCtlSetVal    = nCtlSetValNsec;
            oDcmConfig.u.MasterRefClock.dwInSyncLimit  = dwInSyncLimitNsec;
            oDcmConfig.u.MasterRefClock.bLogEnabled    = g_bDcmLogEnabled ? EC_TRUE : EC_FALSE;
            oDcmConfig.u.MasterRefClock.pGetHostTimeContext = EC_NULL;
            oDcmConfig.u.MasterRefClock.pfnGetHostTime = DcmGetHostTime;
            break;
        case eDcmMode_LinkLayerRefClock:
            oDcmConfig.eMode = eDcmMode_LinkLayerRefClock;
            oDcmConfig.u.LinkLayerRefClock.nCtlSetVal    = nCtlSetValNsec;
            oDcmConfig.u.LinkLayerRefClock.dwInSyncLimit  = dwInSyncLimitNsec;
            oDcmConfig.u.LinkLayerRefClock.bLogEnabled    = g_bDcmLogEnabled ? EC_TRUE : EC_FALSE;
            break;
        case eDcmMode_Dcx:
            oDcmConfig.eMode = eDcmMode_Dcx;
            /* MasterShift part */
            oDcmConfig.u.Dcx.MasterShift.nCtlSetVal    = nCtlSetValNsec;
            oDcmConfig.u.Dcx.MasterShift.dwInSyncLimit  = dwInSyncLimitNsec;
            oDcmConfig.u.Dcx.MasterShift.bLogEnabled    = g_bDcmLogEnabled ? EC_TRUE : EC_FALSE;
            if (g_bDcmSyncToCycleStart)
                oDcmConfig.u.Dcx.MasterShift.pfnGetTimeElapsedSinceCycleStart = EC_NULL;
            oDcmConfig.u.Dcx.MasterShift.pAdjustCycleTimeContext = EC_NULL;
            oDcmConfig.u.Dcx.MasterShift.pfnAdjustCycleTime = DcmAdjustCycleTime;
            /* DCX BusShift part */
            oDcmConfig.u.Dcx.nCtlSetVal    = nCtlSetValNsec;
            oDcmConfig.u.Dcx.dwInSyncLimit  = dwInSyncLimitNsec;
            oDcmConfig.u.Dcx.bLogEnabled    = g_bDcmLogEnabled ? EC_TRUE : EC_FALSE;
            oDcmConfig.u.Dcx.dwExtClockTimeout  = 1000;
            oDcmConfig.u.Dcx.wExtClockFixedAddr = 0;
            break;
        default:
            printf("DCM mode %d not supported\n", (int)g_eDcmMode);
            break;
        }

        dwRes = ecatDcmConfigure(&oDcmConfig, 0);
        switch (dwRes) {
        case EC_E_NOERROR:
            printf("DCM configured: mode=%d, CtlSetVal=%d ns, InSyncLimit=%u ns, Log=%s\n",
                (int)g_eDcmMode, nCtlSetValNsec, dwInSyncLimitNsec,
                g_bDcmLogEnabled ? "ON" : "OFF");
            break;
        case EC_E_FEATURE_DISABLED:
            printf("DCM: feature disabled! Start with -dcmmode off, or configure ENI for DC\n");
            printf("  (In EcEngineer: Advanced settings > Distributed clocks > DC in use + Slave Mode)\n");
            /* Non-fatal: continue without DCM */
            break;
        default:
            printf("ecatDcmConfigure error: 0x%lx\n", (unsigned long)dwRes);
            /* Non-fatal: continue without DCM */
            break;
        }
    }

    return true;
}

/* ============== Slave Detection =================== */
bool DetectSlave()
{
    EC_T_DWORD dwRes;

    /* Transition to INIT */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);
    if (EC_E_NOERROR != dwRes)
        printf("Warning: SetMasterState INIT: 0x%lx\n", (unsigned long)dwRes);
    printf("Master state: INIT\n");

    /* Transition to PREOP (slave discovery happens here) */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_PREOP);
    if (EC_E_NOERROR != dwRes) {
        printf("ecatSetMasterState PREOP error: 0x%lx\n", (unsigned long)dwRes);
        return false;
    }
    printf("Master state: PREOP\n");

    /* Count motor and IO slaves on the bus */
    int motorCount = 0;
    int ioCount    = 0;
    EC_T_DWORD dwNumCfgSlaves = ecatGetNumConfiguredSlaves();
    printf("Number of configured slaves: %u\n", dwNumCfgSlaves);

    for (EC_T_DWORD i = 0; i < dwNumCfgSlaves; i++) {
        EC_T_CFG_SLAVE_INFO oCfgInfo;
        OsMemset(&oCfgInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));
        dwRes = ecatGetCfgSlaveInfo(EC_FALSE, (EC_T_WORD)(0 - i), &oCfgInfo);
        if (EC_E_NOERROR != dwRes) continue;

        /* Determine if motor or IO by checking for DS402 output PDOs */
        bool hasDS402 = false;
        if (oCfgInfo.wNumProcessVarsOutp > 0) {
            EC_T_PROCESS_VAR_INFO_EX* pVars = (EC_T_PROCESS_VAR_INFO_EX*)
                OsMalloc(sizeof(EC_T_PROCESS_VAR_INFO_EX) * oCfgInfo.wNumProcessVarsOutp);
            if (pVars) {
                EC_T_WORD wRead = 0;
                ecatGetSlaveOutpVarInfoEx(EC_TRUE, oCfgInfo.wStationAddress,
                    oCfgInfo.wNumProcessVarsOutp, pVars, &wRead);
                for (EC_T_WORD v = 0; v < wRead; v++) {
                    if (pVars[v].wIndex == DRV_OBJ_CONTROL_WORD) {
                        hasDS402 = true;
                        break;
                    }
                }
                OsFree(pVars);
            }
        }
        if (hasDS402) motorCount++;
        else ioCount++;

        printf("  Slave[%u] addr=%u vendor=0x%08X product=0x%08X %s\n",
            i, oCfgInfo.wStationAddress, oCfgInfo.dwVendorId, oCfgInfo.dwProductCode,
            hasDS402 ? "(motor)" : "(IO)");
    }

    int slaveNums[3] = { (int)dwNumCfgSlaves, ioCount, motorCount };
    sys->detectEcatSlave(slaveNums);

    /* PDO Setup (map process image pointers) */
    dwRes = SetupPDO();
    if (EC_E_NOERROR != dwRes) {
        printf("SetupPDO error: 0x%lx\n", (unsigned long)dwRes);
        return false;
    }

    sys->initIO();

    /* Notify HMI: slave info available */
    EventHandler::Set(sys->evtGotSlaveInfo);

    if (g_bNoHmi) {
        printf("[nohmi] Skipping HMI wait. Slave detection done.\n");
    } else {
        /* Wait for HMI to complete mapping */
        printf("Waiting for HMI mapping command...\n");
        EventHandler::WaitFor(sys->evtHMICommand, INFINITE);
    }

    if (shm->stopRTX) return false;
    return true;
}

bool TransitionToOP()
{
    EC_T_DWORD dwRes;

    /* Transition to SAFEOP */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_SAFEOP);
    if (EC_E_NOERROR != dwRes) {
        printf("ecatSetMasterState SAFEOP error: 0x%lx\n", (unsigned long)dwRes);
        return false;
    }
    if (g_bDcmConfigure)
        printf("DC Synchronized\n");
    printf("Master state: SAFEOP\n");

    /* Transition to OP */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_OP);
    if (EC_E_NOERROR != dwRes) {
        printf("ecatSetMasterState OP error: 0x%lx\n", (unsigned long)dwRes);
        return false;
    }
    printf("Master state: OP\n");
    shm->ecatState = 8; /* 8 = OP */

    return true;
}

bool StartJobTask()
{
    g_bJobTaskShutdown = false;
    int ret = pthread_create(&g_jobTaskThread, NULL, EcMasterJobTaskWrapper, NULL);
    if (ret != 0) {
        printf("pthread_create(JobTask) error: %d\n", ret);
        return false;
    }

    /* Set real-time priority if possible */
    struct sched_param param;
    param.sched_priority = 80;
    pthread_setschedparam(g_jobTaskThread, SCHED_FIFO, &param);

    return true;
}

void StopJobTask()
{
    g_bJobTaskShutdown = true;
    if (g_bJobTaskRunning) {
        pthread_join(g_jobTaskThread, NULL);
    }
}

void waitForHMI()
{
    EventHandler::WaitFor(sys->evtHMICommand, INFINITE);
}

/* ===================== Main  ========================= */
int main(int argc, char *argv[])
{
    printf("\n====== ver.2025-0516%s =======\n", MY_CONFIG);

    /* Parse arguments */
    const char* szENIFile = NULL;
    const char* szLinkLayer = "intelgbe"; /* link layer type */
    const char* szLinkArg1 = "1";  /* instance or device name */
    int linkMode = 1; /* 0=interrupt, 1=polling */
    char szDcmCsvPath[512] = "";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            szENIFile = argv[++i];
        } else if (strcmp(argv[i], "-intelgbe") == 0 && i + 2 < argc) {
            szLinkLayer = "intelgbe";
            szLinkArg1 = argv[++i];
            linkMode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i8254x") == 0 && i + 2 < argc) {
            szLinkLayer = "i8254x";
            szLinkArg1 = argv[++i];
            linkMode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i8255x") == 0 && i + 2 < argc) {
            szLinkLayer = "i8255x";
            szLinkArg1 = argv[++i];
            linkMode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-rtl8169") == 0 && i + 2 < argc) {
            szLinkLayer = "rtl8169";
            szLinkArg1 = argv[++i];
            linkMode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-sockraw") == 0 && i + 2 < argc) {
            szLinkLayer = "sockraw";
            szLinkArg1 = argv[++i]; /* device name, e.g. eth0 */
            linkMode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            g_dwBusCycleTimeUsec = (EC_T_DWORD)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            g_szLicenseKey = argv[++i];
        } else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
            g_dwCpuIndex = (EC_T_DWORD)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-dcmmode") == 0 && i + 1 < argc) {
            i++;
            g_bDcmConfigure = true;
            if (strcmp(argv[i], "off") == 0)
                g_eDcmMode = eDcmMode_Off;
            else if (strcmp(argv[i], "busshift") == 0)
                g_eDcmMode = eDcmMode_BusShift;
            else if (strcmp(argv[i], "mastershift") == 0)
                g_eDcmMode = eDcmMode_MasterShift;
            else if (strcmp(argv[i], "masterrefclock") == 0)
                g_eDcmMode = eDcmMode_MasterRefClock;
            else if (strcmp(argv[i], "linklayerrefclock") == 0)
                g_eDcmMode = eDcmMode_LinkLayerRefClock;
            else if (strcmp(argv[i], "dcx") == 0)
                g_eDcmMode = eDcmMode_Dcx;
            else {
                printf("Unknown DCM mode: %s\n", argv[i]);
                g_bDcmConfigure = false;
            }
            /* Optional: synctocyclestart parameter (0 or 1) */
            if (i + 1 < argc && argv[i+1][0] >= '0' && argv[i+1][0] <= '1' && argv[i+1][1] == '\0') {
                i++;
                g_bDcmSyncToCycleStart = (argv[i][0] == '1');
            }
        } else if (strcmp(argv[i], "-dcmlog") == 0) {
            g_bDcmCsvLog = true;
            if (i + 1 < argc && argv[i+1][0] != '-') {
                snprintf(szDcmCsvPath, sizeof(szDcmCsvPath), "%s/%s", g_szLogDir, argv[++i]);
            }
        } else if (strcmp(argv[i], "-nohmi") == 0) {
            g_bNoHmi = true;
        }
    }

    /* Sync cycle time globals */
    g_nCycleTimeNsec         = (EC_T_INT)(g_dwBusCycleTimeUsec * 1000);
    g_nOriginalCycleTimeNsec = g_nCycleTimeNsec;

    if (!szENIFile) {
        printf("Usage: %s -f <ENI file> [options]\n", argv[0]);
        printf("  -b <usec>           Bus cycle time in usec (default: 1000, must match ENI DC CycleTime)\n");
        printf("  -a <cpu>            CPU affinity (default: 0)\n");
        printf("  -k <key>            License key\n");
        printf("  Link layer (choose one, default: -intelgbe 1 1):\n");
        printf("  -intelgbe <inst> <mode>  Intel GbE (i210/i350/i225)\n");
        printf("  -i8254x   <inst> <mode>  Intel PRO/1000\n");
        printf("  -i8255x   <inst> <mode>  Intel PRO/100\n");
        printf("  -rtl8169  <inst> <mode>  Realtek RTL8169/8168/8111\n");
        printf("  -sockraw  <dev>  <mode>  Linux raw socket (any NIC, e.g. eth0)\n");
        printf("     <inst>  = NIC instance (1=first, 2=second, ...)\n");
        printf("     <mode>  = 0: interrupt, 1: polling\n");
        printf("  -dcmmode <mode> [synctocyclestart]\n");
        printf("     off              DCM off (default)\n");
        printf("     busshift         BusShift mode (default if configured in ENI)\n");
        printf("     mastershift      MasterShift mode\n");
        printf("     masterrefclock   MasterRefClock mode\n");
        printf("     linklayerrefclock LinkLayerRefClock mode\n");
        printf("     dcx              External synchronization mode\n");
        printf("     [0|1]            Sync to cycle start (0=disabled, 1=enabled)\n");
        printf("  -dcmlog [filepath]  Save DCM log to CSV file (default: ecmaster_dcm.csv)\n");
        printf("  -nohmi              No-HMI test mode: verify EtherCAT connection only\n");
        printf("Example: %s -f eni.xml -intelgbe 3 1 -b 1000 -a 3 -dcmmode mastershift 1 -dcmlog\n", argv[0]);
        return 1;
    }

    /* Create log directory */
    if (mkdir(g_szLogDir, 0755) == 0)
        printf("Log directory created: %s\n", g_szLogDir);
    else if (errno == EEXIST)
        printf("Log directory: %s\n", g_szLogDir);
    else
        printf("Warning: cannot create log directory '%s': %s\n", g_szLogDir, strerror(errno));

    EventRegistry::Init();
    signal(SIGINT, sigintHandler);
    sys = new MySystem();
    Eigen::initParallel();

    /* 1. Open shared memory */
    if (!OpenShareMemory()) goto Exit3;

    /* 2. Build link layer parameters */
    {
        EC_T_LINK_PARMS* pBaseLinkParms = EC_NULL;
        EC_T_LINKMODE eLinkMode = (linkMode == 0) ? EcLinkMode_INTERRUPT : EcLinkMode_POLLING;

        if (strcmp(szLinkLayer, "sockraw") == 0) {
            /* SockRaw: uses device name (eth0, enp3s0, etc.) */
            EC_T_LINK_PARMS_SOCKRAW* pL = (EC_T_LINK_PARMS_SOCKRAW*)OsMalloc(sizeof(EC_T_LINK_PARMS_SOCKRAW));
            if (!pL) { printf("OsMalloc link parms failed\n"); goto Exit3; }
            OsMemset(pL, 0, sizeof(EC_T_LINK_PARMS_SOCKRAW));
            pL->linkParms.dwSignature = EC_LINK_PARMS_SIGNATURE_SOCKRAW;
            pL->linkParms.dwSize      = sizeof(EC_T_LINK_PARMS_SOCKRAW);
            snprintf(pL->linkParms.szDriverIdent, MAX_DRIVER_IDENT_LEN, "%s", EC_LINK_PARMS_IDENT_SOCKRAW);
            pL->linkParms.dwInstance  = 1;
            pL->linkParms.eLinkMode   = eLinkMode;
            snprintf(pL->szAdapterName, EC_SOCKRAW_ADAPTER_NAME_SIZE, "%s", szLinkArg1);
            pBaseLinkParms = &pL->linkParms;
            printf("Link layer: SockRaw  dev=%s  mode=%s\n", szLinkArg1,
                (eLinkMode == EcLinkMode_POLLING) ? "POLLING" : "INTERRUPT");
        }
        else if (strcmp(szLinkLayer, "i8254x") == 0) {
            EC_T_LINK_PARMS_I8254X* pL = (EC_T_LINK_PARMS_I8254X*)OsMalloc(sizeof(EC_T_LINK_PARMS_I8254X));
            if (!pL) { printf("OsMalloc link parms failed\n"); goto Exit3; }
            OsMemset(pL, 0, sizeof(EC_T_LINK_PARMS_I8254X));
            pL->linkParms.dwSignature = EC_LINK_PARMS_SIGNATURE_I8254X;
            pL->linkParms.dwSize      = sizeof(EC_T_LINK_PARMS_I8254X);
            snprintf(pL->linkParms.szDriverIdent, MAX_DRIVER_IDENT_LEN, "%s", EC_LINK_PARMS_IDENT_I8254X);
            pL->linkParms.dwInstance  = (EC_T_DWORD)atoi(szLinkArg1);
            pL->linkParms.eLinkMode   = eLinkMode;
            pL->wRxBufferCnt          = 96;
            pBaseLinkParms = &pL->linkParms;
            printf("Link layer: I8254x  instance=%s  mode=%s\n", szLinkArg1,
                (eLinkMode == EcLinkMode_POLLING) ? "POLLING" : "INTERRUPT");
        }
        else if (strcmp(szLinkLayer, "i8255x") == 0) {
            EC_T_LINK_PARMS_I8255X* pL = (EC_T_LINK_PARMS_I8255X*)OsMalloc(sizeof(EC_T_LINK_PARMS_I8255X));
            if (!pL) { printf("OsMalloc link parms failed\n"); goto Exit3; }
            OsMemset(pL, 0, sizeof(EC_T_LINK_PARMS_I8255X));
            pL->linkParms.dwSignature = EC_LINK_PARMS_SIGNATURE_I8255X;
            pL->linkParms.dwSize      = sizeof(EC_T_LINK_PARMS_I8255X);
            snprintf(pL->linkParms.szDriverIdent, MAX_DRIVER_IDENT_LEN, "%s", EC_LINK_PARMS_IDENT_I8255X);
            pL->linkParms.dwInstance  = (EC_T_DWORD)atoi(szLinkArg1);
            pL->linkParms.eLinkMode   = eLinkMode;
            pBaseLinkParms = &pL->linkParms;
            printf("Link layer: I8255x  instance=%s  mode=%s\n", szLinkArg1,
                (eLinkMode == EcLinkMode_POLLING) ? "POLLING" : "INTERRUPT");
        }
        else if (strcmp(szLinkLayer, "rtl8169") == 0) {
            EC_T_LINK_PARMS_RTL8169* pL = (EC_T_LINK_PARMS_RTL8169*)OsMalloc(sizeof(EC_T_LINK_PARMS_RTL8169));
            if (!pL) { printf("OsMalloc link parms failed\n"); goto Exit3; }
            OsMemset(pL, 0, sizeof(EC_T_LINK_PARMS_RTL8169));
            pL->linkParms.dwSignature = EC_LINK_PARMS_SIGNATURE_RTL8169;
            pL->linkParms.dwSize      = sizeof(EC_T_LINK_PARMS_RTL8169);
            snprintf(pL->linkParms.szDriverIdent, MAX_DRIVER_IDENT_LEN, "%s", EC_LINK_PARMS_IDENT_RTL8169);
            pL->linkParms.dwInstance  = (EC_T_DWORD)atoi(szLinkArg1);
            pL->linkParms.eLinkMode   = eLinkMode;
            pBaseLinkParms = &pL->linkParms;
            printf("Link layer: RTL8169  instance=%s  mode=%s\n", szLinkArg1,
                (eLinkMode == EcLinkMode_POLLING) ? "POLLING" : "INTERRUPT");
        }
        else {
            /* Default: IntelGbe */
            EC_T_LINK_PARMS_INTELGBE* pL = (EC_T_LINK_PARMS_INTELGBE*)OsMalloc(sizeof(EC_T_LINK_PARMS_INTELGBE));
            if (!pL) { printf("OsMalloc link parms failed\n"); goto Exit3; }
            OsMemset(pL, 0, sizeof(EC_T_LINK_PARMS_INTELGBE));
            pL->linkParms.dwSignature = EC_LINK_PARMS_SIGNATURE_INTELGBE;
            pL->linkParms.dwSize      = sizeof(EC_T_LINK_PARMS_INTELGBE);
            snprintf(pL->linkParms.szDriverIdent, MAX_DRIVER_IDENT_LEN, "%s", EC_LINK_PARMS_IDENT_INTELGBE);
            pL->linkParms.dwInstance  = (EC_T_DWORD)atoi(szLinkArg1);
            pL->linkParms.eLinkMode   = eLinkMode;
            pL->wRxBufferCnt          = 96;
            pBaseLinkParms = &pL->linkParms;
            printf("Link layer: IntelGbe  instance=%s  mode=%s\n", szLinkArg1,
                (eLinkMode == EcLinkMode_POLLING) ? "POLLING" : "INTERRUPT");
        }

        /* 3. Init EtherCAT Master */
        if (!OpenEthercat(szENIFile, pBaseLinkParms)) goto Exit2;
    }

    /* 3.5 Start cyclic job task BEFORE state transitions (required by acontis) */
    if (!StartJobTask()) goto Exit2;
    Sleep(100); /* let job task stabilize */

    /* 4. Detect slaves & setup PDO */
    if (!DetectSlave()) goto Exit2;

    if (g_bNoHmi) {
        /* ---- No-HMI test mode: verify EtherCAT connection only ---- */
        /* Skip initRobots / linkToEcat (no mapping table from HMI) */

        /* Transition to OP */
        if (!TransitionToOP()) goto Exit2;

        /* Open DCM CSV log file */
        if (g_bDcmCsvLog) {
            if (szDcmCsvPath[0] == '\0')
                snprintf(szDcmCsvPath, sizeof(szDcmCsvPath), "%s/ecmaster_dcm.csv", g_szLogDir);
            g_pDcmCsvFile = fopen(szDcmCsvPath, "w");
            if (g_pDcmCsvFile) {
                /* Header will be written by ecatDcmGetLog on first call */
                printf("DCM CSV log enabled: %s\n", szDcmCsvPath);
            } else {
                printf("Warning: cannot create DCM CSV log file: %s\n", szDcmCsvPath);
                g_bDcmCsvLog = false;
            }
        }

        printf("\n");
        printf("============================================\n");
        printf("  [nohmi] EtherCAT Connection Test Mode\n");
        printf("============================================\n");
        printf("  Master state : OP\n");
        printf("  Configured slaves : %u\n", ecatGetNumConfiguredSlaves());
        printf("  Connected  slaves : %u\n", ecatGetNumConnectedSlaves());
        printf("  Bus cycle         : %u usec\n", g_dwBusCycleTimeUsec);
        printf("============================================\n");

        /* Read each slave's actual state */
        EC_T_DWORD dwNumSlaves = ecatGetNumConfiguredSlaves();
        for (EC_T_DWORD si = 0; si < dwNumSlaves; si++) {
            EC_T_CFG_SLAVE_INFO oCfg;
            OsMemset(&oCfg, 0, sizeof(oCfg));
            EC_T_DWORD dwR = ecatGetCfgSlaveInfo(EC_FALSE, (EC_T_WORD)(0 - si), &oCfg);
            if (EC_E_NOERROR == dwR) {
                const char* szPresent = oCfg.bIsPresent ? "ONLINE" : "OFFLINE";
                printf("  Slave[%u] addr=%u  %-7s  vendor=0x%08X  product=0x%08X  \"%s\"\n",
                    si, oCfg.wStationAddress, szPresent,
                    oCfg.dwVendorId, oCfg.dwProductCode, oCfg.abyDeviceName);
            }
        }

        if (ecatGetNumConfiguredSlaves() == ecatGetNumConnectedSlaves() && ecatGetNumConnectedSlaves() > 0)
            printf("\n  >>> RESULT: ALL SLAVES CONNECTED - EtherCAT OK <<<\n\n");
        else
            printf("\n  >>> RESULT: SLAVE MISMATCH (cfg=%u conn=%u) - CHECK WIRING <<<\n\n",
                ecatGetNumConfiguredSlaves(), ecatGetNumConnectedSlaves());

        /* ---- PDO Verification ---- */
        printf("============ PDO Verification =============\n");
        for (int m = 0; m < sys->slaveMotorNum; m++) {
            AXIS_ECAT* pA = &sys->axisEcat[m];
            printf("  Motor[%d] addr=%u  vendor=0x%08X  product=0x%08X\n",
                m, pA->wStationAddress, pA->dwVendorId, pA->dwProductCode);
            printf("    OUT -> ControlWord:%s  TargetPos:%s  TargetVel:%s  TargetTrq:%s  ModeOp:%s\n",
                pA->pwControlWord   ? "OK" : "--",
                pA->pnTargetPosition? "OK" : "--",
                pA->pnTargetVelocity? "OK" : "--",
                pA->pwTargetTorque  ? "OK" : "--",
                pA->pbyModeOfOperation ? "OK" : "--");
            printf("    INP <- StatusWord:%s  ActPos:%s  ActVel:%s  ActTrq:%s  ErrCode:%s\n",
                pA->pwStatusWord    ? "OK" : "--",
                pA->pnActPosition   ? "OK" : "--",
                pA->pnActVelocity   ? "OK" : "--",
                pA->pwActTorque     ? "OK" : "--",
                pA->pwErrorCode     ? "OK" : "--");
            /* Show live values */
            printf("    VAL -> StatusWord=0x%04X  ActPos=%d  ActVel=%d  ActTrq=%d\n",
                pA->pwStatusWord  ? *pA->pwStatusWord  : 0,
                pA->pnActPosition ? *pA->pnActPosition : 0,
                pA->pnActVelocity ? *pA->pnActVelocity : 0,
                pA->pwActTorque   ? *pA->pwActTorque   : 0);
        }
        for (int io = 0; io < sys->slaveIONum; io++) {
            IO_ECAT* pIo = &sys->ioEcat[io];
            printf("  IO[%d] addr=%u  vendor=0x%08X  product=0x%08X\n",
                io, pIo->wStationAddress, pIo->dwVendorId, pIo->dwProductCode);
            printf("    INP bits=%u  OUT bits=%u\n", pIo->dwInpBitLength, pIo->dwOutBitLength);
            if (pIo->pInp && pIo->dwInpBitLength > 0) {
                EC_T_DWORD dwBytes = (pIo->dwInpBitLength + 7) / 8;
                printf("    INP data:");
                for (EC_T_DWORD b = 0; b < dwBytes && b < 8; b++)
                    printf(" %02X", pIo->pInp[pIo->dwInpByteOff + b]);
                printf("\n");
            }
            if (pIo->pOut && pIo->dwOutBitLength > 0) {
                EC_T_DWORD dwBytes = (pIo->dwOutBitLength + 7) / 8;
                printf("    OUT data:");
                for (EC_T_DWORD b = 0; b < dwBytes && b < 8; b++)
                    printf(" %02X", pIo->pOut[pIo->dwOutByteOff + b]);
                printf("\n");
            }
        }
        printf("============================================\n\n");

        printf("Press Ctrl+C to exit...\n");

        /* Simple monitoring loop — print state every 1 second */
        {
            while (!shm->stopRTX) {
                    EC_T_STATE eState = ecatGetMasterState();
                    const char* szMs = "??";
                    if (eState == eEcatState_INIT)   szMs = "INIT";
                    else if (eState == eEcatState_PREOP)  szMs = "PREOP";
                    else if (eState == eEcatState_SAFEOP) szMs = "SAFEOP";
                    else if (eState == eEcatState_OP)     szMs = "OP";
                    printf("[nohmi] Master=%s  connected=%u/%u\n",
                        szMs, ecatGetNumConnectedSlaves(), ecatGetNumConfiguredSlaves());
                    /* Print live PDO values */
                    for (int m = 0; m < sys->slaveMotorNum; m++) {
                        AXIS_ECAT* pA = &sys->axisEcat[m];
                        printf("  Motor[%d] SW=0x%04X Pos=%-10d Vel=%-8d Trq=%-6d\n",
                            m,
                            pA->pwStatusWord  ? *pA->pwStatusWord  : 0,
                            pA->pnActPosition ? *pA->pnActPosition : 0,
                            pA->pnActVelocity ? *pA->pnActVelocity : 0,
                            pA->pwActTorque   ? *pA->pwActTorque   : 0);
                    }
                
                Sleep(1000);
            }
        }
    } else {
    /* ---- Normal mode (with HMI) ---- */

    /* 5. Init robots */
    if (!sys->initRobots()) goto Exit2;

    /* 6. Link motors to ECAT */
    if (!sys->linkToEcat()) goto Exit2;

    /* 7. Transition to OP */
    if (!TransitionToOP()) goto Exit2;

    /* 8. Open DCM CSV log file (after DCM configured, before job task starts) */
    if (g_bDcmCsvLog) {
        if (szDcmCsvPath[0] == '\0')
            snprintf(szDcmCsvPath, sizeof(szDcmCsvPath), "%s/ecmaster_dcm.csv", g_szLogDir);
        g_pDcmCsvFile = fopen(szDcmCsvPath, "w");
        if (g_pDcmCsvFile) {
            /* Header will be written by ecatDcmGetLog on first call */
            printf("DCM CSV log enabled: %s\n", szDcmCsvPath);
        } else {
            printf("Warning: cannot create DCM CSV log file: %s\n", szDcmCsvPath);
            g_bDcmCsvLog = false;
        }
    }

    /* 9. Open threads */
    if (!OpenLogThread())    goto Exit;
    if (!OpenScriptThread()) goto Exit;
    g_bThreadsStarted = true;

    shm->RTXState = eRTXState::RUNNING;

    Sleep(100);
    Debug::writeln(0, "======== EtherCAT Master initialize successed. ==========\n");

    /* Open log files in log directory */
    Debug::openFile(2, g_szLogDir);
    Debug::openFile(3, g_szLogDir);
    Debug::openFile(4, g_szLogDir);

    /* Main loop: monitor servo on/off commands from HMI + DCM status */
    {
        bool bFirstDcmStatus = true;
        EC_T_DWORD dwDcmStatusTimer = 0;

        while (!shm->stopRTX)
        {
            sys->StartServoOnOff();

            /* DCM status monitoring (every 5 seconds) */
            if (g_bDcmConfigure && g_eDcmMode != eDcmMode_Off && g_eDcmMode != eDcmMode_LinkLayerRefClock)
            {
                dwDcmStatusTimer += 200;
                if (dwDcmStatusTimer >= 5000) {
                    dwDcmStatusTimer = 0;

                    EC_T_DWORD dwDcmStatus = 0;
                    EC_T_INT   nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;
                    EC_T_DWORD dwDcmRes = ecatDcmGetStatus(&dwDcmStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
                    if (EC_E_NOERROR == dwDcmRes) {
                        if (bFirstDcmStatus) {
                            printf("DCM first status after startup\\n");
                            ecatDcmResetStatus();
                            bFirstDcmStatus = false;
                        }
                        if (dwDcmStatus != EC_E_NOERROR && dwDcmStatus != EC_E_NOTREADY && dwDcmStatus != EC_E_BUSY) {
                            printf("DCM Status: 0x%08X\\n", dwDcmStatus);
                        }
                        if (g_bDcmLogEnabled) {
                            printf("DCM Diff [nsec] cur/avg/max: %7d / %7d / %7d  %s\\n",
                                nDiffCur, nDiffAvg, nDiffMax,
                                (dwDcmStatus == EC_E_NOERROR) ? "(InSync)" : "(NOT InSync)");
                        }
                    }
                }
            }

            Sleep(200);
        }
    }

    } /* end normal mode */

    shm->RTXState = eRTXState::OPEN;

    /* Close DCM CSV log file */
    if (g_pDcmCsvFile) {
        fflush(g_pDcmCsvFile);
        fclose(g_pDcmCsvFile);
        g_pDcmCsvFile = NULL;
        printf("DCM CSV log file closed.\n");
    }

    Debug::flushToFile();
    Debug::closeFile();
    if (!g_bNoHmi && g_bThreadsStarted)
        closeThread();

Exit:
    /* Transition to INIT while job task is still running (frames must be processed) */
    printf("[cleanup] Transitioning master to INIT...\n");
    ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);

Exit2:
    StopJobTask();
    ecatDeinitMaster();

    delete sys;
Exit3:
    if (shm) {
        shm->ecatState = 0;
        shm->RTXState = eRTXState::CLOSE;
    }
    printf("========== App Terminated ==============\n");
    return 0;
}

/* ===================== Threads  ======================= */
void scriptStartThread(void* Context)
{
    EventHandler::Reset(sys->evtScriptStart);

    while (!shm->stopRTX) {
        EventHandler::WaitFor(sys->evtScriptStart, INFINITE);

        if (shm->stopRTX) break;
        Debug::write(0, "\n========= Script Start =========\n");

        // reset syncTable
        for (int s = 0; s < MAX_SYNC_NUM; s++) {
            for (int r = 0; r <= MAX_ROBOT_NUM; r++) {
                if (shm->syncTable[s][r] == eSyncState::READY)
                    shm->syncTable[s][r] = eSyncState::NOT_YET;
            }
        }

        // reset other stuff
        for (int r = 0; r < sys->robotNum; r++) {
            sys->robots[r]->script->prepareStart();
        }

        // open flag
        sys->scrRobotNum = sys->robotNum;
        shm->stopScript = false;
        shm->isScriptRunning = true;

        // resume threads
        for (int r = 0; r < sys->robotNum; r++) {
            ThreadHandler::Resume(hScriptHMIThread[r]);
            ThreadHandler::Resume(hScriptPlanningThread[r]);
            ThreadHandler::Resume(hScriptInterpThread[r]);
        }
    }
}

void scriptHMIThread(void* Context)
{
    Script* script = (Script*)Context;
    int r = script->rId;
    ScriptData* shmScrData = &(shm->scriptData[r]);

    while (!shm->stopRTX) {
        if (script->rawQueue.isFull()) {
            Sleep(500);
            continue;
        }

        EventHandler::Set(sys->evtScriptNeed[r]);
        EventHandler::WaitFor(sys->evtScriptSet[r], INFINITE);

        if (shmScrData->isHMICmdTerminate) {
            Debug::writeln(0, "\n scriptHMIThread%d is suspended.\n", r);
            ThreadHandler::Suspend((HANDLE)pthread_self());
            continue;
        }

        script->rawQueue.enqueue(shmScrData->scriptRawCmd);
        Sleep(1);
    }
    Debug::write(0, "scriptHMIThread[%d] end.\n", r);
}

void scriptPlanningThread(void* Context)
{
    Script* script = (Script*)Context;
    Robot* robot = script->robot;
    int r = script->rId;
    int ret;

    ScriptRawCmd rawCmd;
    ScriptPathCmd pathCmd;

    while (!shm->stopRTX) {
        if (script->isRawQueueTerminate) {
            ThreadHandler::Suspend((HANDLE)pthread_self());

            pathCmd.cmdMove.preToolId = shm->jogToolIds[r];
            pathCmd.cmdMove.nowToolId = shm->jogToolIds[r];
        }

        if (script->ppQueue.getCount() > PPQ_SIZE - 2) {
            Sleep(100);
            continue;
        }

        ret = script->rawQueue.getNodeAtFront(&rawCmd);

        if (ret < 0) {
            if (*script->isHMITerminate)
                script->isRawQueueTerminate = true;
            Sleep(100);
            continue;
        }

        if (script->raw2Path(pathCmd, rawCmd) < 0) {
            Sleep(100);
            continue;
        }

        script->updateTransferMatrix(pathCmd.cmdMove.endPose);
        script->ppQueue.enqueue(pathCmd);
        script->rawQueue.dequeue();
        Sleep(1);
    }
    Debug::write(0, "@scriptPlanningThread[%d] end.\n", r);
}

void scriptInterpThread(void* Context)
{
    Script* script = (Script*)Context;
    static int count = 0;
    int r = script->rId;
    int ret;

    Sleep(100);
    while (!shm->stopRTX) {
        if (shm->isScriptRunning == false) {
            ThreadHandler::Suspend((HANDLE)pthread_self());
            Sleep(100);
        }
        if (script->intpQueue.isFull()) { Sleep(25); continue; }

        ret = script->tryToSetNewPath();

        if (count++ == 10) count = 0;
        if (ret < 0 && count == 0) Sleep(1);

        if (script->interpolate() == eIntpResult::FAIL) {
            script->slowStop();
        }
    }
    printf("@scriptInterpThread[%d] end.\n", r);
}

void logThread(void* Context)
{
    (void)Context;
    while (!shm->stopRTX) {
        Debug::flushToConsole();
        Debug::flushToFile();
        Sleep(20);
    }
}

/* ================== Support Functions ================== */
bool OpenScriptThread()
{
    hScriptStartThread = ThreadHandler::Create(0, 300000, (LPTHREAD_START_ROUTINE)scriptStartThread, sys, CREATE_SUSPENDED, NULL);
    if (hScriptStartThread == NULL) { printf("Create scriptStartThread error\n"); return false; }

    for (int r = 0; r < sys->robotNum; r++) {
        LPDWORD id = 0;
        hScriptHMIThread[r] = ThreadHandler::Create(0, 300000, (LPTHREAD_START_ROUTINE)scriptHMIThread, sys->robots[r]->script, CREATE_SUSPENDED, id);
        if (hScriptHMIThread[r] == NULL) { printf("Create scriptHMIThread[%d] error\n", r); return false; }
        ThreadHandler::SetPriority(hScriptHMIThread[r], 10);

        hScriptPlanningThread[r] = ThreadHandler::Create(0, 300000, (LPTHREAD_START_ROUTINE)scriptPlanningThread, sys->robots[r]->script, CREATE_SUSPENDED, id);
        if (hScriptPlanningThread[r] == NULL) { printf("Create scriptPlanningThread[%d] error\n", r); return false; }
        ThreadHandler::SetPriority(hScriptPlanningThread[r], 10);

        hScriptInterpThread[r] = ThreadHandler::Create(0, 300000, (LPTHREAD_START_ROUTINE)scriptInterpThread, sys->robots[r]->script, CREATE_SUSPENDED, id);
        if (hScriptInterpThread[r] == NULL) { printf("Create scriptInterpThread[%d] error\n", r); return false; }
        ThreadHandler::SetPriority(hScriptInterpThread[r], 10);
    }

    ThreadHandler::SetPriority(hScriptStartThread, 10);
    ThreadHandler::Resume(hScriptStartThread);

    return true;
}

bool OpenLogThread()
{
    hLogThread = ThreadHandler::Create(0, 30000, (LPTHREAD_START_ROUTINE)logThread, NULL, CREATE_SUSPENDED, NULL);
    if (hLogThread == NULL) {
        printf("Create LogThread error\n");
        return false;
    }

    ThreadHandler::SetPriority(hLogThread, 10);
    ThreadHandler::Resume(hLogThread);

    return true;
}

void closeThread()
{
    EventHandler::Set(sys->evtScriptStart);

    for (int r = 0; r < sys->robotNum; r++) {
        ThreadHandler::Resume(hScriptHMIThread[r]);
        ThreadHandler::Resume(hScriptPlanningThread[r]);
        ThreadHandler::Resume(hScriptInterpThread[r]);
    }

    if (EventHandler::WaitFor(hScriptStartThread, 3000) == WAIT_TIMEOUT)
        printf("waitTimeOut! [hScriptStartThread]\n");

    for (int r = 0; r < sys->robotNum; r++) {
        if (EventHandler::WaitFor(hScriptHMIThread[r], 3000) == WAIT_TIMEOUT)
            printf("waitTimeOut! [hScriptHMIThread%d]\n", r);

        if (EventHandler::WaitFor(hScriptPlanningThread[r], 3000) == WAIT_TIMEOUT)
            printf("waitTimeOut! [hScriptPlanningThread%d]\n", r);

        if (EventHandler::WaitFor(hScriptInterpThread[r], 3000) == WAIT_TIMEOUT)
            printf("waitTimeOut! [hScriptInterpThread%d]\n", r);
    }

    EventHandler::Close(hScriptStartThread);
    EventHandler::Close(hLogThread);
    EventHandler::Close(sys->evtScriptStart);

    for (int r = 0; r < sys->robotNum; r++) {
        EventHandler::Close(hScriptHMIThread[r]);
        EventHandler::Close(hScriptPlanningThread[r]);
        EventHandler::Close(hScriptInterpThread[r]);
        EventHandler::Close(sys->evtScriptSet[r]);
        EventHandler::Close(sys->evtScriptNeed[r]);
    }
}
