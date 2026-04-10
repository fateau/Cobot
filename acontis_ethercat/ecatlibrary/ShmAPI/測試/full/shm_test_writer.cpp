// shm_test_writer.cpp — 全面測試：透過 libShmAPI.so 寫入所有 shared memory 欄位
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <unistd.h>

// ================== 函式指標型別 ==================
// init / close / reset
typedef int   (*fn_init)();
typedef int   (*fn_closeShm)();
typedef int   (*fn_resetValue)();

// RTX / Ecat
typedef int   (*fn_Set_MasterId)(int);
typedef int   (*fn_Get_RTXState)();
typedef int   (*fn_Get_EcatState)(int&);
typedef int   (*fn_StopRTXProcess)();
typedef int   (*fn_Get_SlaveInfo)(int&, int&);
typedef void  (*fn_Set_RTXMode)(int);
typedef int   (*fn_Get_RTXMode)();
typedef void  (*fn_Set_NIC)(int);
typedef int   (*fn_Get_ErrorCode)(char*);

// Motor
typedef int   (*fn_Set_ServoOn)(int, int, char);
typedef char  (*fn_Get_IsServoOn)(int, int);
typedef void  (*fn_Set_IsApplyToRealMotor)(bool);
typedef char  (*fn_Get_IsApplyToRealMotor)();

// Robot Control
typedef void  (*fn_Get_AxisDeg)(int, double*);
typedef void  (*fn_Get_Pose)(int, double*, double*, double*);
typedef void  (*fn_Get_Velocity)(int, double*);
typedef void  (*fn_Get_Torque)(int, double*);
typedef void  (*fn_Get_MasteringData)(int, double*);
typedef void  (*fn_Set_MasteringData)(int, int, double*);
typedef void  (*fn_Set_ReductionRatio)(int, double*);
typedef void  (*fn_Set_DHtable)(int, double*, double*, double*, double*, double*, double*, double*, int*);
typedef void  (*fn_Set_MaxVelArray)(int, char, double*);
typedef void  (*fn_Set_MaxVel)(int, int, char, double);

// Command
typedef int   (*fn_Set_CmdSource)(int);
typedef int   (*fn_Set_CmdFormat)(int);

// Jog
typedef void  (*fn_Set_JogAcc)(double);
typedef void  (*fn_Set_JogCmdSingle)(int, int, double, double, bool);
typedef void  (*fn_Set_UserStopPressJog)();
typedef void  (*fn_Set_JogFrame)(int);

// HG Cobot (Set_HG6Cobot uses struct by value, test via direct shm in reader)
typedef bool  (*fn_Set_CollisionMode)(int, bool);
typedef int   (*fn_Set_MaxTolerateTime)(int, int);
typedef int   (*fn_Get_CollisionState)(int);
typedef int   (*fn_Set_CollisionState)(int, int);
typedef void  (*fn_Set_URDF)(int, double*, double*, double*, double*, double*, double*, double*, double*, double*, double*);

// PID
typedef void  (*fn_Set_PID)(int, int, double, double, double);
typedef void  (*fn_Get_PID)(int, int, double*, double*, double*);
typedef void  (*fn_Save_MotorParams)(int, int);

// Script
typedef void  (*fn_Clear_SyncTable)();
typedef void  (*fn_Set_SyncTable)(int, int);
typedef void  (*fn_StopScript)(char);
typedef void  (*fn_Set_IsHMIScriptTerminate)(int, char);
typedef char  (*fn_Get_IsScriptRunning)();
typedef int   (*fn_Get_NowLineId)(int);
typedef int   (*fn_Get_NowFuncId)(int);
typedef int   (*fn_Set_VGain)(double);
typedef int   (*fn_Set_ScriptRawCmd_Index)(int, int, int);
typedef int   (*fn_Set_ScriptRawCmd_Delay)(int, int);
typedef int   (*fn_Set_ScriptRawCmd_Sync)(int, int, int);
typedef int   (*fn_Set_ScriptRawCmd_HMISync)(int);
typedef int   (*fn_Set_ScriptRawCmd_MS)(int, int);

// IO
typedef int   (*fn_Get_IOInfo)(int, int&, int&);
typedef int   (*fn_Get_IOBytes)(int, unsigned char*, unsigned char*);
typedef int   (*fn_Set_IO_OUTByte)(int, int, unsigned char);
typedef void  (*fn_Set_IOBit)(int, int, bool, bool);
typedef void  (*fn_Set_IOBit_Toggle)(int, int, bool);

// RobotDeclare
typedef int   (*fn_Set_RobotDeclare)(int, int, double*, int);
typedef int   (*fn_Set_EcatMapping)(int, int, int);

// DataBase
typedef void  (*fn_Set_ToolBase)(int, int, double*);
typedef void  (*fn_Set_JogToolInd)(int, int);
typedef void  (*fn_Set_JogBaseInd)(int, int);

// Scurve
typedef void  (*fn_Set_CurveType)(int);
typedef void  (*fn_Set_Jerk)(double);

// Record
typedef void  (*fn_Set_BinPath)(char*);

// ================== 巨集 ==================
#define LOAD_SYM(type, name) \
    type p_##name = (type)dlsym(handle, #name); \
    if (!p_##name) { fprintf(stderr, "WARN: dlsym(%s) failed\n", #name); }

int main()
{
    printf("========================================\n");
    printf("  SHM 全面功能測試 — Writer 端\n");
    printf("========================================\n\n");

    void* handle = dlopen("./libShmAPI.so", RTLD_NOW);
    if (!handle) { fprintf(stderr, "dlopen failed: %s\n", dlerror()); return 1; }

    // ----- 載入所有函式 -----
    LOAD_SYM(fn_init,                  init);
    LOAD_SYM(fn_closeShm,              closeShm);
    LOAD_SYM(fn_resetValue,            resetValue);
    LOAD_SYM(fn_Set_MasterId,          Set_MasterId);
    LOAD_SYM(fn_Get_RTXState,          Get_RTXState);
    LOAD_SYM(fn_Get_EcatState,         Get_EcatState);
    LOAD_SYM(fn_Get_SlaveInfo,         Get_SlaveInfo);
    LOAD_SYM(fn_Set_RTXMode,           Set_RTXMode);
    LOAD_SYM(fn_Get_RTXMode,           Get_RTXMode);
    LOAD_SYM(fn_Set_NIC,               Set_NIC);
    LOAD_SYM(fn_Get_ErrorCode,         Get_ErrorCode);
    LOAD_SYM(fn_Set_ServoOn,           Set_ServoOn);
    LOAD_SYM(fn_Get_IsServoOn,         Get_IsServoOn);
    LOAD_SYM(fn_Set_IsApplyToRealMotor, Set_IsApplyToRealMotor);
    LOAD_SYM(fn_Get_IsApplyToRealMotor, Get_IsApplyToRealMotor);
    LOAD_SYM(fn_Get_AxisDeg,           Get_AxisDeg);
    LOAD_SYM(fn_Get_Pose,              Get_Pose);
    LOAD_SYM(fn_Get_Velocity,          Get_Velocity);
    LOAD_SYM(fn_Get_Torque,            Get_Torque);
    LOAD_SYM(fn_Get_MasteringData,     Get_MasteringData);
    LOAD_SYM(fn_Set_MasteringData,     Set_MasteringData);
    LOAD_SYM(fn_Set_ReductionRatio,    Set_ReductionRatio);
    LOAD_SYM(fn_Set_DHtable,           Set_DHtable);
    LOAD_SYM(fn_Set_MaxVelArray,       Set_MaxVelArray);
    LOAD_SYM(fn_Set_MaxVel,            Set_MaxVel);
    LOAD_SYM(fn_Set_CmdSource,         Set_CmdSource);
    LOAD_SYM(fn_Set_CmdFormat,         Set_CmdFormat);
    LOAD_SYM(fn_Set_JogAcc,            Set_JogAcc);
    LOAD_SYM(fn_Set_JogCmdSingle,      Set_JogCmdSingle);
    LOAD_SYM(fn_Set_UserStopPressJog,  Set_UserStopPressJog);
    LOAD_SYM(fn_Set_JogFrame,          Set_JogFrame);
    LOAD_SYM(fn_Set_CollisionMode,     Set_CollisionMode);
    LOAD_SYM(fn_Set_MaxTolerateTime,   Set_MaxTolerateTime);
    LOAD_SYM(fn_Get_CollisionState,    Get_CollisionState);
    LOAD_SYM(fn_Set_CollisionState,    Set_CollisionState);
    LOAD_SYM(fn_Set_URDF,              Set_URDF);
    LOAD_SYM(fn_Set_PID,               Set_PID);
    LOAD_SYM(fn_Get_PID,               Get_PID);
    LOAD_SYM(fn_Save_MotorParams,      Save_MotorParams);
    LOAD_SYM(fn_Clear_SyncTable,       Clear_SyncTable);
    LOAD_SYM(fn_Set_SyncTable,         Set_SyncTable);
    LOAD_SYM(fn_StopScript,            StopScript);
    LOAD_SYM(fn_Set_IsHMIScriptTerminate, Set_IsHMIScriptTerminate);
    LOAD_SYM(fn_Get_IsScriptRunning,   Get_IsScriptRunning);
    LOAD_SYM(fn_Get_NowLineId,         Get_NowLineId);
    LOAD_SYM(fn_Get_NowFuncId,         Get_NowFuncId);
    LOAD_SYM(fn_Set_VGain,             Set_VGain);
    LOAD_SYM(fn_Set_ScriptRawCmd_Index, Set_ScriptRawCmd_Index);
    LOAD_SYM(fn_Set_ScriptRawCmd_Delay, Set_ScriptRawCmd_Delay);
    LOAD_SYM(fn_Set_ScriptRawCmd_Sync, Set_ScriptRawCmd_Sync);
    LOAD_SYM(fn_Set_ScriptRawCmd_HMISync, Set_ScriptRawCmd_HMISync);
    LOAD_SYM(fn_Set_ScriptRawCmd_MS,   Set_ScriptRawCmd_MS);
    LOAD_SYM(fn_Get_IOInfo,            Get_IOInfo);
    LOAD_SYM(fn_Get_IOBytes,           Get_IOBytes);
    LOAD_SYM(fn_Set_IO_OUTByte,        Set_IO_OUTByte);
    LOAD_SYM(fn_Set_IOBit,             Set_IOBit);
    LOAD_SYM(fn_Set_IOBit_Toggle,      Set_IOBit_Toggle);
    LOAD_SYM(fn_Set_RobotDeclare,      Set_RobotDeclare);
    LOAD_SYM(fn_Set_EcatMapping,       Set_EcatMapping);
    LOAD_SYM(fn_Set_ToolBase,          Set_ToolBase);
    LOAD_SYM(fn_Set_JogToolInd,        Set_JogToolInd);
    LOAD_SYM(fn_Set_JogBaseInd,        Set_JogBaseInd);
    LOAD_SYM(fn_Set_CurveType,         Set_CurveType);
    LOAD_SYM(fn_Set_Jerk,              Set_Jerk);
    LOAD_SYM(fn_Set_BinPath,           Set_BinPath);

    int section = 0;

    // ============================================
    // [1] init / reset
    // ============================================
    printf("--- [%d] init / resetValue ---\n", ++section);
    int ret = p_init();
    printf("  init() = %d\n", ret);

    // ============================================
    // [2] RTX State / MasterId / NIC / RTXMode
    // ============================================
    printf("--- [%d] RTX/Ecat 基本設定 ---\n", ++section);
    p_Set_MasterId(99);
    printf("  Set_MasterId(99)\n");

    p_Set_NIC(5);
    printf("  Set_NIC(5)\n");

    p_Set_RTXMode(10); // CST=10
    printf("  Set_RTXMode(10=CST)\n");

    int rtxState = p_Get_RTXState();
    printf("  Get_RTXState() = %d\n", rtxState);

    int rtxMode = p_Get_RTXMode();
    printf("  Get_RTXMode() = %d\n", rtxMode);

    int ecatState = 0;
    p_Get_EcatState(ecatState);
    printf("  Get_EcatState() = %d\n", ecatState);

    char errMsg[256] = {};
    int errCode = p_Get_ErrorCode(errMsg);
    printf("  Get_ErrorCode() = %d, msg='%s'\n", errCode, errMsg);

    int sMotor = 0, sIO = 0;
    p_Get_SlaveInfo(sMotor, sIO);
    printf("  Get_SlaveInfo() motorNum=%d, ioNum=%d\n", sMotor, sIO);

    // ============================================
    // [3] Motor Control (Servo)
    // ============================================
    printf("--- [%d] Motor Control ---\n", ++section);
    p_Set_ServoOn(0, 0, 1); // r=0, m=0, ON
    printf("  Set_ServoOn(0, 0, ON)\n");
    p_Set_ServoOn(0, 2, 1); // r=0, m=2, ON
    printf("  Set_ServoOn(0, 2, ON)\n");
    char servoOn0 = p_Get_IsServoOn(0, 0);
    printf("  Get_IsServoOn(0, 0) = %d (isNeedServoOn flag set)\n", (int)servoOn0);

    p_Set_IsApplyToRealMotor(true);
    printf("  Set_IsApplyToRealMotor(true)\n");
    char applyReal = p_Get_IsApplyToRealMotor();
    printf("  Get_IsApplyToRealMotor() = %d\n", (int)applyReal);

    // ============================================
    // [4] Robot Declare / EcatMapping
    // ============================================
    printf("--- [%d] Robot Declare / EcatMapping ---\n", ++section);
    double refRoot[6] = {10.0, 20.0, 30.0, 1.0, 2.0, 3.0};
    p_Set_RobotDeclare(0, 6, refRoot, 0); // robot 0, 6 axis, type=None
    printf("  Set_RobotDeclare(0, 6motors, refRoot=[10,20,30,1,2,3])\n");

    double refRoot1[6] = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    p_Set_RobotDeclare(1, 3, refRoot1, 800); // robot 1, 3 axis, type=Cobot
    printf("  Set_RobotDeclare(1, 3motors, type=Cobot)\n");

    for (int i = 0; i < 6; i++)
        p_Set_EcatMapping(i, 0, i);
    for (int i = 0; i < 3; i++)
        p_Set_EcatMapping(6+i, 1, i);
    printf("  Set_EcatMapping: motor0~5→robot0, motor6~8→robot1\n");

    // ============================================
    // [5] DH table / ReductionRatio / MasteringData / MaxVel
    // ============================================
    printf("--- [%d] Robot Spec ---\n", ++section);
    double a[7]       = {0, 0, 200, 200, 0, 0, 0};
    double alpha[7]   = {0, 90, 0, 0, 90, -90, 0};
    double d[7]       = {300, 0, 0, 0, 300, 0, 100};
    double thInit[7]  = {0, 0, 0, 0, 0, 0, 0};
    double thShift[7] = {0, 0, 0, 0, 0, 0, 0};
    double posLim[7]  = {170, 120, 170, 120, 170, 120, 170};
    double negLim[7]  = {-170, -120, -170, -120, -170, -120, -170};
    int jType[7]      = {0, 0, 0, 0, 0, 0, 0}; // all REVOLUTE
    p_Set_DHtable(0, a, alpha, d, thInit, thShift, posLim, negLim, jType);
    printf("  Set_DHtable(robot=0)\n");

    double ratio[7] = {100, 100, 100, 80, 80, 50, 50};
    p_Set_ReductionRatio(0, ratio);
    printf("  Set_ReductionRatio(robot=0)\n");

    double mastering[7] = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7};
    p_Set_MasteringData(0, 0, mastering);
    printf("  Set_MasteringData(robot=0)\n");

    double axisVels[7] = {180, 180, 180, 360, 360, 360, 360};
    p_Set_MaxVelArray(0, 1, axisVels); // isAxis=1
    printf("  Set_MaxVelArray(robot=0, isAxis=true)\n");

    double tcpVel[1] = {500.0};
    p_Set_MaxVelArray(0, 0, tcpVel); // isAxis=0 → sets maxTCPVel
    printf("  Set_MaxVelArray(robot=0, isAxis=false, vel=500)\n");

    p_Set_MaxVel(0, 3, 1, 999.0); // override axis 3
    printf("  Set_MaxVel(robot=0, motor=3, isAxis=1, vel=999)\n");

    // ============================================
    // [6] Command Source / Format
    // ============================================
    printf("--- [%d] CmdSource / CmdFormat ---\n", ++section);
    p_Set_CmdSource(2); // SCRIPT=2
    printf("  Set_CmdSource(SCRIPT=2)\n");

    p_Set_CmdFormat(1); // AXIS=1
    printf("  Set_CmdFormat(AXIS=1)\n");

    // ============================================
    // [7] Jog
    // ============================================
    printf("--- [%d] Jog ---\n", ++section);
    p_Set_JogAcc(50.0);
    printf("  Set_JogAcc(50.0)\n");

    p_Set_JogCmdSingle(0, 2, 30.0, 10.0, true);
    printf("  Set_JogCmdSingle(r=0, m=2, vel=30, dist=10, limitDist=true)\n");

    p_Set_JogFrame(1); // TOOL=1
    printf("  Set_JogFrame(TOOL=1)\n");

    p_Set_JogToolInd(0, 3);
    printf("  Set_JogToolInd(r=0, toolId=3)\n");

    p_Set_JogBaseInd(0, 7);
    printf("  Set_JogBaseInd(r=0, baseId=7)\n");

    // ============================================
    // [8] HG Cobot / Collision / URDF
    // ============================================
    printf("--- [%d] HG Cobot / Collision / URDF ---\n", ++section);
    p_Set_CollisionMode(0, true);
    printf("  Set_CollisionMode(r=0, true)\n");

    p_Set_CollisionState(0, 2);
    printf("  Set_CollisionState(r=0, state=2)\n");

    int colState = p_Get_CollisionState(0);
    printf("  Get_CollisionState(0) = %d\n", colState);

    p_Set_MaxTolerateTime(0, 500);
    printf("  Set_MaxTolerateTime(r=0, 500)\n");

    double ux[7]={1,2,3,4,5,6,7}, uy[7]={10,20,30,40,50,60,70}, uz[7]={100,200,300,400,500,600,700};
    double urx[7]={1,0,0,0,0,0,0}, ury[7]={0,1,0,0,0,0,0}, urz[7]={0,0,1,0,0,0,0};
    double uw[7]={2.5,3.0,3.5,4.0,4.5,5.0,5.5};
    double ucx[7]={0.1,0.2,0.3,0.4,0.5,0.6,0.7};
    double ucy[7]={0.01,0.02,0.03,0.04,0.05,0.06,0.07};
    double ucz[7]={0.001,0.002,0.003,0.004,0.005,0.006,0.007};
    p_Set_URDF(0, ux, uy, uz, urx, ury, urz, uw, ucx, ucy, ucz);
    printf("  Set_URDF(robot=0)\n");

    // ============================================
    // [9] PID Control
    // ============================================
    printf("--- [%d] PID Control ---\n", ++section);
    p_Set_PID(0, 0, 10.5, 20.5, 30.5);
    printf("  Set_PID(r=0, m=0, kpp=10.5, kvp=20.5, kvi=30.5)\n");

    p_Set_PID(0, 1, 11.0, 22.0, 33.0);
    printf("  Set_PID(r=0, m=1, kpp=11.0, kvp=22.0, kvi=33.0)\n");

    p_Save_MotorParams(0, 0);
    printf("  Save_MotorParams(r=0, m=0) → isNeedSaveParams=true\n");

    // ============================================
    // [10] Script
    // ============================================
    printf("--- [%d] Script ---\n", ++section);
    p_Set_VGain(0.75);
    printf("  Set_VGain(0.75)\n");

    p_Clear_SyncTable();
    printf("  Clear_SyncTable()\n");

    p_Set_SyncTable(0, 5); // robot=0, syncId=5
    printf("  Set_SyncTable(r=0, syncId=5)\n");

    p_Set_ScriptRawCmd_Index(0, 42, 7);
    printf("  Set_ScriptRawCmd_Index(r=0, lineId=42, funcId=7)\n");

    p_Set_ScriptRawCmd_Delay(0, 1500);
    printf("  Set_ScriptRawCmd_Delay(r=0, time=1500ms)\n");

    p_Set_ScriptRawCmd_Sync(0, 3, 0); // syncId=3, NORMAL=0
    printf("  Set_ScriptRawCmd_Sync(r=0, syncId=3, NORMAL)\n");

    p_Set_ScriptRawCmd_HMISync(1);
    printf("  Set_ScriptRawCmd_HMISync(r=1)\n");

    p_Set_ScriptRawCmd_MS(0, 2);
    printf("  Set_ScriptRawCmd_MS(r=0, masterId=2)\n");

    p_Set_IsHMIScriptTerminate(0, 0); // not terminate
    printf("  Set_IsHMIScriptTerminate(r=0, false)\n");

    // ============================================
    // [11] IO
    // ============================================
    printf("--- [%d] IO ---\n", ++section);
    p_Set_IO_OUTByte(0, 0, 0xAB);
    printf("  Set_IO_OUTByte(module=0, byte=0, val=0xAB)\n");

    p_Set_IO_OUTByte(0, 1, 0xCD);
    printf("  Set_IO_OUTByte(module=0, byte=1, val=0xCD)\n");

    p_Set_IOBit(1, 0, true, true);   // module=1, bit=0, isOut=true, value=true
    printf("  Set_IOBit(module=1, bit=0, isOut=true, value=true)\n");

    p_Set_IOBit(1, 3, true, true);
    printf("  Set_IOBit(module=1, bit=3, isOut=true, value=true)\n");

    p_Set_IOBit_Toggle(1, 0, true);  // toggle bit 0 → should become false
    printf("  Set_IOBit_Toggle(module=1, bit=0, isOut=true) → toggle\n");

    // ============================================
    // [12] ToolBase
    // ============================================
    printf("--- [%d] ToolBase ---\n", ++section);
    double toolPose[6] = {10, 20, 30, 0.1, 0.2, 0.3};
    p_Set_ToolBase(0, 0, toolPose); // TOOL=0, ind=0
    printf("  Set_ToolBase(TOOL, ind=0)\n");

    double basePose[6] = {100, 200, 300, 1.0, 2.0, 3.0};
    p_Set_ToolBase(1, 0, basePose); // BASE=1, ind=0
    printf("  Set_ToolBase(BASE, ind=0)\n");

    // ============================================
    // [13] Scurve / Jerk
    // ============================================
    printf("--- [%d] Scurve / Jerk ---\n", ++section);
    p_Set_CurveType(1);
    printf("  Set_CurveType(1=on)\n");

    p_Set_Jerk(1000.0);
    printf("  Set_Jerk(1000.0)\n");

    // ============================================
    // [14] Record / BinPath
    // ============================================
    printf("--- [%d] Record / BinPath ---\n", ++section);
    char binPath[] = "/tmp/test_log.bin";
    p_Set_BinPath(binPath);
    printf("  Set_BinPath('%s')\n", binPath);

    // ============================================
    // Writer 等待 Reader 檢查
    // ============================================
    printf("\n========================================\n");
    printf("  Writer 完成所有寫入！\n");
    printf("  等待 Reader 讀取中...\n");
    printf("========================================\n");

    // 寫入完成旗標到 /tmp，讓 reader 知道
    FILE* f = fopen("/tmp/shm_writer_ready", "w");
    if (f) { fprintf(f, "1"); fclose(f); }

    // 等待 reader 完成
    for (int i = 0; i < 30; i++) {
        if (access("/tmp/shm_reader_done", F_OK) == 0) break;
        sleep(1);
    }
    sleep(1);

    p_closeShm();
    printf("\n[Writer] closeShm() done.\n");

    // 清理
    remove("/tmp/shm_writer_ready");
    remove("/tmp/shm_reader_done");

    dlclose(handle);
    return 0;
}
