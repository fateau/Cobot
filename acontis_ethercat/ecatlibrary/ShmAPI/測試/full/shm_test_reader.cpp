// shm_test_reader.cpp — 全面測試：直接用 POSIX shm_open 讀取所有 shared memory 欄位
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>

#include "ShmAPI/RtxAppLayer/Shm.h"

static const char* SHM_NAME = "/DataSpace";

// ================== 測試框架 ==================
static int g_pass = 0, g_fail = 0, g_section = 0;

#define SECTION(name) printf("\n--- [%d] %s ---\n", ++g_section, name)

#define CHECK_INT(desc, actual, expected) do { \
    if ((actual) == (expected)) { printf("  [PASS] %s = %d\n", desc, (int)(actual)); g_pass++; } \
    else { printf("  [FAIL] %s = %d (expected %d)\n", desc, (int)(actual), (int)(expected)); g_fail++; } \
} while(0)

#define CHECK_DBL(desc, actual, expected) do { \
    if (fabs((double)(actual) - (double)(expected)) < 1e-9) { printf("  [PASS] %s = %.4f\n", desc, (double)(actual)); g_pass++; } \
    else { printf("  [FAIL] %s = %.4f (expected %.4f)\n", desc, (double)(actual), (double)(expected)); g_fail++; } \
} while(0)

#define CHECK_BOOL(desc, actual, expected) do { \
    if ((bool)(actual) == (bool)(expected)) { printf("  [PASS] %s = %s\n", desc, (actual)?"true":"false"); g_pass++; } \
    else { printf("  [FAIL] %s = %s (expected %s)\n", desc, (actual)?"true":"false", (expected)?"true":"false"); g_fail++; } \
} while(0)

#define CHECK_STR(desc, actual, expected) do { \
    if (strcmp((actual), (expected)) == 0) { printf("  [PASS] %s = '%s'\n", desc, (actual)); g_pass++; } \
    else { printf("  [FAIL] %s = '%s' (expected '%s')\n", desc, (actual), (expected)); g_fail++; } \
} while(0)

#define CHECK_BYTE(desc, actual, expected) do { \
    if ((unsigned char)(actual) == (unsigned char)(expected)) { printf("  [PASS] %s = 0x%02X\n", desc, (unsigned char)(actual)); g_pass++; } \
    else { printf("  [FAIL] %s = 0x%02X (expected 0x%02X)\n", desc, (unsigned char)(actual), (unsigned char)(expected)); g_fail++; } \
} while(0)

int main()
{
    printf("========================================\n");
    printf("  SHM 全面功能測試 — Reader 端\n");
    printf("========================================\n");

    // 等待 Writer 準備好
    printf("[Reader] 等待 Writer 就緒...\n");
    for (int i = 0; i < 30; i++) {
        if (access("/tmp/shm_writer_ready", F_OK) == 0) break;
        usleep(200 * 1000);
    }
    usleep(100 * 1000); // 再多等一點確保寫入完畢

    // 開啟共享記憶體
    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
        perror("[Reader] shm_open 失敗");
        return 1;
    }

    int shm_size = sizeof(SHMData);
    SHMData* shm = (SHMData*)mmap(0, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) {
        perror("[Reader] mmap 失敗");
        close(shm_fd);
        return 1;
    }

    printf("[Reader] attach 成功 (SHMData = %d bytes)\n", shm_size);

    // ============================================
    // [1] init / reset 產生的初始狀態
    // ============================================
    SECTION("init / resetValue 基本狀態");
    CHECK_INT("RTXState (CLOSE=0)",    shm->RTXState,   0);
    CHECK_INT("project (BASIC=1)",     shm->project,    1);
    CHECK_BOOL("stopScript",           shm->stopScript, true);

    // ============================================
    // [2] RTX / Ecat 基本設定
    // ============================================
    SECTION("RTX/Ecat 基本設定");
    CHECK_INT("masterId",              shm->masterId,   99);
    CHECK_INT("NIC",                   shm->NIC,        5);
    CHECK_INT("RTXMode (CST=10)",      shm->RTXMode,    10);
    CHECK_INT("ecatState",             shm->ecatState,  0);
    CHECK_INT("errorCode (NONE=0)",    shm->errorCode,  0);
    CHECK_INT("slaveMotorNum",         shm->slaveMotorNum, 0);
    CHECK_INT("slaveIONum",            shm->slaveIONum, 0);

    // ============================================
    // [3] Motor Control (Servo)
    // ============================================
    SECTION("Motor Control");
    CHECK_BOOL("robot[0].motor[0].isNeedServoOn",  shm->robots[0].motors[0].isNeedServoOn,  true);
    CHECK_BOOL("robot[0].motor[1].isNeedServoOn",  shm->robots[0].motors[1].isNeedServoOn,  false);
    CHECK_BOOL("robot[0].motor[2].isNeedServoOn",  shm->robots[0].motors[2].isNeedServoOn,  true);
    CHECK_BOOL("isApplyToRealMotor",               shm->isApplyToRealMotor, true);

    // ============================================
    // [4] Robot Declare / EcatMapping
    // ============================================
    SECTION("Robot Declare / EcatMapping");
    CHECK_INT("robot[0].motorNum",     shm->robotDeclareTable[0].motorNum,   6);
    CHECK_INT("robot[0].gestureNum",   shm->robotDeclareTable[0].gestureNum, 6);
    CHECK_INT("robot[0].robotType",    shm->robotDeclareTable[0].robotType,  0); // None=0
    CHECK_DBL("robot[0].refRoot[0]",   shm->robotDeclareTable[0].refRoot[0], 10.0);
    CHECK_DBL("robot[0].refRoot[1]",   shm->robotDeclareTable[0].refRoot[1], 20.0);
    CHECK_DBL("robot[0].refRoot[2]",   shm->robotDeclareTable[0].refRoot[2], 30.0);
    CHECK_DBL("robot[0].refRoot[3]",   shm->robotDeclareTable[0].refRoot[3], 1.0);
    CHECK_DBL("robot[0].refRoot[4]",   shm->robotDeclareTable[0].refRoot[4], 2.0);
    CHECK_DBL("robot[0].refRoot[5]",   shm->robotDeclareTable[0].refRoot[5], 3.0);

    CHECK_INT("robot[1].motorNum",     shm->robotDeclareTable[1].motorNum,   3);
    CHECK_INT("robot[1].robotType",    shm->robotDeclareTable[1].robotType,  800); // Cobot=800

    // EcatMapping
    for (int i = 0; i < 6; i++) {
        char desc[64];
        snprintf(desc, sizeof(desc), "ecatMap[%d].robotInd", i);
        CHECK_INT(desc, shm->ecatMappingTable[i].robotInd, 0);
        snprintf(desc, sizeof(desc), "ecatMap[%d].axisInd", i);
        CHECK_INT(desc, shm->ecatMappingTable[i].axisInd, i);
    }
    CHECK_INT("ecatMap[6].robotInd",   shm->ecatMappingTable[6].robotInd, 1);
    CHECK_INT("ecatMap[6].axisInd",    shm->ecatMappingTable[6].axisInd,  0);
    CHECK_INT("ecatMap[8].robotInd",   shm->ecatMappingTable[8].robotInd, 1);
    CHECK_INT("ecatMap[8].axisInd",    shm->ecatMappingTable[8].axisInd,  2);

    // ============================================
    // [5] DH / ReductionRatio / MaxVel
    // ============================================
    SECTION("Robot Spec (DH/Ratio/MaxVel)");
    DHtable* DH = &(shm->robots[0].spec.DH);
    CHECK_DBL("DH.a[2]",              DH->a[2],          200.0);
    CHECK_DBL("DH.alphaDeg[1]",       DH->alphaDeg[1],   90.0);
    CHECK_DBL("DH.d[0]",              DH->d[0],          300.0);
    CHECK_DBL("DH.d[4]",              DH->d[4],          300.0);
    CHECK_DBL("DH.d[6]",              DH->d[6],          100.0);
    CHECK_DBL("DH.posLimit[0]",       DH->axisPositiveLimit[0], 170.0);
    CHECK_DBL("DH.negLimit[0]",       DH->axisNegativeLimit[0], -170.0);
    CHECK_INT("DH.jointType[0]",      DH->jointType[0],  0); // REVOLUTE

    RobotSpec* spec = &(shm->robots[0].spec);
    CHECK_DBL("reductionRatio[0]",    spec->reductionRatio[0], 100.0);
    CHECK_DBL("reductionRatio[3]",    spec->reductionRatio[3], 80.0);
    CHECK_DBL("reductionRatio[5]",    spec->reductionRatio[5], 50.0);

    // MasteringData: writer 呼叫的是 non-RTSS 路徑 → 設定 isNeedResetDegree
    CHECK_BOOL("isNeedResetDegree",   shm->isNeedResetDegree, true);
    CHECK_INT("homingData.rId",       shm->homingData.rId, 0);
    CHECK_INT("homingData.mId",       shm->homingData.mId, 0);

    CHECK_DBL("maxAxisVel[0]",        spec->maxAxisVel[0], 180.0);
    CHECK_DBL("maxAxisVel[1]",        spec->maxAxisVel[1], 180.0);
    CHECK_DBL("maxAxisVel[3]",        spec->maxAxisVel[3], 999.0); // overridden by Set_MaxVel
    CHECK_DBL("maxAxisVel[6]",        spec->maxAxisVel[6], 360.0);
    CHECK_DBL("maxTCPVel",            spec->maxTCPVel,     500.0);

    // ============================================
    // [6] CmdSource / CmdFormat
    // ============================================
    SECTION("CmdSource / CmdFormat");
    CHECK_INT("cmdSource (SCRIPT=2)",  shm->cmdSource,   2);
    CHECK_INT("cmdFormat (AXIS=1)",    shm->cmdFormat,    1);

    // ============================================
    // [7] Jog
    // ============================================
    SECTION("Jog");
    CHECK_DBL("jogCmd.acc",            shm->jogCmd.acc,   50.0);
    CHECK_INT("jogCmd.r",             shm->jogCmd.r,     0);
    CHECK_INT("jogCmd.ind",           shm->jogCmd.ind,   2);
    CHECK_DBL("jogCmd.vel",            shm->jogCmd.vel,   30.0);
    CHECK_DBL("jogCmd.dist",           shm->jogCmd.dist,  10.0);
    CHECK_BOOL("jogCmd.isUserPressJOG", shm->jogCmd.isUserPressJOG, true);
    CHECK_BOOL("jogCmd.isLimitDist",   shm->jogCmd.isLimitDist, true);
    CHECK_INT("jogFrame (TOOL=1)",     shm->jogFrame,     1);
    CHECK_INT("jogToolIds[0]",        shm->jogToolIds[0], 3);
    CHECK_INT("jogBaseIds[0]",        shm->jogBaseIds[0], 7);

    // ============================================
    // [8] HG Cobot / Collision / URDF
    // ============================================
    SECTION("HG Cobot / Collision / URDF");
    CHECK_BOOL("CollisionMode[0]",     shm->CollisionMode[0], true);
    CHECK_INT("CollisionState[0]",     shm->CollisionState[0], 2);
    CHECK_INT("hgParams[0].MaxTolerateTime", shm->hgParams[0].MaxTolerateTime, 500);

    URDFtable* URDF = &(shm->robots[0].spec.URDF);
    CHECK_DBL("URDF.x[0]",            URDF->x[0],       1.0);
    CHECK_DBL("URDF.x[3]",            URDF->x[3],       4.0);
    CHECK_DBL("URDF.y[1]",            URDF->y[1],       20.0);
    CHECK_DBL("URDF.z[2]",            URDF->z[2],       300.0);
    CHECK_DBL("URDF.Rx[0]",           URDF->Rx[0],      1.0);
    CHECK_DBL("URDF.Ry[1]",           URDF->Ry[1],      1.0);
    CHECK_DBL("URDF.Rz[2]",           URDF->Rz[2],      1.0);
    CHECK_DBL("URDF.weight[0]",       URDF->weight[0],  2.5);
    CHECK_DBL("URDF.weight[6]",       URDF->weight[6],  5.5);
    CHECK_DBL("URDF.center_x[3]",     URDF->center_x[3], 0.4);
    CHECK_DBL("URDF.center_y[4]",     URDF->center_y[4], 0.05);
    CHECK_DBL("URDF.center_z[5]",     URDF->center_z[5], 0.006);

    // ============================================
    // [9] PID Control
    // ============================================
    SECTION("PID Control");
    MotorData* motor00 = &(shm->robots[0].motors[0]);
    CHECK_DBL("motor[0][0].cmdKpp",   motor00->cmdKpp,  10.5);
    CHECK_DBL("motor[0][0].cmdKvp",   motor00->cmdKvp,  20.5);
    CHECK_DBL("motor[0][0].cmdKvi",   motor00->cmdKvi,  30.5);
    CHECK_BOOL("motor[0][0].isNeedSetPID", motor00->isNeedSetPID, true);
    CHECK_BOOL("motor[0][0].isNeedSaveParams", motor00->isNeedSaveParams, true);

    MotorData* motor01 = &(shm->robots[0].motors[1]);
    CHECK_DBL("motor[0][1].cmdKpp",   motor01->cmdKpp,  11.0);
    CHECK_DBL("motor[0][1].cmdKvp",   motor01->cmdKvp,  22.0);
    CHECK_DBL("motor[0][1].cmdKvi",   motor01->cmdKvi,  33.0);
    CHECK_BOOL("motor[0][1].isNeedSetPID", motor01->isNeedSetPID, true);

    // ============================================
    // [10] Script
    // ============================================
    SECTION("Script");
    CHECK_DBL("vGain",                 shm->vGain,       0.75);
    CHECK_BOOL("isScriptRunning",      shm->isScriptRunning, false);
    CHECK_BOOL("stopScript",           shm->stopScript,  true);

    // SyncTable: syncTable[5][0] should be NOT_YET (=2)
    CHECK_INT("syncTable[5][0] (NOT_YET=2)", shm->syncTable[5][0], 2);
    CHECK_INT("syncTable[0][0] (NONE=0)",    shm->syncTable[0][0], 0);

    ScriptData* sd0 = &(shm->scriptData[0]);
    CHECK_BOOL("scriptData[0].isHMICmdTerminate", sd0->isHMICmdTerminate, false);

    // Writer 最後呼叫了 Set_ScriptRawCmd_MS(0, 2)
    CHECK_INT("scriptData[0].rawCmd.info.type (MS=5)", sd0->scriptRawCmd.info.type, 5);
    CHECK_INT("scriptData[0].rawCmd.info.myMasterId",  sd0->scriptRawCmd.info.myMasterId, 2);
    CHECK_INT("scriptData[0].rawCmd.info.lineId",      sd0->scriptRawCmd.info.lineId, 42);
    CHECK_INT("scriptData[0].rawCmd.info.funcId",      sd0->scriptRawCmd.info.funcId, 7);

    // scriptData[1] should have HMI_SYNC (=4) from Set_ScriptRawCmd_HMISync(1)
    CHECK_INT("scriptData[1].rawCmd.info.type (HMI_SYNC=4)", shm->scriptData[1].scriptRawCmd.info.type, 4);

    // ============================================
    // [11] IO
    // ============================================
    SECTION("IO");
    CHECK_BYTE("ioModules[0].outBytesCmd[0]", shm->ioModules[0].outBytesCmd[0], 0xAB);
    CHECK_BYTE("ioModules[0].outBytesCmd[1]", shm->ioModules[0].outBytesCmd[1], 0xCD);

    // module 1: bit 0 set then toggled → 0, bit 3 set → 1
    // So outBytesCmd[0] should be 0b00001000 = 0x08
    CHECK_BYTE("ioModules[1].outBytesCmd[0]", shm->ioModules[1].outBytesCmd[0], 0x08);

    // ============================================
    // [12] ToolBase
    // ============================================
    SECTION("ToolBase");
    CHECK_DBL("tools[0].Pose[0]",     shm->tools[0].Pose[0],  10.0);
    CHECK_DBL("tools[0].Pose[1]",     shm->tools[0].Pose[1],  20.0);
    CHECK_DBL("tools[0].Pose[2]",     shm->tools[0].Pose[2],  30.0);
    CHECK_DBL("tools[0].Pose[3]",     shm->tools[0].Pose[3],  0.1);
    CHECK_DBL("tools[0].Pose[4]",     shm->tools[0].Pose[4],  0.2);
    CHECK_DBL("tools[0].Pose[5]",     shm->tools[0].Pose[5],  0.3);

    CHECK_DBL("bases[0].Pose[0]",     shm->bases[0].Pose[0],  100.0);
    CHECK_DBL("bases[0].Pose[1]",     shm->bases[0].Pose[1],  200.0);
    CHECK_DBL("bases[0].Pose[2]",     shm->bases[0].Pose[2],  300.0);
    CHECK_DBL("bases[0].Pose[3]",     shm->bases[0].Pose[3],  1.0);
    CHECK_DBL("bases[0].Pose[4]",     shm->bases[0].Pose[4],  2.0);
    CHECK_DBL("bases[0].Pose[5]",     shm->bases[0].Pose[5],  3.0);

    // ============================================
    // [13] Scurve / Jerk
    // ============================================
    SECTION("Scurve / Jerk");
    CHECK_INT("onScurve",             shm->onScurve,    1);
    CHECK_INT("Jerk",                 shm->Jerk,        1000);

    // ============================================
    // [14] Record / BinPath
    // ============================================
    SECTION("Record / BinPath");
    CHECK_STR("logPath",              shm->logPath,     "/tmp/test_log.bin");

    // ============================================
    // 結果彙總
    // ============================================
    printf("\n========================================\n");
    printf("  測試結果: %d PASS, %d FAIL (共 %d 項)\n", g_pass, g_fail, g_pass + g_fail);
    printf("========================================\n");

    if (g_fail == 0)
        printf("  ✓ 全部通過！所有 ShmAPI 功能正常！\n");
    else
        printf("  ✗ 有 %d 項失敗，請檢查上方 [FAIL] 項目。\n", g_fail);

    printf("========================================\n");

    munmap(shm, shm_size);
    close(shm_fd);

    // 通知 Writer 可以結束
    FILE* f = fopen("/tmp/shm_reader_done", "w");
    if (f) { fprintf(f, "1"); fclose(f); }

    return (g_fail > 0) ? 1 : 0;
}
