// shm_reader.cpp — 讀取端：直接用 POSIX shm_open 讀取 Writer 寫入的共享記憶體
// 不呼叫 init() (因為 init 會 resetValue)，而是直接 attach 已存在的 shm

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>

// 直接引入 SHMData 結構定義
#include "ShmAPI/RtxAppLayer/Shm.h"

static const char* SHM_NAME = "/DataSpace";

int main()
{
    printf("=== SHM Reader (模擬 HMI/上位機側) ===\n");

    // 開啟已存在的共享記憶體 (不帶 O_CREAT，若不存在則失敗)
    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
        perror("[Reader] shm_open 失敗 (Writer 是否已啟動?)");
        return 1;
    }

    int shm_size = sizeof(SHMData);
    printf("[Reader] SHMData 大小: %d bytes\n", shm_size);

    SHMData* shm = (SHMData*)mmap(0, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) {
        perror("[Reader] mmap 失敗");
        close(shm_fd);
        return 1;
    }

    printf("[Reader] 成功 attach 到共享記憶體 '%s'\n\n", SHM_NAME);

    // ===== 讀取 Writer 寫入的資料 =====
    printf("--- 讀取結果 ---\n");
    printf("  masterId      = %d  (預期: 42)\n", shm->masterId);
    printf("  NIC           = %d  (預期: 3)\n",  shm->NIC);
    printf("  RTXState      = %d  (0=CLOSE)\n",  shm->RTXState);
    printf("  RTXMode       = %d  (8=CSP)\n",    shm->RTXMode);
    printf("  vGain         = %.1f (預期: 1.0)\n", shm->vGain);
    printf("  isApplyToReal = %d  (預期: 0)\n",  shm->isApplyToRealMotor);

    // Robot Declaration
    printf("\n  Robot[0].motorNum  = %d  (預期: 6)\n",  shm->robotDeclareTable[0].motorNum);
    printf("  Robot[0].refRoot   = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
        shm->robotDeclareTable[0].refRoot[0], shm->robotDeclareTable[0].refRoot[1],
        shm->robotDeclareTable[0].refRoot[2], shm->robotDeclareTable[0].refRoot[3],
        shm->robotDeclareTable[0].refRoot[4], shm->robotDeclareTable[0].refRoot[5]);

    // EcatMapping
    printf("\n  EcatMapping:\n");
    for (int i = 0; i < 6; i++) {
        printf("    [%d] robotInd=%d, axisInd=%d\n",
            i, shm->ecatMappingTable[i].robotInd, shm->ecatMappingTable[i].axisInd);
    }

    // MaxVel
    printf("\n  Robot[0].maxAxisVel[0] = %.1f  (預期: 180.0)\n", 
        shm->robots[0].spec.maxAxisVel[0]);

    // 驗證結果
    printf("\n--- 驗證 ---\n");
    int pass = 0, fail = 0;

    #define CHECK(name, cond) do { \
        if (cond) { printf("  [PASS] %s\n", name); pass++; } \
        else      { printf("  [FAIL] %s\n", name); fail++; } \
    } while(0)

    CHECK("masterId == 42",           shm->masterId == 42);
    CHECK("NIC == 3",                 shm->NIC == 3);
    CHECK("vGain == 1.0",             shm->vGain == 1.0);
    CHECK("Robot[0].motorNum == 6",   shm->robotDeclareTable[0].motorNum == 6);
    CHECK("refRoot[0] == 100.0",      shm->robotDeclareTable[0].refRoot[0] == 100.0);
    CHECK("refRoot[1] == 200.0",      shm->robotDeclareTable[0].refRoot[1] == 200.0);
    CHECK("refRoot[2] == 300.0",      shm->robotDeclareTable[0].refRoot[2] == 300.0);
    CHECK("EcatMapping[0].robotInd",  shm->ecatMappingTable[0].robotInd == 0);
    CHECK("EcatMapping[5].axisInd",   shm->ecatMappingTable[5].axisInd == 5);
    CHECK("maxAxisVel[0] == 180.0",   shm->robots[0].spec.maxAxisVel[0] == 180.0);

    printf("\n=== 結果: %d PASS, %d FAIL ===\n", pass, fail);

    munmap(shm, shm_size);
    close(shm_fd);

    return (fail > 0) ? 1 : 0;
}
