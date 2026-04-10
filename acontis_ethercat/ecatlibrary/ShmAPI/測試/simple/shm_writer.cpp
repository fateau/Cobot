// shm_writer.cpp — 寫入端：透過 libShmAPI.so 初始化共享記憶體並寫入資料
#include <stdio.h>
#include <dlfcn.h>
#include <unistd.h>

// 匯入的函式指標型別
typedef int  (*fn_init)();
typedef int  (*fn_closeShm)();
typedef int  (*fn_resetValue)();
typedef int  (*fn_Set_MasterId)(int);
typedef void (*fn_Set_NIC)(int);
typedef int  (*fn_Set_RobotDeclare)(int, int, double*, int);
typedef int  (*fn_Set_EcatMapping)(int, int, int);
typedef void (*fn_Set_MaxVel)(int, int, char, double);
typedef int  (*fn_Get_RTXState)();
typedef int  (*fn_Get_SlaveInfo)(int&, int&);

int main()
{
    printf("=== SHM Writer (模擬 RTX/EtherCAT 側) ===\n");

    // 動態載入 libShmAPI.so
    void* handle = dlopen("./libShmAPI.so", RTLD_NOW);
    if (!handle) {
        fprintf(stderr, "dlopen failed: %s\n", dlerror());
        return 1;
    }

    // 取得函式指標
    auto p_init           = (fn_init)dlsym(handle, "init");
    auto p_closeShm       = (fn_closeShm)dlsym(handle, "closeShm");
    auto p_Set_MasterId   = (fn_Set_MasterId)dlsym(handle, "Set_MasterId");
    auto p_Set_NIC        = (fn_Set_NIC)dlsym(handle, "Set_NIC");
    auto p_Set_RobotDeclare = (fn_Set_RobotDeclare)dlsym(handle, "Set_RobotDeclare");
    auto p_Set_EcatMapping  = (fn_Set_EcatMapping)dlsym(handle, "Set_EcatMapping");
    auto p_Set_MaxVel     = (fn_Set_MaxVel)dlsym(handle, "Set_MaxVel");

    if (!p_init || !p_closeShm) {
        fprintf(stderr, "dlsym failed: %s\n", dlerror());
        dlclose(handle);
        return 1;
    }

    // 1) 初始化共享記憶體
    int ret = p_init();
    printf("[Writer] init() returned: %d\n", ret);

    // 2) 設定一些值
    p_Set_MasterId(42);
    printf("[Writer] Set_MasterId(42)\n");

    p_Set_NIC(3);
    printf("[Writer] Set_NIC(3)\n");

    double refRoot[6] = {100.0, 200.0, 300.0, 0.0, 0.0, 0.0};
    p_Set_RobotDeclare(0, 6, refRoot, 0);
    printf("[Writer] Set_RobotDeclare(robot=0, motorNum=6)\n");

    for (int i = 0; i < 6; i++) {
        p_Set_EcatMapping(i, 0, i);
    }
    printf("[Writer] Set_EcatMapping for 6 motors\n");

    p_Set_MaxVel(0, 0, 1, 180.0);
    printf("[Writer] Set_MaxVel(robot=0, motor=0, isAxis=1, vel=180.0)\n");

    printf("\n[Writer] 資料已寫入共享記憶體 '/DataSpace'\n");
    printf("[Writer] 等待 Reader 讀取... (10 秒後自動關閉)\n");
    printf("[Writer] 請在另一個終端執行: ./shm_reader\n\n");

    sleep(10);

    p_closeShm();
    printf("[Writer] closeShm() done. 共享記憶體已釋放。\n");

    dlclose(handle);
    return 0;
}
