"""
shm_api.py - Python ctypes wrapper for libShmAPI.so
Provides access to all shared-memory functions exposed by the C library.
"""

import ctypes
import os
import json

# ─── Constants (must match Def.h) ───────────────────────────────────────────
MAX_MOTOR_PER_ROBOT = 7
MAX_REDUNDANCY = 7
MAX_ROBOT_NUM = 4
MAX_MOTOR_NUM = 30
MAX_IO_NUM = 32
MAX_INP_BYTE_PER_IO = 4
MAX_OUT_BYTE_PER_IO = 4
MAX_TOOL_NUM = 10
MAX_BASE_NUM = 10
MAX_IO_CMD_NUM = 10

# ─── Enum replicas ──────────────────────────────────────────────────────────
class eRTXState:
    CLOSE = 0
    OPEN = 1
    RUNNING = 2
    _names = {0: "CLOSE", 1: "OPEN", 2: "RUNNING"}

class eRTXMode:
    CSP = 8
    CST = 10
    _names = {8: "CSP", 10: "CST"}

class eRobotType:
    NONE = 0
    Ext = 600
    Cobot = 800

class eCmdFormat:
    UNKNOWN = 0
    AXIS = 1
    POSE = 2
    PT_TCP = 3
    PT_AXIS = 4
    _names = {0: "UNKNOWN", 1: "AXIS", 2: "POSE", 3: "PT_TCP", 4: "PT_AXIS"}

class eCmdSource:
    NONE = 0
    JOG = 1
    SCRIPT = 2
    HANDGUIDE = 3

class eFrame:
    BASE = 0
    TOOL = 1

class eMovePathType:
    JOINT = 0
    P2P = 1
    LINE = 2
    CIRCLE = 3

class eJointType:
    REVOLUTE = 0
    PRISMATIC = 1

class eSyncType:
    NORMAL = 0
    NONE = 1

class eToolBaseType:
    TOOL = 0
    BASE = 1

class eError:
    NONE = 0
    SCRIPT = 1
    IK_FAIL = 2
    ANGLE_LIMIT = 3
    AXIS_JUMP = 4
    TORQ_LIMIT = 5
    TORQ_COLLISION = 6
    DRIVER_ERROR = 100
    DRIVER_POWER_OFF = 101
    LINK_BROKEN = 102
    _names = {
        0: "NONE", 1: "SCRIPT", 2: "IK_FAIL", 3: "ANGLE_LIMIT",
        4: "AXIS_JUMP", 5: "TORQ_LIMIT", 6: "TORQ_COLLISION",
        100: "DRIVER_ERROR", 101: "DRIVER_POWER_OFF", 102: "LINK_BROKEN",
    }

# ─── ctypes struct for MoveData ─────────────────────────────────────────────
class MoveData(ctypes.Structure):
    _fields_ = [
        ("maxV", ctypes.c_double),
        ("maxA", ctypes.c_double),
        ("contD", ctypes.c_double),
    ]

class IOData(ctypes.Structure):
    _fields_ = [
        ("moduleInd", ctypes.c_int),
        ("bitInd", ctypes.c_int),
        ("status", ctypes.c_bool),
        ("logic", ctypes.c_int),
    ]

class HGParams(ctypes.Structure):
    _fields_ = [
        ("aidRatio", ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("g_ratio",  ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("g_shift",  ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("KV",       ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("KV0",      ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("KA",       ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("KA0",      ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("KC",       ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("errorValue", ctypes.c_int * MAX_MOTOR_PER_ROBOT),
        ("ratedTorq", ctypes.c_double * MAX_MOTOR_PER_ROBOT),
        ("MaxTolerateTime", ctypes.c_int),
        ("payload", ctypes.c_double),
        ("centerX", ctypes.c_double),
        ("centerY", ctypes.c_double),
        ("centerZ", ctypes.c_double),
    ]


# ─── ShmAPI Wrapper ─────────────────────────────────────────────────────────
class ShmAPI:
    """Wraps libShmAPI.so via ctypes. All methods mirror the C API."""

    def __init__(self, lib_path: str | None = None):
        if lib_path is None:
            lib_path = os.path.join(os.path.dirname(__file__), "libShmAPI.so")
        self._lib = ctypes.CDLL(lib_path)
        self._setup_prototypes()
        self._connected = False

    # ── prototype declarations ──────────────────────────────────────────────
    def _setup_prototypes(self):
        L = self._lib

        # init / close / reset
        L.init.restype = ctypes.c_int
        L.closeShm.restype = ctypes.c_int
        L.resetValue.restype = ctypes.c_int

        # RTX process
        L.Set_MasterId.argtypes = [ctypes.c_int]
        L.Set_MasterId.restype = ctypes.c_int
        L.Get_RTXState.restype = ctypes.c_int
        L.Get_EcatState.argtypes = [ctypes.POINTER(ctypes.c_int)]
        L.Get_EcatState.restype = ctypes.c_int
        L.StopRTXProcess.restype = ctypes.c_int
        L.Get_SlaveInfo.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        L.Get_SlaveInfo.restype = ctypes.c_int
        L.Set_RTXMode.argtypes = [ctypes.c_int]
        L.Get_RTXMode.restype = ctypes.c_int
        L.Set_NIC.argtypes = [ctypes.c_int]
        L.Get_ErrorCode.argtypes = [ctypes.c_char_p]
        L.Get_ErrorCode.restype = ctypes.c_int

        # Motor
        L.Set_ServoOn.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char]
        L.Set_ServoOn.restype = ctypes.c_int
        L.Get_IsServoOn.argtypes = [ctypes.c_int, ctypes.c_int]
        L.Get_IsServoOn.restype = ctypes.c_char
        L.Set_IsApplyToRealMotor.argtypes = [ctypes.c_bool]
        L.Get_IsApplyToRealMotor.restype = ctypes.c_char

        # Robot state
        DA = ctypes.c_double * MAX_MOTOR_PER_ROBOT
        DR = ctypes.c_double * MAX_REDUNDANCY
        L.Get_AxisDeg.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        L.Get_Pose.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double),
                               ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        L.Get_Velocity.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        L.Get_Torque.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

        # Mastering & config
        L.Get_MasteringData.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        L.Set_MasteringData.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        L.Set_ReductionRatio.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        L.Set_DHtable.argtypes = [ctypes.c_int] + [ctypes.POINTER(ctypes.c_double)] * 7 + [ctypes.POINTER(ctypes.c_int)]
        L.Set_MaxVelArray.argtypes = [ctypes.c_int, ctypes.c_char, ctypes.POINTER(ctypes.c_double)]
        L.Set_MaxVel.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char, ctypes.c_double]

        # Command
        L.Set_CmdSource.argtypes = [ctypes.c_int]
        L.Set_CmdSource.restype = ctypes.c_int
        L.Set_CmdFormat.argtypes = [ctypes.c_int]
        L.Set_CmdFormat.restype = ctypes.c_int

        # Jog
        L.Set_JogAcc.argtypes = [ctypes.c_double]
        L.Set_JogCmdSingle.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_bool]
        L.Set_UserStopPressJog.restype = None
        L.Set_JogFrame.argtypes = [ctypes.c_int]

        # HG Cobot
        L.Set_HG6Cobot.argtypes = [ctypes.c_int, HGParams]
        L.Set_CollisionMode.argtypes = [ctypes.c_int, ctypes.c_bool]
        L.Set_CollisionMode.restype = ctypes.c_bool
        L.Set_MaxTolerateTime.argtypes = [ctypes.c_int, ctypes.c_int]
        L.Set_MaxTolerateTime.restype = ctypes.c_int
        L.Get_CollisionState.argtypes = [ctypes.c_int]
        L.Get_CollisionState.restype = ctypes.c_int
        L.Set_CollisionState.argtypes = [ctypes.c_int, ctypes.c_int]
        L.Set_CollisionState.restype = ctypes.c_int
        L.Set_URDF.argtypes = [ctypes.c_int] + [ctypes.POINTER(ctypes.c_double)] * 10

        # PID
        L.Set_PID.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        L.Get_PID.argtypes = [ctypes.c_int, ctypes.c_int,
                              ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        L.Save_MotorParams.argtypes = [ctypes.c_int, ctypes.c_int]

        # Script
        L.Clear_SyncTable.restype = None
        L.Set_SyncTable.argtypes = [ctypes.c_int, ctypes.c_int]
        L.StopScript.argtypes = [ctypes.c_char]
        L.Set_IsHMIScriptTerminate.argtypes = [ctypes.c_int, ctypes.c_char]
        L.Get_IsScriptRunning.restype = ctypes.c_char
        L.Get_NowLineId.argtypes = [ctypes.c_int]
        L.Get_NowLineId.restype = ctypes.c_int
        L.Get_NowFuncId.argtypes = [ctypes.c_int]
        L.Get_NowFuncId.restype = ctypes.c_int
        L.Set_VGain.argtypes = [ctypes.c_double]
        L.Set_VGain.restype = ctypes.c_int

        L.Set_ScriptRawCmd_Index.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
        L.Set_ScriptRawCmd_Index.restype = ctypes.c_int
        L.Set_ScriptRawCmd_Move.argtypes = [
            ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.c_char, MoveData,
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_char),
            ctypes.c_int, ctypes.c_int]
        L.Set_ScriptRawCmd_Move.restype = ctypes.c_int
        L.Set_ScriptRawCmd_IO.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(IOData)]
        L.Set_ScriptRawCmd_IO.restype = ctypes.c_int
        L.Set_ScriptRawCmd_Delay.argtypes = [ctypes.c_int, ctypes.c_int]
        L.Set_ScriptRawCmd_Delay.restype = ctypes.c_int
        L.Set_ScriptRawCmd_Sync.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
        L.Set_ScriptRawCmd_Sync.restype = ctypes.c_int
        L.Set_ScriptRawCmd_HMISync.argtypes = [ctypes.c_int]
        L.Set_ScriptRawCmd_HMISync.restype = ctypes.c_int

        # IO
        L.Get_IOInfo.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        L.Get_IOInfo.restype = ctypes.c_int
        L.Get_IOBytes.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_ubyte)]
        L.Get_IOBytes.restype = ctypes.c_int
        L.Set_IO_OUTByte.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_ubyte]
        L.Set_IO_OUTByte.restype = ctypes.c_int
        L.Set_IOBit.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_bool, ctypes.c_bool]
        L.Set_IOBit_Toggle.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_bool]

        # Robot declare
        L.Set_RobotDeclare.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        L.Set_RobotDeclare.restype = ctypes.c_int
        L.Set_EcatMapping.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
        L.Set_EcatMapping.restype = ctypes.c_int

        # Tool/Base
        L.Set_ToolBase.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        L.Set_JogToolInd.argtypes = [ctypes.c_int, ctypes.c_int]
        L.Set_JogBaseInd.argtypes = [ctypes.c_int, ctypes.c_int]

        # Scurve / Jerk
        L.Set_CurveType.argtypes = [ctypes.c_int]
        L.Set_Jerk.argtypes = [ctypes.c_double]

        # Record
        L.Set_BinPath.argtypes = [ctypes.c_char_p]

        # Cross-process HMI signaling
        L.Set_HMICommandReady.argtypes = []
        L.Set_HMICommandReady.restype = None

    # ── High-level API ──────────────────────────────────────────────────────
    def init(self) -> bool:
        rc = self._lib.init()
        self._connected = (rc == 1)
        return self._connected

    def close(self):
        """Unmap shm (does NOT unlink — ecatApp owns lifecycle)."""
        if self._connected:
            self._lib.closeShm()
            self._connected = False

    def reset_value(self):
        return self._lib.resetValue()

    # ── RTX state ───────────────────────────────────────────────────────────
    def get_rtx_state(self) -> int:
        return self._lib.Get_RTXState()

    def get_ecat_state(self) -> int:
        val = ctypes.c_int(0)
        self._lib.Get_EcatState(ctypes.byref(val))
        return val.value

    def get_slave_info(self) -> tuple[int, int]:
        m = ctypes.c_int(0)
        io = ctypes.c_int(0)
        self._lib.Get_SlaveInfo(ctypes.byref(m), ctypes.byref(io))
        return m.value, io.value

    def stop_rtx_process(self):
        return self._lib.StopRTXProcess()

    def set_rtx_mode(self, mode: int):
        self._lib.Set_RTXMode(mode)

    def get_rtx_mode(self) -> int:
        return self._lib.Get_RTXMode()

    def set_nic(self, nic: int):
        self._lib.Set_NIC(nic)

    def get_error_code(self) -> tuple[int, str]:
        buf = ctypes.create_string_buffer(256)
        code = self._lib.Get_ErrorCode(buf)
        return code, buf.value.decode("utf-8", errors="replace")

    # ── Motor ───────────────────────────────────────────────────────────────
    def set_servo_on(self, r: int, m: int, on: bool):
        self._lib.Set_ServoOn(r, m, ctypes.c_char(1 if on else 0))

    def get_is_servo_on(self, r: int, m: int) -> bool:
        return bool(ord(self._lib.Get_IsServoOn(r, m)))

    def get_status_word(self, r: int, m: int) -> int:
        self._lib.Get_StatusWord.restype = ctypes.c_ushort
        return self._lib.Get_StatusWord(r, m)

    def set_apply_to_real_motor(self, real: bool):
        self._lib.Set_IsApplyToRealMotor(real)

    def get_is_apply_to_real_motor(self) -> bool:
        return bool(ord(self._lib.Get_IsApplyToRealMotor()))

    # ── Robot state ─────────────────────────────────────────────────────────
    def get_axis_deg(self, r: int) -> list[float]:
        buf = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)()
        self._lib.Get_AxisDeg(r, buf)
        return list(buf)

    def get_pose(self, r: int) -> tuple[list[float], list[float], list[float]]:
        w = (ctypes.c_double * MAX_REDUNDANCY)()
        root = (ctypes.c_double * MAX_REDUNDANCY)()
        base = (ctypes.c_double * MAX_REDUNDANCY)()
        self._lib.Get_Pose(r, w, root, base)
        return list(w), list(root), list(base)

    def get_velocity(self, r: int) -> list[float]:
        buf = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)()
        self._lib.Get_Velocity(r, buf)
        return list(buf)

    def get_torque(self, r: int) -> list[float]:
        buf = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)()
        self._lib.Get_Torque(r, buf)
        return list(buf)

    # ── Mastering & Config ──────────────────────────────────────────────────
    def get_mastering_data(self, r: int) -> list[float]:
        buf = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)()
        self._lib.Get_MasteringData(r, buf)
        return list(buf)

    def set_mastering_data(self, r: int, m: int, data: list[float]):
        buf = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)(*data[:MAX_MOTOR_PER_ROBOT])
        self._lib.Set_MasteringData(r, m, buf)

    def set_reduction_ratio(self, r: int, ratios: list[float]):
        buf = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)(*ratios[:MAX_MOTOR_PER_ROBOT])
        self._lib.Set_ReductionRatio(r, buf)

    def set_dh_table(self, r: int, a, alpha, d, th_init, th_shift, pos_limit, neg_limit, joint_type):
        def _da(lst): return (ctypes.c_double * MAX_MOTOR_PER_ROBOT)(*lst[:MAX_MOTOR_PER_ROBOT])
        jt = (ctypes.c_int * MAX_MOTOR_PER_ROBOT)(*joint_type[:MAX_MOTOR_PER_ROBOT])
        self._lib.Set_DHtable(r, _da(a), _da(alpha), _da(d), _da(th_init),
                              _da(th_shift), _da(pos_limit), _da(neg_limit), jt)

    def set_max_vel_array(self, r: int, is_axis: bool, vel: list[float]):
        buf = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)(*vel[:MAX_MOTOR_PER_ROBOT])
        self._lib.Set_MaxVelArray(r, ctypes.c_char(1 if is_axis else 0), buf)

    # ── Command ─────────────────────────────────────────────────────────────
    def set_cmd_source(self, source: int):
        self._lib.Set_CmdSource(source)

    def set_cmd_format(self, fmt: int):
        self._lib.Set_CmdFormat(fmt)

    # ── Jog ─────────────────────────────────────────────────────────────────
    def set_jog_acc(self, acc: float):
        self._lib.Set_JogAcc(acc)

    def set_jog_cmd_single(self, r: int, m: int, vel: float, dist: float, limit_dist: bool):
        self._lib.Set_JogCmdSingle(r, m, vel, dist, limit_dist)

    def stop_jog(self):
        self._lib.Set_UserStopPressJog()

    def set_jog_frame(self, frame: int):
        self._lib.Set_JogFrame(frame)

    # ── PID ─────────────────────────────────────────────────────────────────
    def set_pid(self, r: int, m: int, kpp: float, kvp: float, kvi: float):
        self._lib.Set_PID(r, m, kpp, kvp, kvi)

    def get_pid(self, r: int, m: int) -> tuple[float, float, float]:
        kpp = ctypes.c_double(0)
        kvp = ctypes.c_double(0)
        kvi = ctypes.c_double(0)
        self._lib.Get_PID(r, m, ctypes.byref(kpp), ctypes.byref(kvp), ctypes.byref(kvi))
        return kpp.value, kvp.value, kvi.value

    def save_motor_params(self, r: int, m: int):
        self._lib.Save_MotorParams(r, m)

    # ── Script ──────────────────────────────────────────────────────────────
    def stop_script(self, slow: bool = True):
        self._lib.StopScript(ctypes.c_char(1 if slow else 0))

    def set_hmi_script_terminate(self, r: int, terminate: bool):
        self._lib.Set_IsHMIScriptTerminate(r, ctypes.c_char(1 if terminate else 0))

    def get_is_script_running(self) -> bool:
        return bool(ord(self._lib.Get_IsScriptRunning()))

    def get_now_line_id(self, r: int) -> int:
        return self._lib.Get_NowLineId(r)

    def get_now_func_id(self, r: int) -> int:
        return self._lib.Get_NowFuncId(r)

    def set_vgain(self, vgain: float):
        self._lib.Set_VGain(vgain)

    def set_script_raw_cmd_index(self, r: int, line_id: int, func_id: int):
        self._lib.Set_ScriptRawCmd_Index(r, line_id, func_id)

    def set_script_raw_cmd_move(self, r, fmt, path, frame, is_abs, mv: MoveData,
                                 axis_pose, axis_pose_mask, base_idx, tool_idx):
        ap = (ctypes.c_double * MAX_MOTOR_PER_ROBOT)(*axis_pose[:MAX_MOTOR_PER_ROBOT])
        am = (ctypes.c_char * MAX_MOTOR_PER_ROBOT)(*[ctypes.c_char(x) for x in axis_pose_mask[:MAX_MOTOR_PER_ROBOT]])
        self._lib.Set_ScriptRawCmd_Move(r, fmt, path, frame, ctypes.c_char(1 if is_abs else 0),
                                         mv, ap, am, base_idx, tool_idx)

    def set_script_raw_cmd_delay(self, r: int, time_ms: int):
        self._lib.Set_ScriptRawCmd_Delay(r, time_ms)

    def set_script_raw_cmd_hmi_sync(self, r: int):
        self._lib.Set_ScriptRawCmd_HMISync(r)

    def clear_sync_table(self):
        self._lib.Clear_SyncTable()

    # ── IO ──────────────────────────────────────────────────────────────────
    def get_io_info(self, io_ind: int) -> tuple[int, int]:
        inp = ctypes.c_int(0)
        out = ctypes.c_int(0)
        self._lib.Get_IOInfo(io_ind, ctypes.byref(inp), ctypes.byref(out))
        return inp.value, out.value

    def get_io_bytes(self, ind: int) -> tuple[list[int], list[int]]:
        inp = (ctypes.c_ubyte * MAX_INP_BYTE_PER_IO)()
        out = (ctypes.c_ubyte * MAX_OUT_BYTE_PER_IO)()
        self._lib.Get_IOBytes(ind, inp, out)
        return list(inp), list(out)

    def set_io_out_byte(self, m_ind: int, b_ind: int, val: int):
        self._lib.Set_IO_OUTByte(m_ind, b_ind, ctypes.c_ubyte(val))

    def set_io_bit(self, m_id: int, bit_id: int, is_out: bool, value: bool):
        self._lib.Set_IOBit(m_id, bit_id, is_out, value)

    def set_io_bit_toggle(self, m_id: int, bit_id: int, is_out: bool):
        self._lib.Set_IOBit_Toggle(m_id, bit_id, is_out)

    # ── Robot Declare ───────────────────────────────────────────────────────
    def set_robot_declare(self, ind: int, motor_num: int, ref_root: list[float],
                          robot_type: int = eRobotType.Ext):
        buf = (ctypes.c_double * 6)(*ref_root[:6])
        self._lib.Set_RobotDeclare(ind, motor_num, buf, robot_type)

    def set_ecat_mapping(self, ind: int, robot_ind: int, axis_ind: int):
        self._lib.Set_EcatMapping(ind, robot_ind, axis_ind)

    # ── Tool / Base ─────────────────────────────────────────────────────────
    def set_tool_base(self, tb_type: int, ind: int, pose: list[float]):
        buf = (ctypes.c_double * 6)(*pose[:6])
        self._lib.Set_ToolBase(tb_type, ind, buf)

    def set_jog_tool_ind(self, r: int, tool_id: int):
        self._lib.Set_JogToolInd(r, tool_id)

    def set_jog_base_ind(self, r: int, base_id: int):
        self._lib.Set_JogBaseInd(r, base_id)

    # ── Scurve ──────────────────────────────────────────────────────────────
    def set_curve_type(self, on_scurve: int):
        self._lib.Set_CurveType(on_scurve)

    def set_jerk(self, jerk: float):
        self._lib.Set_Jerk(jerk)

    # ── Collision / HG Cobot ────────────────────────────────────────────────
    def set_collision_mode(self, r: int, mode: bool):
        self._lib.Set_CollisionMode(r, mode)

    def get_collision_state(self, r: int) -> int:
        return self._lib.Get_CollisionState(r)

    def set_collision_state(self, r: int, state: int):
        self._lib.Set_CollisionState(r, state)

    # ── URDF ────────────────────────────────────────────────────────────────
    def set_urdf(self, r: int, x, y, z, rx, ry, rz, weight, cx, cy, cz):
        def _da(lst): return (ctypes.c_double * MAX_MOTOR_PER_ROBOT)(*lst[:MAX_MOTOR_PER_ROBOT])
        self._lib.Set_URDF(r, _da(x), _da(y), _da(z), _da(rx), _da(ry),
                           _da(rz), _da(weight), _da(cx), _da(cy), _da(cz))

    # ── HG6Cobot ────────────────────────────────────────────────────────────
    def set_hg6_cobot(self, r: int, params: HGParams):
        self._lib.Set_HG6Cobot(r, params)

    # ── Cross-process HMI signaling ─────────────────────────────────────────
    def set_hmi_command_ready(self):
        """Signal ecatApp that HMI mapping config is done."""
        self._lib.Set_HMICommandReady()

    # ── Config from JSON ────────────────────────────────────────────────────
    def send_robot_config(self, config_path: str | None = None):
        """Load robot_config.json → Set_RobotDeclare, Set_EcatMapping,
           Set_ReductionRatio, Set_DHtable, Set_MaxVelArray, Set_MasteringData,
           Set_URDF, Set_HG6Cobot."""
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "robot_config.json")
        with open(config_path, "r") as f:
            cfg = json.load(f)

        for rob in cfg.get("robots", []):
            rid = rob["id"]
            self.set_robot_declare(rid, rob["motorNum"], rob["refRoot"], rob.get("robotType", eRobotType.Ext))
            self.set_reduction_ratio(rid, rob["reductionRatio"])
            dh = rob["DH"]
            self.set_dh_table(rid, dh["a"], dh["alpha"], dh["d"],
                              dh["thInit"], dh["thShift"],
                              dh["posLimit"], dh["negLimit"], dh["jointType"])
            self.set_max_vel_array(rid, True, rob["maxAxisVel"])
            if "maxTCPVel" in rob:
                self.set_max_vel_array(rid, False, [rob["maxTCPVel"]])
            self.set_mastering_data(rid, 0, rob.get("masteringData", [0]*MAX_MOTOR_PER_ROBOT))

            # URDF
            urdf = rob.get("URDF")
            if urdf:
                self.set_urdf(rid,
                              urdf["x"], urdf["y"], urdf["z"],
                              urdf["Rx"], urdf["Ry"], urdf["Rz"],
                              urdf["weight"],
                              urdf.get("center_x", [0]*MAX_MOTOR_PER_ROBOT),
                              urdf.get("center_y", [0]*MAX_MOTOR_PER_ROBOT),
                              urdf.get("center_z", [0]*MAX_MOTOR_PER_ROBOT))

            # HGParams
            hg = rob.get("HGParams")
            if hg:
                p = HGParams()
                for i, v in enumerate(hg.get("aidRatio", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.aidRatio[i] = v
                for i, v in enumerate(hg.get("g_ratio", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.g_ratio[i] = v
                for i, v in enumerate(hg.get("g_shift", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.g_shift[i] = v
                for i, v in enumerate(hg.get("KV", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.KV[i] = v
                for i, v in enumerate(hg.get("KV0", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.KV0[i] = v
                for i, v in enumerate(hg.get("KA", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.KA[i] = v
                for i, v in enumerate(hg.get("KA0", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.KA0[i] = v
                for i, v in enumerate(hg.get("KC", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.KC[i] = v
                for i, v in enumerate(hg.get("errorValue", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.errorValue[i] = v
                for i, v in enumerate(hg.get("ratedTorq", [])[:MAX_MOTOR_PER_ROBOT]):
                    p.ratedTorq[i] = v
                p.MaxTolerateTime = hg.get("MaxTolerateTime", 100)
                p.payload = hg.get("payload", 0.0)
                p.centerX = hg.get("centerX", 0.0)
                p.centerY = hg.get("centerY", 0.0)
                p.centerZ = hg.get("centerZ", 0.0)
                self.set_hg6_cobot(rid, p)

        for mp in cfg.get("ecatMapping", []):
            self.set_ecat_mapping(mp["slaveIndex"], mp["robotId"], mp["axisId"])

        # Signal ecatApp that mapping is complete
        self.set_hmi_command_ready()
