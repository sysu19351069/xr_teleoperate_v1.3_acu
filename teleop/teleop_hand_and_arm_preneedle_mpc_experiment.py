import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Value, Array, Lock
import threading
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, Acu_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, Acu_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller, Dex1_1_Gripper_Controller
from teleop.robot_control.robot_hand_inspire import Inspire_Controller
from teleop.robot_control.robot_hand_brainco import Brainco_Controller
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from sshkeyboard import listen_keyboard, stop_listening
from teleop.robot_control.acu_retargeting import AcuNeedleRetargeting
from teleop.robot_control.acu_adaptive_impedance import (
    AcuAdaptiveImpedanceController,
    VelocityOutputLimits,
    SimContactEnv,
    AcuNeedleTeleopImpedanceRunner,
)

# for simulation
# NOTE: unitree_sdk2py is only required in simulation/motion scenarios.
# Import lazily to avoid import errors in environments without unitree_sdk2py.
# from unitree_sdk2py.core.channel import ChannelPublisher
# from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

def publish_reset_category(category: int, publisher):  # Scene Reset signal
    # String_ type is provided by unitree_sdk2py in sim mode.
    msg = publisher._msg_type(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

def _make_pose_dict(position_xyz, quat_xyzw):
    return {
        "position": np.asarray(position_xyz, dtype=float).reshape(3),
        "quat": np.asarray(quat_xyzw, dtype=float).reshape(4),
    }


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _interp_linear(p0: np.ndarray, p1: np.ndarray, t: float) -> np.ndarray:
    t = float(np.clip(t, 0.0, 1.0))
    return (1.0 - t) * p0 + t * p1


def _rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion (xyzw)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = float(np.trace(R))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        # find the major diagonal element
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

    q = np.array([x, y, z, w], dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _yaw_quat(delta_yaw_rad: float) -> np.ndarray:
    """z-axis yaw quaternion (xyzw)."""
    half = 0.5 * float(delta_yaw_rad)
    return np.array([0.0, 0.0, np.sin(half), np.cos(half)], dtype=float)


def _roll_quat(delta_roll_rad: float) -> np.ndarray:
    """x-axis roll quaternion (xyzw)."""
    half = 0.5 * float(delta_roll_rad)
    return np.array([np.sin(half), 0.0, 0.0, np.cos(half)], dtype=float)


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Quaternion multiply (xyzw)."""
    x1, y1, z1, w1 = np.asarray(q1, dtype=float).reshape(4)
    x2, y2, z2, w2 = np.asarray(q2, dtype=float).reshape(4)
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def _quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between quaternions (xyzw)."""
    t = float(np.clip(t, 0.0, 1.0))
    q0 = _quat_normalize(q0)
    q1 = _quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    # handle double-cover: pick the shortest path
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        # very close: fallback to lerp
        return _quat_normalize((1.0 - t) * q0 + t * q1)

    theta_0 = float(np.arccos(dot))
    sin_theta_0 = float(np.sin(theta_0))
    theta = theta_0 * t
    sin_theta = float(np.sin(theta))

    s0 = float(np.sin(theta_0 - theta) / sin_theta_0)
    s1 = float(sin_theta / sin_theta_0)
    return _quat_normalize(s0 * q0 + s1 * q1)


def _quat_angle(q1: np.ndarray, q2: np.ndarray) -> float:
    """Quaternion angular distance (radians), xyzw."""
    q1 = np.asarray(q1, dtype=float).reshape(4)
    q2 = np.asarray(q2, dtype=float).reshape(4)
    q1 = q1 / (np.linalg.norm(q1) + 1e-12)
    q2 = q2 / (np.linalg.norm(q2) + 1e-12)
    if float(np.dot(q1, q2)) < 0.0:
        q2 = -q2
    w = float(np.clip(float(np.dot(q1, q2)), -1.0, 1.0))
    return 2.0 * float(np.arccos(w))


def _safe_np_savez(path: str, **kwargs) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez_compressed(path, **kwargs)


class TeleopCompareDataLogger:
    """采集 MPC vs IK 两次实验的对比数据（右臂8DoF，离线分析用）。

    约束：
    - 只在非 acu_needle_teleop 模式下采集（该模式覆盖了控制逻辑，不属于 MPC/IK 对比范畴）。
    - q/dq 必须来自 arm_ctrl.get_current_dual_arm_q/dq 的实测值。
    - ee_meas 由 teleop_mpc_api.ctl.robot.fk(q_meas[:8]) 计算。
    - xr_ref 使用当前 teleop_ref（已 anchor 映射到机器人坐标系的一致参考）。
    """

    def __init__(self, *, out_dir: str, method: str) -> None:        
        self.out_dir = out_dir
        self.method = str(method)
        self._t0 = time.time()
        self._rows: list[dict] = []

    def add(
        self,
        *,
        stamp: float,
        q_meas8: np.ndarray,
        dq_meas8: np.ndarray | None,
        ee_p: np.ndarray,
        ee_quat_xyzw: np.ndarray,
        xr_p: np.ndarray,
        xr_quat_xyzw: np.ndarray,
        pf_dbg: dict | None = None,
        # MPC-PF specific optional fields
        proj_p: np.ndarray | None = None,
        p_des: np.ndarray | None = None,
        target_p: np.ndarray | None = None,
        target_quat_xyzw: np.ndarray | None = None,
    ) -> None:
        q_meas8 = np.asarray(q_meas8, dtype=float).reshape(8)
        dq_meas8 = None if dq_meas8 is None else np.asarray(dq_meas8, dtype=float).reshape(8)
        ee_p = np.asarray(ee_p, dtype=float).reshape(3)
        ee_quat_xyzw = _quat_normalize(np.asarray(ee_quat_xyzw, dtype=float).reshape(4))
        xr_p = np.asarray(xr_p, dtype=float).reshape(3)
        xr_quat_xyzw = _quat_normalize(np.asarray(xr_quat_xyzw, dtype=float).reshape(4))

        # Optional PF geometry
        proj_p_arr = None if proj_p is None else np.asarray(proj_p, dtype=float).reshape(3)
        p_des_arr = None if p_des is None else np.asarray(p_des, dtype=float).reshape(3)
        target_p_arr = None if target_p is None else np.asarray(target_p, dtype=float).reshape(3)
        target_q_arr = None if target_quat_xyzw is None else _quat_normalize(np.asarray(target_quat_xyzw, dtype=float).reshape(4))

        err_p = ee_p - xr_p
        err_pos_norm = float(np.linalg.norm(err_p))
        
        # err_ori_rad = float(_quat_angle(ee_quat_xyzw, xr_quat_xyzw))
        # Orientation error:
        # - Default: compare to XR quat
        # - If target_quat is available (MPC-PF): compare to segment-end quat
        # - On segment switch: freeze last orientation error for one sample (to avoid using
        #   a just-updated XR ref or a not-yet-consistent target)
        err_ori_ref = target_q_arr if target_q_arr is not None else xr_quat_xyzw
        err_ori_rad_raw = float(_quat_angle(ee_quat_xyzw, err_ori_ref))

        seg_i = None
        if isinstance(pf_dbg, dict):
            try:
                seg_i = int(pf_dbg.get("seg_i")) if pf_dbg.get("seg_i") is not None else None
            except Exception:
                seg_i = None

        if seg_i is not None and (self._last_pf_seg_i is None):
            self._last_pf_seg_i = seg_i

        if seg_i is not None and (self._last_pf_seg_i is not None) and (seg_i != self._last_pf_seg_i):
            # segment switched: freeze last last-segment error for the next sample
            self._freeze_next_ori_err = True
            self._last_pf_seg_i = seg_i

        if self._freeze_next_ori_err and (self._last_ori_err_rad is not None):
            err_ori_rad = float(self._last_ori_err_rad)
            self._freeze_next_ori_err = False
        else:
            err_ori_rad = float(err_ori_rad_raw)
            self._last_ori_err_rad = err_ori_rad

        # PF errors: projection / lookahead / segment-end
        err_proj = None
        err_des = None
        err_end_pos = None
        if proj_p_arr is not None:
            err_proj = float(np.linalg.norm(ee_p - proj_p_arr))
        if p_des_arr is not None:
            err_des = float(np.linalg.norm(ee_p - p_des_arr))
        if target_p_arr is not None:
            err_end_pos = float(np.linalg.norm(ee_p - target_p_arr))

        self._rows.append(
            {
                "t": float(stamp - self._t0),
                "stamp": float(stamp),
                "q_meas8": q_meas8,
                "dq_meas8": dq_meas8,
                "ee_p": ee_p,
                "ee_quat": ee_quat_xyzw,
                "xr_p": xr_p,
                "xr_quat": xr_quat_xyzw,
                "err_p": err_p,
                "err_pos": err_pos_norm,
                "err_ori": err_ori_rad,
                "pf_dbg": pf_dbg,
                "proj_p": proj_p_arr,
                "p_des": p_des_arr,
                "target_p": target_p_arr,
                "target_quat": target_q_arr,
                "err_proj": err_proj,
                "err_des": err_des,
                "err_end_pos": err_end_pos,
            }
        )

    def save(self, *, extra_meta: dict | None = None) -> str:
        if len(self._rows) == 0:
            return ""

        def _stack(key: str, *, allow_none: bool = False):
            vals = [r[key] for r in self._rows]
            if allow_none and any(v is None for v in vals):
                return None
            return np.stack(vals, axis=0)

        def _stack_or_nan_1d(key: str) -> np.ndarray:
            vals = [r.get(key, None) for r in self._rows]
            out = np.full((len(vals),), np.nan, dtype=float)
            for i, v in enumerate(vals):
                if v is None:
                    continue
                out[i] = float(v)
            return out

        def _stack_or_nan_3d(key: str) -> np.ndarray:
            vals = [r.get(key, None) for r in self._rows]
            out = np.full((len(vals), 3), np.nan, dtype=float)
            for i, v in enumerate(vals):
                if v is None:
                    continue
                out[i, :] = np.asarray(v, dtype=float).reshape(3)
            return out

        def _stack_or_nan_4d(key: str) -> np.ndarray:
            vals = [r.get(key, None) for r in self._rows]
            out = np.full((len(vals), 4), np.nan, dtype=float)
            for i, v in enumerate(vals):
                if v is None:
                    continue
                out[i, :] = np.asarray(v, dtype=float).reshape(4)
            return out

        meta = {
            "method": self.method,
            "t0": float(self._t0),
            "n": int(len(self._rows)),
        }
        if extra_meta:
            meta.update(extra_meta)

        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(self._t0))
        out_path = os.path.join(self.out_dir, f"teleop_compare_{self.method}_{ts}.npz")

        # pf_dbg 可能是 dict（长度可变），用 object 数组保存
        pf_dbg_arr = np.array([r["pf_dbg"] for r in self._rows], dtype=object)

        _safe_np_savez(
            out_path,
            meta=np.array(meta, dtype=object),
            t=_stack("t"),
            stamp=_stack("stamp"),
            q_meas8=_stack("q_meas8"),
            dq_meas8=_stack("dq_meas8", allow_none=True),
            ee_p=_stack("ee_p"),
            ee_quat=_stack("ee_quat"),
            xr_p=_stack("xr_p"),
            xr_quat=_stack("xr_quat"),
            err_p=_stack("err_p"),
            err_pos=_stack("err_pos"),
            err_ori=_stack("err_ori"),
            pf_dbg=pf_dbg_arr,
            proj_p=_stack_or_nan_3d("proj_p"),
            p_des=_stack_or_nan_3d("p_des"),
            target_p=_stack_or_nan_3d("target_p"),
            target_quat=_stack_or_nan_4d("target_quat"),
            err_proj=_stack_or_nan_1d("err_proj"),
            err_des=_stack_or_nan_1d("err_des"),
            err_end_pos=_stack_or_nan_1d("err_end_pos"),
        )
        return out_path


class TeleopRefReplayer:
    def __init__(self, ref_npz_path: str):
        self._d = np.load(ref_npz_path, allow_pickle=True)
        # 新格式：XR参考按主循环频率保存，回放按计数器推进
        self.xr_p = np.asarray(self._d["xr_p"], dtype=float).reshape(-1, 3)
        self.xr_quat = np.asarray(self._d["xr_quat"], dtype=float).reshape(-1, 4)
        self._i = 0
        # truncate_tail = 0
        # self.xr_p = self.xr_p[:-truncate_tail]
        # self.xr_quat = self.xr_quat[:-truncate_tail]

        self.xr_p[-30:, 2] -= 0.045

    def reset(self) -> None:
        self._i = 0

    def sample_next(self) -> dict | None:
        if self._i >= len(self.xr_p):
            return None
        pose = _make_pose_dict(self.xr_p[self._i], self.xr_quat[self._i])
        self._i += 1
        return pose


class XRTeleopRefLogger:
    """以主循环频率保存XR参考(teleop_ref)，用于IK第二次实验按帧回放。"""

    def __init__(self) -> None:
        self._t0 = time.time()
        self._rows: list[dict] = []

    def add(self, *, stamp: float, teleop_ref: dict) -> None:
        if teleop_ref is None:
            return
        p = teleop_ref.get("position")
        q = teleop_ref.get("quat")
        if p is None or q is None:
            return
        self._rows.append(
            {
                "t": float(stamp - self._t0),
                "stamp": float(stamp),
                "xr_p": np.asarray(p, dtype=float).reshape(3).copy(),
                "xr_quat": _quat_normalize(np.asarray(q, dtype=float).reshape(4).copy()),
            }
        )

    def save(self, *, out_dir: str, tag: str = "") -> str:
        if not self._rows:
            return ""
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(self._t0))
        suffix = f"_{tag}" if tag else ""
        out_path = os.path.join(str(out_dir), f"teleop_xr_ref{suffix}_{ts}.npz")
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        np.savez_compressed(
            out_path,
            meta=np.array({"t0": float(self._t0), "n": int(len(self._rows)), "tag": str(tag)}, dtype=object),
            t=np.asarray([r["t"] for r in self._rows], dtype=float),
            stamp=np.asarray([r["stamp"] for r in self._rows], dtype=float),
            xr_p=np.stack([r["xr_p"] for r in self._rows], axis=0),
            xr_quat=np.stack([r["xr_quat"] for r in self._rows], axis=0),
        )
        return out_path


# state transition
START          = False  # Enable to start robot following VR user motion  
STOP           = False  # Enable to begin system exit procedure
RECORD_TOGGLE  = False  # [Ready] ⇄ [Recording] ⟶ [AutoSave] ⟶ [Ready]         (⇄ manual) (⟶ auto)
RECORD_RUNNING = False  # True if [Recording]
RECORD_READY   = True   # True if [Ready], False if [Recording] / [AutoSave]
# task info
TASK_NAME = None
TASK_DESC = None
ITEM_ID = None
def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def on_info(info):
    """Only handle CMD_TOGGLE_RECORD's task info"""
    global TASK_NAME, TASK_DESC, ITEM_ID
    TASK_NAME   = info.get("task_name")
    TASK_DESC   = info.get("task_desc")
    ITEM_ID     = info.get("item_id")
    logger_mp.debug(f"[on_info] Updated globals: {TASK_NAME}, {TASK_DESC}, {ITEM_ID}")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, RECORD_READY
    return {
        "START": START,
        "STOP": STOP,
        "RECORD_RUNNING": RECORD_RUNNING,
        "RECORD_READY": RECORD_READY,
    }

# ------------------------------
# ACU teleop_api MPC: high-rate path-following thread
# ------------------------------
mpc_pf_thread = None
mpc_pf_stop = threading.Event()

# 主线程最新遥操参考（用于 MPC 线程记录 ground truth，不用于 MPC 控制）
latest_teleop_ref = None
latest_teleop_ref_lock = threading.Lock()


def _acu_mpc_pf_thread(frequency_hz: float):
    """高频线程：对 teleop_mpc_api 的 path-following 做 step + 下发控制。

    - 与 teleop_api.py 的设计一致：
      * 主线程 ingest_teleop_pose（构建 WaypointPath）
      * 高频线程 step_with_path_following（投影+前瞻产生 Sd，并把 segment end pose 作为 target_pose）
    - 重要：线程内必须实时读取电机 q，不能用主循环缓存。
    """
    global mpc_pf_stop
    global teleop_mpc_api, arm_ctrl
    global args, mpc_ref_enabled
    global compare_logger
    global latest_teleop_ref, latest_teleop_ref_lock

    dt = 1.0 / float(max(1e-6, frequency_hz))
    q_next = None
    sol_q = None
    while not mpc_pf_stop.is_set():
        tick_t0 = time.time()
        try:
            if teleop_mpc_api is None or arm_ctrl is None:
                time.sleep(dt)
                continue

            # gate off: do not move
            if not bool(mpc_ref_enabled):
                time.sleep(dt)
                continue
            
            # realtime measured q (right arm 8dof)
            # q_all = arm_ctrl.get_current_dual_arm_q()
            # q_meas = np.asarray(q_all, dtype=float).reshape(-1)[:8].copy()
            # q_meas[6] = 0.0

            # q_all = arm_ctrl.get_current_dual_arm_q()
            q_meas = np.asarray([0.0, 0.29175, 1.31388, 0.0, 1.51938, 3.14159, 0.0, 0.0], dtype=float).reshape(-1)[:8].copy() if sol_q is None else sol_q.copy()
            q_meas[6] = 0.0

            # current pose for projection should be consistent with the frame used in teleop_api path
            fk = teleop_mpc_api.ctl.robot.fk(q_meas)
            cur_p = np.asarray(fk.translation).reshape(3).copy()
            cur_pose_for_pf = {"position": cur_p}

            now = time.time()
            q_next = teleop_mpc_api.step_with_path_following(cur_pose_for_pf, now, qk=q_meas)
            # print(q_next)
            if q_next is not None:
                sol_q = np.asarray(q_next, dtype=float).reshape(-1)
                sol_tauff = np.zeros_like(sol_q)
                # print(sol_q)
                a = sol_q[5]
                sol_q[5] = 3.14159
                print(sol_q)
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
                sol_q[5] = a
                # print("i am here")
                # print(sol_q)

            # Compare-log sampling for MPC-PF
            if compare_logger is not None and bool(getattr(args, 'compare_log', False)) and bool(getattr(args, 'use_mpc_control', False)):
                # measured dq optional
                dq_meas8 = None
                try:
                    dq_all = arm_ctrl.get_current_dual_arm_dq()
                    dq_meas8 = np.asarray(dq_all, dtype=float).reshape(-1)[:8].copy()
                except Exception:
                    dq_meas8 = None

                # EE quat from FK rotation
                ee_quat = _quat_normalize(_rot_to_quat_xyzw(np.asarray(fk.rotation)))

                # pull pf debug snapshot (contains proj/p_des/target)
                dbg = getattr(teleop_mpc_api, '_dbg_last_pf', None)
                proj_p = None
                p_des = None
                target_p = None
                target_q = None
                if isinstance(dbg, dict):
                    proj_p = dbg.get('proj_p', None)
                    p_des = dbg.get('p_des', None)
                    target_p = dbg.get('target_p', None)
                    target_q = dbg.get('target_q', None)

                # XR ground truth for logging: use latest teleop reference (not segment-end)
                xr_p = None
                xr_quat = None
                try:
                    with latest_teleop_ref_lock:
                        _ref = latest_teleop_ref
                    if isinstance(_ref, dict) and (_ref.get('position') is not None) and (_ref.get('quat') is not None):
                        xr_p = np.asarray(_ref['position'], dtype=float).reshape(3).copy()
                        xr_quat = _quat_normalize(np.asarray(_ref['quat'], dtype=float).reshape(4).copy())
                except Exception:
                    xr_p, xr_quat = None, None

                # print(xr_p)
                # fallback (保证字段存在)：如果暂时没有最新参考，则用 cur_p/identity
                if xr_p is None:
                    xr_p = cur_p
                if xr_quat is None:
                    xr_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

                compare_logger.add(
                    stamp=float(now),
                    q_meas8=q_meas,
                    dq_meas8=dq_meas8,
                    ee_p=cur_p,
                    ee_quat_xyzw=ee_quat,
                    xr_p=xr_p,
                    xr_quat_xyzw=xr_quat,
                    pf_dbg=dbg,
                    proj_p=proj_p,
                    p_des=p_des,
                    target_p=target_p,
                    target_quat_xyzw=target_q,
                )

        except Exception:
            pass

        # keep frequency
        elapsed = time.time() - tick_t0
        time.sleep(max(0.0, dt - elapsed))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'save data\'s frequency')

    # basic control parameters
    parser.add_argument('--xr-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device tracking source')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'ACU'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire1', 'brainco'], help='Select end effector controller')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task name for recording')
    parser.add_argument('--task-desc', type = str, default = 'e.g. pick the red cube on the table.', help = 'task goal for recording')

    # teleop_api path-following test inputs (ACU only)
    parser.add_argument('--mpc-teleop-api', action='store_true', help='Use mpc_py.teleop_api TeleopAcuMPC with path following (ACU only)')
    parser.add_argument('--mpc-test-input', action='store_true', help='Feed built-in waypoint test inputs instead of real XR right wrist pose (ACU only)')
    parser.add_argument('--mpc-test-hold', type=float, default=1.0, help='Seconds to hold each test segment end before switching')
    parser.add_argument('--mpc-test-speed', type=float, default=0.05, help='m/s along the segment for generating intermediate points')

    # gate teleop reference streaming by controller trigger
    parser.add_argument('--mpc-stream-on-trigger', action='store_true', help='Only ingest XR teleop reference when right controller trigger is pressed (ACU + controller mode)')
    parser.add_argument('--mpc-trigger-threshold', type=float, default=0.2, help='Trigger threshold in [0,10] to start ingesting teleop reference')
    parser.add_argument('--mpc-stream-toggle', action='store_true', help='Latch teleop streaming ON after the first right-trigger press (one-shot start)')
    parser.add_argument('--use-mpc-control', action='store_true', help='True to use mpc control for teleop or False to use pure ik control') 

    # Acu needle manipulation (行针手法) tracking (hand mode)
    parser.add_argument('--acu-needle-teleop', action='store_true', help='Enable Acu needle manipulation retargeting (insert+twist) from right hand tracking; overrides MPC ingest/PF when enabled')
    parser.add_argument('--acu-needle-mode', type=str, default='incremental', choices=['absolute', 'incremental'], help='Needle retargeting mode (absolute vs incremental)')
    parser.add_argument('--acu-needle-insert-angle-deg', type=float, default=35.0, help='Insert gate angle threshold (deg) for dv_index vs horizontal')
    parser.add_argument('--acu-needle-twist-angle-deg', type=float, default=20.0, help='Twist gate angle threshold (deg) for horizontal motion')
    parser.add_argument('--acu-needle-circumference-mm', type=float, default=2.0, help='Needle circumference in mm for mapping lateral motion to twist angle')
    parser.add_argument('--acu-needle-insert-gain', type=float, default=1.0, help='Insert gain')
    parser.add_argument('--acu-needle-twist-gain', type=float, default=1.0, help='Twist gain')
    parser.add_argument('--acu-needle-insert-positive-down', action='store_true', help='Define insertion positive direction as -Z of TeleVuer coordinates')

    # compare logger
    parser.add_argument('--compare-log', action='store_true', help='采集 MPC vs IK 离线对比数据（非 acu_needle_teleop）。')
    parser.add_argument('--compare-out-dir', type=str, default='./teleop_compare_logs_experiment', help='对比数据输出目录（npz）。')
    parser.add_argument('--compare-tag', type=str, default='', help='可选：写入meta的实验tag（例如被试/场景/日期）。')

    parser.add_argument('--replay-ref', type=str, default='', help='IK对比用：回放MPC实验保存的XR参考npz（teleop_compare_mpc_*.npz）。')

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    # compare data logger (init after parsing)
    compare_logger = None
    if bool(args.compare_log):
        method = 'mpc' if bool(args.use_mpc_control) else 'ik'
        compare_logger = TeleopCompareDataLogger(out_dir=str(args.compare_out_dir), method=method)

    # XR ref logger always follows main loop frequency when compare_log is enabled
    xr_ref_logger = XRTeleopRefLogger() if bool(args.compare_log) else None

    ref_replayer = None
    if str(getattr(args, 'replay_ref', '')).strip():
        try:
            ref_replayer = TeleopRefReplayer(str(args.replay_ref).strip())
            logger_mp.info(f"[compare] replay-ref enabled: {args.replay_ref}")
        except Exception as e:
            logger_mp.error(f"[compare] failed to load replay-ref: {e}")
            ref_replayer = None

    try:
        # ipc communication. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press, on_info=on_info, get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False,}, daemon=True)
            listen_keyboard_thread.start()

        # image client: img_config should be the same as the configuration in image_server.py (of Robot's development computing unit)
        if args.sim:
            img_config = {
                'fps': 30,
                'head_camera_type': 'opencv',
                'head_camera_image_shape': [480, 640],  # Head camera resolution
                'head_camera_id_numbers': [0],
                'wrist_camera_type': 'opencv',
                'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
                'wrist_camera_id_numbers': [2, 4],
            }
        else:
            img_config = {
                'fps': 30,
                'head_camera_type': 'opencv',
                'head_camera_image_shape': [720, 1280],  # Head camera resolution
                'head_camera_id_numbers': [0],
                'wrist_camera_type': 'opencv',
                'wrist_camera_image_shape': [720, 1280],  # Wrist camera resolution
                'wrist_camera_id_numbers': [2, 4],
            }


        ASPECT_RATIO_THRESHOLD = 2.0 # If the aspect ratio exceeds this value, it is considered binocular
        if len(img_config['head_camera_id_numbers']) > 1 or (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
            BINOCULAR = True
        else:
            BINOCULAR = False
        if 'wrist_camera_type' in img_config:
            WRIST = True
        else:
            WRIST = False
        
        # Determine tv (full) image shape.
        # NOTE: ImageServer publishes full_color = hconcat([head_color, wrist_color]) when wrist is enabled.
        # ImageClient now writes the full decoded frame into tv_img_array (no slicing), so tv_img_shape must
        # match the full image.
        head_h = int(img_config['head_camera_image_shape'][0])
        head_w = int(img_config['head_camera_image_shape'][1])
        wrist_w = 0
        if WRIST:
            # wrist_image_shape width is per single wrist camera; number of wrist cameras determines concat width
            wrist_w = int(img_config['wrist_camera_image_shape'][1]) * int(len(img_config.get('wrist_camera_id_numbers', []) or []))

        if BINOCULAR and not (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
            head_full_w = head_w * 2
        else:
            head_full_w = head_w

        head_full_w = head_w
        tv_img_shape = (head_h, head_w, 3)
        print(tv_img_shape)

        tv_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(tv_img_shape) * np.uint8().itemsize)
        tv_img_array = np.ndarray(tv_img_shape, dtype = np.uint8, buffer = tv_img_shm.buf)

        if WRIST and args.sim:
            wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
            wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
            wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                    wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name, server_address="127.0.0.1")
        elif WRIST and not args.sim:
            wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
            wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
            wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                    wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name, server_address="172.20.10.5")
        else:
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, server_address="172.20.10.5")

        image_receive_thread = threading.Thread(target = img_client.receive_process, daemon = True)
        image_receive_thread.daemon = True
        image_receive_thread.start()

        is_acu = (args.arm == 'ACU')
        # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(binocular=BINOCULAR, use_hand_tracking=args.xr_mode == "hand", img_shape=tv_img_shape, img_shm_name=tv_img_shm.name, 
                                    return_state_data=True, return_hand_rot_data = False,
                                    # enable Acu controller teleop mapping only for Acu + controller tracking
                                    use_acu_controller_teleop=(is_acu and args.xr_mode == 'controller'),
                                    use_acu_hand_teleop=(is_acu and args.xr_mode == 'hand'),)

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "ACU":
            arm_ik = Acu_ArmIK()
            arm_ctrl = Acu_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        
        current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
        arm_ik.reset_init_joint_pos(current_lr_arm_q)

        # ------------------------------
        # MPC init (moved here, after arm_ctrl exists)
        # ------------------------------
        if args.arm == 'ACU':
            if args.mpc_teleop_api:
                try:
                    from mpc_py.teleop_api import TeleopAcuMPC, TeleopMPCConfig, _quat_to_rot

                    urdf_path = os.path.join(parent_dir, 'assets', 'acu', 'acu.urdf')
                    teleop_mpc_api = TeleopAcuMPC(
                        TeleopMPCConfig(
                            urdf_path=urdf_path,
                            ee_frame='Acu_ee',
                            dt=0.1,
                            horizon_T=1.0,
                            solver_backend='osqp',
                        )
                    )

                    _init_q = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)
                    _init_q[6] = 0.0  
                    # _init_q = np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 3.13, -0.0, 0.0], dtype=float)
                    q0 = _init_q[:8].copy() if _init_q.size >= 8 else np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 3.13, -0.0, 0.0], dtype=float)

                    # rough goal only used to initialize internal mode/state; then we drive via path following.
                    teleop_mpc_api.reset_with_rough_goal(q0=q0, goal_xyz=np.array([0.250, 0.450, -0.35], dtype=float))

                    # teleop_mpc_api.switch_to_accurate(v=np.array([-0.0, -0.5, 2.0], dtype=float), r=0.05)

                    logger_mp.info('[MPC] teleop_api TeleopAcuMPC initialized.')
                except Exception as e:
                    logger_mp.error(f"[MPC] Failed to init teleop_api TeleopAcuMPC, fallback. err={e}")
                    teleop_mpc_api = None

            # keep legacy init for backward compatibility
            if teleop_mpc_api is None:
                try:
                    from mpc_py.controller import AcuMPCController, MPCMode
                    from mpc_py.planner_sd import compute_sd_rough, init_rough_plan

                    try:
                        import pinocchio as pin
                    except Exception:  # pragma: no cover
                        import pin  # type: ignore

                    urdf_path = os.path.join(parent_dir, 'assets', 'acu', 'acu.urdf')
                    mpc_ctl = AcuMPCController(
                        urdf_path=urdf_path,
                        dt=0.1,
                        horizon_T=1.0,
                        ee_frame='Acu_ee',
                        solver_backend='osqp',
                    )

                    _init_q = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)
                    if _init_q.size >= 8:
                        mpc_qk = _init_q[:8].copy()
                    else:
                        mpc_qk = np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 0.0, -0.0, 0.0], dtype=float)

                    mpc_qk = np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 3.13, -0.0, 0.0], dtype=float)
                    curpos = mpc_ctl.robot.fk(mpc_qk)
                    cur_p = np.asarray(curpos.translation).reshape(3)
                    O = np.array([0.250, 0.450, -0.35], dtype=float)
                    mpc_rough_plan = init_rough_plan(P0=cur_p, O=O)
                    mpc_target_rough = pin.SE3(np.asarray(curpos.rotation), O)

                    logger_mp.info('[MPC] ACU MPC controller initialized.')
                except Exception as e:
                    logger_mp.error(f"[MPC] Failed to init ACU MPC controller, fallback to IK. err={e}")
                    mpc_ctl = None

        # end-effector
        if args.ee == "dex3":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "dex1":
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire1":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "brainco":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20) # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            try:
                from unitree_sdk2py.core.channel import ChannelPublisher
                from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

                reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
                # stash msg type for publish_reset_category
                reset_pose_publisher._msg_type = String_
                reset_pose_publisher.Init()
                from teleop.utils.sim_state_topic import start_sim_state_subscribe
                sim_state_subscriber = start_sim_state_subscribe()
            except Exception as e:
                logger_mp.error(f"unitree_sdk2py not available for --sim mode: {e}")
                args.sim = False

        # controller + motion mode
        if args.xr_mode == "controller" and args.motion:
            try:
                from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
                sport_client = LocoClient()
                sport_client.SetTimeout(0.0001)
                sport_client.Init()
            except Exception as e:
                logger_mp.error(f"unitree_sdk2py not available for --motion mode: {e}")
                args.motion = False

        # record + headless mode
        if args.record and args.headless:
            recorder = EpisodeWriter(task_dir = args.task_dir + args.task_name, task_goal = args.task_desc, frequency = args.frequency, rerun_log = False)
        elif args.record and not args.headless:
            recorder = EpisodeWriter(task_dir = args.task_dir + args.task_name, task_goal = args.task_desc, frequency = args.frequency, rerun_log = True)

        # after arm_ctrl is created and before main loop
        if args.arm == 'ACU' and args.acu_needle_teleop:
            needle_right_hand_pos_array = Array('d', 75, lock=True)
            needle_lock = Lock()
            needle_state_array = Array('d', 2, lock=False)
            needle_action_array = Array('d', 2, lock=False)
            needle_meta_array = Array('d', 3, lock=False)  # [ts, insert_vel, twist_vel]
            needle_retargeter = AcuNeedleRetargeting(
                needle_right_hand_pos_array,
                data_lock=needle_lock,
                needle_state_array_out=needle_state_array,
                needle_action_array_out=needle_action_array,
                needle_meta_array_out=needle_meta_array,
                fps=float(max(1.0, args.frequency)),
                needle_circumference_m=float(max(1e-9, args.acu_needle_circumference_mm * 1e-3)),
                insert_angle_threshold_deg=float(args.acu_needle_insert_angle_deg),
                twist_angle_threshold_deg=float(args.acu_needle_twist_angle_deg),
                insert_gain=float(args.acu_needle_insert_gain),
                twist_gain=float(args.acu_needle_twist_gain),
                mode=str(args.acu_needle_mode),
                insert_positive_down=bool(args.acu_needle_insert_positive_down),
            )

            # Init adaptive impedance controller (use simple default stable numbers; can be tuned)
            # Filter coefficients: placeholder 2nd order IIR; replace with paper-identified coefficients if available.
            acu_imp = AcuAdaptiveImpedanceController(
                kf=1.0,
                f_safe=20.0,  # N
                filter_coeffs=(2.732e-5, 5.464e-5, 2.932e-5, -0.8415, -0.0929),
                fr=20.0,  # N target during impedance mode
                dt=float(1.0 / max(args.frequency, 1e-6)),
                hysteresis=0.2,
                latch=False,
                v_limits=VelocityOutputLimits(vmax=30),
                integrator_leak=0.02,
            )
            acu_imp_runner = AcuNeedleTeleopImpedanceRunner(
                controller=acu_imp,
                # env=SimContactEnv(ke=1.4, be=0.1, xe=0.0, mode='const', noise_mode='gaussian', noise_sigma=0.05, noise_seed=0),
                env=SimContactEnv(ke=1.4, be=0.1, xe=0.0),
                arm_ctrl=arm_ctrl,
                needle_lock=needle_lock,
                needle_action_array=needle_action_array,
                needle_meta_array=needle_meta_array,
                frequency_hz=float(max(1.0, args.frequency)),
                joint_index=6,
                reference_source='retargeting',
                stop_after_s=8.0,
                plot_save_path=os.path.join(os.getcwd(), 'acu_adaptive_impedance_retargeting_summary.png'),
            )

        logger_mp.info("Please enter the start signal (enter 'r' to start the subsequent program)")
        while not START and not STOP:
            time.sleep(0.01)
        logger_mp.info("start program.")
        arm_ctrl.speed_gradual_max()

        time.sleep(3.0)
        # For incremental Acu IK: cache last right wrist pose. The first iteration only records it.
        right_wrist_pose_last = None

        # MPC teleop streaming gate state
        mpc_ref_enabled = False
        mpc_ref_prev_pressed = False
        mpc_ref_latched = False
        # Hand mode: run the "clear path + anchors + seed" init only once per session
        # to avoid repeated triggers due to pinch jitter.
        mpc_hand_init_done = False

        # ACU needle teleop: start only after a single pinch trigger in hand mode
        needle_ref_enabled = False
        needle_init_done = False
        needle_base_q = None  # cache once: first 6 DOF (and other non-needle joints) baseline
        # step_once scheme: no background loop; runner is called once per main tick

        # Anchors for ACU XR->robot mapping (only set when trigger is pressed)
        args._acu_xr_p0 = None
        args._acu_ee_p0 = None

        # teleop_api test inputs (ACU only)
        # will be overwritten by the initial EE FK orientation when the test path is initialized.
        test_quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        test_waypoints = None
        test_seg_i = 0
        test_seg_start_t = None
        test_seg_p0 = None

        while not STOP:
            start_time = time.time()
            now = time.time()

            # Ensure variables exist for downstream logging/recording even if this tick early-continues.
            sol_q = None
            sol_tauff = None
            # teleop_ref = None

            # # --- Pure accurate test (ACU + TeleopAcuMPC) ---
            # # Only inject a minimal loop body here; skip the rest of the teleop pipeline.
            # if args.arm == 'ACU' and teleop_mpc_api is not None:
            #     current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
            #     q_meas = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8].copy()
            #     q_meas[6] = 0.0
            #     q_meas = teleop_mpc_api.state.qk.copy()

            #     q_next = teleop_mpc_api.step(q_meas=q_meas)
            #     sol_q = np.asarray(q_next, dtype=float).reshape(-1)
                
            #     # sol_q = np.array([-0.08047,  0.7515 ,  1.0407 ,  0.01011,  1.29989,  1.62477,  0.00004,  1.37585], dtype=float)
            #     sol_tauff = np.zeros_like(sol_q) 

            #     arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            #     time_elapsed = time.time() - start_time
            #     time.sleep(max(0.0, (1.0 / float(args.frequency)) - time_elapsed))
            #     continue

            # print("i am here")

            # If ACU needle impedance runner is active but has stopped, exit.
            try:
                if args.arm == 'ACU' and args.acu_needle_teleop and (acu_imp_runner is not None):
                    if hasattr(acu_imp_runner, '_running') and (acu_imp_runner._running is False) and needle_ref_enabled:
                        STOP = True
                        break
            except Exception:
                pass

            if not args.headless:
                # Preview: if wrist camera is enabled, stitch head(tv_img_array) and wrist(wrist_img_array).
                show_img = tv_img_array
                if WRIST:
                    try:
                        show_img = np.concatenate([tv_img_array, wrist_img_array], axis=1)
                    except Exception:
                        show_img = tv_img_array
                # show_img = tv_img_array
                h, w = show_img.shape[:2]
                show_resized = cv2.resize(show_img, (w // 2, h // 2))
                cv2.imshow("record image", show_resized)
                # opencv GUI communication
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    START = False
                    STOP = True
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)
                elif key == ord('s'):
                    RECORD_TOGGLE = True
                elif key == ord('a'):
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)

            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

                # # If recording action triggers a sim reset / scene reset, also allow re-init next time.
                # # (Keeps behavior intuitive when user starts a new episode.)
                # if args.arm == 'ACU' and args.xr_mode == 'hand':
                #     mpc_hand_init_done = False
                #     needle_ref_enabled = False  # 如果不是遥操获取，需要设置为True
                #     needle_init_done = False
                #     needle_base_q = None
                #     if acu_imp is not None:
                #         acu_imp.reset()
                #     if acu_imp_runner is not None:
                #         acu_imp_runner.reset()

            # get input data
            tele_data = tv_wrapper.get_motion_state_data()
            # ACU incremental mode: first frame only records last pose and skips IK/control.
            if args.arm == 'ACU' and right_wrist_pose_last is None:
                right_wrist_pose_last = tele_data.right_arm_pose
                # still allow recording/images/etc; just skip arm IK/control this cycle
                time_elapsed = time.time() - start_time
                time.sleep(max(0.0, (1.0 / args.frequency) - time_elapsed))
                continue
            if (args.ee == "dex3" or args.ee == "inspire1" or args.ee == "brainco") and args.xr_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "dex1" and args.xr_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_trigger_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_trigger_value
            elif args.ee == "dex1" and args.xr_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_pinch_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_pinch_value
            else:
                pass        
            
            # high level control
            if args.xr_mode == "controller" and args.motion:
                # quit teleoperate
                if tele_data.tele_state.right_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.tele_state.left_thumbstick_state and tele_data.tele_state.right_thumbstick_state:
                    sport_client.Damp()
                # control, limit velocity to within 0.3
                sport_client.Move(-tele_data.tele_state.left_thumbstick_value[1]  * 0.3,
                                  -tele_data.tele_state.left_thumbstick_value[0]  * 0.3,
                                  -tele_data.tele_state.right_thumbstick_value[0] * 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # ------------------------------
            # ACU needle manipulation retargeting override
            # ------------------------------
            if args.arm == 'ACU' and args.acu_needle_teleop:
                # In this mode, do NOT use teleop_api MPC ingest/PF.
                if args.xr_mode != 'hand':
                    logger_mp.warning('[acu-needle-teleop] requires --xr-mode hand; fallback to normal pipeline.')
                else:
                    # Gate needle retargeting by pinch (trigger once to start)
                    try:
                        pinch_v = float(getattr(tele_data, 'right_pinch_value', 1e9))
                    except Exception:
                        pinch_v = 1e9

                    pressed_now = (pinch_v <= float(args.mpc_trigger_threshold))
                    if pressed_now and (not needle_init_done):
                        needle_ref_enabled = True
                        needle_init_done = True
                        try:
                            needle_base_q = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8].copy()
                        except Exception:
                            needle_base_q = None
                        if acu_imp is not None:
                            try:
                                acu_imp.reset(x0=float(np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[6]))
                            except Exception:
                                acu_imp.reset()
                        if acu_imp_runner is not None:
                            try:
                                # acu_imp_runner.reset(
                                #     x0=float(np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[6]),
                                #     base_q8=np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8].copy(),
                                # )
                                acu_imp_runner.start_session(now=float(now))
                            except Exception:
                                acu_imp_runner.reset()
                        logger_mp.info('[acu-needle-teleop] right pinch pressed: needle teleop enabled.')

                    # Not enabled yet -> skip this override and fall back to normal pipeline.
                    if not needle_ref_enabled:
                        pass
                    else:
                        try:
                            # Exit if runner has stopped (e.g., trajectory finished)
                            if acu_imp_runner is not None and hasattr(acu_imp_runner, '_running') and (acu_imp_runner._running is False):
                                STOP = True
                                break

                             # feed retargeter input
                            rh = tele_data.right_hand_pos
                            if rh is None:
                                raise RuntimeError('right_hand_pos is None')
                            rh = np.asarray(rh, dtype=float).reshape(25, 3)
                            with needle_right_hand_pos_array.get_lock():
                                needle_right_hand_pos_array[:] = rh.reshape(-1)

                            if acu_imp_runner is None:
                                raise RuntimeError('acu_imp_runner is None')
                            try:
                                acu_imp_runner.step_once(now=float(now))
                            except StopIteration:
                                STOP = True
                                break

                            time_elapsed = time.time() - start_time
                            time.sleep(max(0.0, (1.0 / float(args.frequency)) - time_elapsed))
                            continue
                        except Exception as e:
                            logger_mp.error(f"[acu-needle-teleop] failed, fallback to normal pipeline. err={e}")
                            break

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            uselast = False
            if args.arm == 'ACU':
                # Prefer teleop_api path-following MPC when enabled.
                if teleop_mpc_api is not None:
                    # Start high-rate MPC thread once
                    if mpc_pf_thread is None and args.use_mpc_control:
                        pf_hz = float(max(1.0, args.frequency * 3.0))
                        mpc_pf_stop.clear()
                        mpc_pf_thread = threading.Thread(target=_acu_mpc_pf_thread, args=(pf_hz,), daemon=True)
                        mpc_pf_thread.start()
                        logger_mp.info(f"[MPC] ACU PF thread started at {pf_hz:.1f} Hz")

                    # IMPORTANT: keep the original ingest/test_waypoints logic below unchanged.
                    try:
                        now = time.time()
                        if args.mpc_test_input:
                            hold = float(max(0.0, args.mpc_test_hold))
                            speed = float(max(1e-4, args.mpc_test_speed))

                            wp0 = test_waypoints[test_seg_i]
                            wp1 = test_waypoints[(test_seg_i + 1) % len(test_waypoints)]

                            if isinstance(wp0, tuple):
                                p0, q0_seg = wp0
                            else:
                                p0, q0_seg = wp0, test_quat_xyzw

                            if isinstance(wp1, tuple):
                                p1, q1_seg = wp1
                            else:
                                p1, q1_seg = wp1, q0_seg

                            # dt = 1.0 / float(max(1e-6, args.frequency))
                            dt = 1.0 / float(max(1e-6, 20))
                            step = speed * dt

                            if test_seg_p0 is None:
                                test_seg_p0 = p0.copy()

                            seg_vec = (p1 - p0)
                            seg_len = float(np.linalg.norm(seg_vec))
                            if seg_len < 1e-9:
                                p_ref = p1.copy()
                                alpha = 1.0
                            else:
                                seg_dir = seg_vec / seg_len
                                dist_remain = float(np.linalg.norm(p1 - test_seg_p0))
                                if dist_remain <= step:
                                    p_ref = p1.copy()
                                    test_seg_p0 = p1.copy()
                                    alpha = 1.0
                                else:
                                    test_seg_p0 = test_seg_p0 + seg_dir * step
                                    p_ref = test_seg_p0.copy()
                                    alpha = float(np.clip(np.linalg.norm(test_seg_p0 - p0) / seg_len, 0.0, 1.0))

                            # orientation policy: slerp with the same alpha as position progress
                            q_ref = _quat_slerp(q0_seg, q1_seg, alpha)

                            if alpha >= 1.0:
                                if (now - float(test_seg_start_t)) >= hold:
                                    test_seg_i = (test_seg_i + 1) % len(test_waypoints)
                                    test_seg_start_t = now
                                    wp = test_waypoints[test_seg_i]
                                    test_seg_p0 = (wp[0].copy() if isinstance(wp, tuple) else wp.copy())

                            teleop_ref = _make_pose_dict(p_ref, q_ref)

                        else:
                             # use XR as reference (or replay for IK run).
                            if ref_replayer is not None:
                                # replay XR reference stream for either IK run or MPC run
                                teleop_ref = ref_replayer.sample_next()
                                rp = None
                                if teleop_ref is None:
                                    # record data
                                    if args.record:
                                        RECORD_READY = recorder.is_ready()
                                        # dex hand or gripper
                                        if args.ee == "dex3" and args.xr_mode == "hand":
                                            with dual_hand_data_lock:
                                                left_ee_state = dual_hand_state_array[:7]
                                                right_ee_state = dual_hand_state_array[-7:]
                                                left_hand_action = dual_hand_action_array[:7]
                                                right_hand_action = dual_hand_action_array[:7]
                                                current_body_state = []
                                                current_body_action = []
                                        elif args.ee == "dex1" and args.xr_mode == "hand":
                                            with dual_gripper_data_lock:
                                                left_ee_state = [dual_gripper_state_array[0]]
                                                right_ee_state = [dual_gripper_state_array[1]]
                                                left_hand_action = [dual_gripper_action_array[0]]
                                                right_hand_action = [dual_gripper_action_array[0]]
                                                current_body_state = []
                                                current_body_action = []
                                        elif args.ee == "dex1" and args.xr_mode == "controller":
                                            with dual_gripper_data_lock:
                                                left_ee_state = [dual_gripper_state_array[0]]
                                                right_ee_state = [dual_gripper_state_array[1]]
                                                left_hand_action = [dual_gripper_action_array[0]]
                                                right_hand_action = [dual_gripper_action_array[1]]
                                                current_body_state = arm_ctrl.get_current_motor_q().tolist()
                                                current_body_action = [-tele_data.tele_state.left_thumbstick_value[1]  * 0.3,
                                                                    -tele_data.tele_state.left_thumbstick_value[0]  * 0.3,
                                                                    -tele_data.tele_state.right_thumbstick_value[0] * 0.3]
                                        elif (args.ee == "inspire1" or args.ee == "brainco") and args.xr_mode == "hand":
                                            with dual_hand_data_lock:
                                                left_ee_state = dual_hand_state_array[:6]
                                                right_ee_state = dual_hand_state_array[-6:]
                                                left_hand_action = dual_hand_action_array[:6]
                                                right_hand_action = dual_hand_action_array[:6]
                                                current_body_state = []
                                                current_body_action = []
                                        else:
                                            left_ee_state = []
                                            right_ee_state = []
                                            left_hand_action = []
                                            right_hand_action = []
                                            current_body_state = []
                                            current_body_action = []
                                        # head image
                                        current_tv_image = tv_img_array.copy()
                                        # wrist image
                                        if WRIST:
                                            current_wrist_image = wrist_img_array.copy()
                                        # arm state and action
                                        if args.arm == 'ACU':
                                            # single arm: store in right_arm by convention (keep schema stable)
                                            left_arm_state = []
                                            left_arm_action = []
                                            right_arm_state = current_lr_arm_q.tolist()
                                            right_arm_action = sol_q.tolist() if sol_q is not None else []
                                        else:
                                            left_arm_state  = current_lr_arm_q[:7]
                                            right_arm_state = current_lr_arm_q[-7:]
                                            left_arm_action = sol_q[:7]
                                            right_arm_action = sol_q[-7:]
                                        if RECORD_RUNNING:
                                            colors = {}
                                            depths = {}
                                            if BINOCULAR:
                                                colors[f"color_{0}"] = current_tv_image[:, :tv_img_shape[1]//2]
                                                colors[f"color_{1}"] = current_tv_image[:, tv_img_shape[1]//2:]
                                                if WRIST:
                                                    colors[f"color_{2}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                                                    colors[f"color_{3}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                                            else:
                                                colors[f"color_{0}"] = current_tv_image
                                                if WRIST:
                                                    colors[f"color_{1}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                                                    colors[f"color_{2}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                                            states = {
                                                "left_arm": {                                                                    
                                                    "qpos":   left_arm_state if args.arm == 'ACU' else left_arm_state.tolist(),    # numpy.array -> list
                                                    "qvel":   [],                          
                                                    "torque": [],                        
                                                }, 
                                                "right_arm": {                                                                    
                                                    "qpos":   right_arm_state if args.arm == 'ACU' else right_arm_state.tolist(),       
                                                    "qvel":   [],                          
                                                    "torque": [],                         
                                                },                        
                                                "left_ee": {                                                                    
                                                    "qpos":   left_ee_state,           
                                                    "qvel":   [],                           
                                                    "torque": [],                          
                                                }, 
                                                "right_ee": {                                                                    
                                                    "qpos":   right_ee_state,       
                                                    "qvel":   [],                           
                                                    "torque": [],  
                                                }, 
                                                "body": {
                                                    "qpos": current_body_state,
                                                }, 
                                            }
                                            actions = {
                                                "left_arm": {                                   
                                                    "qpos":   left_arm_action if args.arm == 'ACU' else left_arm_action.tolist(),       
                                                    "qvel":   [],       
                                                    "torque": [],      
                                                }, 
                                                "right_arm": {                                   
                                                    "qpos":   right_arm_action if args.arm == 'ACU' else right_arm_action.tolist(),  
                                                    "qvel":   [],       
                                                    "torque": [],       
                                                },                         
                                                "left_ee": {                                   
                                                    "qpos":   left_hand_action,       
                                                    "qvel":   [],       
                                                    "torque": [],       
                                                }, 
                                                "right_ee": {                                   
                                                    "qpos":   right_hand_action,       
                                                    "qvel":   [],       
                                                    "torque": [], 
                                                }, 
                                                "body": {
                                                    "qpos": current_body_action,
                                                }, 
                                            }
                                            if args.sim:
                                                sim_state = sim_state_subscriber.read_data()            
                                                recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                                            else:
                                                recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)
                                                current_time = time.time()
                                                time_elapsed = current_time - start_time
                                                sleep_time = max(0, (1 / args.frequency) - time_elapsed)
                                                time.sleep(sleep_time)
                                    continue  # end of replay; exit teleop loop
                            else:
                                rp = tele_data.right_arm_pose
                                # TeleVuerWrapper returns right_arm_pose as a 4x4 homogeneous matrix (np.ndarray)
                                # under (basis) Robot convention (see tv_wrapper.py).
                                if args.xr_mode == 'hand':
                                    # Hand-tracking mode: keep orientation from wrist pose, but use index tip position.
                                    # right_hand_pos is (25,3) with OpenXR joint order; index tip is joint 10 (1-based) => idx 9 (0-based).
                                    try:
                                        xr_p = np.asarray(rp[:3, 3], dtype=float).reshape(3)

                                        if isinstance(rp, np.ndarray) and rp.shape == (4, 4):
                                            xr_R = np.asarray(rp[:3, :3], dtype=float).reshape(3, 3)
                                        else:
                                            xr_R = np.eye(3, dtype=float)

                                        if tele_data.right_hand_pos is not None and np.asarray(tele_data.right_hand_pos).shape[0] >= 10:
                                            idx_tip = np.asarray(tele_data.right_hand_pos, dtype=float).reshape(25, 3)[9].copy()
                                        else:
                                            idx_tip = None

                                        # Map position by anchors if available.
                                        if idx_tip is not None and args._acu_xr_p0 is not None and args._acu_ee_p0 is not None:
                                            pos = args._acu_ee_p0 + (idx_tip - args._acu_xr_p0)
                                            # pos = args._acu_ee_p0 + (xr_p - args._acu_xr_p0)
                                        else:
                                            pos = None

                                        quat = _rot_to_quat_xyzw(xr_R)
                                        teleop_ref = _make_pose_dict(pos, quat) if pos is not None else None
                                    except Exception:
                                        teleop_ref = None

                                elif isinstance(rp, np.ndarray) and rp.shape == (4, 4):
                                    xr_p = np.asarray(rp[:3, 3], dtype=float).reshape(3)
                                    xr_R = np.asarray(rp[:3, :3], dtype=float).reshape(3, 3)

                                    # Only build mapped position if anchors are available.
                                    if args._acu_xr_p0 is not None and args._acu_ee_p0 is not None:
                                        pos = args._acu_ee_p0 + (xr_p - args._acu_xr_p0)*0.5
                                    else:
                                        # Not enabled yet (or just reset). Provide None to prevent ingest.
                                        pos = None

                                    quat = _rot_to_quat_xyzw(xr_R)
                                    teleop_ref = _make_pose_dict(pos, quat) if pos is not None else None

                                elif isinstance(rp, dict) and ("position" in rp or "pos" in rp):
                                    pos = rp.get("position", rp.get("pos"))
                                    quat = rp.get("quat", rp.get("orientation", test_quat_xyzw))
                                    teleop_ref = _make_pose_dict(pos, quat)
                                else:
                                    teleop_ref = None

                        # ---- Gate: ingest only when right trigger is pressed (optional) ----
                        # Controller mode: use trigger as before.
                        # Hand mode: use pinch distance/value as gate.
                        if (args.mpc_stream_on_trigger or args.mpc_stream_toggle) and (not args.mpc_test_input):
                            if args.xr_mode == 'hand':
                                # TeleVuerWrapper: right_pinch_value is a distance-like value (scaled in tv_wrapper.py by *100).
                                # Smaller value => stronger pinch.
                                try:
                                    pinch_v = float(getattr(tele_data, 'right_pinch_value', 1e9))
                                except Exception:
                                    pinch_v = 1e9

                                # Use the same threshold semantics as controller trigger: "pressed" when pinch is small enough.
                                pressed_now = (pinch_v <= float(args.mpc_trigger_threshold))

                                # pinch "pressed" -> enable streaming; also do one-time init (clear path + anchors + seed)
                                # NOTE: pinch may jitter and repeatedly generate rising edges; guard init with mpc_hand_init_done.
                                if pressed_now:
                                    if (not mpc_hand_init_done) and (not mpc_ref_enabled):
                                        print("MPC teleop: right pinch pressed (init once).")
                                        if args.mpc_stream_toggle:
                                            mpc_ref_latched = True
                                        mpc_ref_enabled = True

                                        try:
                                            teleop_mpc_api._path.clear()
                                        except Exception:
                                            pass

                                        args._acu_xr_p0 = None
                                        args._acu_ee_p0 = None
                                        try:
                                            # anchor xr p0 uses index tip in hand mode
                                            if tele_data.right_hand_pos is not None:
                                                _rh = np.asarray(tele_data.right_hand_pos, dtype=float).reshape(25, 3)
                                                args._acu_xr_p0 = _rh[9].copy()
                                                # args._acu_xr_p0 = np.asarray(rp[:3, 3], dtype=float).reshape(3).copy()
                                            q_fk = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8]
                                            fk0 = teleop_mpc_api.ctl.robot.fk(q_fk)
                                            args._acu_ee_p0 = np.asarray(fk0.translation).reshape(3).copy()
                                        except Exception:
                                            args._acu_xr_p0 = None
                                            args._acu_ee_p0 = None

                                        try:
                                            q_seed = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8]
                                            fk_seed = teleop_mpc_api.ctl.robot.fk(q_seed)
                                            p_seed = np.asarray(fk_seed.translation).reshape(3).copy()
                                            q_seed_xyzw = _quat_normalize(_rot_to_quat_xyzw(np.asarray(fk_seed.rotation)))
                                            teleop_mpc_api.ingest_teleop_pose(_make_pose_dict(p_seed, q_seed_xyzw), now)
                                        except Exception:
                                            pass

                                        mpc_hand_init_done = True

                                if args.mpc_stream_toggle:
                                    mpc_ref_enabled = bool(mpc_ref_latched)
                                else:
                                    mpc_ref_enabled = bool(pressed_now)

                                mpc_ref_prev_pressed = pressed_now

                                if mpc_ref_enabled and (teleop_ref is not None) and args.use_mpc_control:
                                    teleop_mpc_api.ingest_teleop_pose(teleop_ref, now)
                                elif (teleop_ref is not None) and not args.use_mpc_control:
                                    target_p, target_q = teleop_ref["position"], teleop_ref["quat"]
                                    try:
                                        import pinocchio as pin
                                    except Exception:  # pragma: no cover
                                        import pin  # type: ignore
                                    # include orientation in target_pose
                                    R = _quat_to_rot(target_q)
                                    target_pose = pin.SE3(np.asarray(R).copy(), np.asarray(target_p).reshape(3).copy())
                                    ik_sol = teleop_mpc_api.ctl._ik.solve(target = target_pose, q0 = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8].copy()) if target_pose is not None else None
                                    sol_q = ik_sol.q.reshape(-1) if ik_sol is not None else None
                                    sol_tauff = np.zeros_like(sol_q)

                            else:
                                # -------- existing controller gate logic --------
                                trig_state = bool(getattr(tele_data.tele_state, 'right_trigger_state', False))
                                trig_val_raw = getattr(tele_data, 'right_trigger_value', None)
                                if trig_val_raw is None:
                                    trig_val_raw = getattr(tele_data.tele_state, 'right_trigger_value', 0.0)
                                try:
                                    trig_val = float(trig_val_raw)
                                except Exception:
                                    trig_val = 0.0

                                pressed_now = trig_state or (trig_val <= float(args.mpc_trigger_threshold))

                                # rising edge: initialize anchors + clear path + seed
                                if pressed_now and not mpc_ref_prev_pressed:
                                    if args.mpc_stream_toggle:
                                        mpc_ref_latched = True
                                    # in hold-to-run mode, this just enables while pressed
                                    mpc_ref_enabled = True

                                    try:
                                        teleop_mpc_api._path.clear()
                                    except Exception:
                                        pass

                                    args._acu_xr_p0 = None
                                    args._acu_ee_p0 = None
                                    try:
                                        if isinstance(rp, np.ndarray) and rp.shape == (4, 4):
                                            args._acu_xr_p0 = np.asarray(rp[:3, 3], dtype=float).reshape(3).copy()
                                        q_fk = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8]
                                        fk0 = teleop_mpc_api.ctl.robot.fk(q_fk)
                                        args._acu_ee_p0 = np.asarray(fk0.translation).reshape(3).copy()
                                    except Exception:
                                        args._acu_xr_p0 = None
                                        args._acu_ee_p0 = None

                                    # seed with current EE pose (FK)
                                    try:
                                        q_seed = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8]
                                        fk_seed = teleop_mpc_api.ctl.robot.fk(q_seed)
                                        p_seed = np.asarray(fk_seed.translation).reshape(3).copy()
                                        q_seed_xyzw = _quat_normalize(_rot_to_quat_xyzw(np.asarray(fk_seed.rotation)))
                                        teleop_mpc_api.ingest_teleop_pose(_make_pose_dict(p_seed, q_seed_xyzw), now)
                                    except Exception:
                                        pass

                                # determine enabled state
                                if args.mpc_stream_toggle:
                                    mpc_ref_enabled = bool(mpc_ref_latched)
                                else:
                                    mpc_ref_enabled = bool(pressed_now)

                                mpc_ref_prev_pressed = pressed_now

                                if mpc_ref_enabled and (teleop_ref is not None) and args.use_mpc_control:
                                    teleop_mpc_api.ingest_teleop_pose(teleop_ref, now)

                                elif mpc_ref_enabled and (teleop_ref is not None) and not args.use_mpc_control:
                                    target_p, target_q = teleop_ref["position"], teleop_ref["quat"]
                                    # include orientation in target_pose
                                    R = _quat_to_rot(target_q)
                                    target_pose = pin.SE3(np.asarray(R).copy(), np.asarray(target_p).reshape(3).copy())
                                    sol_q = teleop_mpc_api.ctl._ik.solve(target_pose) if target_pose is not None else None
                                    sol_tauff = np.zeros_like(sol_q)

                        else:
                            if teleop_ref is not None:
                                teleop_mpc_api.ingest_teleop_pose(teleop_ref, now)
                        # ---------------------------------------------------------------

                    except Exception as e:
                        logger_mp.error(f"[teleop_api MPC] step failed, fallback to IK. err={e}")
                        sol_q, sol_tauff  = arm_ik.solve_ik(
                            tele_data.left_arm_pose,
                            tele_data.right_arm_pose,
                            current_lr_arm_q,
                            current_lr_arm_dq,
                            right_wrist_last=right_wrist_pose_last,
                        )
                        right_wrist_pose_last = tele_data.right_arm_pose

                else:
                    # ------------------------------
                    # Legacy MPC (rough planner) or IK fallback
                    # ------------------------------
                    if mpc_ctl is not None and mpc_qk is not None and mpc_rough_plan is not None:
                        try:
                            curpos = mpc_ctl.robot.fk(mpc_qk)
                            P = np.asarray(curpos.translation).reshape(3)
                            Sd, Vv, dot_value = compute_sd_rough(mpc_rough_plan, P=P)
                            if mpc_rough_plan.flag == 1:
                                break
                            _res = mpc_ctl.step(qk=mpc_qk, Sd=Sd, mode=MPCMode.ROUGH, target_pose=mpc_target_rough)
                            mpc_qk = _res.q_next.reshape(-1)
                            sol_q = mpc_qk.copy()
                            sol_tauff = np.zeros_like(sol_q)
                            mpc_step_i += 1
                            print(sol_q)
                        except Exception as e:
                            logger_mp.error(f"[MPC] step failed, fallback to IK. err={e}")
                            sol_q, sol_tauff  = arm_ik.solve_ik(
                                tele_data.left_arm_pose,
                                tele_data.right_arm_pose,
                                current_lr_arm_q,
                                current_lr_arm_dq,
                                right_wrist_last=right_wrist_pose_last,
                            )
                            right_wrist_pose_last = tele_data.right_arm_pose
                    else:
                        sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq)
            else:
                sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            
            # print(sol_q)
            # print(tele_data.right_arm_pose)
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")

            # # Only control in main loop when ACU PF thread is not responsible.
            # if not (args.arm == 'ACU' and teleop_mpc_api is not None):
            #     arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            try:
                # Only control in main loop when ACU PF thread is not responsible.
                if args.arm == 'ACU' and teleop_mpc_api is not None and not args.use_mpc_control and sol_q is not None:
                    print(sol_q)
                    # arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
            except Exception as e:
                pass

            # ------------------------------
            # Compare-log sampling (EXCLUDE acu_needle_teleop)
            # Ground truth: mapped XR reference teleop_ref (only right arm), measured: motor q/dq.
            # ------------------------------
            try:
                if (
                    compare_logger is not None
                    and args.arm == 'ACU'
                    and (not args.acu_needle_teleop)
                    and (teleop_mpc_api is not None)
                    and (teleop_ref is not None)
                    and (isinstance(teleop_ref, dict))
                    and (teleop_ref.get('position') is not None)
                    and (teleop_ref.get('quat') is not None)
                    and (not bool(args.use_mpc_control))
                ):
                    q_meas8 = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8].copy()
                    dq_meas8 = None
                    try:
                        dq_meas8 = np.asarray(current_lr_arm_dq, dtype=float).reshape(-1)[:8].copy()
                    except Exception:
                        dq_meas8 = None

                    # align FK convention with control code
                    q_meas8_fk = q_meas8.copy()
                    q_meas8_fk[6] = 0.0
                    fk = teleop_mpc_api.ctl.robot.fk(q_meas8_fk)
                    ee_p = np.asarray(fk.translation).reshape(3).copy()
                    ee_quat = _quat_normalize(_rot_to_quat_xyzw(np.asarray(fk.rotation)))

                    xr_p = np.asarray(teleop_ref['position'], dtype=float).reshape(3).copy()
                    xr_quat = _quat_normalize(np.asarray(teleop_ref['quat'], dtype=float).reshape(4).copy())

                    pf_dbg = getattr(teleop_mpc_api, '_dbg_last_pf', None)
                    compare_logger.add(
                        stamp=float(now),
                        q_meas8=q_meas8,
                        dq_meas8=dq_meas8,
                        ee_p=ee_p,
                        ee_quat_xyzw=ee_quat,
                        xr_p=xr_p,
                        xr_quat_xyzw=xr_quat,
                        pf_dbg=pf_dbg,
                    )
            except Exception as e:
                logger_mp.debug(f"[compare-log] skip sample: {e}")

            # 在主循环中，每次生成/更新 teleop_ref 后同步 latest_teleop_ref（供 MPC 线程记录用）
            # （不要求与 MPC 线程采样频率一致，数量可不同）
            try:
                if teleop_ref is not None:
                    with latest_teleop_ref_lock:
                        latest_teleop_ref = teleop_ref.copy()
                    if xr_ref_logger is not None:
                        xr_ref_logger.add(stamp=float(now), teleop_ref=teleop_ref)
            except Exception as e:
                logger_mp.debug(f"[latest_teleop_ref] update failed: {e}")

            # record data
            if args.record:
                RECORD_READY = recorder.is_ready()
                # dex hand or gripper
                if args.ee == "dex3" and args.xr_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[:7]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.xr_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[0]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.xr_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.tele_state.left_thumbstick_value[1]  * 0.3,
                                               -tele_data.tele_state.left_thumbstick_value[0]  * 0.3,
                                               -tele_data.tele_state.right_thumbstick_value[0] * 0.3]
                elif (args.ee == "inspire1" or args.ee == "brainco") and args.xr_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[:6]
                        current_body_state = []
                        current_body_action = []
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []
                # head image
                current_tv_image = tv_img_array.copy()
                # wrist image
                if WRIST:
                    current_wrist_image = wrist_img_array.copy()
                # arm state and action
                if args.arm == 'ACU':
                    # single arm: store in right_arm by convention (keep schema stable)
                    left_arm_state = []
                    left_arm_action = []
                    right_arm_state = current_lr_arm_q.tolist()
                    right_arm_action = sol_q.tolist() if sol_q is not None else []
                else:
                    left_arm_state  = current_lr_arm_q[:7]
                    right_arm_state = current_lr_arm_q[-7:]
                    left_arm_action = sol_q[:7]
                    right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if BINOCULAR:
                        colors[f"color_{0}"] = current_tv_image[:, :tv_img_shape[1]//2]
                        colors[f"color_{1}"] = current_tv_image[:, tv_img_shape[1]//2:]
                        if WRIST:
                            colors[f"color_{2}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                            colors[f"color_{3}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                    else:
                        colors[f"color_{0}"] = current_tv_image
                        if WRIST:
                            colors[f"color_{1}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                            colors[f"color_{2}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state if args.arm == 'ACU' else left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state if args.arm == 'ACU' else right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action if args.arm == 'ACU' else left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action if args.arm == 'ACU' else right_arm_action.tolist(),  
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("KeyboardInterrupt, exiting program...")
    except Exception as e:
        logger_mp.error(f"main process exception: {e}")
    finally:
        try:
            if acu_imp_runner is not None:
                acu_imp_runner.stop()
                acu_imp_runner.join(timeout=1.0)
        except Exception:
            pass
        arm_ctrl.ctrl_dual_arm_go_home()

        if args.ipc:
            ipc_server.stop()
        else:
            stop_listening()
            listen_keyboard_thread.join()

        if args.sim:
            sim_state_subscriber.stop_subscribe()
        tv_img_shm.close()
        tv_img_shm.unlink()
        if WRIST:
            wrist_img_shm.close()
            wrist_img_shm.unlink()

        if args.record:
            recorder.close()

        try:
            mpc_pf_stop.set()
            if mpc_pf_thread is not None:
                mpc_pf_thread.join(timeout=1.0)
        except Exception:
            pass

        # save compare log
        try:
            if compare_logger is not None:
                saved = compare_logger.save(extra_meta={
                    'arm': str(args.arm),
                    'xr_mode': str(args.xr_mode),
                    'use_mpc_control': bool(args.use_mpc_control),
                    'mpc_teleop_api': bool(args.mpc_teleop_api),
                    'mpc_stream_on_trigger': bool(args.mpc_stream_on_trigger),
                    'mpc_stream_toggle': bool(args.mpc_stream_toggle),
                    'mpc_test_input': bool(args.mpc_test_input),
                    'tag': str(args.compare_tag),
                })
                if saved:
                    logger_mp.info(f"[compare-log] saved: {saved}")
        except Exception as e:
            logger_mp.error(f"[compare-log] save failed: {e}")

        # save XR ref stream (main loop frequency)
        try:
            if xr_ref_logger is not None and not args.use_mpc_control:
                xr_saved = xr_ref_logger.save(out_dir=str(args.compare_out_dir), tag=str(args.compare_tag))
                if xr_saved:
                    logger_mp.info(f"[xr-ref] saved: {xr_saved}")
        except Exception as e:
            logger_mp.error(f"[xr-ref] save failed: {e}")

        logger_mp.info("Finally, exiting program.")
        exit(0)
