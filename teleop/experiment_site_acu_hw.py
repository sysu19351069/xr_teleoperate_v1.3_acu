#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实机实验端（针灸机器人）一体化服务

提供给 teleop_hand_and_arm.py (Acu_ArmController) 的实机侧能力：
1) 图像：调用 teleop.image_server.image_server.ImageServer 发布 head/wrist 相机 JPEG 拼接流（ZMQ PUB）。
2) 机械臂：通过 Acupuncture_Software/CPS.py 的 CPSClient 读取/控制 6DOF 机械臂。
3) 末端2DOF针灸执行器：通过 Acupuncture_Software/Force_Sensor/Motor1.py 串口控制两轴（速度/方向）。
4) 统一成 8DOF：DDS 使用与 teleop/robot_control/robot_arm.py 完全一致的消息与话题：
   - cmd: rt/acu_lowcmd (unitree_hg.msg.dds_.LowCmdAcu_)
   - state: rt/acu_lowstate (unitree_hg.msg.dds_.LowStateAcu_)
   实机端订阅低命令并控制硬件，同时发布当前8DoF状态。
5) 末端 1D 力：通过 Acupuncture_Software/Force_Sensor/transducer.py 读取，DDS(String_) 发布（按你要求不走 ROS）。

注意：
- ACU 的 LowCmdAcu_ 中每个 motor_cmd[i] 通常含 q/dq/tau/kp/kd/mode 等字段。
  teleop 侧 Acu_ArmController 主要写 motor_cmd[i].q（位置目标）并附带增益。
  本实机端把 q 解释为：前6轴机械臂关节角目标；后2轴为末端电机“位置编码目标”或“保留字段”。
  由于末端两电机当前串口控制更适合速度模式，默认不直接执行 q[6:8]，而是支持：
  - 若 motor_cmd[6/7].dq 或 tau 被用作速度通道（你可在遥操端映射），则按速度控制。

- unitree_sdk2py 在编辑器可能报“无法解析导入”，但在部署环境只要包已安装即可运行。
"""

from __future__ import annotations

import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import numpy as np

# ---- path fix for workspace imports ----
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.dirname(THIS_DIR)
if WS_ROOT not in sys.path:
    sys.path.append(WS_ROOT)

# 1) image server
from teleop.image_server.image_server import ImageServer

# 2) arm CPS client
from Acupuncture_Software.CPS import CPSClient

# 3) needle motors + force transducer
from Acupuncture_Software.Force_Sensor import Motor1 as FS_m
from Acupuncture_Software.Force_Sensor import transducer as FS_t

# 4/5) DDS
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmdAcu_ as Acu_LowCmd, LowStateAcu_ as Acu_LowState
from unitree_sdk2py.idl.default import unitree_Acu_msg_dds__LowState_

# 与 robot_arm.py 对齐
kTopicLowCommand_Debug = "rt/acu_lowcmd"
kTopicLowState = "rt/acu_lowstate"
TOPIC_ACU_FORCE = "rt/acu_force"

import contextlib

@contextlib.contextmanager
def _silence_fds(enable: bool = True):
    """Silence C/Python prints by redirecting stdout/stderr FDs to /dev/null.
    Useful when vendor SDK prints in C (printf).
    """
    if not enable:
        yield
        return
    devnull_fd = None
    saved = None
    try:
        devnull_fd = os.open("/dev/null", os.O_WRONLY)
        saved = (os.dup(1), os.dup(2))
        os.dup2(devnull_fd, 1)
        os.dup2(devnull_fd, 2)
        yield
    finally:
        try:
            if saved is not None:
                os.dup2(saved[0], 1)
                os.dup2(saved[1], 2)
        finally:
            if saved is not None:
                os.close(saved[0])
                os.close(saved[1])
            if devnull_fd is not None:
                os.close(devnull_fd)



@dataclass
class ImageConfig:
    fps: int = 30
    head_camera_type: str = "realsense"  # 'opencv' | 'realsense'
    head_camera_image_shape: Tuple[int, int] = (720, 1280)  # (H,W)
    head_camera_id_numbers: Tuple[str, ...] = ("220222069039",)  # realsense serial

    wrist_camera_type: Optional[str] = "realsense"
    wrist_camera_image_shape: Tuple[int, int] = (720, 1280)
    wrist_camera_id_numbers: Optional[Tuple[str, ...]] = ("220322060052",)

    port: int = 5555


@dataclass
class CPSConfig:
    ip: str = "192.168.0.10"
    port: int = 10003
    box_id: int = 0
    rbt_id: int = 0


@dataclass
class NeedleMotorConfig:
    motor1_port: str = "/dev/ttyUSB0"  # insert axis
    motor2_port: str = "/dev/ttyUSB1"  # twist axis


@dataclass
class ForceSensorConfig:
    port: str = "/dev/ttyUSB2"
    publish_hz: float = 30.0
    force_scale: float = 1.0 / 100.0 / (1.63200 * 5.0) * 49.0


class AcuNeedleMotors:
    """末端两自由度电机（串口）。"""

    def __init__(self, cfg: NeedleMotorConfig):
        self.cfg = cfg
        FS_m.status()
        self.motor1 = FS_m.init(cfg.motor1_port)
        self.motor2 = FS_m.init(cfg.motor2_port)

        self._lock = threading.Lock()
        self._dir_cur1 = 0
        self._dir_cur2 = 0

        self.pos1 = 0
        self.pos2 = 0

        self._init_motor_params()

    def _init_motor_params(self):
        # 参考 PubAndSub.py
        try:
            self.motor1.write([0x01, 0x10, 0x00, 0x70, 0x00, 0x02, 0x04, 0x5F, 0x90, 0x00, 0x01, 0x27, 0x72])
            time.sleep(0.01)
            self.motor1.write([0x01, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x74, 0x0B])
            time.sleep(0.01)
            self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x38, 0x80, 0x00, 0x01, 0xB2, 0x62])
            time.sleep(0.01)

            self.motor2.write([0x02, 0x10, 0x00, 0x70, 0x00, 0x02, 0x04, 0x4E, 0x20, 0x00, 0x00, 0xED, 0x2D])
            time.sleep(0.01)
            self.motor2.write([0x02, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x4F])
            time.sleep(0.01)
            self.motor2.write([0x02, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x27, 0x10, 0x00, 0x00, 0x7B, 0x1F])
            time.sleep(0.01)
        except Exception as e:
            logger_mp.warning(f"[NeedleMotors] init params failed: {e}")

    def set_velocity(self, v_insert_mm_s: float, v_twist_r_s: float):
        with self._lock:
            dir1 = FS_m.dirGet(-float(v_insert_mm_s))
            dir2 = FS_m.dirGet(float(v_twist_r_s))

            ordV1, ordV2 = FS_m.vConvert(abs(float(v_insert_mm_s)), abs(float(v_twist_r_s)))
            ordDir1, ordDir2 = FS_m.dirConvert(dir1, dir2)

            if self._dir_cur1 != dir1:
                self.motor1.write([0x01, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x34])
                time.sleep(0.005)
                self.motor1.write(ordV1)
                time.sleep(0.005)
                self.motor1.write(ordDir1)
                time.sleep(0.005)
            else:
                self.motor1.write(ordV1)
                time.sleep(0.005)

            if self._dir_cur2 != dir2:
                self.motor2.write([0x02, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x07])
                time.sleep(0.005)
                self.motor2.write(ordV2)
                time.sleep(0.005)
                self.motor2.write(ordDir2)
                time.sleep(0.005)
            else:
                self.motor2.write(ordV2)
                time.sleep(0.005)

            self._dir_cur1 = dir1
            self._dir_cur2 = dir2

    def read_position_counts(self) -> Tuple[float, float]:
        with self._lock:
            _ = FS_m.readLocation(self.motor1)
            _ = FS_m.readLocation(self.motor2)

            self.motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])
            self.motor2.write([0x02, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xF9])
            time.sleep(0.005)

            p1 = FS_m.readLocation(self.motor1)
            p2 = FS_m.readLocation(self.motor2)

            self.pos1 = 0 if p1 < 0 else p1
            self.pos2 = 0 if p2 < 0 else p2
            return float(self.pos1), float(self.pos2)


class AcuForceSensorDDS:
    def __init__(self, cfg: ForceSensorConfig, pub: ChannelPublisher):
        self.cfg = cfg
        self.pub = pub
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._tran = FS_t.init(cfg.port)
        self._pF_raw = 0.0
        self._pT_raw = 0.0

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _loop(self):
        dt = 1.0 / max(1e-6, float(self.cfg.publish_hz))
        while self._running:
            t0 = time.time()
            try:
                F_raw, T_raw, ok = FS_t.read_data(self._tran)
                if ok:
                    self._pF_raw, self._pT_raw = float(F_raw), float(T_raw)
                else:
                    F_raw, T_raw = self._pF_raw, self._pT_raw

                F = float(F_raw) * float(self.cfg.force_scale)
                # 按要求用 String_，参考 sim_state_topic.py：json 序列化
                payload = {
                    "t": time.time(),
                    "force": F,
                    "force_raw": float(F_raw),
                    "torque_raw": float(T_raw),
                }
                self.pub.Write(String_(data=str(payload).replace("'", '"')))
            except Exception as e:
                logger_mp.warning(f"[ForceSensor] read/publish failed: {e}")

            time.sleep(max(0.0, dt - (time.time() - t0)))


class Acu8DoFDDSServer:
    """订阅 rt/acu_lowcmd 并驱动硬件；发布 rt/acu_lowstate."""

    def __init__(self, cps_cfg: CPSConfig, needle_cfg: NeedleMotorConfig):
        self.cps_cfg = cps_cfg

        self.arm = CPSClient()
        self._arm_connected = False

        # 保存配置，末端2DoF默认不开启时不创建串口对象
        self._needle_cfg = needle_cfg
        self.needle: Optional[AcuNeedleMotors] = None

        self.sub_cmd = ChannelSubscriber(kTopicLowCommand_Debug, Acu_LowCmd)
        self.pub_state = ChannelPublisher(kTopicLowState, Acu_LowState)
        self.sub_cmd.Init()
        self.pub_state.Init()

        self._running = False
        self._thread_sub: Optional[threading.Thread] = None
        self._thread_pub: Optional[threading.Thread] = None

        # caches
        self._last_cmd: Optional[Acu_LowCmd] = None
        self._lock = threading.Lock()

        self._needle_deadband = 1e-3
        # 速度限幅：insert(mm/s), twist(r/s)
        self._needle_v_insert_limit = 5.0
        self._needle_v_twist_limit = 5.0

        # 末端2DoF默认不开启（只保留8DoF收发框架与状态位占位）
        self._needle_enabled = False
        # 当未使能硬件读取时，使用订阅到的上一帧命令(q)作为末端2DoF状态占位
        self._needle_last_pos = (0.0, 0.0)
        self._needle_last_vel = (0.0, 0.0)

        # ---- real-arm joint convention fix (CPS real robot vs teleop predefined) ----
        # 实测：真实机械臂 J3/J5 方向与上层相反；J6 相差 180deg。
        # 这里在“实机侧”做转换，保证 DDS 侧仍保持预定义关节语义。
        self._arm_flip_idx = {2, 4}  # 0-based indices: J3, J5
        self._arm_j6_offset_rad = float(np.pi)  # works in *command semantic(rad)* space

        # ---- unit conversion ----
        # CPS API 的关节角读写单位为 deg；DDS 侧统一使用 rad。
        self._cps_deg_per_rad = float(180.0 / np.pi)

    def _wrap_pi(self, a: float) -> float:
        # wrap to (-pi, pi]
        # x = (float(a)) % (2.0 * np.pi)
        x = min(2.0 * np.pi, max(-2.0 * np.pi, a))
        return float(x)

    def _cmd_to_real_arm_q6(self, q6_cmd):
        """DDS(cmd)关节(rad, 预定义语义) -> 真实机械臂(CPS)关节(deg)."""
        q = [float(x) for x in q6_cmd]
        
        # print(q6_cmd)
        # convention fix in rad space
        for i in range(min(6, len(q))):
            if i in self._arm_flip_idx:
                q[i] = -q[i]
        if len(q) >= 6:
            q[5] = self._wrap_pi(q[5] - self._arm_j6_offset_rad)

        # convert to deg for CPS
        q_deg = [v * self._cps_deg_per_rad for v in q[:6]]
        return q_deg

    def _real_to_cmd_arm_q6(self, q6_real):
        """真实机械臂(CPS)关节(deg) -> DDS(state)关节(rad, 预定义语义)."""
        q_deg = [float(x) for x in q6_real]
        q = [v / self._cps_deg_per_rad for v in q_deg[:6]]

        # convention fix back in rad space
        for i in range(min(6, len(q))):
            if i in self._arm_flip_idx:
                q[i] = -q[i]
        if len(q) >= 6:
            q[5] = self._wrap_pi(q[5] + self._arm_j6_offset_rad)

        return q[:6]

    def move_arm_to_initial(self, q6_init_rad, speed: float = 50.0) -> bool:
        """启动时回到指定初始关节角(q6_init_rad, rad)。

        说明：这里依旧使用 HRIF_MoveJ（非伺服），适合“回零/回初始位”。
        """
        if not self._arm_connected:
            return False
        if not hasattr(self.arm, "HRIF_MoveJ"):
            return False
        try:
            q6_init_rad = [float(x) for x in q6_init_rad][:6]
            q6_real_deg = self._cmd_to_real_arm_q6(q6_init_rad)

            self.arm.HRIF_MoveJ(
                self.cps_cfg.box_id,
                self.cps_cfg.rbt_id,
                list(q6_real_deg),
                list(q6_real_deg),
                "TCP",
                "Base",
                float(speed),
                float(speed),
                0,
                1,
                0,
                0,
                0,
                "0",
            )
            return True
        except Exception as e:
            logger_mp.warning(f"[CPS] move to initial failed: {e}")
            return False

    def move_arm_to_initial_deg(self, q6_init_deg, speed: float = 50.0) -> bool:
        """启动时回到指定初始关节角（单位 deg，且可直接发送给机械臂的真实关节角）。

        注意：
        - 这里 *不做* rad/deg 单位换算；认为输入就是 CPS 侧实际要用的 deg。
        - 仍保持对真实机械臂的约定修正（J3/J5取反、J6+180deg），让 main 里填写的角度
          与“直接发给机械臂”的口径一致。
        """
        if not self._arm_connected:
            return False
        if not hasattr(self.arm, "HRIF_MoveJ"):
            return False
        try:
            q = [float(x) for x in q6_init_deg][:6]

            # convention fix in deg space
            for i in range(min(6, len(q))):
                if i in self._arm_flip_idx:
                    q[i] = -q[i]
            if len(q) >= 6:
                q[5] = float(q[5]) - 180.0

            self.arm.HRIF_MoveJ(
                self.cps_cfg.box_id,
                self.cps_cfg.rbt_id,
                list(q),
                list(q),
                "TCP",
                "Base",
                float(speed),
                float(speed),
                0,
                1,
                0,
                0,
                0,
                "0",
            )
            return True
        except Exception as e:
            logger_mp.warning(f"[CPS] move to initial(deg) failed: {e}")
            return False

    def connect_arm(self) -> bool:
        try:
            if hasattr(self.arm, "HRIF_Connect"):
                ret = self.arm.HRIF_Connect(self.cps_cfg.box_id, self.cps_cfg.ip, self.cps_cfg.port)
                self._arm_connected = (ret == 0)
                if not self._arm_connected:
                    logger_mp.error(f"[CPS] HRIF_Connect failed ret={ret}")
                return self._arm_connected
            logger_mp.error("[CPS] HRIF_Connect not found in CPSClient")
            return False
        except Exception as e:
            logger_mp.error(f"[CPS] connect exception: {e}")
            return False

    def start(self, sub_hz: float = 60.0, pub_hz: float = 200.0):
        if self._running:
            return
        self._running = True
        self._thread_sub = threading.Thread(target=self._sub_loop, args=(sub_hz,), daemon=True)
        self._thread_pub = threading.Thread(target=self._pub_loop, args=(pub_hz,), daemon=True)
        self._thread_sub.start()
        self._thread_pub.start()

    def stop(self):
        self._running = False
        if self._thread_sub is not None:
            self._thread_sub.join(timeout=1.0)
        if self._thread_pub is not None:
            self._thread_pub.join(timeout=1.0)

    def enable_needle(self, enabled: bool = True) -> None:
        """显式启用/关闭末端2DoF。

        enabled=True 时才会创建 AcuNeedleMotors（打开串口并写初始化寄存器）。
        """
        if enabled and not self._needle_enabled:
            if self.needle is None:
                try:
                    self.needle = AcuNeedleMotors(self._needle_cfg)
                    logger_mp.info("[NeedleMotors] enabled and initialized")
                except Exception as e:
                    logger_mp.error(f"[NeedleMotors] enable failed: {e}")
                    self.needle = None
                    self._needle_enabled = False
                    return
            self._needle_enabled = True
            return

        if (not enabled) and self._needle_enabled:
            # 不主动关闭串口（Motor1.py未提供统一close封装）；仅停止下发并保留占位状态
            self._needle_enabled = False

    def _read_arm_q6_impl(self):
        q6 = [0.0] * 6
        if self._arm_connected and hasattr(self.arm, "HRIF_ReadActJointPos"):
            result = []
            self._cps_quiet_read = True  # 需要静默就保持 True
            # ret = self.arm.HRIF_ReadActJointPos(self.cps_cfg.box_id, self.cps_cfg.rbt_id, result)
            with _silence_fds(self._cps_quiet_read):
                ret = self.arm.HRIF_ReadActJointPos(self.cps_cfg.box_id, self.cps_cfg.rbt_id, result)

            if ret == 0 and len(result) >= 6:
                try:
                    q6_real = [float(x) for x in result[:6]]
                    q6 = self._real_to_cmd_arm_q6(q6_real)
                except Exception:
                    pass
        return q6

    def _read_arm_q6(self):
        return self._read_arm_q6_impl()

    def _write_arm_q6(self, q6):
        # 最小实现：MoveJ；如需伺服请替换为 CPS 可用的更实时接口
        if self._arm_connected and hasattr(self.arm, "HRIF_MoveJ"):
            try:
                q6_real = self._cmd_to_real_arm_q6(q6)
                print(q6_real)
                # q6_real[5] = 0
                with _silence_fds(self._cps_quiet_read):
                    self.arm.HRIF_MoveJ(
                        self.cps_cfg.box_id,
                        self.cps_cfg.rbt_id,
                        list(q6_real),
                        list(q6_real),
                        "TCP",
                        "Base",
                        100,
                        100,
                        0,
                        1,
                        0,
                        0,
                        0,
                        "0",
                    )
                # self.arm.HRIF_MoveJ(
                #         self.cps_cfg.box_id,
                #         self.cps_cfg.rbt_id,
                #         list(q6_real),
                #         list(q6_real),
                #         "TCP",
                #         "Base",
                #         100,
                #         100,
                #         0,
                #         1,
                #         0,
                #         0,
                #         0,
                #         "0",
                #     )
            except Exception as e:
                logger_mp.warning(f"[CPS] HRIF_MoveJ failed: {e}")

    def _apply_needle_from_cmd(self, cmd: Acu_LowCmd):
        """末端2DOF默认速度控制（但默认关闭执行）。

        - 保留8DoF命令接收范式：仍解析 motor_cmd[6/7]。
        - 默认不对硬件下发，只有在 self._needle_enabled=True 时才执行。
        - 当关闭时，仅更新缓存的“上一帧速度”便于后续开启时平滑。
        """
        try:
            v_insert = float(cmd.motor_cmd[6].dq)
            v_twist = float(cmd.motor_cmd[7].dq)

            if abs(v_insert) < self._needle_deadband and abs(v_twist) < self._needle_deadband:
                v_insert = float(getattr(cmd.motor_cmd[6], "tau", 0.0))
                v_twist = float(getattr(cmd.motor_cmd[7], "tau", 0.0))

            if abs(v_insert) < self._needle_deadband:
                v_insert = 0.0
            if abs(v_twist) < self._needle_deadband:
                v_twist = 0.0

            v_insert = max(-self._needle_v_insert_limit, min(self._needle_v_insert_limit, v_insert))
            v_twist = max(-self._needle_v_twist_limit, min(self._needle_v_twist_limit, v_twist))

            # 始终更新缓存（即使不执行）
            self._needle_last_vel = (v_insert, v_twist)

            if not self._needle_enabled or self.needle is None:
                return

            if v_insert == 0.0 and v_twist == 0.0:
                return

            self.needle.set_velocity(v_insert, v_twist)
        except Exception:
            return

    def _update_needle_placeholder_from_cmd(self, cmd: Acu_LowCmd) -> None:
        """末端关闭时，用cmd里的q(位置目标)更新占位状态，避免一直为0。"""
        try:
            p1 = float(cmd.motor_cmd[6].q)
            p2 = float(cmd.motor_cmd[7].q)
            # 过滤NaN/inf
            if not (p1 == p1 and p2 == p2):
                return
            if abs(p1) > 1e9 or abs(p2) > 1e9:
                return
            self._needle_last_pos = (p1, p2)
        except Exception:
            return

    def _get_needle_pos_for_state(self) -> Tuple[float, float]:
        """用于8DoF lowstate 的末端2DoF位置。

        默认不开启时，不主动采集/驱动末端硬件（避免意外动作与串口阻塞），
        直接复用上一帧缓存位置作为占位。
        """
        if not self._needle_enabled or self.needle is None:
            return self._needle_last_pos
        try:
            self._needle_last_pos = self.needle.read_position_counts()
        except Exception:
            # keep last
            pass
        return self._needle_last_pos

    def _build_state_msg(self):
        """Build LowStateAcu message (must use default class)."""
        msg = unitree_Acu_msg_dds__LowState_()

        q6 = Acu8DoFDDSServer._read_arm_q6_impl(self)
        p1, p2 = self._get_needle_pos_for_state()
        q8 = list(q6) + [p1, p2]

        # only update tick and motor_state; other header fields (version, mode_pr, mode_machine) 
        # are handled by serialization or left at default.
        try:
            msg.tick = int(time.time() * 1000) & 0xFFFFFFFF
        except Exception:
            pass

        # motor_state[0..7]
        for i in range(8):
            msg.motor_state[i].q = float(q8[i])
            msg.motor_state[i].dq = 0.0

        # crc keep 0
        try:
            msg.crc = 0
        except Exception:
            pass

        return msg

    def _sub_loop(self, hz: float):
        dt = 1.0 / max(1e-6, float(hz))
        while self._running:
            t0 = time.time()
            try:
                cmd = self.sub_cmd.Read()
                if cmd is not None:
                    # 更新末端占位状态（无论是否启用硬件）
                    if not self._needle_enabled:
                        self._update_needle_placeholder_from_cmd(cmd)

                    with self._lock:
                        self._last_cmd = cmd

                    # 控制（订阅到新cmd时才更新控制）
                    try:
                        q6_target = [float(cmd.motor_cmd[i].q) for i in range(6)]
                        print(q6_target)
                        self._write_arm_q6(q6_target)
                    except Exception:
                        pass

                    self._apply_needle_from_cmd(cmd)
            except Exception as e:
                logger_mp.warning(f"[AcuDDS] sub loop error: {e}")

            time.sleep(max(0.0, dt - (time.time() - t0)))

    def _pub_loop(self, hz: float):
        dt = 1.0 / max(1e-6, float(hz))
        while self._running:
            t0 = time.time()
            try:
                # print("type(self._build_state_msg):", type(self._build_state_msg), callable(self._build_state_msg))
                # print("type(self.pub_state):", type(self.pub_state))
                # print("type(self.pub_state.Write):", type(self.pub_state.Write), callable(self.pub_state.Write))

                msg = self._build_state_msg()
                # print("type(msg):", type(msg))

                self.pub_state.Write(msg)
            except Exception as e:
                import traceback
                traceback.print_exc()
                logger_mp.warning(f"[AcuDDS] pub loop error: {e}")
            time.sleep(max(0.0, dt - (time.time() - t0)))


def _start_image_server(cfg: ImageConfig) -> threading.Thread:
    config = {
        "fps": int(cfg.fps),
        "head_camera_type": str(cfg.head_camera_type),
        "head_camera_image_shape": [int(cfg.head_camera_image_shape[0]), int(cfg.head_camera_image_shape[1])],
        "head_camera_id_numbers": list(cfg.head_camera_id_numbers),
    }

    if cfg.wrist_camera_type and cfg.wrist_camera_id_numbers:
        config.update(
            {
                "wrist_camera_type": str(cfg.wrist_camera_type),
                "wrist_camera_image_shape": [int(cfg.wrist_camera_image_shape[0]), int(cfg.wrist_camera_image_shape[1])],
                "wrist_camera_id_numbers": list(cfg.wrist_camera_id_numbers),
            }
        )

    server = ImageServer(config=config, port=int(cfg.port), Unit_Test=False)
    th = threading.Thread(target=server.send_process, daemon=True)
    th.start()
    logger_mp.info(f"[Image] server started at tcp://*:{cfg.port}")
    return th


def main():
    try:
        # 与 teleop/robot_control/robot_arm.py 一致，真实硬件 domain 一般是0
        ChannelFactoryInitialize(0)  # domain_id=1 避免和仿真环境冲突
    except Exception as e:
        logger_mp.warning(f"ChannelFactoryInitialize failed: {e}")

    print("i am here")
    # ---- user-defined initial pose (deg, directly sendable to real arm) ----
    # 按需修改为你的初始位（6关节，单位 deg）。
    # 该脚本启动后会先 MoveJ 到该关节角，再启动 DDS 收发循环。
    ARM_INIT_Q6_DEG = [0.0, 16.715999603271484, 75.27999877929688, 0.0, 87.05400085449219, 180.0]
    ARM_INIT_SPEED = 300.0

    img_cfg = ImageConfig()
    cps_cfg = CPSConfig()
    needle_cfg = NeedleMotorConfig()
    force_cfg = ForceSensorConfig()

    _start_image_server(img_cfg)

    acu_dds = Acu8DoFDDSServer(cps_cfg, needle_cfg)
    if not acu_dds.connect_arm():
        logger_mp.warning("[CPS] arm not connected; will still publish lowstate with zeros for arm joints.")
    else:
        acu_dds.move_arm_to_initial_deg(ARM_INIT_Q6_DEG, speed=ARM_INIT_SPEED)
        pass

    acu_dds.start(sub_hz=50.0, pub_hz=100.0)

    # force_pub = ChannelPublisher(TOPIC_ACU_FORCE, String_)
    # force_pub.Init()
    # force = AcuForceSensorDDS(force_cfg, force_pub)
    # force.start()

    logger_mp.info("[AcuHW] running (image+dds+force). Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger_mp.info("[AcuHW] KeyboardInterrupt")
    finally:
        # force.stop()
        acu_dds.stop()
        pass


if __name__ == "__main__":
    main()
