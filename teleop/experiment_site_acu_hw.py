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


# 与 robot_arm.py 对齐
kTopicLowCommand_Debug = "rt/acu_lowcmd"
kTopicLowState = "rt/acu_lowstate"
TOPIC_ACU_FORCE = "rt/acu_force"


@dataclass
class ImageConfig:
    fps: int = 30
    head_camera_type: str = "realsense"  # 'opencv' | 'realsense'
    head_camera_image_shape: Tuple[int, int] = (720, 1280)  # (H,W)
    head_camera_id_numbers: Tuple[str, ...] = ("220222067428",)  # realsense serial

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

        self.needle = AcuNeedleMotors(needle_cfg)

        self.sub_cmd = ChannelSubscriber(kTopicLowCommand_Debug, Acu_LowCmd)
        self.pub_state = ChannelPublisher(kTopicLowState, Acu_LowState)
        self.sub_cmd.Init()
        self.pub_state.Init()

        self._running = False
        self._thread: Optional[threading.Thread] = None

        # caches
        self._last_cmd: Optional[Acu_LowCmd] = None
        self._lock = threading.Lock()

        self._needle_deadband = 1e-3
        # 速度限幅：insert(mm/s), twist(r/s)
        self._needle_v_insert_limit = 5.0
        self._needle_v_twist_limit = 5.0

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

    def start(self, hz: float = 250.0):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(hz,), daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _read_arm_q6(self):
        q6 = [0.0] * 6
        if self._arm_connected and hasattr(self.arm, "HRIF_ReadActJointPos"):
            result = []
            ret = self.arm.HRIF_ReadActJointPos(self.cps_cfg.box_id, self.cps_cfg.rbt_id, result)
            if ret == 0 and len(result) >= 6:
                try:
                    q6 = [float(x) for x in result[:6]]
                except Exception:
                    pass
        return q6

    def _write_arm_q6(self, q6):
        # 最小实现：MoveJ；如需伺服请替换为 CPS 可用的更实时接口
        if self._arm_connected and hasattr(self.arm, "HRIF_MoveJ"):
            try:
                self.arm.HRIF_MoveJ(
                    self.cps_cfg.box_id,
                    self.cps_cfg.rbt_id,
                    list(q6),
                    list(q6),
                    "TCP",
                    "Base",
                    50,
                    50,
                    0,
                    1,
                    0,
                    0,
                    0,
                    "0",
                )
            except Exception as e:
                logger_mp.warning(f"[CPS] HRIF_MoveJ failed: {e}")

    def _apply_needle_from_cmd(self, cmd: Acu_LowCmd):
        """末端2DOF默认速度控制。

        默认读取：motor_cmd[6].dq / motor_cmd[7].dq
        兼容读取：若dq全0且tau非0，则用tau作为速度通道（便于你后续在遥操端快速映射）。
        """
        try:
            v_insert = float(cmd.motor_cmd[6].dq)
            v_twist = float(cmd.motor_cmd[7].dq)

            if abs(v_insert) < self._needle_deadband and abs(v_twist) < self._needle_deadband:
                # fallback: use tau as velocity channel when dq is not populated
                v_insert = float(getattr(cmd.motor_cmd[6], "tau", 0.0))
                v_twist = float(getattr(cmd.motor_cmd[7], "tau", 0.0))

            # deadband again
            if abs(v_insert) < self._needle_deadband:
                v_insert = 0.0
            if abs(v_twist) < self._needle_deadband:
                v_twist = 0.0

            # clamp
            v_insert = max(-self._needle_v_insert_limit, min(self._needle_v_insert_limit, v_insert))
            v_twist = max(-self._needle_v_twist_limit, min(self._needle_v_twist_limit, v_twist))

            if v_insert == 0.0 and v_twist == 0.0:
                return

            self.needle.set_velocity(v_insert, v_twist)
        except Exception:
            return

    def _build_state_msg(self) -> Acu_LowState:
        msg = Acu_LowState()
        # 读取6DOF + 2DOF
        q6 = self._read_arm_q6()
        p1, p2 = self.needle.read_position_counts()
        q8 = list(q6) + [p1, p2]

        # 填充 LowStateAcu_ motor_state[i].q / dq
        try:
            for i in range(8):
                msg.motor_state[i].q = float(q8[i])
                msg.motor_state[i].dq = 0.0
        except Exception:
            pass
        return msg

    def _loop(self, hz: float):
        dt = 1.0 / max(1e-6, float(hz))
        while self._running:
            t0 = time.time()
            try:
                cmd = self.sub_cmd.Read()
                if cmd is not None:
                    with self._lock:
                        self._last_cmd = cmd

                with self._lock:
                    cmd_local = self._last_cmd

                if cmd_local is not None:
                    # 前6轴：位置目标（q）
                    try:
                        q6_target = [float(cmd_local.motor_cmd[i].q) for i in range(6)]
                        self._write_arm_q6(q6_target)
                    except Exception:
                        pass

                    # 末端2DOF：默认用 dq 当速度通道
                    self._apply_needle_from_cmd(cmd_local)

                # publish state
                self.pub_state.Write(self._build_state_msg())

            except Exception as e:
                logger_mp.warning(f"[AcuDDS] loop error: {e}")

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
        ChannelFactoryInitialize(0)
    except Exception as e:
        logger_mp.warning(f"ChannelFactoryInitialize failed: {e}")

    img_cfg = ImageConfig()
    cps_cfg = CPSConfig()
    needle_cfg = NeedleMotorConfig()
    force_cfg = ForceSensorConfig()

    _start_image_server(img_cfg)

    acu_dds = Acu8DoFDDSServer(cps_cfg, needle_cfg)
    if not acu_dds.connect_arm():
        logger_mp.warning("[CPS] arm not connected; will still publish lowstate with zeros for arm joints.")
    acu_dds.start(hz=60.0)

    force_pub = ChannelPublisher(TOPIC_ACU_FORCE, String_)
    force_pub.Init()
    force = AcuForceSensorDDS(force_cfg, force_pub)
    force.start()

    logger_mp.info("[AcuHW] running (image+dds+force). Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger_mp.info("[AcuHW] KeyboardInterrupt")
    finally:
        force.stop()
        acu_dds.stop()


if __name__ == "__main__":
    main()
