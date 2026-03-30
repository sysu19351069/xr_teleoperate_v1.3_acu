#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Quick checks for ACU real-hardware experiment.

提供几个“单机/单模块”的快速测试入口：
1) camera: 连接 image_server(ZMQ) 并显示图像
2) cps:   连接 CPS 机器人，读取关节角，并做一个很小的关节抖动动作（可选）
3) dds_state: 订阅 rt/acu_lowstate，打印 8DoF q
4) force: 订阅 rt/acu_force(String_)，打印力

注意：
- 这些测试会驱动真实硬件（尤其 cps_test_move）。执行前请确保安全区域、急停可用。
"""

from __future__ import annotations

import argparse
import ast
import json
import time
from typing import List

import numpy as np

import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os
import sys
# ---- path fix for workspace imports ----
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.dirname(THIS_DIR)
if WS_ROOT not in sys.path:
    sys.path.append(WS_ROOT)

def camera_test(server_ip: str, port: int = 5555):
    import cv2
    import zmq
    import numpy as np

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{server_ip}:{port}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "")

    logger_mp.info(f"[camera_test] connected to tcp://{server_ip}:{port}")
    try:
        while True:
            jpg_bytes = sock.recv()
            np_img = np.frombuffer(jpg_bytes, dtype=np.uint8)
            img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
            if img is None:
                logger_mp.warning("[camera_test] decode failed")
                continue
            h, w = img.shape[:2]
            show = cv2.resize(img, (w // 2, h // 2))
            cv2.imshow("ACU Camera Stream", show)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        sock.close()
        ctx.term()
        cv2.destroyAllWindows()


def cps_test_read(ip: str, port: int, box_id: int = 0, rbt_id: int = 0):
    from Acupuncture_Software.CPS import CPSClient

    cps = CPSClient()
    if not hasattr(cps, "HRIF_Connect"):
        raise RuntimeError("CPSClient.HRIF_Connect not found")

    ret = cps.HRIF_Connect(box_id, ip, port)
    print(ret)
    if ret != 0:
        raise RuntimeError(f"HRIF_Connect failed ret={ret}")

    if not hasattr(cps, "HRIF_ReadActJointPos"):
        raise RuntimeError("CPSClient.HRIF_ReadActJointPos not found")

    for _ in range(50):
        result: List[str] = []
        ret = cps.HRIF_ReadActJointPos(box_id, rbt_id, result)
        if ret != 0:
            logger_mp.warning(f"HRIF_ReadActJointPos ret={ret}")
            time.sleep(0.01)
            continue
        try:
            q = [float(x) for x in result[:6]]
        except Exception:
            q = result[:6]
        logger_mp.info(f"[cps_test_read] q={q}")
        time.sleep(0.1)


def cps_test_move_small(ip: str, port: int, box_id: int = 0, rbt_id: int = 0, delta_deg: float = 1.0, joint: int = 0):
    """做一个非常小的关节抖动动作：q[joint] += delta，然后回到原位。

    目的：验证通讯链路和 MoveJ 是否可用。
    """

    from Acupuncture_Software.CPS import CPSClient

    cps = CPSClient()
    ret = cps.HRIF_Connect(box_id, ip, port)
    if ret != 0:
        raise RuntimeError(f"HRIF_Connect failed ret={ret}")

    result: List[str] = []
    ret = cps.HRIF_ReadActJointPos(box_id, rbt_id, result)
    if ret != 0:
        raise RuntimeError(f"HRIF_ReadActJointPos failed ret={ret}")

    q0 = [float(x) for x in result[:6]]
    q1 = q0.copy()
    q1[int(joint)] += float(delta_deg)

    if not hasattr(cps, "HRIF_MoveJ"):
        raise RuntimeError("CPSClient.HRIF_MoveJ not found")

    logger_mp.warning("[cps_test_move_small] WILL MOVE REAL ROBOT. Ensure safety, press Ctrl+C to cancel within 2s...")
    time.sleep(2.0)

    def _move(q):
        # 参数参考 Acupuncture_Robot.py 注释段
        cps.HRIF_MoveJ(
            box_id,
            rbt_id,
            q,
            q,
            "TCP",
            "Base",
            30,
            30,
            0,
            1,
            0,
            0,
            0,
            "0",
        )

    logger_mp.info(f"[cps_test_move_small] move to q1={q1}")
    _move(q1)
    time.sleep(3.0)
    logger_mp.info(f"[cps_test_move_small] move back q0={q0}")
    _move(q0)


def dds_state_watch(domain_id: int = 0, hz: float = 10.0):
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowStateAcu_ as Acu_LowState

    ChannelFactoryInitialize(int(domain_id))
    sub = ChannelSubscriber("rt/acu_lowstate", Acu_LowState)
    sub.Init()

    dt = 1.0 / max(1e-6, float(hz))
    logger_mp.info(f"[dds_state_watch] subscribed rt/acu_lowstate domain={domain_id}")
    while True:
        msg = sub.Read()
        if msg is not None:
            try:
                q = [float(msg.motor_state[i].q) for i in range(8)]
            except Exception:
                q = []
            logger_mp.info(f"[dds_state_watch] q={q}")
        time.sleep(dt)


def force_watch(domain_id: int = 0, hz: float = 10.0):
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

    ChannelFactoryInitialize(int(domain_id))
    sub = ChannelSubscriber("rt/acu_force", String_)
    sub.Init()

    dt = 1.0 / max(1e-6, float(hz))
    logger_mp.info(f"[force_watch] subscribed rt/acu_force domain={domain_id}")
    while True:
        msg = sub.Read()
        if msg is not None and getattr(msg, "data", None):
            s = msg.data
            # payload 在 experiment_site_acu_hw.py 里是 dict->str 的 JSON-like
            data = None
            try:
                data = json.loads(s)
            except Exception:
                try:
                    data = ast.literal_eval(s)
                except Exception:
                    data = s
            logger_mp.info(f"[force_watch] {data}")
        time.sleep(dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", required=True, choices=["camera", "cps_read", "cps_move", "dds_state", "force"], help="which quickcheck to run")

    # camera
    ap.add_argument("--img-server-ip", default="127.0.0.1")
    ap.add_argument("--img-port", type=int, default=5555)

    # cps
    ap.add_argument("--cps-ip", default="192.168.0.10")
    ap.add_argument("--cps-port", type=int, default=10003)
    ap.add_argument("--box-id", type=int, default=0)
    ap.add_argument("--rbt-id", type=int, default=0)
    ap.add_argument("--delta-deg", type=float, default=1.0)
    ap.add_argument("--joint", type=int, default=0)

    # dds
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--hz", type=float, default=10.0)

    args = ap.parse_args()

    if args.mode == "camera":
        camera_test(args.img_server_ip, args.img_port)
    elif args.mode == "cps_read":
        cps_test_read(args.cps_ip, args.cps_port, args.box_id, args.rbt_id)
    elif args.mode == "cps_move":
        cps_test_move_small(args.cps_ip, args.cps_port, args.box_id, args.rbt_id, args.delta_deg, args.joint)
    elif args.mode == "dds_state":
        dds_state_watch(args.domain, args.hz)
    elif args.mode == "force":
        force_watch(args.domain, args.hz)


if __name__ == "__main__":
    main()
