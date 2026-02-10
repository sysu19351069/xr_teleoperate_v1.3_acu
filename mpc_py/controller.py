from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
import scipy.sparse as sp
import osqp
import json
import subprocess
import os
import sys
import atexit
import time
from typing import Literal

try:
    import pinocchio as pin
except Exception:  # pragma: no cover
    import pin  # type: ignore

try:
    import zmq  # type: ignore
except Exception:  # pragma: no cover
    zmq = None

from .lifted import LiftedSystem, build_lifted_matrices
from .mpc2qp import QPData, mpc2qp_accurate, mpc2qp_rough
from .robot_pinocchio import CasadiSingleArmIK, PinRobot, detect_linear_jacobian_rows, ik_dls


class MPCMode(str, Enum):
    ROUGH = "rough"  # LSR
    ACCURATE = "accurate"  # LAR


@dataclass
class MPCStepResult:
    q_next: np.ndarray
    u_next: np.ndarray
    x_opt: np.ndarray
    qp: QPData
    qkd: np.ndarray


def _build_Q_R_T(*, num_jont: int, num_ctrl: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """复刻 C++ accurate_positioning.cpp 的 Q/R/T 设定。"""

    n = num_jont
    N = num_ctrl

    Qi = np.eye(2 * n)
    Ri = 0.1 * np.eye(n)

    P = np.zeros((2 * n, 2 * n))
    diag = [
        19.1677,
        41.7573,
        31.6742,
        35.2202,
        39.2995,
        43.9919,
        49.3902,
        55.6025,
        366.2165,
        685.1713,
        560.9258,
        607.9773,
        659.7306,
        716.7489,
        779.6759,
        849.2492,
    ]
    for i, v in enumerate(diag):
        P[i, i] = v

    P[0, 8] = P[8, 0] = -43.0184
    P[1, 9] = P[9, 1] = -129.7105
    P[2, 10] = P[10, 2] = -93.5113
    P[3, 11] = P[11, 3] = -106.6855

    P[4, 12] = P[12, 4] = -121.4779
    P[5, 13] = P[13, 5] = -138.1033
    P[6, 14] = P[14, 6] = -156.8090
    P[7, 15] = P[15, 7] = -177.8805

    T = 10.0 * P[n : 2 * n, n : 2 * n]

    Q = np.zeros((2 * n * N, 2 * n * N))
    for i in range(N):
        Q[(2 * n) * i : (2 * n) * (i + 1), (2 * n) * i : (2 * n) * (i + 1)] = Qi if i != (N - 1) else P

    R = np.zeros((n * (N + 1), n * (N + 1)))
    for i in range(N + 1):
        R[n * i : n * (i + 1), n * i : n * (i + 1)] = Ri if i != N else T

    return Q, R, T


# ============================================================
# NOTE(2026-01-23): 仅使用 ZMQ 通信。
# 旧的“子进程 + stdin/stdout JSONL”的 qpoases-ipc 方案容易被 stdout 调试输出污染，
# 且你已明确不再使用，因此这里将其代码整体禁用（保留注释以便未来回滚）。
# ============================================================

# class _QPOASESIPCClient:
#     """通过子进程 + JSONL 协议调用另一个环境中的 qpOASES Python 求解器。
#
#     约定协议（一行一个 JSON 对象）：
#     - 请求：{"cmd":"init"|"hotstart"|"close", "H":..., "g":..., "A":..., "lb":..., "ub":..., "lbA":..., "ubA":..., "nWSR": 200}
#       其中矩阵/向量都用 list（row-major 的二维 list），A 对应一般约束矩阵 G。
#       lb/ub 可为空（null）表示无 box 约束。
#     - 响应：{"status":"ok","x": [...],"obj": float, "nWSR": int} 或 {"status":"err","message": "..."}
#     """
#
#     def __init__(self, cmd: list[str]) -> None:
#         # ...existing code...
#         pass
#
#     def solve(self, *, H: np.ndarray, g: np.ndarray, A: np.ndarray, lbA: np.ndarray, ubA: np.ndarray, nWSR: int = 200) -> np.ndarray:
#         # ...existing code...
#         raise NotImplementedError
#
#     def close(self) -> None:
#         # ...existing code...
#         pass


class _QPOASESZMQClient:
    def __init__(self, *, addr: str = "tcp://127.0.0.1:5555", timeout_ms: int = 2000) -> None:
        if zmq is None:
            raise RuntimeError("pyzmq not installed; cannot use qpoases-zmq backend")
        self._addr = addr
        self._timeout_ms = int(timeout_ms)
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        self._sock.connect(self._addr)
        self._inited = False

    def _rpc(self, payload: dict) -> dict:
        try:
            self._sock.send_string(json.dumps(payload))
            msg = self._sock.recv_string()
        except Exception as e:
            raise RuntimeError(f"qpoases zmq rpc failed: {e}")
        resp = json.loads(msg)
        if resp.get("status") != "ok":
            raise RuntimeError(f"qpoases zmq error: {resp}")
        return resp

    def solve(self, *, H: np.ndarray, g: np.ndarray, A: np.ndarray, lbA: np.ndarray, ubA: np.ndarray, nWSR: int = 200) -> np.ndarray:
        payload = {
            "cmd": "init" if not self._inited else "hotstart",
            "H": H.tolist(),
            "g": g.reshape(-1).tolist(),
            "A": A.tolist(),
            "lb": None,
            "ub": None,
            "lbA": lbA.reshape(-1).tolist(),
            "ubA": ubA.reshape(-1).tolist(),
            "nWSR": int(nWSR),
        }
        resp = self._rpc(payload)
        self._inited = True
        return np.asarray(resp["x"], dtype=float).reshape(-1, 1)

    def close(self) -> None:
        try:
            self._rpc({"cmd": "close"})
        except Exception:
            pass
        try:
            self._sock.close(0)
        except Exception:
            pass


class AcuMPCController:
    """统一对外的 MPC 控制器。

    设计目标（对齐你 C++ rough_positioning / accurate_positioning 的结构）：
    - 两阶段 mpc2qp 接口一致；差异主要体现在：
      1) Sd 的来源（外部规划器）
      2) rough 多了姿态保持等式约束
      3) 约束上下限的数值略不同
    - 控制器只负责：
      1) 机器人学：FK/Jacobian/IK（Pinocchio + CasADi）
      2) MPC：构造 QP 并解出 x_opt
      3) 输出：q_next/u_next

    对外尽量少参数：
    - step(qk, Sd, mode, target_pose=None)
      - qk: (8,) or (8,1)
      - Sd: (3,) or (3,1)
      - target_pose: 可选。若传入，则内部先 IK 得到 qkd（近似 RobCoGen ikine）。
        若不传入，则默认 qkd=qk（只按 Sd 做局部修正）。
    """

    def __init__(
        self,
        *,
        urdf_path: str | Path,
        dt: float = 0.1,
        horizon_T: float = 1.0,
        num_jont: int = 8,
        ee_frame: str = "Acu_ee",
        solver_backend: Literal["osqp", "qpoases-zmq"] = "osqp",
        qpoases_ipc_cmd: list[str] | None = None,
        qpoases_zmq_addr: str = "tcp://127.0.0.1:5555",
    ) -> None:
        self.dt = float(dt)
        self.num_jont = int(num_jont)
        self.num_ctrl = int(round(float(horizon_T) / float(dt)))

        self.lifted: LiftedSystem = build_lifted_matrices(num_ctrl=self.num_ctrl, num_jont=self.num_jont, dt=self.dt)
        self.Q, self.R, self.T = _build_Q_R_T(num_jont=self.num_jont, num_ctrl=self.num_ctrl)

        self.robot = PinRobot.from_urdf(urdf_path, ee_frame=ee_frame)

        # 兼容：默认 IK 仍指向 Acu_ee；精定位时在 step() 内按需用 needle_head IK。
        self._ik = CasadiSingleArmIK(self.robot, frame="Acu_ee")
        self._ik_needle_head = CasadiSingleArmIK(self.robot, frame=self.robot.needle_head_frame)

        self._solver_backend = solver_backend

        # 旧 stdout/stderr IPC 已禁用：不再创建子进程客户端
        self._qpoases = None

        self._qpoases_zmq: _QPOASESZMQClient | None = None
        if self._solver_backend == "qpoases-zmq":
            self._qpoases_zmq = _QPOASESZMQClient(addr=qpoases_zmq_addr)

        # solver 缓存（OSQP）
        self._solver: osqp.OSQP | None = None

        # 内部状态缓存（如果用户不提供 qk_1/uk_1，就用内部缓存）
        self._qk_1: np.ndarray | None = None
        self._uk_1: np.ndarray | None = None

        self._linear_rows: slice | None = None
        self._angular_rows: slice | None = None
        self._linear_rows_needle: slice | None = None
        self._angular_rows_needle: slice | None = None

        self._ik_res: np.ndarray | None = None

        self._Sd_real: np.ndarray | None = None

    def reset(self) -> None:
        self._qk_1 = None
        self._uk_1 = None

    def _ensure_jacobian_rows(self, qk: np.ndarray, *, frame: str | int | None = None) -> tuple[slice, slice]:
        """缓存不同 frame 的 Jacobian 行排列。

        - frame=None: 兼容旧行为（Acu_ee）
        - frame='needle_head': 精定位使用
        """
        if frame is None or frame == self.robot.ee_frame or frame == "Acu_ee":
            if self._linear_rows is None or self._angular_rows is None:
                self._linear_rows, self._angular_rows = detect_linear_jacobian_rows(self.robot, qk, frame=None)
            return self._linear_rows, self._angular_rows

        if frame == self.robot.needle_head_frame:
            if self._linear_rows_needle is None or self._angular_rows_needle is None:
                self._linear_rows_needle, self._angular_rows_needle = detect_linear_jacobian_rows(self.robot, qk, frame=self.robot.needle_head_frame)
            return self._linear_rows_needle, self._angular_rows_needle

        # 未知 frame：不缓存
        return detect_linear_jacobian_rows(self.robot, qk, frame=frame)

    def _solve_qp(self, qp: QPData) -> np.ndarray:
        # 支持两种后端（osqp / qpoases-zmq）
        if self._solver_backend == "qpoases-zmq":
            if self._qpoases_zmq is None:
                raise RuntimeError("qpoases zmq client not initialized")
            H = qp.H
            return self._qpoases_zmq.solve(H=H, g=qp.g, A=qp.G, lbA=qp.lbA, ubA=qp.ubA, nWSR=200)

        if self._solver_backend != "osqp":
            raise ValueError(f"unsupported solver_backend: {self._solver_backend} (expected 'osqp' or 'qpoases-zmq')")

        # OSQP
        H = qp.H
        g = qp.g
        G = qp.G
        lbA = qp.lbA
        ubA = qp.ubA

        P = sp.csc_matrix(0.5 * (H + H.T))
        # P = sp.csc_matrix(H)
        A = sp.csc_matrix(G)
        q = g.reshape(-1)
        l = lbA.reshape(-1)
        u = ubA.reshape(-1)

        solver = osqp.OSQP()
        solver.setup(P=P, q=q, A=A, l=l, u=u, verbose=False)
        res = solver.solve()
        if res.info.status_val not in (1, 2):
            raise RuntimeError(f"OSQP failed: {res.info.status}")
        return res.x.reshape(-1, 1)

    def close(self) -> None:
        if self._qpoases is not None:
            self._qpoases.close()
            self._qpoases = None
        if self._qpoases_zmq is not None:
            self._qpoases_zmq.close()
            self._qpoases_zmq = None

    def _post_process(self, x_opt: np.ndarray, qk: np.ndarray, uk_1: np.ndarray, J_pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # 与 C++ 完全一致：
        qc = self.lifted.psi @ qk + self.lifted.omega @ uk_1 + self.lifted.theta @ x_opt
        Uk_1 = np.tile(uk_1, (self.num_ctrl, 1))
        U = Uk_1 + self.lifted.Gq_dot @ x_opt
        q_next = qc[0 : self.num_jont, :]
        u_next = U[0 : self.num_jont, :]

        n = self.num_jont
        N = self.num_ctrl
        tmp = np.hstack([np.eye(n), np.zeros((n, n * N))])
        J_pos = J_pos
        # thetapp = self.dt * (J_pos @ tmp)
        Sd_real = self.dt * J_pos @ (tmp @ x_opt + uk_1)

        return q_next, u_next, Sd_real

    def step(
        self,
        *,
        qk: np.ndarray,
        Sd: np.ndarray,
        mode: MPCMode = MPCMode.ACCURATE,
        target_pose: pin.SE3 | None = None,
        qk_1: np.ndarray | None = None,
        uk_1: np.ndarray | None = None,
    ) -> MPCStepResult:
        qk = np.asarray(qk).reshape(self.num_jont, 1)
        Sd = np.asarray(Sd).reshape(3, 1)

        if qk_1 is None:
            qk_1 = self._qk_1 if self._qk_1 is not None else qk.copy()
        qk_1 = np.asarray(qk_1).reshape(self.num_jont, 1)

        if uk_1 is None:
            uk_1 = self._uk_1 if self._uk_1 is not None else np.zeros((self.num_jont, 1))
        uk_1 = np.asarray(uk_1).reshape(self.num_jont, 1)

        # Jacobian（精定位使用 needle_head；粗定位保持 Acu_ee）
        if mode == MPCMode.ACCURATE:
            linear_rows, _ = self._ensure_jacobian_rows(qk, frame=self.robot.needle_head_frame)
            J6 = self.robot.jacobian_frame(qk, frame=self.robot.needle_head_frame)
        else:
            linear_rows, _ = self._ensure_jacobian_rows(qk, frame=None)
            J6 = self.robot.jacobian(qk)
        J_pos = J6[linear_rows, :]

        # artificial reference joint angle：qkd
        if target_pose is None:
            qkd = qk.copy()
        else:
            # 精定位：target_pose 表达的是“针头”目标（调用侧只需传目标点即可）
            if mode == MPCMode.ACCURATE:
                ik_res = self._ik_needle_head.solve(target_pose, q0=qk)
            else:
                ik_res = self._ik.solve(target_pose, q0=qk)

            self._ik_res = ik_res.q
            if not ik_res.success:
                ik_res = ik_dls(self.robot, target_pose, qk, position_only=False)
            qkd = ik_res.q

        if mode == MPCMode.ACCURATE:
            qp = mpc2qp_accurate(
                lifted=self.lifted,
                Q=self.Q,
                R=self.R,
                T=self.T,
                Jacob_pos_3xn=J_pos,
                qk=qk,
                qkd=qkd,
                qk_1=qk_1,
                uk_1=uk_1,
                Sd=Sd,
            )
        else:
            qp = mpc2qp_rough(
                lifted=self.lifted,
                Q=self.Q,
                R=self.R,
                T=self.T,
                Jacob_6xn=J6,
                qk=qk,
                qkd=qkd,
                qk_1=qk_1,
                uk_1=uk_1,
                Sd=Sd,
            )

        x_opt = self._solve_qp(qp)
        q_next, u_next, Sd_real = self._post_process(x_opt, qk, uk_1, J_pos)

        # 更新内部缓存（与 C++ 循环一致）
        self._qk_1 = qk
        self._uk_1 = u_next
        self._Sd_real = Sd_real

        return MPCStepResult(q_next=q_next, u_next=u_next, x_opt=x_opt, qp=qp, qkd=qkd)
