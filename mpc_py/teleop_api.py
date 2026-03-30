from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Deque, Optional, Tuple, List
from collections import deque
import threading

import numpy as np

try:
    import pinocchio as pin
except Exception:  # pragma: no cover
    import pin  # type: ignore

from .controller import AcuMPCController, MPCMode
from .planner_sd import AccuratePlanState, RoughPlanState, compute_sd_accurate, compute_sd_rough, init_accurate_plan, init_rough_plan


@dataclass
class TeleopMPCConfig:
    urdf_path: str | Path
    ee_frame: str = "Acu_ee"
    dt: float = 0.1
    horizon_T: float = 1.0
    solver_backend: Literal["osqp", "qpoases-zmq"] = "osqp"


@dataclass
class TeleopMPCState:
    qk: np.ndarray  # (8,)
    mode: MPCMode
    rough_plan: RoughPlanState | None = None
    accurate_plan: AccuratePlanState | None = None
    target_pose: "pin.SE3" | None = None
    # accurate 轨迹推进（0~1）
    acc_step_ratio: float = 0.0


@dataclass
class Waypoint:
    """A sampled teleop reference pose."""
    p: np.ndarray            # (3,)
    quat: np.ndarray         # (4,) xyzw
    stamp: float = 0.0


def _quat_angle(q1: np.ndarray, q2: np.ndarray) -> float:
    """Angular distance between quaternions (radians)."""
    q1 = np.asarray(q1, dtype=float).reshape(4)
    q2 = np.asarray(q2, dtype=float).reshape(4)
    q1 = q1 / (np.linalg.norm(q1) + 1e-12)
    q2 = q2 / (np.linalg.norm(q2) + 1e-12)
    # handle double-cover
    if float(np.dot(q1, q2)) < 0.0:
        q2 = -q2
    w = float(np.clip(np.dot(q1, q2), -1.0, 1.0))
    return 2.0 * float(np.arccos(w))


def _quat_to_rot(q_xyzw: np.ndarray) -> np.ndarray:
    """Quaternion (xyzw) -> rotation matrix."""
    q = np.asarray(q_xyzw, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = (q / n).tolist()
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def _project_point_to_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, float, float, float]:
    """Project point p to segment [a,b].

    Returns (proj_clamped, s_clamped in [0,1], dist_to_proj, s_raw).

    - s_raw: unclamped projection coefficient along (b-a) in segment-parameter space.
    - s_clamped/proj_clamped: clamped to the segment, for downstream lookahead.

    Segment selection can optionally use |s_raw| when debuging segment ambiguity.
    """
    u = b - a
    uu = float(np.dot(u, u))
    if uu < 1e-12:
        proj = a.copy()
        return proj, 0.0, float(np.linalg.norm(p - proj)), 0.0

    ap = p - a
    s_raw = float(np.dot(ap, u) / uu)
    s_clamped = float(np.clip(s_raw, 0.0, 1.0))
    proj = a + s_clamped * u
    dist = float(np.linalg.norm(p - proj))
    return proj, s_clamped, dist, s_raw


class WaypointPath:
    """Bounded polyline path built from streaming teleop poses."""

    def __init__(
        self,
        max_points: int = 120,
        min_dist: float = 0.05,
        min_ang: float = np.deg2rad(5.0),
        reach_eps: float = 0.01,
        win_fwd: int = 6,
        win_back: int = 1,
        max_track_steps: int = 5,
    ):
        self._wpts: Deque[Waypoint] = deque()
        self.max_points = int(max_points)
        self.min_dist = float(min_dist)
        self.min_ang = float(min_ang)
        self.reach_eps = float(reach_eps)
        self.win_fwd = int(win_fwd)
        self.win_back = int(win_back)

        # active segment start index in the current deque (0 means front)
        self._active_i = 0

        # Anti-stuck: if a segment is tracked too many cycles, force-pop it.
        self.max_track_steps = int(max_track_steps)
        self._track_seg_i: int | None = None
        self._track_steps: int = 0

    def clear(self) -> None:
        self._wpts.clear()
        self._active_i = 0
        self._track_seg_i = None
        self._track_steps = 0

    def _reset_track_counter(self) -> None:
        self._track_seg_i = None
        self._track_steps = 0

    def _bump_track_counter(self, seg_i: int) -> None:
        """Update per-segment tracking counter for anti-stuck logic."""
        if self._track_seg_i is None or self._track_seg_i != int(seg_i):
            self._track_seg_i = int(seg_i)
            self._track_steps = 1
        else:
            self._track_steps += 1

    def _force_pop_if_stuck(self) -> bool:
        """Force-pop the current front segment if it has been tracked too long.

        Returns True if a pop happened.
        """
        if self.max_track_steps <= 0:
            return False
        if self._track_seg_i is None:
            return False
        if self._track_steps < self.max_track_steps:
            return False
        if len(self._wpts) < 2:
            return False

        # Do not pop if it would leave us with <2 points (would make PF return None forever
        # until new points arrive).
        if len(self._wpts) <= 2:
            self._reset_track_counter()
            return False

        # If the tracked segment is the current front segment, we can safely drop its start point.
        # If active_i > 0, we only pop points strictly before active segment elsewhere.
        if self._active_i == 0:
            self._wpts.popleft()
        else:
            # drop points up to the active start (defensive)
            while self._active_i > 0 and len(self._wpts) > 0:
                self._wpts.popleft()
                self._active_i -= 1

        self._active_i = max(0, self._active_i - 1)
        self._reset_track_counter()
        return True

    def _segment_index_window(self) -> Tuple[int, int]:
        """Return [i0, i1] segment index window for local projection search."""
        if len(self._wpts) < 2:
            return 0, -1

        # Defensive clamp: _active_i must always be a valid segment start index
        # (0..len-2). If it drifts out of range, the window can become empty and
        # select_segment_by_projection() will return None even though len(_wpts)>2.
        self._active_i = int(np.clip(self._active_i, 0, len(self._wpts) - 2))

        i0 = max(0, self._active_i - self.win_back)
        i1 = min(len(self._wpts) - 2, self._active_i + self.win_fwd)
        return i0, i1

    def __len__(self) -> int:
        return len(self._wpts)

    def push(self, p: np.ndarray, quat: np.ndarray, stamp: float = 0.0) -> None:
        p = np.asarray(p, dtype=float).reshape(3)
        quat = np.asarray(quat, dtype=float).reshape(4)

        if len(self._wpts) > 0:
            last = self._wpts[-1]
            if np.linalg.norm(p - last.p) < self.min_dist and _quat_angle(quat, last.quat) < self.min_ang:
                return

        self._wpts.append(Waypoint(p=p, quat=quat, stamp=float(stamp)))

        # bound memory/compute:
        # never drop the active segment start point while that segment is still being tracked.
        while len(self._wpts) > self.max_points and self._active_i > 0:
            self._wpts.popleft()
            self._active_i -= 1

        # If still over the limit but _active_i==0, keep the front point to avoid breaking the tracked segment.

    def _prune_front_reached(self, cur_p: np.ndarray) -> None:
        """Drop reached segments from the front.

        Pop must be driven by segment completion (reach/overrun of the *end* point b),
        not by being close to the start point a (which is often true at initialization).
        """
        cur_p = np.asarray(cur_p, dtype=float).reshape(3)

        # Only prune points that are strictly before the active segment start.
        while self._active_i > 0 and len(self._wpts) > 0:
            self._wpts.popleft()
            self._active_i -= 1

        # Now consider pruning the current front segment [0,1] only when reaching its end.
        while len(self._wpts) >= 2:
            a = self._wpts[0].p
            b = self._wpts[1].p
            u = b - a
            seg_len = float(np.linalg.norm(u))

            # Degenerate/duplicate segment: safe to drop a.
            if seg_len < 1e-6:
                self._wpts.popleft()
                self._active_i = max(0, self._active_i - 1)
                continue

            # progress along the segment (can be >1 if overrun beyond b)
            s_unclipped = float(np.dot(cur_p - a, u) / max(1e-12, seg_len * seg_len))
            proj = a + float(np.clip(s_unclipped, 0.0, 1.0)) * u
            dist = float(np.linalg.norm(cur_p - proj))

            if (dist < self.reach_eps and s_unclipped > 0.95) or (float(np.linalg.norm(cur_p - b)) < self.reach_eps):
                self._wpts.popleft()
                self._active_i = max(0, self._active_i - 1)
                self._reset_track_counter()
                continue

            break

    def select_segment_by_projection(self, cur_p: np.ndarray) -> Optional[Tuple[int, np.ndarray, float]]:
        """Return (i, proj, s) for the best segment [i,i+1] in a local window.

        Selection criterion (as requested):
        1) minimize distance to the clamped projection point
        2) tie-break: minimize |s_raw|

        Note: s_raw < 0 / > 1 is allowed; we only use it for tie-breaking.
        """
        cur_p = np.asarray(cur_p, dtype=float).reshape(3)
        if len(self._wpts) < 2:
            return None

        # Local window search first
        i0, i1 = self._segment_index_window()

        # If window is empty for any reason, fall back to a full search to avoid
        # returning None while we still have a valid polyline.
        if i1 < i0:
            i0, i1 = 0, len(self._wpts) - 2

        best_d = float("inf")
        best_key = float("inf")
        best_i = i0
        best_proj = None
        best_s = 0.0

        for i in range(i0, i1 + 1):
            a = self._wpts[i].p
            b = self._wpts[i + 1].p
            proj, s_clamped, d, s_raw = _project_point_to_segment(cur_p, a, b)
            key = float(abs(s_raw))

            if d < best_d - 1e-12 or (abs(d - best_d) <= 1e-12 and key < best_key):
                best_d = d
                best_key = key
                best_i = i
                best_proj = proj
                best_s = s_clamped

        if best_proj is None:
            return None

        self._active_i = int(best_i)
        self._bump_track_counter(best_i)

        # If stuck for too long, force-pop and re-select once
        if self._force_pop_if_stuck():
            if len(self._wpts) < 2:
                return None
            return self.select_segment_by_projection(cur_p)

        return best_i, best_proj, best_s

    def get_target_pose_for_segment_end(self, i: int) -> Tuple[np.ndarray, np.ndarray]:
        """Return (target_p, target_quat) as the end waypoint of segment [i,i+1]."""
        if len(self._wpts) < 2:
            raise IndexError("not enough waypoints")
        i = int(np.clip(i, 0, len(self._wpts) - 2))
        w = self._wpts[i + 1]
        return w.p.copy(), w.quat.copy()

    def get_lookahead_target_on_segment(
        self,
        *,
        seg_i: int,
        proj_s: float,
        cur_p: np.ndarray,
        base_step: float = 0.001,
        min_step: float = 0.001,
        max_step: float = 0.05,
        eps_step: float = 1e-4,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Return a point on the active segment slightly ahead of the projection.

        Returns (p_des, quat_des, s_des).

        Note:
        - If dist_to_end becomes extremely small, the previous logic could yield step==0,
          producing Sd≈0 and causing the controller to appear "stuck" before pruning triggers.
          We enforce a tiny forward step (eps_step) when still not exactly at the end.
        """
        cur_p = np.asarray(cur_p, dtype=float).reshape(3)
        seg_i = int(np.clip(seg_i, 0, len(self._wpts) - 2))
        a = self._wpts[seg_i]
        b = self._wpts[seg_i + 1]

        u = b.p - a.p
        L = float(np.linalg.norm(u))
        if L < 1e-9:
            return b.p.copy(), b.quat.copy(), 1.0

        # projection point on the segment
        proj_s = float(np.clip(proj_s, 0.0, 1.0))
        proj_p = a.p + proj_s * u

        # lookahead is "how much should move forward along the segment", so adapt to remaining distance
        dist_to_end = float(np.linalg.norm(b.p - proj_p))
        step = float(np.clip(max(base_step, 0.5 * dist_to_end), min_step, max_step))
        # step = float(np.clip(0.5 * dist_to_end, min_step, max_step))
        step = float(min(step, dist_to_end))

        # If we are not exactly at the end but step collapses to 0, force a tiny forward progress.
        if dist_to_end > 0.0 and step <= 0.0:
            step = float(min(max(eps_step, 0.0), dist_to_end, max_step))

        ds = step / L
        s_des = float(np.clip(proj_s + ds, 0.0, 1.0))
        p_des = a.p + s_des * u

        quat_des = b.quat.copy()
        return p_des, quat_des, s_des

    def current_target_pose(self, cur_p: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Compute projection on the path and choose target_pose as the end of the segment B.

        Returns (proj_p, target_p, target_quat).
        """
        self._prune_front_reached(cur_p)
        sel = self.select_segment_by_projection(cur_p)
        if sel is None:
            return None
        i, proj, _s = sel
        target_p, target_q = self.get_target_pose_for_segment_end(i)
        return proj, target_p, target_q


class TeleopAcuMPC:
    """给遥操用的最小 MPC 接口封装。

    设计点：
    - 遥操循环只需要给当前关节角(可选) + 目标点/参数，即可获得下一步关节角。
    - 内部缓存 qk_1/uk_1 已由 AcuMPCController 处理；这里再缓存 plan/state。
    - 默认采取“rough(直角折线) -> accurate(球面旋转)”两段式，但也允许调用方自己提供 Sd。
    """

    def __init__(self, cfg: TeleopMPCConfig) -> None:
        self.cfg = cfg
        self.ctl = AcuMPCController(
            urdf_path=str(cfg.urdf_path),
            dt=cfg.dt,
            horizon_T=cfg.horizon_T,
            ee_frame=cfg.ee_frame,
            solver_backend=cfg.solver_backend,
        )
        self.state: TeleopMPCState | None = None
        self._path = WaypointPath()
        self._path_lock = threading.Lock()
        # last debug snapshot (optional)
        self._dbg_last_pf: dict | None = None

    def reset_with_rough_goal(self, *, q0: np.ndarray, goal_xyz: np.ndarray) -> TeleopMPCState:
        q0 = np.asarray(q0, dtype=float).reshape(-1)
        qk = q0[:8].copy()

        cur = self.ctl.robot.fk(qk)
        P0 = np.asarray(cur.translation).reshape(3).copy()
        O = np.asarray(goal_xyz, dtype=float).reshape(3)

        rough_plan = init_rough_plan(P0=P0, O=O)
        target_pose = pin.SE3(np.asarray(cur.rotation).copy(), O.copy())

        self.ctl.reset()
        self.state = TeleopMPCState(qk=qk, mode=MPCMode.ROUGH, rough_plan=rough_plan, target_pose=target_pose)
        return self.state

    def switch_to_accurate(self, *, v: np.ndarray, r: float = 0.05, target_pose: "pin.SE3" | None = None) -> None:
        if self.state is None:
            raise RuntimeError("call reset_with_rough_goal() first")
        cur = self.ctl.robot.fk_frame(self.state.qk, frame=self.ctl.robot.needle_head_frame)
        plan = init_accurate_plan(curpos=cur.homogeneous, v=np.asarray(v).reshape(3), r=float(r))
        self.ctl.reset()
        self.state.mode = MPCMode.ACCURATE
        self.state.accurate_plan = plan
        self.state.acc_step_ratio = 0.0
        # NOTE: 精定位时 target_pose 的平移目标应当理解为“针头(needle_head)”目标点；
        # 具体由 controller 内部在 ACCURATE 模式下切换 IK/Jacobian frame 来保证。
        self.state.target_pose = target_pose

    def step(self, *, q_meas: np.ndarray | None = None, Sd: np.ndarray | None = None) -> np.ndarray:
        """执行一次 MPC。

        - q_meas: 可选。若提供，则覆盖内部 qk（更贴近真实电机状态）。
        - Sd: 可选。若提供，则直接使用；否则根据当前模式与 plan 自动计算。

        返回：q_next shape (8,)

        说明：
        - ROUGH：围绕针尖(Acu_ee)规划/控制
        - ACCURATE：Sd/target_pose 的目标点按照“针头(needle_head)”理解（controller 内部处理 frame 切换）
        """

        if self.state is None:
            raise RuntimeError("TeleopAcuMPC not initialized. Call reset_with_rough_goal() first.")

        if q_meas is not None:
            q_meas = np.asarray(q_meas, dtype=float).reshape(-1)
            if q_meas.size >= 8:
                self.state.qk = q_meas[:8].copy()

        qk = self.state.qk

        if Sd is None:
            if self.state.mode == MPCMode.ROUGH:
                if self.state.rough_plan is None:
                    raise RuntimeError("rough_plan is None")
                cur = self.ctl.robot.fk(qk)
                P = np.asarray(cur.translation).reshape(3).copy()
                Sd, _, _ = compute_sd_rough(self.state.rough_plan, P=P)
                # rough 完成判据：planner flag==1 且已经足够接近拐点/目标
                if self.state.rough_plan.flag == 1:
                    # 仍返回一步 MPC 结果，由调用侧决定是否切换
                    pass
            else:
                if self.state.accurate_plan is None:
                    raise RuntimeError("accurate_plan is None")

                # ACCURATE: 使用 needle_head 的当前位姿
                cur_head = self.ctl.robot.fk_frame(qk, frame=self.ctl.robot.needle_head_frame).copy()

                # 固定步数推进（0~1）
                self.state.acc_step_ratio = float(min(1.0, self.state.acc_step_ratio + 1.0 / max(1.0, self.ctl.num_ctrl)))
                Sd = compute_sd_accurate(self.state.accurate_plan, curpos=cur_head.homogeneous, step_ratio=self.state.acc_step_ratio)

                # 目标位姿：优先外部显式指定；否则使用 planner 初始化得到的 obj
                if self.state.target_pose is None:
                    obj = np.asarray(self.state.accurate_plan.obj, dtype=float).reshape(4, 4)
                    self.state.target_pose = pin.SE3(obj[0:3, 0:3].copy(), obj[0:3, 3].copy())

        Sd = np.asarray(Sd, dtype=float).reshape(3)

        res = self.ctl.step(qk=qk, Sd=Sd, mode=self.state.mode, target_pose=self.state.target_pose)
        self.state.qk = res.q_next.reshape(-1)
        return self.state.qk

    def ingest_teleop_pose(self, teleop_pose, stamp: float) -> None:
        """Collect incoming teleop pose into waypoint path.

        Expected teleop_pose to provide position and quaternion (xyzw).
        Adapt the field access below to your actual message/type.
        """
        # --- Adapt these getters to your actual teleop_pose object ---
        if isinstance(teleop_pose, dict):
            p = np.array(teleop_pose["position"], dtype=float)
            quat = np.array(teleop_pose["quat"], dtype=float)
        else:
            # common ROS geometry_msgs/Pose
            p = np.array([teleop_pose.position.x, teleop_pose.position.y, teleop_pose.position.z], dtype=float)
            quat = np.array([teleop_pose.orientation.x, teleop_pose.orientation.y,
                             teleop_pose.orientation.z, teleop_pose.orientation.w], dtype=float)
        # ------------------------------------------------------------

        with self._path_lock:
            self._path.push(p, quat, stamp)

    def step_with_path_following(self, cur_pose, stamp: float, qk: np.ndarray | None = None):
        """Compute target_pose via projection on waypoint polyline, then call mpc.step.

        Sd 采用投影 + 自适应前瞻：
        - 在局部窗口内找最佳线段投影
        - 沿线段向前推进一小步得到 p_des
        - Sd = p_des - cur_p

        参数：
        - qk: 可选，实测关节角(>=8)。若提供则覆盖内部状态并用于本次 self.ctl.step；
              若不提供则默认使用内部缓存的 q_next (self.state.qk)。

        Returns next q (8,) or None if insufficient waypoints.
        """
        if self.state is None:
            raise RuntimeError("TeleopAcuMPC not initialized. Call reset_with_rough_goal() first.")

        # If caller provides measured joint angles, use them for this step.
        if qk is not None:
            qk = np.asarray(qk, dtype=float).reshape(-1)
            if qk.size < 8:
                raise ValueError(f"qk must have at least 8 elements, got {qk.size}")
            self.state.qk = qk[:8].copy()

        # --- Adapt these getters to your actual cur_pose object ---
        if isinstance(cur_pose, dict):
            cur_p = np.array(cur_pose["position"], dtype=float)
        else:
            cur_p = np.array([cur_pose.position.x, cur_pose.position.y, cur_pose.position.z], dtype=float)
        # ----------------------------------------------------------

        # pop only when segment end is reached
        with self._path_lock:
            self._path._prune_front_reached(cur_p)

            ref = self._path.select_segment_by_projection(cur_p)
            if ref is None:
                # print("here 555")
                # print(len(self._path._wpts))
                return None

            seg_i, proj, s = ref

            p_des, _quat_des, s_des = self._path.get_lookahead_target_on_segment(seg_i=seg_i, proj_s=s, cur_p=cur_p)
            target_p, target_q = self._path.get_target_pose_for_segment_end(seg_i)

        # print(len(self._path._wpts))
        # include orientation in target_pose
        R = _quat_to_rot(target_q)
        target_pose = pin.SE3(np.asarray(R).copy(), np.asarray(target_p).reshape(3).copy())

        Sd = (p_des - cur_p).reshape(3)
        # print(Sd)
        # print(np.linalg.norm(Sd))

        # store debug info for inspection
        try:
            self._dbg_last_pf = {
                "stamp": float(stamp),
                "seg_i": int(seg_i),
                "proj_s": float(s),
                "s_des": float(s_des),
                "cur_p": np.asarray(cur_p).reshape(3).copy(),
                "proj_p": np.asarray(proj).reshape(3).copy(),
                "p_des": np.asarray(p_des).reshape(3).copy(),
                "target_p": np.asarray(target_p).reshape(3).copy(),
                "target_q": np.asarray(target_q).reshape(4).copy(),
                "Sd_norm": float(np.linalg.norm(Sd)),
                "wpts_len": int(len(self._path._wpts)),
            }
        except Exception:
            self._dbg_last_pf = None

        res = self.ctl.step(qk=self.state.qk, Sd=Sd, mode=self.state.mode, target_pose=target_pose)
        self.state.qk = res.q_next.reshape(-1)
        return self.state.qk

    def close(self) -> None:
        self.ctl.close()
