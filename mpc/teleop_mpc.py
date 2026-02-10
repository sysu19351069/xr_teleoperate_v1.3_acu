from __future__ import annotations
import numpy as np
from collections import deque
from typing import Deque, Tuple, List, Optional
import pinocchio as pin

class Waypoint:
    def __init__(self, p: np.ndarray, quat: np.ndarray, stamp: float):
        self.p = p
        self.quat = quat
        self.stamp = stamp

def _quat_angle(quat1: np.ndarray, quat2: np.ndarray) -> float:
    """Compute the angle between two quaternions."""
    dot_product = np.dot(quat1, quat2)
    return 2 * np.arccos(np.clip(dot_product, -1.0, 1.0))

def _project_point_to_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, float, float]:
    """Project a point onto a segment defined by points a and b."""
    ab = b - a
    ab_len = np.linalg.norm(ab)
    if ab_len < 1e-9:
        return a, 0.0, np.linalg.norm(point - a)
    ab_unit = ab / ab_len
    ap = point - a
    s = np.dot(ap, ab_unit) / ab_len
    s_clamped = np.clip(s, 0.0, 1.0)
    proj = a + s_clamped * ab
    dist = np.linalg.norm(point - proj)
    return proj, s_clamped, dist

class WaypointPath:
    """Bounded polyline path built from streaming teleop poses."""

    def __init__(
        self,
        max_points: int = 120,
        min_dist: float = 0.003,
        min_ang: float = np.deg2rad(1.0),
        reach_eps: float = 0.01,
        win_fwd: int = 12,
        win_back: int = 1,
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

    def _refresh_windows(self) -> None:
        """Refresh front/back windows around the active segment."""
        if len(self._wpts) == 0:
            self.win_front = []
            self.win_back_pts = []
            return
        i0 = max(0, self._active_i - self.win_back)
        i1 = min(len(self._wpts), self._active_i + 1 + self.win_fwd)
        self.win_back_pts = list(self._wpts)[i0 : self._active_i + 1]
        self.win_front = list(self._wpts)[self._active_i : i1]

    def clear(self) -> None:
        self._wpts.clear()
        self._active_i = 0
        self._refresh_windows()

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
        # IMPORTANT: never drop the active segment start point while that segment is still being tracked.
        # Only drop points strictly before _active_i (already-finished segments).
        while len(self._wpts) > self.max_points and self._active_i > 0:
            self._wpts.popleft()
            self._active_i -= 1

        # If we are still over the limit but _active_i==0, we keep the front point to avoid breaking the
        # currently-tracked segment. (Better to temporarily exceed max_points than to introduce jump errors.)

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

            proj, s, dist = _project_point_to_segment(cur_p, a, b)

            # Reached the segment end (near b) or essentially at the next waypoint.
            # Note: do NOT abs() dot-products; s<0 means we are behind a.
            if (dist < self.reach_eps and s > 0.95) or (np.linalg.norm(cur_p - b) < self.reach_eps):
                self._wpts.popleft()
                self._active_i = max(0, self._active_i - 1)
                continue

            break

    def _segment_index_window(self) -> Tuple[int, int]:
        """Return [i0, i1] segment index window for local projection search."""
        if len(self._wpts) < 2:
            return 0, -1
        i0 = max(0, self._active_i - self.win_back)
        i1 = min(len(self._wpts) - 2, self._active_i + self.win_fwd)
        return i0, i1

    def get_target_pose_for_segment_end(self, seg_i: int) -> Tuple[np.ndarray, np.ndarray]:
        """Return (target_p, target_quat) for segment end B = wpts[seg_i+1]."""
        if len(self._wpts) < 2:
            raise IndexError("need at least 2 waypoints")
        seg_i = int(np.clip(seg_i, 0, len(self._wpts) - 2))
        target = self._wpts[seg_i + 1]
        return target.p.copy(), target.quat.copy()

    def select_segment_by_projection(self, cur_p: np.ndarray) -> Optional[Tuple[int, np.ndarray, float]]:
        """Return (i, proj, s) for the best segment [i,i+1] in a local window."""
        cur_p = np.asarray(cur_p, dtype=float).reshape(3)
        if len(self._wpts) < 2:
            return None

        i0, i1 = self._segment_index_window()
        if i1 < i0:
            return None

        best_d = float("inf")
        best_i = i0
        best_proj = None
        best_s = 0.0

        for i in range(i0, i1 + 1):
            a = self._wpts[i].p
            b = self._wpts[i + 1].p
            proj, s, d = _project_point_to_segment(cur_p, a, b)
            if d < best_d:
                best_d = d
                best_i = i
                best_proj = proj
                best_s = s

        self._active_i = best_i
        return best_i, best_proj, best_s

    def get_lookahead_target_on_segment(
        self,
        *,
        seg_i: int,
        proj_s: float,
        cur_p: np.ndarray,
        base_step: float = 0.02,
        min_step: float = 0.005,
        max_step: float = 0.05,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Return a point on the active segment slightly ahead of the projection.

        This avoids large errors under high-frequency waypoint streaming.
        Returns (p_des, quat_des, s_des).
        """
        cur_p = np.asarray(cur_p, dtype=float).reshape(3)
        seg_i = int(np.clip(seg_i, 0, len(self._wpts) - 2))
        a = self._wpts[seg_i]
        b = self._wpts[seg_i + 1]

        u = b.p - a.p
        L = float(np.linalg.norm(u))
        if L < 1e-9:
            return b.p.copy(), b.quat.copy(), 1.0

        # adaptive lookahead: proportional to distance to the path, but clamped
        proj_p = a.p + float(np.clip(proj_s, 0.0, 1.0)) * u
        dist_to_path = float(np.linalg.norm(cur_p - proj_p))
        step = float(np.clip(base_step + 0.5 * dist_to_path, min_step, max_step))
        ds = step / L
        s_des = float(np.clip(proj_s + ds, 0.0, 1.0))
        p_des = a.p + s_des * u

        # quaternion: keep segment end quaternion (simple + stable). If you need SLERP, add here.
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
        target = self._wpts[i + 1]
        return proj, target.p.copy(), target.quat.copy()

class TeleopAcuMPC:
    def __init__(self, ctl, state, path: WaypointPath):
        self.ctl = ctl
        self.state = state
        self._path = path

    def step_with_path_following(self, cur_pose, stamp: float):
        """Compute target_pose via projection on waypoint polyline, then call mpc.step.

        Sd is computed like a single-segment rough tracker:
        - project current point onto the best segment
        - advance a small (adaptive) step along the segment (clamped)
        - Sd = p_des - cur_p

        Returns q_next (8,) or None if no path.
        """
        if self.state is None:
            raise RuntimeError("TeleopAcuMPC not initialized")

        # --- Adapt these getters to your actual cur_pose object ---
        if isinstance(cur_pose, dict):
            cur_p = np.array(cur_pose["position"], dtype=float)
        else:
            cur_p = np.array([cur_pose.position.x, cur_pose.position.y, cur_pose.position.z], dtype=float)
        # ----------------------------------------------------------

        # Pop only when segment end is reached.
        self._path._prune_front_reached(cur_p)

        ref = self._path.select_segment_by_projection(cur_p)
        if ref is None:
            return None

        seg_i, proj, s = ref

        # segment endpoints
        a = self._path._wpts[seg_i].p
        b = self._path._wpts[seg_i + 1].p
        u = b - a
        seg_len = float(np.linalg.norm(u))
        if seg_len < 1e-6:
            return None

        # adaptive step: bigger when far, smaller when close
        # (values can be tuned; kept conservative)
        base_step = 0.01
        max_step = 0.05
        dist_to_b = float(np.linalg.norm(b - cur_p))
        step = float(np.clip(0.3 * dist_to_b, base_step, max_step))

        # move desired point forward along the segment from the projection
        s_des = float(np.clip(s + step / seg_len, 0.0, 1.0))
        p_des = a + s_des * u

        target_p, target_q = self._path.get_target_pose_for_segment_end(seg_i)
        target_pose = pin.SE3(np.eye(3), target_p)

        Sd = (p_des - cur_p).reshape(3)

        res = self.ctl.step(qk=self.state.qk, Sd=Sd, mode=self.state.mode, target_pose=target_pose)
        self.state.qk = res.q_next.reshape(-1)
        return self.state.qk