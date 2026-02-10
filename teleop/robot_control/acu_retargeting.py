import time
import numpy as np
from multiprocessing import Process, Array, Lock
from typing import Optional, Any

import logging_mp
logger_mp = logging_mp.get_logger(__name__)


# OpenXR hand joint indices (0-based)
XR_HAND_JOINT_WRIST = 0
XR_HAND_JOINT_THUMB_TIP = 4
XR_HAND_JOINT_INDEX_PROXIMAL = 6
XR_HAND_JOINT_INDEX_TIP = 9


def _safe_norm(v: np.ndarray, eps: float = 1e-9) -> float:
    return float(np.linalg.norm(v) + eps)


def _angle_to_horizontal_rad(v: np.ndarray, eps: float = 1e-9) -> float:
    """Return angle between vector v and the horizontal plane (xy-plane).

    - 0 rad: vector lies in horizontal plane
    - pi/2: vertical

    Using z component vs horizontal magnitude.
    """
    vz = float(v[2])
    vxy = float(np.linalg.norm(v[:2]))
    return float(np.arctan2(abs(vz), max(vxy, eps)))


class AcuNeedleRetargeting:
    """Human hand -> acupuncture needle motion retargeting.

    Inputs:
      - right_hand_pos_array_in: multiprocessing.Array('d', 75) holding (25,3) joint positions.
        Coordinate convention (as given): x,y horizontal; z up.

    Outputs:
      - needle_state_array_out: Array('d', 2) [insert_depth, twist_angle]
      - needle_action_array_out: Array('d', 2) [insert_depth_cmd, twist_angle_cmd]
      - (optional) needle_meta_array_out: Array('d', 3) [timestamp_s, insert_vel, twist_vel]

    DOFs:
      - insert (提插): based on index finger segment length change (proximal->tip).
      - twist (捻转): based on thumb-index relative motion in horizontal plane mapped by needle circumference.

    Notes:
      - Initial positions are taken as 0.
      - This module only computes a reference; downstream mapping to robot actuator is out of scope.

    Modes:
      - mode='absolute': relative-to-baseline mapping (original behavior, kept for compatibility)
      - mode='incremental': integrate per-frame increments (recommended when input rate < control fps)
    """

    def __init__(
        self,
        right_hand_pos_array_in: Any,
        data_lock: Optional[Any] = None,
        needle_state_array_out: Optional[Any] = None,
        needle_action_array_out: Optional[Any] = None,
        needle_meta_array_out: Optional[Any] = None,
        fps: float = 60.0,
        needle_circumference_m: float = 2.0e-3,
        insert_angle_threshold_deg: float = 35.0,
        twist_angle_threshold_deg: float = 20.0,
        insert_gain: float = 1.0,
        twist_gain: float = 1.0,
        max_insert_m: float = 0.05,
        max_twist_rad: float = np.deg2rad(360.0),
        mode: str = 'absolute',
        lp_tau_s: float = 0.15,
        input_change_eps: float = 1e-6,
        insert_positive_down: bool = True,
    ):
        self.fps = float(fps)
        self.needle_circumference_m = float(max(needle_circumference_m, 1e-9))
        self.insert_angle_threshold_rad = float(np.deg2rad(insert_angle_threshold_deg))
        self.twist_angle_threshold_rad = float(np.deg2rad(twist_angle_threshold_deg))
        self.insert_gain = float(insert_gain)
        self.twist_gain = float(twist_gain)
        self.max_insert_m = float(max_insert_m)
        self.max_twist_rad = float(max_twist_rad)

        self.mode = str(mode)
        self.lp_tau_s = float(max(lp_tau_s, 1e-6))
        self.input_change_eps = float(max(input_change_eps, 0.0))

        self.data_lock = data_lock if data_lock is not None else Lock()

        # outputs: 2 dof
        self.needle_state_array_out = needle_state_array_out
        self.needle_action_array_out = needle_action_array_out
        self.needle_meta_array_out = needle_meta_array_out

        self.insert_positive_down = bool(insert_positive_down)

        self._proc = Process(
            target=self.control_process,
            args=(right_hand_pos_array_in, self.data_lock, self.needle_state_array_out, self.needle_action_array_out, self.needle_meta_array_out),
        )
        self._proc.daemon = True
        self._proc.start()

        logger_mp.info("Initialize AcuNeedleRetargeting OK!")

    def control_process(
        self,
        right_hand_pos_array_in: Any,
        data_lock: Any,
        needle_state_array_out: Optional[Any],
        needle_action_array_out: Optional[Any],
        needle_meta_array_out: Optional[Any],
    ):
        running = True

        insert_depth = 0.0
        twist_angle = 0.0
        insert_vel = 0.0
        twist_vel = 0.0

        # Baselines / previous-frame cache
        base_index_v = None
        base_rel_xy = None
        prev_index_v = None
        prev_rel_xy = None
        prev_hand_key = None
        prev_ts = None

        # Low-pass states (for incremental step smoothing)
        d_insert_lp = 0.0
        d_twist_lp = 0.0

        last_insert_depth = None
        last_twist_angle = None

        while running:
            t0 = time.time()

            with right_hand_pos_array_in.get_lock():
                hand = np.array(right_hand_pos_array_in[:], dtype=float).reshape(25, 3).copy()

            if np.allclose(hand, 0.0):
                time.sleep(0.01)
                continue

            # Only process when input changes (otherwise we would re-consume the same frame many times if input rate < fps)
            # Use a cheap key based on a few stable points.
            key_pts = np.concatenate([hand[XR_HAND_JOINT_INDEX_PROXIMAL], hand[XR_HAND_JOINT_INDEX_TIP], hand[XR_HAND_JOINT_THUMB_TIP]])
            hand_key = key_pts.round(6).tobytes()
            if prev_hand_key is not None and hand_key == prev_hand_key:
                # no new input frame
                dt_sleep = max(0.0, (1.0 / max(self.fps, 1e-6)) - (time.time() - t0))
                time.sleep(dt_sleep)
                continue
            prev_hand_key = hand_key

            now = time.time()
            if prev_ts is None:
                dt = 1.0 / max(self.fps, 1e-6)
            else:
                dt = max(1e-4, now - prev_ts)
            prev_ts = now

            # dt-adaptive 1st order LPF coefficient
            alpha = 1.0 - float(np.exp(-dt / self.lp_tau_s))

            idx_prox = hand[XR_HAND_JOINT_INDEX_PROXIMAL]
            idx_tip = hand[XR_HAND_JOINT_INDEX_TIP]
            th_tip = hand[XR_HAND_JOINT_THUMB_TIP]

            # ---- vectors ----
            v_index = idx_tip - idx_prox
            v_rel = th_tip - idx_tip
            rel_xy = v_rel[:2].copy()

            if base_index_v is None:
                base_index_v = v_index.copy()
            if base_rel_xy is None:
                base_rel_xy = rel_xy.copy()

            if prev_index_v is None:
                prev_index_v = v_index.copy()
            if prev_rel_xy is None:
                prev_rel_xy = rel_xy.copy()

            # =========================
            # INSERT (提插)
            # =========================
            if self.mode == 'incremental':
                dv_index = v_index - prev_index_v
            else:
                dv_index = v_index - base_index_v

            # Gate based on motion vector w.r.t. horizontal plane
            ang_index_motion = _angle_to_horizontal_rad(dv_index)

            # Mapping: use motion vector norm ("index motion vector norm tracking").
            # To preserve direction (sign), use the z sign of dv_index:
            #   - z up is positive in TeleVuerWrapper output
            #   - define "insert down" as positive when insert_positive_down=True
            dv_norm = float(np.linalg.norm(dv_index))
            if dv_norm < 1e-12:
                d_insert_raw = 0.0
            else:
                z_sign = 1.0 if self.insert_positive_down else -1.0
                # If dv_index points downward (dv_index.z < 0), insertion should be positive (after z_sign).
                d_insert_raw = (z_sign * float(np.sign(dv_index[2])) * dv_norm) * self.insert_gain

            if ang_index_motion < self.insert_angle_threshold_rad:
                d_insert_raw = 0.0

            d_insert_lp = (1.0 - alpha) * d_insert_lp + alpha * d_insert_raw

            if self.mode == 'incremental':
                insert_depth = float(np.clip(insert_depth + d_insert_lp, -self.max_insert_m, self.max_insert_m))
            else:
                insert_depth = float(np.clip(d_insert_lp, -self.max_insert_m, self.max_insert_m))

            # =========================
            # TWIST (捻转)
            # =========================
            if self.mode == 'incremental':
                d_rel_xy = rel_xy - prev_rel_xy
            else:
                d_rel_xy = rel_xy - base_rel_xy

            ang_rel_motion = _angle_to_horizontal_rad(np.array([d_rel_xy[0], d_rel_xy[1], 0.0], dtype=float))

            # signed direction in xy-plane
            def signed_angle_2d(a: np.ndarray, b: np.ndarray, eps: float = 1e-9) -> float:
                a = np.asarray(a, dtype=float).reshape(2)
                b = np.asarray(b, dtype=float).reshape(2)
                na = float(np.linalg.norm(a))
                nb = float(np.linalg.norm(b))
                if na < eps or nb < eps:
                    return 0.0
                a_u = a / na
                b_u = b / nb
                cross = float(a_u[0] * b_u[1] - a_u[1] * b_u[0])
                dot = float(np.clip(a_u[0] * b_u[0] + a_u[1] * b_u[1], -1.0, 1.0))
                return float(np.arctan2(cross, dot))

            # For incremental, sign from prev->cur; for absolute, sign from base->cur.
            ref_a = prev_rel_xy if self.mode == 'incremental' else base_rel_xy
            signed_ang = signed_angle_2d(ref_a, rel_xy)

            s = float(np.linalg.norm(d_rel_xy))
            d_theta = (2.0 * np.pi) * (s / self.needle_circumference_m)
            d_theta *= float(np.sign(signed_ang))
            d_theta *= self.twist_gain

            # Gate: only when relative motion is sufficiently horizontal (angle small)
            if ang_rel_motion > self.twist_angle_threshold_rad:
                d_theta = 0.0

            d_twist_lp = (1.0 - alpha) * d_twist_lp + alpha * d_theta

            if self.mode == 'incremental':
                twist_angle = float(np.clip(twist_angle + d_twist_lp, -self.max_twist_rad, self.max_twist_rad))
            else:
                twist_angle = float(np.clip(d_twist_lp, -self.max_twist_rad, self.max_twist_rad))

            # -------------------------
            # Velocity estimates (for upstream sync / xdot_ref)
            # -------------------------
            if last_insert_depth is None:
                insert_vel = 0.0
            else:
                insert_vel = float((insert_depth - last_insert_depth) / max(dt, 1e-6))
            if last_twist_angle is None:
                twist_vel = 0.0
            else:
                twist_vel = float((twist_angle - last_twist_angle) / max(dt, 1e-6))
            last_insert_depth = float(insert_depth)
            last_twist_angle = float(twist_angle)

            # update prev caches
            prev_index_v = v_index.copy()
            prev_rel_xy = rel_xy.copy()

            # publish
            if needle_state_array_out is not None and needle_action_array_out is not None:
                with data_lock:
                    needle_state_array_out[:] = [insert_depth, twist_angle]
                    needle_action_array_out[:] = [insert_depth, twist_angle]
                    if needle_meta_array_out is not None:
                        # timestamp uses the same 'now' used for dt
                        needle_meta_array_out[:] = [float(now), float(insert_vel), float(twist_vel)]

            dt_sleep = max(0.0, (1.0 / max(self.fps, 1e-6)) - (time.time() - t0))
            time.sleep(dt_sleep)
