import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, Callable

import numpy as np
import os

import logging_mp

logger_mp = logging_mp.get_logger(__name__)


def _save_summary_plots(out_path: str, t_s, x_ref, x_m, p_m, f_safe: float):
    """Save position/force comparison plots to out_path (PNG)."""
    import matplotlib  # type: ignore
    matplotlib.use('Agg', force=True)
    import matplotlib.pyplot as plt  # type: ignore

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    ax1.set_title('Position: reference vs measured')
    ax1.plot(t_s, x_ref, label='x_ref')
    ax1.plot(t_s, x_m, label='x_m')
    ax1.set_ylabel('x')
    ax1.grid(True)
    ax1.legend(loc='upper right')

    ax2.set_title('Force: measured vs f_safe')
    ax2.plot(t_s, p_m, label='p_m')
    ax2.plot(t_s, [float(f_safe)] * len(t_s), label='f_safe')
    ax2.set_xlabel('t (s)')
    ax2.set_ylabel('F (N)')
    ax2.grid(True)
    ax2.legend(loc='upper right')

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


class IIR2Filter:
    """Second-order IIR filter (Direct Form I).

    y[k] = b0*x[k] + b1*x[k-1] + b2*x[k-2] - a1*y[k-1] - a2*y[k-2]

    Notes:
      - a0 assumed 1.
      - This is a generic utility used to implement the impedance filter (35) after discretization.
    """

    def __init__(self, b0: float, b1: float, b2: float, a1: float, a2: float):
        self.b0 = float(b0)
        self.b1 = float(b1)
        self.b2 = float(b2)
        self.a1 = float(a1)
        self.a2 = float(a2)

        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

    def reset(self, x: float = 0.0, y: float = 0.0):
        self.x1 = float(x)
        self.x2 = float(x)
        self.y1 = float(y)
        self.y2 = float(y)

    def process(self, x: float) -> float:
        x = float(x)
        y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2

        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        return float(y)


@dataclass
class AdaptiveEnvParams:
    """Adaptive estimates of spring-damper contact model (36)/(47)."""

    ke_hat: float = 100.0  # N/m
    be_hat: float = 0.5   # N*s/m
    xe_hat: float = 0.0   # m

    ke_min: float = 1e-3  # avoid division by 0
    ke_max: float = 5e4
    be_min: float = 0.0
    be_max: float = 5e3


class AdaptiveImpedanceReference:
    """Adaptive impedance reference generator (paper Eq. (43)-(47)).

    It estimates environment parameters (Ke, Be, xe), and generates a force-regulated
    reference xr used by an impedance filter / outer loop.

    This module is meant to be used when measured force exceeds a safe threshold.

    Key equations (from the provided figure/text):
      - Contact force model:    Fe = Ke*(xm - xe) + Be*xm_dot   (Eq. 36 / 47)
      - Adaptive reference:     xr = Fr/Ke_hat + xe_hat - xdot_d/kF   (Eq. 43)
      - Parameter adaptation:   Eq. (44)-(46) (continuous) -> discretized with dt

    Discretization here is forward Euler; practical stability is controlled by small gammas.
    """

    def __init__(
        self,
        kf: float,
        params: Optional[AdaptiveEnvParams] = None,
        gamma1: float = 1e-3,
        gamma2: float = 1e-3,
        gamma3: float = 1e-4,
        dt: float = 0.01,
    ):
        self.kf = float(max(kf, 1e-9))
        self.p = params if params is not None else AdaptiveEnvParams()

        self.gamma1 = float(gamma1)
        self.gamma2 = float(gamma2)
        self.gamma3 = float(gamma3)
        self.dt = float(max(dt, 1e-6))

        self._last_ts: Optional[float] = None

    def reset(self):
        self._last_ts = None

    def _step_dt(self, now: Optional[float]) -> float:
        if now is None:
            return self.dt
        if self._last_ts is None:
            self._last_ts = float(now)
            return self.dt
        dt = float(max(1e-6, float(now) - self._last_ts))
        self._last_ts = float(now)
        return dt

    def predict_force(self, xm: float, xm_dot: float) -> float:
        return float(self.p.ke_hat * (float(xm) - self.p.xe_hat) + self.p.be_hat * float(xm_dot))

    def update(self, fr: float, xm: float, xm_dot: float, xdot_d: float, pm: float, now: Optional[float] = None) -> Tuple[float, float]:
        """Update estimates and output xr.

        Args:
          fr: desired force Fr
          xm: measured insertion depth x_m
          xm_dot: measured insertion speed xdot_m
          xdot_d: desired insertion speed xdot_d (from technique tracking)
          pm: measured force (F_e)

        Returns:
          xr: adaptive reference position
          f_hat: predicted force
        """
        dt = self._step_dt(now)

        xm = float(xm)
        xm_dot = float(xm_dot)
        fr = float(fr)
        xdot_d = float(xdot_d)
        pm = float(pm)

        # force prediction (Eq. 47)
        f_hat = self.predict_force(xm, xm_dot)
        err = float(f_hat - pm)

        # reference (Eq. 43)
        ke_safe = float(np.clip(self.p.ke_hat, self.p.ke_min, self.p.ke_max))
        xr = float(fr / ke_safe + self.p.xe_hat - (xdot_d / self.kf))

        # parameter adaptation (Eq. 44-46) - Euler discretization
        # Ke_hat_dot = -gamma1 * x_m * (F_hat - F_e)
        self.p.ke_hat = float(self.p.ke_hat - self.gamma1 * xm * err * dt)
        self.p.ke_hat = float(np.clip(self.p.ke_hat, self.p.ke_min, self.p.ke_max))

        ke_safe = float(np.clip(self.p.ke_hat, self.p.ke_min, self.p.ke_max))

        # xe_hat_dot =  gamma1 * (F_hat - F_e)/Ke_hat * (x_m*xe_hat + gamma2/gamma1)
        # NOTE: matches the provided C++ snippet; in practice you may want a bounded term.
        self.p.xe_hat = float(self.p.xe_hat + self.gamma1 * (err / ke_safe) * (xm * self.p.xe_hat + (self.gamma2 / max(self.gamma1, 1e-12))) * dt)

        # Be_hat_dot = -gamma3 * xdot_m * (F_hat - F_e)
        self.p.be_hat = float(self.p.be_hat - self.gamma3 * xm_dot * err * dt)
        self.p.be_hat = float(np.clip(self.p.be_hat, self.p.be_min, self.p.be_max))

        return xr, f_hat


class SafeForceSwitch:
    """Force detector + mode switch.

    If measured force pm exceeds safety threshold, enable adaptive impedance mode.

    Hysteresis is supported to avoid chattering.
    """

    def __init__(
        self,
        f_safe: float,
        hysteresis: float = 0.5,
        latch: bool = False,
        adaptive_hysteresis: bool = True,
        hysteresis_min: float = 0.05,
        hysteresis_max: float = 1.5,
        # if |dF/dt| < slow_dfdt -> treat as slow, reduce hysteresis
        slow_dfdt: float = 0.5,  # N/s
        fast_dfdt: float = 5.0,  # N/s
        # optional min dwell time to prevent rapid toggling
        min_dwell_s: float = 0.05,
    ):
        self.f_safe = float(f_safe)
        self.hysteresis = float(max(hysteresis, 0.0))
        self.adaptive_hysteresis = bool(adaptive_hysteresis)
        self.hysteresis_min = float(max(0.0, hysteresis_min))
        self.hysteresis_max = float(max(self.hysteresis_min, hysteresis_max))
        self.slow_dfdt = float(max(1e-6, slow_dfdt))
        self.fast_dfdt = float(max(self.slow_dfdt, fast_dfdt))
        self.min_dwell_s = float(max(0.0, min_dwell_s))
        self.latch = bool(latch)

        self.enabled = False
        self._pm_prev: Optional[float] = None
        self._t_prev: Optional[float] = None
        self._hyst_cur: float = float(max(hysteresis, 0.0))
        self._last_toggle_t: Optional[float] = None

    def reset(self):
        self.enabled = False
        self._pm_prev = None
        self._t_prev = None
        self._hyst_cur = float(max(self.hysteresis, 0.0))
        self._last_toggle_t = None

    def update(self, pm: float, now: Optional[float] = None) -> bool:
        pm = float(pm)
        if now is None:
            now = time.time()

        # adapt hysteresis based on force change rate |dF/dt|
        if self.adaptive_hysteresis:
            if self._pm_prev is not None and self._t_prev is not None:
                dt = float(max(1e-6, float(now) - float(self._t_prev)))
                dfdt = abs((pm - float(self._pm_prev)) / dt)
                # map dfdt in [slow, fast] -> scale in [max, min]
                # slow change => larger hysteresis (faster exiting by pm < f_safe + hyst)
                if dfdt <= self.slow_dfdt:
                    target = self.hysteresis_max
                elif dfdt >= self.fast_dfdt:
                    target = self.hysteresis_min
                else:
                    u = (dfdt - self.slow_dfdt) / (self.fast_dfdt - self.slow_dfdt)
                    target = float(self.hysteresis_max - u * (self.hysteresis_max - self.hysteresis_min))
                # smooth a bit to avoid abrupt changes
                self._hyst_cur = float(0.9 * self._hyst_cur + 0.1 * target)
            else:
                self._hyst_cur = float(np.clip(self.hysteresis, self.hysteresis_min, self.hysteresis_max))

        self._pm_prev = pm
        self._t_prev = float(now)

        # dwell-time gate
        def can_toggle() -> bool:
            if self.min_dwell_s <= 0.0:
                return True
            if self._last_toggle_t is None:
                return True
            return (float(now) - float(self._last_toggle_t)) >= self.min_dwell_s

        if self.latch:
            if (not self.enabled) and (pm > self.f_safe):
                self.enabled = True
                self._last_toggle_t = float(now)
            return bool(self.enabled)

        # non-latched with (possibly adaptive) hysteresis
        hyst = float(self._hyst_cur if self.adaptive_hysteresis else self.hysteresis)
        if not self.enabled:
            if pm > self.f_safe and can_toggle():
                self.enabled = True
                self._last_toggle_t = float(now)
        else:
            # exit condition uses f_safe + hysteresis (as per your updated switching rule)
            if pm < (self.f_safe + hyst) and can_toggle():
                self.enabled = False
                self._last_toggle_t = float(now)
        return bool(self.enabled)


@dataclass
class VelocityOutputLimits:
    vmax: Optional[float] = None


@dataclass
class SimContactEnv:
    """Simple simulated contact force environment.

    Placeholder for pm modeling until real force sensing is wired.
    Fe = Ke*(xm - xe) + Be*xm_dot
    """

    ke: float = 1.4
    be: float = 0.1
    xe: float = 0.0

    # variable stiffness/damping mode
    #   - 'const': constant ke/be (default)
    #   - 'time_sine': ke/be vary sinusoidally with time
    #   - 'xm_linear': ke/be vary linearly with xm (clipped)
    mode: str = 'const'

    # noise settings
    #   - noise_mode='none': no noise
    #   - noise_mode='gaussian': additive Gaussian noise
    #   - noise_mode='relative': multiplicative noise, F = F*(1+N(0,rel_sigma))
    noise_mode: str = 'none'
    noise_sigma: float = 0.05      # N (for gaussian)
    noise_rel_sigma: float = 0.0  # unitless (for relative)
    noise_seed: Optional[int] = None
    noise_contact_only: bool = True

    # time-varying parameters
    ke_base: float = 1.4
    ke_amp: float = 0.0
    ke_freq_hz: float = 0.2
    be_base: float = 0.1
    be_amp: float = 0.0
    be_freq_hz: float = 0.2

    # xm-varying parameters (linear ramp)
    xm0: float = 0.0
    xm1: float = 6.0
    ke0: float = 1.4
    ke1: float = 2.8
    be0: float = 0.1
    be1: float = 0.2

    ke_min: float = 0.0
    ke_max: float = 1e6
    be_min: float = 0.0
    be_max: float = 1e6

    def _interp(self, a: float, b: float, u: float) -> float:
        u = float(np.clip(u, 0.0, 1.0))
        return float((1.0 - u) * float(a) + u * float(b))

    def get_params(self, xm: float, now: Optional[float] = None) -> Tuple[float, float]:
        """Return (ke, be) at current state."""
        m = str(self.mode).lower()
        xm = float(xm)
        if now is None:
            now = time.time()

        if m == 'const':
            ke = float(self.ke)
            be = float(self.be)
        elif m == 'time_sine':
            w_ke = 2.0 * np.pi * float(self.ke_freq_hz)
            w_be = 2.0 * np.pi * float(self.be_freq_hz)
            ke = float(self.ke_base + self.ke_amp * np.sin(w_ke * float(now)))
            be = float(self.be_base + self.be_amp * np.sin(w_be * float(now)))
        elif m == 'xm_linear':
            den = float(max(1e-9, self.xm1 - self.xm0))
            u = (xm - float(self.xm0)) / den
            ke = self._interp(self.ke0, self.ke1, u)
            be = self._interp(self.be0, self.be1, u)
        else:
            # fallback
            ke = float(self.ke)
            be = float(self.be)

        ke = float(np.clip(ke, self.ke_min, self.ke_max))
        be = float(np.clip(be, self.be_min, self.be_max))
        return ke, be

    def force(self, xm: float, xm_dot: float, now: Optional[float] = None) -> float:
        xm = float(xm)
        in_contact = (xm > 0.0)
        if not in_contact:
            f = 0.0
        else:
            ke, be = self.get_params(xm=xm, now=now)
            f = float(ke * (xm - float(self.xe)) + be * float(xm_dot))

        # noise
        if self.noise_mode and str(self.noise_mode).lower() != 'none':
            if (not self.noise_contact_only) or in_contact:
                # create rng on demand; keep deterministic if seed is set
                if not hasattr(self, '_rng') or getattr(self, '_rng') is None:
                    setattr(self, '_rng', np.random.default_rng(self.noise_seed))
                rng = getattr(self, '_rng')
                nm = str(self.noise_mode).lower()
                if nm == 'gaussian':
                    sig = float(max(0.0, self.noise_sigma))
                    if sig > 0.0:
                        f = float(f + rng.normal(0.0, sig))
                elif nm == 'relative':
                    rs = float(max(0.0, self.noise_rel_sigma))
                    if rs > 0.0:
                        f = float(f * (1.0 + rng.normal(0.0, rs)))

        return float(f)


@dataclass
class PolySegVelProfile:
    """Piecewise polynomial velocity profile defined by the provided C++ snippet.

    Base polynomial:
        P(t) = t*(t-3)*(t-5)*(t-8)*(t-10)

    Velocity in each segment is a scaled version of P(t) so that max |v(t)| over the segment
    equals given maxRange.

    Segments: [0,3), [3,5), [5,8), [8,10]
    """

    # velocity limits per segment
    max_range1: float = 10.0
    max_range2: float = 15.0
    max_range3: float = 20.0
    max_range4: float = 25.0

    # time boundaries
    t0: float = 0.0
    t1: float = 3.0
    t2: float = 5.0
    t3: float = 8.0
    t4: float = 10.0

    # discretization for computing max |P(t)| in each segment
    sample_n: int = 1000


class PolyTrajectoryReference:
    """Generate (x_d, xdot_d) from a mathematical formula.

    - xdot_d(t): from PolySegVelProfile
    - x_d(t): numerical integration online at controller dt (incremental sum)

    Designed for testing acu_adaptive_impedance without retargeting.
    """

    def __init__(self, profile: Optional[PolySegVelProfile] = None):
        self.profile = profile if profile is not None else PolySegVelProfile()

        # # cache per-segment scale factors
        # self._scale_1 = None
        # self._scale_2 = None
        # self._scale_3 = None
        # self._scale_4 = None

        # cache per-segment scale factors
        self._scale_1 = 0.006
        self._scale_2 = 0.031
        self._scale_3 = 0.0083
        self._scale_4 = 0.0156


        self._last_t: Optional[float] = None
        self._x: float = 0.
        
        # self._ensure_scales()

    @staticmethod
    def _P(t):
        """Polynomial P(t) supporting scalar or numpy array input."""
        t = np.asarray(t, dtype=float)
        return t * (t - 3.0) * (t - 5.0) * (t - 8.0) * (t - 10.0)

    def _max_abs_P(self, ta: float, tb: float) -> float:
        n = int(max(2, self.profile.sample_n))
        ts = np.linspace(float(ta), float(tb), n, dtype=float)
        vals = np.abs(self._P(ts))
        return float(np.max(vals))

    def _ensure_scales(self):
        if self._scale_1 is None:
            m1 = max(self._max_abs_P(self.profile.t0, self.profile.t1), 1e-12)
            self._scale_1 = float(self.profile.max_range1) / m1
        if self._scale_2 is None:
            m2 = max(self._max_abs_P(self.profile.t1, self.profile.t2), 1e-12)
            self._scale_2 = float(self.profile.max_range2) / m2
        if self._scale_3 is None:
            m3 = max(self._max_abs_P(self.profile.t2, self.profile.t3), 1e-12)
            self._scale_3 = float(self.profile.max_range3) / m3
        if self._scale_4 is None:
            m4 = max(self._max_abs_P(self.profile.t3, self.profile.t4), 1e-12)
            self._scale_4 = float(self.profile.max_range4) / m4

    def reset(self, x0: float = 0.0):
        self._last_t = None
        self._x = float(x0)

    def desired_vel(self, t: float) -> float:
        t = float(t)
        if t < self.profile.t1:
            return float(self._scale_1 * self._P(t))
        if t < self.profile.t2:
            return float(self._scale_2 * self._P(t))
        if t < self.profile.t3:
            return float(self._scale_3 * self._P(t))
        return float(self._scale_4 * self._P(t))

    def step(self, t: float, dt: float) -> Tuple[float, float]:
        """Return (x_d, xdot_d) at time t.

        Position is updated by incremental integration using dt.
        """
        t = float(t)
        dt = float(max(1e-6, dt))

        v = self.desired_vel(t)
        if self._last_t is None:
            self._last_t = t
            return float(self._x), float(v)

        # integrate one step (rectangle rule) to match controller dt
        self._x = float(self._x + v * dt)
        self._last_t = t
        return float(self._x), float(v)


class AcuNeedleTeleopImpedanceRunner:
    """Glue module for --acu-needle-teleop using adaptive impedance.

    Responsibilities moved out of teleop_hand_and_arm.py:
      - Time/step synchronization for the impedance controller
      - Simulated environment force pm (until hardware force feedback exists)
      - Mapping: retargeting (ins, ins_vel) -> (x_d, xdot_d)
      - Mapping: robot joint7 (xm, xm_dot) -> measured state
      - Output: commanded joint7 position x_cmd

    This keeps teleop_hand_and_arm.py thin.
    """

    def __init__(
        self,
        controller: 'AcuAdaptiveImpedanceController',
        env: Optional[SimContactEnv] = None,
        arm_ctrl=None,
        needle_lock=None,
        needle_action_array=None,
        needle_meta_array=None,
        frequency_hz: float = 30.0,
        joint_index: int = 6,
        reference_source: str = 'retargeting',
        traj_ref: Optional[PolyTrajectoryReference] = None,
        log_data: bool = True,
        plot_on_finish: bool = True,
        plot_save_path: Optional[str] = None,
        stop_after_s: Optional[float] = None,
    ):
        self.ctrl = controller
        self.env = env if env is not None else SimContactEnv()

        # dependencies for running loop
        self.arm_ctrl = arm_ctrl
        self.needle_lock = needle_lock
        self.needle_action_array = needle_action_array
        self.needle_meta_array = needle_meta_array
        self.frequency_hz = float(max(1e-6, frequency_hz))
        self.joint_index = int(joint_index)

        self.reference_source = str(reference_source)
        self.traj_ref = traj_ref if traj_ref is not None else PolyTrajectoryReference()
        self._traj_t0: Optional[float] = None

        self._running = True
        self._thread: Optional[threading.Thread] = None

        # cached baseline joint vector (first 8 dof)
        self._base_q8 = None

        # last commanded state (used as xm/xm_dot placeholder when measurement is not reliable)
        self._last_cmd_x: Optional[float] = None
        self._last_cmd_v: float = 0.0
        self._last_cmd_ts: Optional[float] = None

        # data logging (for post-run plots)
        self.log_data = bool(log_data)
        self.plot_on_finish = bool(plot_on_finish)
        self.plot_save_path = plot_save_path

        self._t_buf = []
        self._x_ref_buf = []
        self._xm_buf = []
        self._pm_buf = []

        self.stop_after_s = None if stop_after_s is None else float(max(0.0, stop_after_s))
        self._session_t0: Optional[float] = None

    def start_session(self, now: Optional[float] = None):
        """Mark the start of a retargeting test session (e.g., after needle_ref_enabled becomes True)."""
        if now is None:
            now = time.time()
        self._session_t0 = float(now)
        # reset time base for plotting
        if self._traj_t0 is None:
            self._traj_t0 = float(now)

    def reset(self, x0: Optional[float] = None, base_q8: Optional[np.ndarray] = None):
        self.ctrl.reset(x0=x0)
        self._base_q8 = None if base_q8 is None else np.asarray(base_q8, dtype=float).reshape(-1)[:8].copy()
        
        self._traj_t0 = None
        if x0 is not None:
            self.traj_ref.reset(x0=float(x0))
        else:
            self.traj_ref.reset(x0=0.0)

        self._last_cmd_x = None if x0 is None else float(x0)
        self._last_cmd_v = 0.0
        self._last_cmd_ts = None

        # reset log buffers
        self._t_buf.clear()
        self._x_ref_buf.clear()
        self._xm_buf.clear()
        self._pm_buf.clear()

        self._session_t0 = None

    def configure_io(self, arm_ctrl=None, needle_lock=None, needle_action_array=None, needle_meta_array=None):
        self.arm_ctrl = arm_ctrl if arm_ctrl is not None else self.arm_ctrl
        self.needle_lock = needle_lock if needle_lock is not None else self.needle_lock
        self.needle_action_array = needle_action_array if needle_action_array is not None else self.needle_action_array
        self.needle_meta_array = needle_meta_array if needle_meta_array is not None else self.needle_meta_array

    def _read_retargeting(self) -> Tuple[float, float, float]:
        """Return (ins_ref, ins_vel_ref, tws_ref)"""
        if self.needle_lock is None or self.needle_action_array is None:
            raise RuntimeError('needle shared arrays not configured')
        with self.needle_lock:
            ins = float(self.needle_action_array[0])
            tws = float(self.needle_action_array[1])
            if self.needle_meta_array is not None:
                ins_vel = float(self.needle_meta_array[1])
            else:
                ins_vel = 0.0
        return ins, ins_vel, tws

    def step_once(self, now: Optional[float] = None, fr: Optional[float] = None) -> dict:
        """Single step: read q/dq + retargeting, compute x_cmd, publish ctrl_dual_arm."""
        if self.arm_ctrl is None:
            raise RuntimeError('arm_ctrl not configured')

        if now is None:
            now = time.time()

        q = np.asarray(self.arm_ctrl.get_current_dual_arm_q(), dtype=float).reshape(-1)[:8]
        # dq is not used for xm/xm_dot in this test mode

        if self._base_q8 is None:
            self._base_q8 = q.copy()

        if self.reference_source == 'trajectory':
            if self._traj_t0 is None:
                self._traj_t0 = float(now)
            t_rel = float(now) - float(self._traj_t0)
            # Trajectory is defined up to 10s in the provided polynomial profile.
            if t_rel > 10.0:
                # finalize plots before exit
                if self.log_data and self.plot_on_finish and len(self._t_buf) > 2:
                    try:
                        save_path = self.plot_save_path
                        if save_path is None:
                            save_path = os.path.join(os.getcwd(), 'acu_adaptive_impedance_summary.png')
                        _save_summary_plots(
                            out_path=save_path,
                            t_s=list(self._t_buf),
                            x_ref=list(self._x_ref_buf),
                            x_m=list(self._xm_buf),
                            p_m=list(self._pm_buf),
                            f_safe=float(self.ctrl.switch.f_safe),
                        )
                        logger_mp.info(f"[acu-needle-imp-runner] saved summary plot: {save_path}")
                    except Exception as e:
                        logger_mp.warning(f"[acu-needle-imp-runner] failed to save summary plot: {e}")
                self.stop()
                raise StopIteration('trajectory finished (t_rel > 10s)')
            # Use controller internal dt for integration consistency
            dt = float(self.ctrl.adapt.dt)
            ins_ref, ins_vel_ref = self.traj_ref.step(t=t_rel, dt=dt)
            tws_ref = 0.0
        else:
            ins_ref, ins_vel_ref, tws_ref = self._read_retargeting()
            ins_ref = ins_ref * 1e3  
            ins_vel_ref = ins_vel_ref * 1e3

            if self._traj_t0 is None:
                self._traj_t0 = float(now)
            t_rel = float(now) - float(self._traj_t0)

            # retargeting test auto-stop (e.g., 5s after session starts)
            if self.stop_after_s is not None and self._session_t0 is not None:
                if (float(now) - float(self._session_t0)) > float(self.stop_after_s):
                    if self.log_data and self.plot_on_finish and len(self._t_buf) > 2:
                        try:
                            save_path = self.plot_save_path
                            if save_path is None:
                                save_path = os.path.join(os.getcwd(), 'acu_adaptive_impedance_summary.png')
                            _save_summary_plots(
                                out_path=save_path,
                                t_s=list(self._t_buf),
                                x_ref=list(self._x_ref_buf),
                                x_m=list(self._xm_buf),
                                p_m=list(self._pm_buf),
                                f_safe=float(self.ctrl.switch.f_safe),
                            )
                            logger_mp.info(f"[acu-needle-imp-runner] saved summary plot: {save_path}")
                        except Exception as e:
                            logger_mp.warning(f"[acu-needle-imp-runner] failed to save summary plot: {e}")
                    self.stop()
                    raise StopIteration(f'retargeting session finished (> {self.stop_after_s:.3f}s)')

        # For now, use last commanded state as (xm, xm_dot) instead of measurement
        # to avoid noise/lag in get_current_dual_arm_q/dq during early integration tests.
        if self._last_cmd_x is None:
            # initialize from current measured q once
            self._last_cmd_x = float(q[self.joint_index])
            self._last_cmd_v = 0.0
            self._last_cmd_ts = float(now)
        xm = float(self._last_cmd_x)
        xm_dot = float(self._last_cmd_v)

        print(ins_ref)

        out = self.step(
            ins_ref=ins_ref,
            ins_vel_ref=ins_vel_ref,
            xm=xm,
            xm_dot=xm_dot,
            now=float(now),
            fr=fr,
        )

        sol_q = self._base_q8.copy()
        sol_q[self.joint_index] = float(out["x_cmd"])
        # keep last two joints from retargeting (compat with existing pipeline)
        sol_q[-2] = float(ins_ref) * 1e-3  # scale down to m
        sol_q[-1] = float(tws_ref)
        sol_tauff = np.zeros_like(sol_q)
        self.arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

        # update last commanded state so next step uses commands as (xm, xm_dot)
        try:
            self._last_cmd_x = float(out.get("x_cmd", sol_q[self.joint_index]))
            self._last_cmd_v = float(out.get("v_cmd", 0.0))
            self._last_cmd_ts = float(now)
        except Exception:
            pass

        # log
        if self.log_data:
            try:
                self._t_buf.append(float(t_rel))
                # self._x_ref_buf.append(float(out.get('x_ref', 0.0)))
                self._x_ref_buf.append(float(ins_ref))
                self._xm_buf.append(float(xm))
                self._pm_buf.append(float(out.get('pm', float('nan'))))
            except Exception:
                pass

        out["sol_q"] = sol_q
        return out

    def run(self):
        """Blocking loop: call from the main thread when needle teleop is enabled."""
        self._running = True
        while self._running:
            t0 = time.time()
            try:
                self.step_once(now=t0)
            except Exception as e:
                logger_mp.error(f"[acu-needle-imp-runner] step failed: {e}")
            dt_sleep = max(0.0, (1.0 / self.frequency_hz) - (time.time() - t0))
            time.sleep(dt_sleep)

    def start(self):
        """Start runner loop in a daemon thread (non-blocking)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def join(self, timeout: Optional[float] = None):
        if self._thread is None:
            return
        self._thread.join(timeout=timeout)

    def stop(self):
        self._running = False
        # do not block here; caller may join()

    def step(
        self,
        ins_ref: float,
        ins_vel_ref: float,
        xm: float,
        xm_dot: float,
        now: Optional[float] = None,
        fr: Optional[float] = None,
    ) -> dict:
        """Compute x_cmd for joint7.

        Args:
          ins_ref: retargeting insertion position command
          ins_vel_ref: retargeting insertion velocity command (numerical derivative)
          xm/xm_dot: measured joint7 position/velocity
          now: timestamp for synchronization
          fr: optional desired force
        """
        pm = self.env.force(xm=xm, xm_dot=xm_dot, now=now)
        out = self.ctrl.step(
            x_d=float(ins_ref),
            xdot_d=float(ins_vel_ref),
            xm=float(xm),
            xm_dot=float(xm_dot),
            pm=float(pm),
            fr=fr,
            now=now,
        )
        out["pm"] = float(pm)
        return out


class AcuAdaptiveImpedanceController:
    """High-level wrapper: normal reference tracking or adaptive impedance force control.

    Compared to the initial version, this class now follows Fig.5 more closely:
      - Switch between different position references (x_d vs x_c)
      - Keep the SAME reference velocity xdot_d for both modes
      - Use a PD(FFC) block to generate a velocity command v_cmd
      - Provide the final position command x_cmd by integrating v_cmd

    Notes:
      - This is 1-DoF scalar (needle insertion depth) controller.
      - If you later extend it to 3D, treat all scalars as vectors and apply per-axis.
    """

    def __init__(
        self,
        kf: float,
        f_safe: float,
        filter_coeffs: Tuple[float, float, float, float, float],
        fr: float = 0.0,
        gamma1: float = 5,
        gamma2: float = 10,
        gamma3: float = 0.5,
        dt: float = 0.01,
        hysteresis: float = 0.5,
        latch: bool = False,
        v_limits: Optional[VelocityOutputLimits] = None,
        integrator_leak: float = 0.0,
    ):
        self.filter = IIR2Filter(*[float(x) for x in filter_coeffs])
        self.switch = SafeForceSwitch(f_safe=f_safe, hysteresis=hysteresis, latch=latch)
        self.adapt = AdaptiveImpedanceReference(kf=kf, gamma1=gamma1, gamma2=gamma2, gamma3=gamma3, dt=dt)

        self.fr = float(fr)
        self.v_limits = v_limits if v_limits is not None else VelocityOutputLimits()
        self.integrator_leak = float(np.clip(integrator_leak, 0.0, 1.0))

        self._last_mode: Optional[bool] = None
        self._last_ts: Optional[float] = None

        # integrated command position from velocity output
        self.x_cmd_int: Optional[float] = None

    def reset(self, x0: Optional[float] = None):
        """Reset internal filters/switch/adaptation and (optionally) integrator state."""
        self.filter.reset(0.0, 0.0)
        self.switch.reset()
        self.adapt.reset()
        self._last_mode = None
        self._last_ts = None
        self.x_cmd_int = None if x0 is None else float(x0)

    def _step_dt(self, now: Optional[float]) -> float:
        if now is None:
            return self.adapt.dt
        if self._last_ts is None:
            self._last_ts = float(now)
            return self.adapt.dt
        dt = float(max(1e-6, float(now) - self._last_ts))
        self._last_ts = float(now)
        return dt

    def _clip_v(self, v: float) -> float:
        vmax = self.v_limits.vmax
        if vmax is None:
            return float(v)
        vmax = float(abs(vmax))
        return float(np.clip(v, -vmax, vmax))

    def step(
        self,
        x_d: float,
        xdot_d: float,
        xm: float,
        xm_dot: float,
        pm: float,
        fr: Optional[float] = None,
        now: Optional[float] = None,
        x_cmd_init: Optional[float] = None,
    ) -> dict:
        """Run one control step.

        Inputs:
          - x_d / xdot_d: technique reference (position/velocity)
          - xm / xm_dot: measured position/velocity
          - pm: measured force
          - fr: desired force (if None uses self.fr)

        Outputs (dict):
          - mode_impedance: bool
          - x_ref: chosen position reference (x_d or x_c)
          - xdot_ref: reference velocity (always xdot_d)
          - v_cmd: velocity command from FFC PD
          - x_cmd: integrated position command based on v_cmd
          - xr, xc, f_hat, dF, dx: intermediate signals

        Important:
          - x_ref switches with mode, but xdot_ref remains xdot_d (per Fig.5 assumption).
          - If robot interface accepts velocity, use v_cmd directly.
          - If it accepts position, use x_cmd; it is computed by integrating v_cmd.
        """
        if fr is None:
            fr = self.fr
        fr = float(fr)

        dt = self._step_dt(now)

        xm = float(xm)
        xm_dot = float(xm_dot)
        x_d = float(x_d)
        xdot_d = float(xdot_d)
        pm = float(pm)

        # initialize integrator state on first call
        if self.x_cmd_int is None:
            if x_cmd_init is not None:
                self.x_cmd_int = float(x_cmd_init)
            else:
                # default: start from current measured position
                self.x_cmd_int = float(xm)

        mode_imp = self.switch.update(pm, now=now)
        # mode_imp = False
        if self._last_mode is None or (mode_imp != self._last_mode):
            logger_mp.info(f"[acu-adaptive-imp] mode_impedance={mode_imp} (pm={pm:.3f}, f_safe={self.switch.f_safe:.3f})")
            self._last_mode = bool(mode_imp)

        # Same reference velocity for both modes (as required)
        xdot_ref = float(xdot_d)

        # Compute x_ref (switch), plus diagnostic signals
        xr = x_d
        xc = x_d
        f_hat = float('nan')
        dF = float(fr) - pm
        dx = 0.0

        if not mode_imp:
            x_ref = float(x_d)
        else:
            # adaptive reference xr (Eq.43) + update(K,B,xe)
            xr, f_hat = self.adapt.update(fr=fr, xm=xm, xm_dot=xm_dot, xdot_d=xdot_ref, pm=pm, now=now)

            # impedance filter: dx = H(z) * (Fr - Fe)
            dx = float(self.filter.process(dF))

            # xc = xr + dx
            xc = float(xr + dx)
            x_ref = float(xc)

        # FFC (Fig.5): velocity command
        # v_cmd = kF * (x_ref - x_m) + xdot_d
        # IMPORTANT: kF is the same kf used in AdaptiveImpedanceReference.
        # NOTE: Paper indicates kd=1, but the implemented FFC law here does not use velocity error.
        v_cmd = float(self.adapt.kf * (x_ref - xm) + xdot_ref)
        v_cmd = self._clip_v(v_cmd)

        # integrate velocity to position command
        # optional leak to prevent drift: x_int = (1-leak)*x_int + leak*xm + v*dt
        leak = self.integrator_leak
        if leak > 0.0:
            self.x_cmd_int = float((1.0 - leak) * self.x_cmd_int + leak * xm)
        self.x_cmd_int = float(self.x_cmd_int + v_cmd * dt)

        return {
            "mode_impedance": bool(mode_imp),
            "x_ref": float(x_ref),
            "xdot_ref": float(xdot_ref),
            "v_cmd": float(v_cmd),
            "x_cmd": float(self.x_cmd_int),
            "xr": float(xr),
            "xc": float(xc),
            "f_hat": float(f_hat),
            "dF": float(dF),
            "dx": float(dx),
            "dt": float(dt),
            "ke_hat": float(self.adapt.p.ke_hat),
            "be_hat": float(self.adapt.p.be_hat),
            "xe_hat": float(self.adapt.p.xe_hat),
        }
