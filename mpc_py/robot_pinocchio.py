from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np 

try:
    import pinocchio as pin
except Exception:  # pragma: no cover
    import pin  # type: ignore




def _resolve_frame_id(model: "pin.Model", ee_frame: str | None) -> int:
    if ee_frame is None:
        # 尽量选择最后一个 link 对应的 frame（URDF 没有显式 tool frame 时很常见）
        return model.nframes - 1
    frame_id = model.getFrameId(ee_frame)
    if frame_id == len(model.frames):
        raise ValueError(f"frame '{ee_frame}' not found in model")
    return frame_id


@dataclass
class PinRobot:
    """针灸机器人(8DOF) Pinocchio 模型封装。

    目标：替代 RobCoGen 的 transforms/jacobians/ikine，并把对外 API 尽量收敛到：
    - fk(q) -> SE3
    - jacobian(q) -> 6xn

    兼容性说明：
    - 现有 fk()/jacobian() 默认仍围绕 Acu_ee（针尖）
    - 新增 needle_head（针头）frame：相对 Acu_ee 沿 -Z 平移 0.05m
    """

    model: "pin.Model"
    data: "pin.Data"
    ee_frame: str | None
    ee_frame_id: int

    # 新增：针头 frame（用于精定位）
    needle_head_frame: str
    needle_head_frame_id: int

    @classmethod
    def from_urdf(cls, urdf_path: str | Path, *, ee_frame: str | None = None) -> "PinRobot":
        urdf_path = Path(urdf_path)
        if not urdf_path.exists():
            raise FileNotFoundError(str(urdf_path))

        robot = pin.RobotWrapper.BuildFromURDF(str(urdf_path), str(urdf_path.parent))
        # model = pin.buildModelFromUrdf(str(urdf_path), str(urdf_path.parent))
        mixed_jointsToLockIDs = []
        redeced_robot = robot.buildReducedRobot(
            list_of_joints_to_lock=mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * robot.model.nq),
        )
        model = redeced_robot.model
        data = redeced_robot.data

        # 若 URDF 中缺少 Acu_ee（你在 teleop/robot_control/robot_arm_ik.py 里是额外加的），则在这里补一个。
        # 默认将其挂在 link8（末端 link）上，placement 先用单位变换。
        if ee_frame == "Acu_ee":
            frame_id = model.getFrameId("Acu_ee")
            if frame_id == len(model.frames):
                # 找 parent：优先 link8，否则用最后一个 link
                parent_frame = "link8" if "link8" in [f.name for f in model.frames] else None
                if parent_frame is not None:
                    parent_id = model.getFrameId(parent_frame)
                else:
                    # 使用最后一个 body frame 的 parent joint
                    parent_id = model.nframes - 1

                parent_joint = model.frames[parent_id].parentJoint

                # 以父 frame 的 placement 作为基准：Acu_ee = parent_frame * I
                # placement = pin.SE3.Identity()
                placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.34513]).T)
                new_frame = pin.Frame(
                    "Acu_ee",
                    parent_joint,
                    parent_id,
                    placement,
                    pin.FrameType.OP_FRAME,
                )
                model.addFrame(new_frame)
                data = model.createData()

        ee_frame_id = _resolve_frame_id(model, ee_frame)

        # 新增 needle_head：挂在 Acu_ee 上，沿 -Z 平移 0.05m。
        needle_head_name = "needle_head"
        nh_id = model.getFrameId(needle_head_name)
        if nh_id == len(model.frames):
            # 以 Acu_ee 为 parent；若 Acu_ee 不存在则回退到当前 ee_frame_id。
            # parent_id = model.getFrameId("Acu_ee")
            parent_id = model.getFrameId("link8")
            if parent_id == len(model.frames):
                parent_id = ee_frame_id
            parent_joint = model.frames[parent_id].parentJoint

            # placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.05]))
            placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.29513]))

            new_frame = pin.Frame(
                needle_head_name,
                parent_joint,
                parent_id,
                placement,
                pin.FrameType.OP_FRAME,
            )
            model.addFrame(new_frame)
            data = model.createData()
            nh_id = model.getFrameId(needle_head_name)

        return cls(
            model=model,
            data=data,
            ee_frame=ee_frame,
            ee_frame_id=ee_frame_id,
            needle_head_frame=needle_head_name,
            needle_head_frame_id=int(nh_id),
        )

    def _frame_id(self, frame: str | int | None) -> int:
        if frame is None:
            return int(self.ee_frame_id)
        if isinstance(frame, int):
            return int(frame)
        if frame == self.needle_head_frame:
            return int(self.needle_head_frame_id)
        return int(_resolve_frame_id(self.model, frame))

    def fk(self, q: np.ndarray) -> "pin.SE3":
        q = np.asarray(q).reshape(-1)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id]

    def fk_frame(self, q: np.ndarray, *, frame: str | int | None = None) -> "pin.SE3":
        """按指定 frame 做 FK。

        frame:
        - None: 默认 ee_frame（兼容旧接口）
        - "needle_head": 针头
        - 其他字符串：模型中的 frame 名
        - int：frame_id
        """
        q = np.asarray(q).reshape(-1)
        fid = self._frame_id(frame)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[fid]

    def jacobian(self, q: np.ndarray, *, ref: int | None = None) -> np.ndarray:
        q = np.asarray(q).reshape(-1)
        if ref is None:
            ref = pin.LOCAL_WORLD_ALIGNED
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, ref)
        return np.asarray(J)

    def jacobian_frame(self, q: np.ndarray, *, frame: str | int | None = None, ref: int | None = None) -> np.ndarray:
        """按指定 frame 计算 6D Jacobian（返回 6xn）。"""
        q = np.asarray(q).reshape(-1)
        if ref is None:
            ref = pin.LOCAL_WORLD_ALIGNED
        fid = self._frame_id(frame)
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.computeFrameJacobian(self.model, self.data, q, fid, ref)
        return np.asarray(J)


def detect_linear_jacobian_rows(robot: PinRobot, q: np.ndarray, *, eps: float = 1e-7, frame: str | int | None = None) -> tuple[slice, slice]:
    """用数值差分自动判断 frame Jacobian 中哪三行对应平移。

    支持指定 frame（用于精定位的 needle_head 等）。

    返回： (linear_rows, angular_rows)
    """

    q = np.asarray(q).reshape(-1)
    n = q.shape[0]

    oMf = robot.fk_frame(q, frame=frame)
    p0 = np.asarray(oMf.translation).reshape(3)

    J = robot.jacobian_frame(q, frame=frame)

    dq = np.zeros(n)
    dq[0] = eps

    oMf_eps = robot.fk_frame(q + dq, frame=frame)
    p1 = np.asarray(oMf_eps.translation).reshape(3)
    dp = (p1 - p0)

    dp_top = (J[0:3, :] @ dq).reshape(3)
    dp_bottom = (J[3:6, :] @ dq).reshape(3)

    err_top = float(np.linalg.norm(dp - dp_top))
    err_bottom = float(np.linalg.norm(dp - dp_bottom))

    if err_top <= err_bottom:
        return slice(0, 3), slice(3, 6)
    return slice(3, 6), slice(0, 3)


def _se3_log6(oMf: "pin.SE3") -> np.ndarray:
    """SE3 -> 6D error (log)。返回 shape (6,)"""
    err = pin.log6(oMf).vector
    return np.asarray(err).reshape(6)


def _clamp(x: np.ndarray, lo: np.ndarray | None, hi: np.ndarray | None) -> np.ndarray:
    if lo is not None:
        x = np.maximum(x, lo)
    if hi is not None:
        x = np.minimum(x, hi)
    return x


# =========================
# IK 实现（CasADi 优先，DLS 兜底）
# =========================


@dataclass
class IKResult:
    q: np.ndarray
    success: bool
    iters: int
    err_norm: float


class CasadiSingleArmIK:
    """参考 teleop/robot_control/robot_arm_ik.py 的 CasADi+Pinocchio(cpin) 做法，做单臂 IK。

    新增：支持指定用于 IK 的末端 frame。
    - 默认依旧用 Acu_ee（兼容旧调用）
    - 精定位可以传入 frame="needle_head"，让目标点以针头为参考
    """

    def __init__(
        self,
        robot: PinRobot,
        *,
        w_pos: float = 50.0,
        w_rot: float = 1.0,
        w_reg: float = 0.02,
        w_smooth: float = 0.1,
        max_iter: int = 50,
        tol: float = 1e-6,
        frame: str | None = None,
    ) -> None:
        import casadi as cs
        from pinocchio import casadi as cpin

        self.robot = robot
        self.model = robot.model

        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()

        self.cq = cs.SX.sym("q", self.model.nq, 1)
        self.cTf = cs.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # frame id（默认 Acu_ee；允许 needle_head）
        if frame is None:
            frame = "Acu_ee"
        self._frame_name = str(frame)
        fid = self.model.getFrameId(self._frame_name)
        if fid == len(self.model.frames):
            raise ValueError(f"IK frame '{self._frame_name}' not found")
        self._frame_id = int(fid)

        self.translational_error = cs.Function(
            "translational_error",
            [self.cq, self.cTf],
            [self.cdata.oMf[self._frame_id].translation - self.cTf[:3, 3]],
        )
        self.rotational_error = cs.Function(
            "rotational_error",
            [self.cq, self.cTf],
            [cpin.log3(self.cdata.oMf[self._frame_id].rotation @ self.cTf[:3, :3].T)],
        )

        self.opti = cs.Opti()
        self.var_q = self.opti.variable(self.model.nq)
        self.var_q_last = self.opti.parameter(self.model.nq)
        self.param_tf = self.opti.parameter(4, 4)

        self.translational_cost = cs.sumsqr(self.translational_error(self.var_q, self.param_tf))
        self.rotation_cost = cs.sumsqr(self.rotational_error(self.var_q, self.param_tf))
        self.regularization_cost = cs.sumsqr(self.var_q)
        self.smooth_cost = cs.sumsqr(self.var_q - self.var_q_last)

        self.opti.subject_to(self.opti.bounded(
            np.array([-3.14  , -1.1775, -1.31  , -3.14  , -1.3   , -3.14  , -0.   ,
       -3.14  ]),
            self.var_q,
            np.array([3.14  , 1.1775, 1.31  , 3.14  , 1.3   , 3.14  , 0.00   , 3.14  ]))
        )

        # self.opti.minimize(50 * self.translational_cost + 5 * self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)
        self.opti.minimize(50 * self.translational_cost + 0.5 * self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

        opts = {
            'expand': True,
            'detect_simple_bounds': True,
            'calc_lam_p': False,
            'print_time': False,
            'ipopt.sb': 'yes',
            'ipopt.print_level': 0,
            'ipopt.max_iter': 50,
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 5e-4,
            'ipopt.acceptable_iter': 5,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.derivative_test': 'none',
            'ipopt.jacobian_approximation': 'exact',
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.model.nq)
        from teleop.utils.weighted_moving_filter import WeightedMovingFilter
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), max(4, self.model.nq))

        self.init_joint_pos = None

        self._q_last = np.zeros((self.model.nq,))

    def solve(self, target: "pin.SE3", *, q0: np.ndarray | None = None) -> IKResult:
        import casadi as cs

        if q0 is None:
            q0 = self._q_last
        q0 = np.asarray(q0).reshape(-1)

        self.opti.set_initial(self.var_q, q0)
        self.opti.set_value(self.var_q_last, self._q_last)
        self.opti.set_value(self.param_tf, target.homogeneous)

        try:
            sol = self.opti.solve()
            q_sol = np.asarray(sol.value(self.var_q)).reshape(-1)
            self.smooth_filter.add_data(q_sol)
            q_sol = self.smooth_filter.filtered_data

            self._q_last = q_sol

            # 误差复算
            # NOTE: err_norm 按 IK 的 frame 计算
            oMf = self.robot.fk_frame(q_sol, frame=self._frame_id)
            e = pin.log6(oMf.inverse() * target).vector
            err_norm = float(np.linalg.norm(np.asarray(e)))

            return IKResult(q=q_sol.reshape(-1, 1), success=True, iters=int(sol.stats().get("iter_count", 0)), err_norm=err_norm)
        except Exception:
            # fallback: 返回 debug 值，标记失败
            try:
                q_dbg = np.asarray(self.opti.debug.value(self.var_q)).reshape(-1)
            except Exception:
                q_dbg = q0

            oMf = self.robot.fk_frame(q_dbg, frame=self._frame_id)
            e = pin.log6(oMf.inverse() * target).vector
            err_norm = float(np.linalg.norm(np.asarray(e)))
            self._q_last = q_dbg
            return IKResult(q=q_dbg.reshape(-1, 1), success=False, iters=0, err_norm=err_norm)


def ik_dls(
    robot: PinRobot,
    target: "pin.SE3",
    q0: np.ndarray,
    *,
    tol: float = 1e-4,
    max_iters: int = 200,
    damping: float = 1e-6,
    dq_max: float = 0.2,
    position_only: bool = False,
    ref: int | None = None,
) -> IKResult:
    """轻量 DLS IK（当 casadi/ipopt 不可用时兜底）。"""

    if ref is None:
        ref = pin.LOCAL_WORLD_ALIGNED

    q = np.asarray(q0).reshape(-1).copy()

    for it in range(1, max_iters + 1):
        oMf = robot.fk(q)

        if position_only:
            e = np.asarray(target.translation - oMf.translation).reshape(3)
            err_norm = float(np.linalg.norm(e))
        else:
            e6 = pin.log6(oMf.inverse() * target).vector
            e = np.asarray(e6).reshape(6)
            err_norm = float(np.linalg.norm(e))

        if err_norm < tol:
            return IKResult(q=q.reshape(-1, 1), success=True, iters=it, err_norm=err_norm)

        J = robot.jacobian(q, ref=ref)
        if position_only:
            J = J[3:6, :]

        JJt = J @ J.T
        A = JJt + damping * np.eye(J.shape[0])
        dq = J.T @ np.linalg.solve(A, e)

        nrm = float(np.linalg.norm(dq))
        if nrm > dq_max:
            dq *= dq_max / nrm

        q = q + dq

    return IKResult(q=q.reshape(-1, 1), success=False, iters=max_iters, err_norm=err_norm)
