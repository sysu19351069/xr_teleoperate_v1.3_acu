from __future__ import annotations

import os
import sys

import numpy as np

try:
    import pinocchio as pin
except Exception:  # pragma: no cover
    import pin  # type: ignore

# 兼容两种运行方式：
# 1) python -m mpc_py.main  -> __package__ == 'mpc_py'，可用相对导入
# 2) python mpc_py/main.py  -> __package__ is None，需要把项目根目录加入 sys.path 后再绝对导入
if __package__:
    from .controller import AcuMPCController, MPCMode
    from .planner_sd import (
        compute_sd_accurate,
        compute_sd_rough,
        init_accurate_plan,
        init_rough_plan,
    )
else:
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    from mpc_py.controller import AcuMPCController, MPCMode
    from mpc_py.planner_sd import (
        compute_sd_accurate,
        compute_sd_rough,
        init_accurate_plan,
        init_rough_plan,
    )


def main() -> int:
    urdf_path = "/home/shiyanshi/xr_teleoperate_v1.3/assets/acu/acu.urdf"

    # 仅使用 ZMQ 求解器后端（不再使用旧 stdin/stdout IPC 子进程模式）
    use_qpoases_zmq = False

    ctl = AcuMPCController(
        urdf_path=urdf_path,
        dt=0.1,
        horizon_T=1.0,
        ee_frame="Acu_ee",
        solver_backend="qpoases-zmq" if use_qpoases_zmq else "osqp",
        # qpoases_ipc_cmd 已废弃（旧IPC模式），保留注释防止误用：
        # qpoases_ipc_cmd=["/home/shiyanshi/anaconda3/envs/qpbuild/bin/python", "-u", "-m", "mpc_py.qpoases_ipc_server"],
    )

    # 初始关节角（沿用 accurate_positioning.cpp 的示例）
    # qk = np.array([1.63482, 0.0199317, -1.00999, 0.0587652, -2.15094, 0.152961, 0.0, 0.0], dtype=float)
    qk = np.array([-0. ,      0.63453,  1.19118, -0. ,      1.29998 , 0.  ,    -0.  ,    0.     ], dtype=float)

    # 当前末端位姿（用 numpy 拷贝，避免 pinocchio 内部缓存引用导致外部数组被意外更新）
    curpos = ctl.robot.fk(qk)
    cur_p = np.asarray(curpos.translation).reshape(3).copy()

    # =====================
    # Phase 1: rough positioning（复刻 C++ main.cpp 粗定位“直角线型规划”）
    # =====================
    # 目标点 O：C++ 示例里 obj_rough.topRightCorner(3,1) << -0.250, -0.750, 0
    # 这里采用米单位（与 URDF 一致）。
    O = np.array([0.250, 0.450, -0.35], dtype=float)

    rough_plan = init_rough_plan(P0=cur_p, O=O)

    # rough 阶段是否需要 target_pose（用于 IK 得到 qkd）：
    # C++ 里 qkd = ikine(obj_rough, qk, ...)；
    # 这里我们给一个目标位姿（只改平移，姿态保持当前）。
    target_rough = pin.SE3(np.asarray(curpos.rotation), O)

    for i in range(70):
        curpos = ctl.robot.fk(qk)
        print(curpos.translation)
        # print(qk)
        # try:
        #     print(ctl._ik_res.reshape(-1).T)
        # except Exception:
        #     pass
        # try:
        #     print(ctl.robot.fk(ctl._ik_res.reshape(-1)))
        # except Exception:
        #     pass
        
        curpos = ctl.robot.fk(qk)
        P = np.asarray(curpos.translation).reshape(3).copy()

        Sd, Vv, dot_value = compute_sd_rough(rough_plan, P=P)
        print(rough_plan.flag)

        res = ctl.step(qk=qk, Sd=Sd, mode=MPCMode.ROUGH, target_pose=target_rough)
        qk = res.q_next.reshape(-1)
        print(qk)
        # print(f"[rough] i={i:02d} Sd={Sd} q6={qk[6]:.3f} IK_qkd_err~{float(np.linalg.norm(res.qkd.reshape(-1)-qk)):.3e}")

        print(f"[rough] i={i:02d} Sd={Sd} \n Sd_real={ctl._Sd_real.reshape(-1).T}")

    # =====================
    # Phase 2: accurate positioning（复刻 accurate_positioning.cpp 的 Sd 球面旋转规划）
    # =====================
    # v：沿用 C++ 示例（注意其单位/含义在原代码存在混用；这里作为方向/尺度参量使用）
    # v = np.array([0.1, 0.1, 2.0], dtype=float)

    # curpos = ctl.robot.fk(qk)
    # plan_acc = init_accurate_plan(curpos=curpos.homogeneous, v=v, r=0.05)

    # # accurate 阶段：示例 target_pose 先保持当前姿态，平移由 planner 主导。
    # # 如果你希望严格复刻 C++ obj 构造（curpos * trans + FP），可以把那段目标位姿构造迁进来。
    # target_acc = None

    # for i in range(10):
    #     curpos = ctl.robot.fk(qk)
    #     step_ratio = (i + 1) / 10.0
    #     Sd = compute_sd_accurate(plan_acc, curpos=curpos.homogeneous, step_ratio=step_ratio)

    #     res = ctl.step(qk=qk, Sd=Sd, mode=MPCMode.ACCURATE, target_pose=target_acc)
    #     qk = res.q_next.reshape(-1)

    #     print(f"[acc] i={i:02d} ratio={step_ratio:.2f} Sd={Sd} q6={qk[6]:.3f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
