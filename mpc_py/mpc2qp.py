from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .lifted import LiftedSystem, matrix_power


@dataclass
class QPData:
    """标准 QP 数据：min 0.5 x^T H x + g^T x s.t. lbA <= Gx <= ubA"""

    H: np.ndarray
    g: np.ndarray
    G: np.ndarray
    lbA: np.ndarray
    ubA: np.ndarray


def _moas_block(
    *,
    num_ctrl: int,
    num_jont: int,
    A: np.ndarray,
    B: np.ndarray,
    Theta_plus: np.ndarray,
    Epsilon: np.ndarray,
    qmin: np.ndarray,
    qmax: np.ndarray,
    umin: np.ndarray,
    umax: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """构造与你 C++ 中 MOAS 约束一致的 (Gmoas, lb, ub)。"""

    # T1,T2,T3 见 C++：mpc2qp.cpp / mpc2qp_rough_positioning.cpp
    T1 = np.block(
        [
            [A, B],
            [np.zeros_like(A), np.eye(num_jont)],
        ]
    )

    T2 = np.block(
        [
            [A - np.eye(num_jont), B],
            [A, B],
        ]
    )

    T3 = np.eye(2 * num_jont, 3 * num_jont)
    T3[num_jont : 2 * num_jont, 2 * num_jont : 3 * num_jont] = np.eye(num_jont)

    Cv = T1 @ np.linalg.inv(T2) @ T3
    Ce = Cv[:, 0 : 2 * num_jont]
    Cq = Cv[:, 2 * num_jont : 3 * num_jont]

    T4 = np.hstack([np.zeros((2 * num_jont, num_jont * num_ctrl)), Cq])

    # 注意：accurate 与 rough 的 C++ 两处，Theta_plus 格式不一致：
    # - accurate: block(2*NUM_JONT*(NUM_CTRL-1), ...)
    # - rough:    block(NUM_JONT*(NUM_CTRL-1), ...)
    # 这里严格跟 accurate/rough 各自外层调用传入的 Theta_plus 约定：
    # 我们要求调用者传进来的 Theta_plus 是 (2nN) x (n(N+1))，因此取最后一步的 2n block。
    last = Theta_plus[(2 * num_jont) * (num_ctrl - 1) : (2 * num_jont) * num_ctrl, :]
    Gmoas = Ce @ last + T4

    qmin_umin = np.vstack([qmin, umin])
    qmax_umax = np.vstack([qmax, umax])

    eps_last = Epsilon[(2 * num_jont) * (num_ctrl - 1) : (2 * num_jont) * num_ctrl, :]
    lb = qmin_umin + Ce @ eps_last
    ub = qmax_umax + Ce @ eps_last

    return Gmoas, lb, ub


def mpc2qp_accurate(
    *,
    lifted: LiftedSystem,
    Q: np.ndarray,
    R: np.ndarray,
    T: np.ndarray,
    Jacob_pos_3xn: np.ndarray,
    qk: np.ndarray,
    qkd: np.ndarray,
    qk_1: np.ndarray,
    uk_1: np.ndarray,
    Sd: np.ndarray,
) -> QPData:
    """Python 复刻 teleop/mpc/mpc2qp.cpp 中最终使用的 mpc2qp（LAR/accurate）。

    约定：
    - Jacob_pos_3xn 对应 C++ 里 dt * Jacobk * u 产生末端平移的那部分（你现在传的是 block(3,0,3,8)）。
    - qk,qkd,qk_1,uk_1 都是 (n,1) 列向量。
    - Sd 是 (3,1)。

    输出：H,g,G,lbA,ubA（与 C++ 同形状）。
    """

    n = lifted.num_jont
    N = lifted.num_ctrl
    dt = lifted.dt

    # --- Delta / Epsilon（增广系统偏置）
    dqk = qk - qk_1
    dqk_qk = np.vstack([dqk, qk])

    Delta = np.zeros((2 * n * N, 1))
    for i in range(N):
        tmp = matrix_power(lifted.Av, i + 1)
        Delta[(2 * n) * i : (2 * n) * (i + 1), :] = tmp @ dqk_qk
    Epsilon = -Delta

    # --- thetapp / guk：第一步 soft tracking (Ts*J*u - Sd)
    tmp = np.hstack([np.eye(n), np.zeros((n, n * N))])
    thetapp = dt * (Jacob_pos_3xn @ tmp)  # (3, n(N+1))
    w = 5e3

    guk = w * thetapp.T @ (Sd - dt * Jacob_pos_3xn @ uk_1)

    # --- gqd：最后一段 artificial reference 跟踪 qkd
    gqd = np.zeros((n * (N + 1), 1))
    gqd[n * N : n * (N + 1), :] = T @ qkd

    # == gq1: 第一段的状态写进去
    gq1 = np.zeros((n * (N + 1), 1))
    # gq1[n * N : n * (N + 1), :] = np.eye(n) @ qk

    g = -2.0 * (lifted.Theta_plus.T @ Q @ Epsilon + gqd + guk + gq1)
    H = 2.0 * (lifted.Theta_plus.T @ Q @ lifted.Theta_plus + R + w * (thetapp.T @ thetapp))

    # --- 约束：vel-con（q/u/du） + MOAS
    # 关节角上下限（与 C++ accurate 版本一致：q6 >=0, q6<=2）
    qmin = (-2.0 * np.pi) * np.ones((n, 1))
    qmin[6, 0] = -0.00
    # qmin = np.array([-3.14, -1.177, -1.310, -3.140, -1.300, -3.140, -0.0, -3.140]).T.reshape((n,1))
    qmin = np.array([-6.28319  , -2.35619, -2.61799  , -6.28319  , -2.56563   , -6.28319  , -0.   ,-6.28319 ]).T.reshape((n,1))

    qmax = (2.0 * np.pi) * np.ones((n, 1))
    qmax[6, 0] = 0.0
    # qmax = np.array([3.14, 1.177, 1.310, 3.140, 1.300, 3.140, 0.20, 3.140]).T.reshape((n,1))
    qmax = np.array([6.28319  , 2.35619, 2.61799  , 6.28319  , 2.56563   , 6.28319  , 0.2   , 6.28319  ]).T.reshape((n,1))
    

    Qmin = np.tile(qmin, (N, 1))
    Qmax = np.tile(qmax, (N, 1))

    # 速度上下限（与 C++ accurate：0.2, q6:2）
    umin = (-0.2) * np.ones((n, 1))
    umin[6, 0] = -0.00

    umax = 0.2 * np.ones((n, 1))
    umax[6, 0] = 0.00

    Umin = np.tile(umin, (N, 1))
    Umax = np.tile(umax, (N, 1))

    dumin = (-0.05) * np.ones((n, 1))
    dumax = 0.05 * np.ones((n, 1))
    dUmin = np.tile(dumin, (N, 1))
    dUmax = np.tile(dumax, (N, 1))

    # nC = 3nN + 2n
    nC = (3 * n * N) + (2 * n)
    G = np.zeros((nC, n * (N + 1)))
    lbA = np.zeros((nC, 1))
    ubA = np.zeros((nC, 1))

    # (1) q constraints: theta*x in [Qmin - psi*qk - omega*uk_1, Qmax - ...]
    G[0 : n * N, :] = lifted.theta
    lbA[0 : n * N, :] = Qmin - lifted.psi @ qk - lifted.omega @ uk_1
    ubA[0 : n * N, :] = Qmax - lifted.psi @ qk - lifted.omega @ uk_1

    # (2) u constraints: Gq_dot*x in [Umin-Uk_1, Umax-Uk_1]
    G[n * N : 2 * n * N, :] = lifted.Gq_dot
    Uk_1 = np.tile(uk_1, (N, 1))
    lbA[n * N : 2 * n * N, :] = Umin - Uk_1
    ubA[n * N : 2 * n * N, :] = Umax - Uk_1

    # (3) du constraints: I*x in [dUmin,dUmax]
    G[2 * n * N : 3 * n * N, :] = np.eye(n * N, n * (N + 1))
    lbA[2 * n * N : 3 * n * N, :] = dUmin
    ubA[2 * n * N : 3 * n * N, :] = dUmax

    # (4) MOAS
    Gmoas, lb_moas, ub_moas = _moas_block(
        num_ctrl=N,
        num_jont=n,
        A=lifted.A,
        B=lifted.B,
        Theta_plus=lifted.Theta_plus,
        Epsilon=Epsilon,
        qmin=qmin,
        qmax=qmax,
        umin=umin,
        umax=umax,
    )

    G[3 * n * N : 3 * n * N + 2 * n, :] = Gmoas
    lbA[3 * n * N : 3 * n * N + 2 * n, :] = lb_moas
    ubA[3 * n * N : 3 * n * N + 2 * n, :] = ub_moas

    return QPData(H=H, g=g, G=G, lbA=lbA, ubA=ubA)


def mpc2qp_rough(
    *,
    lifted: LiftedSystem,
    Q: np.ndarray,
    R: np.ndarray,
    T: np.ndarray,
    Jacob_6xn: np.ndarray,
    qk: np.ndarray,
    qkd: np.ndarray,
    qk_1: np.ndarray,
    uk_1: np.ndarray,
    Sd: np.ndarray,
    enable_attitude_eq: bool = False,
) -> QPData:
    """Python 复刻 teleop/mpc/positioning/mpc2qp_rough_positioning.cpp（LSR/rough）。

    关键差异（对齐论文 LSR）：
    - Sd 构造不同（由外部 rough planner 提供）；
    - rough 可选姿态保持等式约束：保持绕 x,y 角速度为 0（J_rxy u_k = 0）。

    约定：
    - Jacob_6xn 必须是 6xn（C++ 内部使用 block(0,0,2,n) 和 block(3,0,3,n)）。
    """

    n = lifted.num_jont
    N = lifted.num_ctrl
    dt = lifted.dt

    dqk = qk - qk_1
    dqk_qk = np.vstack([dqk, qk])

    Delta = np.zeros((2 * n * N, 1))
    for i in range(N):
        tmp = matrix_power(lifted.Av, i + 1)
        Delta[(2 * n) * i : (2 * n) * (i + 1), :] = tmp @ dqk_qk
    Epsilon = -Delta

    # 第一阶段软跟踪项：用末端平移 Jacobian（C++ 里用 block(3,0,3,8)）
    J_pos = Jacob_6xn[0:3, :]
    tmp = np.hstack([np.eye(n), np.zeros((n, n * N))])
    thetapp = dt * (J_pos @ tmp)

    w = 500
    guk = w * thetapp.T @ (Sd - (dt * J_pos @ uk_1))

    gqd = np.zeros((n * (N + 1), 1))
    gqd[n * N : n * (N + 1), :] = T @ qkd

    # == gq1: 第一段的状态写进去
    gq1 = np.zeros((n * (N + 1), 1))
    # gq1[n * N : n * (N + 1), :] = np.eye(n) @ qk

    g = -2.0 * (lifted.Theta_plus.T @ Q @ Epsilon + gqd + guk + gq1)
    H = 2.0 * (lifted.Theta_plus.T @ Q @ lifted.Theta_plus + R + w * (thetapp.T @ thetapp))

    # --- 约束：vel-con（q/u/du） + attitude eq + MOAS
    # 关节角（rough 版本：q6>=0, q6<=30）
    # qmin = (-2.0 * np.pi) * np.ones((n, 1))
    # qmin[6, 0] = 0.0
    # qmin = np.array([-3.14, -1.177, -1.310, -3.140, -1.300, -3.140, -0.1, -3.140]).T.reshape((n,1))
    qmin = np.array([-6.28319  , -2.35619, -2.61799  , -6.28319  , -2.56563   , -6.28319  , -0.   ,-6.28319 ]).T.reshape((n,1))


    # qmax = (2.0 * np.pi) * np.ones((n, 1))
    # qmax[6, 0] = 30.0
    # qmax = np.array([3.14, 1.177, 1.310, 3.140, 1.300, 3.140, 0.1, 3.140]).T.reshape((n,1))
    qmax = np.array([6.28319  , 2.35619, 2.61799  , 6.28319  , 2.56563   , 6.28319  , 0.2   , 6.28319  ]).T.reshape((n,1))

    Qmin = np.tile(qmin, (N, 1))
    Qmax = np.tile(qmax, (N, 1))

    # 速度（rough 版本：0.5, q6: 5）
    umin = (-0.5) * np.ones((n, 1))
    umin[6, 0] = -0.0

    umax = 0.5 * np.ones((n, 1))
    umax[6, 0] = 0.0

    Umin = np.tile(umin, (N, 1))
    Umax = np.tile(umax, (N, 1))

    # 加速度（rough 版本：0.5）
    dumin = (-0.5) * np.ones((n, 1))
    dumax = 0.5 * np.ones((n, 1))
    dUmin = np.tile(dumin, (N, 1))
    dUmax = np.tile(dumax, (N, 1))

    # nC = 3nN + (optional)2(roll/pitch eq) + 2n(MOAS)
    n_eq = 2 if enable_attitude_eq else 0
    nC = (3 * n * N) + n_eq + (2 * n)
    G = np.zeros((nC, n * (N + 1)))
    lbA = np.zeros((nC, 1))
    ubA = np.zeros((nC, 1))

    # (1) q
    G[0 : n * N, :] = lifted.theta
    lbA[0 : n * N, :] = Qmin - lifted.psi @ qk - lifted.omega @ uk_1
    ubA[0 : n * N, :] = Qmax - lifted.psi @ qk - lifted.omega @ uk_1

    # (2) u
    G[n * N : 2 * n * N, :] = lifted.Gq_dot
    Uk_1 = np.tile(uk_1, (N, 1))
    lbA[n * N : 2 * n * N, :] = Umin - Uk_1
    ubA[n * N : 2 * n * N, :] = Umax - Uk_1

    # (3) du
    G[2 * n * N : 3 * n * N, :] = np.eye(n * N, n * (N + 1))
    lbA[2 * n * N : 3 * n * N, :] = dUmin
    ubA[2 * n * N : 3 * n * N, :] = dUmax

    row0 = 3 * n * N

    # (4) Optional equality constraint: J_rxy * u_k = 0
    # This corresponds to the original C++ rough formulation.
    # Enable it only when needed, as it can make the QP infeasible in some debug/test setups.
    if enable_attitude_eq:
        J_rxy = Jacob_6xn[3:5, :]

        # C++ 的 Gv 构造比较绕：先对 u 做约束，再乘 tmp1 把 du 映射进去。
        Gv = np.zeros((N, n * (N + 1)))
        Gv[0:2, 0:n] = J_rxy

        tmp1 = np.vstack([lifted.Gq_dot, np.zeros((n, n * (N + 1)))])
        GvGq_dot = Gv @ tmp1

        G[row0 : row0 + 2, :] = GvGq_dot[0:2, :]

        nUk_1 = np.vstack([Uk_1, np.zeros((n, 1))])
        tmp2 = Gv[0:2, :] @ nUk_1
        lbA[row0 : row0 + 2, :] = -tmp2
        ubA[row0 : row0 + 2, :] = -tmp2

        row0 += 2

    # (5) MOAS
    Gmoas, lb_moas, ub_moas = _moas_block(
        num_ctrl=N,
        num_jont=n,
        A=lifted.A,
        B=lifted.B,
        Theta_plus=lifted.Theta_plus,
        Epsilon=Epsilon,
        qmin=qmin,
        qmax=qmax,
        umin=umin,
        umax=umax,
    )

    G[row0 : row0 + 2 * n, :] = Gmoas
    lbA[row0 : row0 + 2 * n, :] = lb_moas
    ubA[row0 : row0 + 2 * n, :] = ub_moas

    return QPData(H=H, g=g, G=G, lbA=lbA, ubA=ubA)
