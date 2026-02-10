from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def matrix_power(A: np.ndarray, exponent: int) -> np.ndarray:
    """与 C++ matrixPower(A, exponent) 保持一致（朴素乘法）。

    说明：
    - 这里不直接用 np.linalg.matrix_power，是为了在数值/类型上更显式地对齐 C++。
    - 之后如果确认 A=I 等特例，可替换为更快实现。
    """
    if exponent < 0:
        raise ValueError("exponent must be >= 0")
    r = np.eye(A.shape[0], dtype=float)
    for _ in range(exponent):
        r = r @ A
    return r


@dataclass(frozen=True)
class LiftedSystem:
    """保存 MPC 里那些只依赖 (A,B,Av,Bv,NUM_CTRL,NUM_JONT) 的常量大矩阵。"""

    num_ctrl: int
    num_jont: int
    dt: float

    A: np.ndarray
    B: np.ndarray
    Av: np.ndarray
    Bv: np.ndarray

    Theta: np.ndarray
    Psi: np.ndarray
    Theta_plus: np.ndarray

    theta: np.ndarray
    omega: np.ndarray
    psi: np.ndarray

    Gq_dot: np.ndarray


def build_lifted_matrices(
    *,
    num_ctrl: int,
    num_jont: int,
    dt: float,
) -> LiftedSystem:
    """构造与你 C++ accurate_positioning.cpp 初始化阶段一致的矩阵。

    注意：
    - 这里完全按 C++ 的算法构造（包含 Av 幂、Toeplitz 堆叠）。
    - 之后做性能优化时，再把 A=I 的闭式形式替换掉。
    """

    A = np.eye(num_jont)
    B = dt * np.eye(num_jont)

    Av = np.block(
        [
            [A, np.zeros((num_jont, num_jont))],
            [A, np.eye(num_jont)],
        ]
    )
    Bv = np.vstack([B, B])

    # Theta: (2nN) x (n(N+1))
    Theta = np.zeros((2 * num_jont * num_ctrl, num_jont * (num_ctrl + 1)))
    for i in range(num_ctrl):
        for j in range(i + 1):
            Theta[
                (2 * num_jont) * i : (2 * num_jont) * (i + 1),
                num_jont * j : num_jont * (j + 1),
            ] = matrix_power(Av, i - j) @ Bv

    # Psi: (2nN) x (n(N+1))，只在最后一个 block 列非零
    Psi = np.zeros_like(Theta)
    for i in range(num_ctrl):
        tmp = matrix_power(Av, i + 1)
        Psi[
            (2 * num_jont) * i : (2 * num_jont) * (i + 1),
            num_jont * num_ctrl : num_jont * (num_ctrl + 1),
        ] = tmp[:, num_jont : 2 * num_jont]

    Theta_plus = Theta - Psi

    # 关节角与控制增量的关系：qc = psi*qk + omega*uk_1 + theta*x
    theta = np.zeros((num_jont * num_ctrl, num_jont * (num_ctrl + 1)))
    for i in range(num_ctrl):
        for j in range(i + 1):
            tmp = np.zeros((num_jont, num_jont))
            for k in range(i - j + 1):
                tmp = tmp + matrix_power(A, k) @ B
            theta[
                num_jont * i : num_jont * (i + 1),
                num_jont * j : num_jont * (j + 1),
            ] = tmp

    omega = np.zeros((num_jont * num_ctrl, num_jont))
    for i in range(num_ctrl):
        tmp = np.zeros((num_jont, num_jont))
        for k in range(i + 1):
            tmp = tmp + matrix_power(A, k) @ B
        omega[num_jont * i : num_jont * (i + 1), :] = tmp

    psi = np.zeros((num_jont * num_ctrl, num_jont))
    for i in range(num_ctrl):
        psi[num_jont * i : num_jont * (i + 1), :] = matrix_power(A, i + 1)

    # Gq_dot：速度约束映射（下三角累加）
    Gq_dot = np.zeros((num_jont * num_ctrl, num_jont * (num_ctrl + 1)))
    I = np.eye(num_jont)
    for i in range(num_ctrl):
        for j in range(i + 1):
            Gq_dot[num_jont * i : num_jont * (i + 1), num_jont * j : num_jont * (j + 1)] = I

    return LiftedSystem(
        num_ctrl=num_ctrl,
        num_jont=num_jont,
        dt=dt,
        A=A,
        B=B,
        Av=Av,
        Bv=Bv,
        Theta=Theta,
        Psi=Psi,
        Theta_plus=Theta_plus,
        theta=theta,
        omega=omega,
        psi=psi,
        Gq_dot=Gq_dot,
    )
