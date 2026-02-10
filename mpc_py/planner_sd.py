from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RoughPlanState:
    """粗定位（LSR）规划内部状态。

    对齐 C++ main.cpp 粗定位逻辑：
    - P0: 初始末端位置
    - T0: 中间拐点（保持 z 不变）
    - O : 目标位置
    - Pv/Tv: 两段方向srcsrc
    - flag: 是否进入第二段
    """

    P0: np.ndarray
    T0: np.ndarray
    O: np.ndarray
    T0: np.ndarray
    O: np.ndarray
    Pv: np.ndarray
    Pv: np.ndarray
    Tv: np.ndarray
    flag: int = 0
    _i: int = 1


def init_rough_plan(*, P0: np.ndarray, O: np.ndarray) -> RoughPlanState:
    P0 = np.asarray(P0).reshape(3)
    O = np.asarray(O).reshape(3)

    T0 = np.array([O[0], O[1], P0[2]], dtype=float)
    Pv = (T0 - P0)
    Pv = Pv / (np.linalg.norm(Pv) + 1e-12)

    Tv = (O - T0)
    Tv = Tv / (np.linalg.norm(Tv) + 1e-12)

    return RoughPlanState(P0=P0, T0=T0, O=O, Pv=Pv, Tv=Tv)


def compute_sd_rough(plan: RoughPlanState, *, P: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """C++ main.cpp（粗定位）中“直角线型规划”的一步 Sd。

    返回 (Sd, Vv, dotValue)
    - Sd: (3,)
    - Vv: (3,)
    - dotValue = Vv·Sd

    PM 推进逻辑按你给的写法复刻：
    - 第一段：PM = P_proj + (T0 - P_proj) / (2*NUM_CTRL) * min(i, 2*NUM_CTRL)
    - 第二段：PM = P_proj + (O  - P_proj) / (2*NUM_CTRL) * min(i, 2*NUM_CTRL)

    说明：
    - 原 C++ 里 NUM_CTRL=10；这里沿用等价行为：把 (2*NUM_CTRL) 视作 20 步的归一推进。
    - i 从 0 开始时 PM==P_proj，随着 i 增加逐步靠近目标点。
    """

    P = np.asarray(P).reshape(3)

    # 与 C++ 相同：靠近 T0 就切段
    if np.linalg.norm(plan.T0[0:2] - P[0:2]) < 0.05 and plan.flag == 0:
        plan.flag = 1
        plan._i = 0  # 切段时重置计数

    # 复刻 min(i, 2*NUM_CTRL) 的推进上限
    NUM_CTRL = 10
    s = float(min(getattr(plan, "_i", 1), 2 * NUM_CTRL))
    plan._i = getattr(plan, "_i", 1) + 1

    if plan.flag == 0:
        w = P - plan.P0
        P_proj = plan.P0 + float(w.dot(plan.Pv)) * plan.Pv
        PM = P_proj + (plan.T0 - P_proj) / (2.0 * NUM_CTRL) * s
        Sd = PM - P
        Vv = plan.Tv
    else:
        w = P - plan.T0
        P_proj = plan.T0 + float(w.dot(plan.Tv)) * plan.Tv
        PM = P_proj + (plan.O - P_proj) / (2.0 * NUM_CTRL) * s 
        Sd = PM - P
        Vv = plan.Pv

    dot_value = float(Vv.dot(Sd))
    return Sd, Vv, dot_value


@dataclass
class AccuratePlanState:
    """精定位（LAR）规划内部状态（对齐 accurate_positioning.cpp）。

    约定：curpos 输入为“针头(needle_head)”位姿。

    - r: 针长（m）
    - c_T: 针尖位姿（由 curpos 沿局部 +Z 平移 r 得到）
    - dO2: 目标球面方向向量（世界系，长度为 r）
    - trans_R: 目标姿态附加旋转（世界系，curpos.R @ trans_R 为 obj.R）
    - obj: 目标位姿（4x4）
    """

    r: float
    c_T: np.ndarray  # (4,4)
    dO2: np.ndarray  # (3,)
    trans_R: np.ndarray  # (3,3)
    obj: np.ndarray  # (4,4)


def _skew(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=float,
    )


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(3)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.eye(3)
    a = axis / n
    K = _skew(a)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def init_accurate_plan(*, curpos: np.ndarray, v: np.ndarray, r: float = 0.05) -> AccuratePlanState:
    """初始化精定位规划，并生成目标位姿 obj。

    严格对齐 C++ accurate_positioning.cpp：
    - 输入 curpos 为当前“针头”位姿（世界系 4x4）
    - trans: 由 z 旋到 v 的旋转（AngleAxis，轴=z×v，角=acos(v·z/|v||z|)）
    - f_T = curpos * Tl，Tl 的平移为 -v（局部平移）
    - dO2 = r * normalize(f_T.p - curpos.p)
    - c_T = curpos * Tz(r)，Tz 的平移为 (0,0,r)（局部平移），表示针尖位姿
    - FP = c_T.p + dO2
    - obj = curpos * trans（先旋转），再把 obj.p 设为 FP

    返回：
    - plan: AccuratePlanState
    """

    curpos = np.asarray(curpos, dtype=float).reshape(4, 4)
    v = np.asarray(v, dtype=float).reshape(3)
    r = float(r)

    # -------- trans (z -> v) --------
    z = np.array([0.0, 0.0, 1.0], dtype=float)
    vn = np.cross(z, v)
    v_norm = float(np.linalg.norm(v))
    if v_norm < 1e-12:
        trans_R = np.eye(3)
    else:
        cosang = float(np.clip(float(np.dot(v, z) / (v_norm * (np.linalg.norm(z) + 1e-12))), -1.0, 1.0))
        angle = float(np.arccos(cosang))
        if float(np.linalg.norm(vn)) < 1e-12:
            # v 与 z 平行/反平行：轴不定。平行时 trans=I；反平行时绕 X 轴旋转 pi。
            if cosang > 0.0:
                trans_R = np.eye(3)
            else:
                trans_R = _rodrigues(np.array([1.0, 0.0, 0.0]), np.pi)
        else:
            trans_R = _rodrigues(vn, angle)

    # -------- f_T = curpos * Tl (local translate -v) --------
    Tl = np.eye(4)
    Tl[0:3, 3] = -v
    f_T = curpos @ Tl

    dO2_dir = f_T[0:3, 3] - curpos[0:3, 3]
    # dO2_dir = -v
    dO2_dir = dO2_dir / (float(np.linalg.norm(dO2_dir)) + 1e-12)
    dO2 = r * dO2_dir

    # -------- c_T = curpos * Tz(r) (needle tip) --------
    Tz = np.eye(4)
    Tz[0:3, 3] = np.array([0.0, 0.0, r], dtype=float)
    c_T = curpos @ Tz

    # -------- obj --------
    obj = curpos.copy()
    obj[0:3, 0:3] = curpos[0:3, 0:3] @ trans_R
    FP = c_T[0:3, 3] + dO2
    obj[0:3, 3] = FP

    plan = AccuratePlanState(r=r, c_T=c_T, dO2=dO2, trans_R=trans_R, obj=obj)
    return plan


def compute_sd_accurate(plan: AccuratePlanState, *, curpos: np.ndarray, step_ratio: float) -> np.ndarray:
    """复刻 accurate_positioning.cpp 中 Sd = OM - OP。

    - curpos: 当前“针头”位姿 (4,4)
    - plan.c_T: 固定的“针尖”位姿 (curpos0 * Tz(r))

    step_ratio: 0~1，对应 C++ 中 min(i+1,NUM_CTRL)/NUM_CTRL。

    返回 Sd shape (3,)
    """

    curpos = np.asarray(curpos, dtype=float).reshape(4, 4)
    cur_p = curpos[0:3, 3]

    c_p = np.asarray(plan.c_T, dtype=float).reshape(4, 4)[0:3, 3]

    # OP = curpos.p - c_T.p
    OP = cur_p - c_p
    OPn = float(np.linalg.norm(OP))
    if OPn < 1e-12:
        return np.zeros((3,), dtype=float)

    # OPr = r * normalize(OP)
    OPr = plan.r * (OP / OPn)

    dO2 = np.asarray(plan.dO2, dtype=float).reshape(3)

    # rotate_axis = OPr x dO2
    rotate_axis = np.cross(OPr, dO2)
    axis_norm = float(np.linalg.norm(rotate_axis))
    if axis_norm < 1e-12:
        OM = OPr
    else:
        # anglesum = acos(OPr.dot(dO2)/(r*r))
        denom = (plan.r * plan.r) + 1e-12
        cosang = float(np.clip(float(np.dot(OPr, dO2) / denom), -1.0, 1.0))
        anglesum = float(np.arccos(cosang))
        angle = anglesum * float(np.clip(step_ratio, 0.0, 1.0))

        R = _rodrigues(rotate_axis, angle)
        OM = R @ OPr

    Sd = OM - OP
    return Sd
