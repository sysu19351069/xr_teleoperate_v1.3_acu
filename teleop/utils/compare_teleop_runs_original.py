"""离线对比：MPC vs IK（两次运行采集的 teleop_compare_*.npz）。

用法示例：
  python teleop/utils/compare_teleop_runs.py \
    --mpc teleop_compare_logs/teleop_compare_mpc_*.npz \
    --ik  teleop_compare_logs/teleop_compare_ik_*.npz \
    --out teleop_compare_logs/report.html

输出：
- 一个自包含 HTML（Plotly 离线）
- 包含：3D轨迹、误差时间序列、分布与统计表

约定：
- ground truth 为 npz 内 xr_p/xr_quat（已映射到机器人坐标系）
- 实测为 ee_p/ee_quat（由 q_meas8 的 FK 得到）
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


def _load_npz(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    d = np.load(p, allow_pickle=True)
    out: dict[str, Any] = {k: d[k] for k in d.files}
    # meta 是 object array（dict）
    if "meta" in out:
        try:
            out["meta"] = out["meta"].item()
        except Exception:
            pass
    return out


def _ensure_same_len(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    n = min(len(a), len(b))
    return a[:n], b[:n]


def _rmse(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float).reshape(-1)
    return float(np.sqrt(np.mean(np.square(x))))


def _mae(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float).reshape(-1)
    return float(np.mean(np.abs(x)))


def _p95(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float).reshape(-1)
    return float(np.quantile(x, 0.95))


@dataclass
class RunData:
    name: str
    meta: dict[str, Any]
    t: np.ndarray
    ee_p: np.ndarray
    xr_p: np.ndarray
    err_pos: np.ndarray
    err_ori: np.ndarray

    # joint measurements (optional)
    q_meas8: np.ndarray | None = None

    # MPC-PF extra
    err_proj: np.ndarray | None = None
    err_des: np.ndarray | None = None
    err_end_pos: np.ndarray | None = None

    def t0_aligned(self) -> np.ndarray:
        t = np.asarray(self.t, dtype=float).reshape(-1)
        if t.size == 0:
            return t
        return t - float(t[0])


def _to_run(d: dict[str, Any], *, name: str) -> RunData:
    meta = d.get("meta", {}) if isinstance(d.get("meta", {}), dict) else {}
    err_proj = np.asarray(d.get("err_proj"), dtype=float).reshape(-1) if "err_proj" in d else None
    err_des = np.asarray(d.get("err_des"), dtype=float).reshape(-1) if "err_des" in d else None
    err_end_pos = np.asarray(d.get("err_end_pos"), dtype=float).reshape(-1) if "err_end_pos" in d else None
    q_meas8 = None
    if "q_meas8" in d:
        try:
            q_meas8 = np.asarray(d["q_meas8"], dtype=float).reshape(-1, 8)
        except Exception:
            q_meas8 = None
    return RunData(
        name=name,
        meta=meta,
        t=np.asarray(d["t"], dtype=float).reshape(-1),
        ee_p=np.asarray(d["ee_p"], dtype=float).reshape(-1, 3),
        xr_p=np.asarray(d["xr_p"], dtype=float).reshape(-1, 3),
        err_pos=np.asarray(d["err_pos"], dtype=float).reshape(-1),
        err_ori=np.asarray(d["err_ori"], dtype=float).reshape(-1),
        q_meas8=q_meas8,
        err_proj=err_proj,
        err_des=err_des,
        err_end_pos=err_end_pos,
    )


def _finite(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float).reshape(-1)
    return x[np.isfinite(x)]


def _joint_jitter_series(q: np.ndarray) -> np.ndarray:
    """Compute per-step absolute joint delta: |q[k]-q[k-1]|, shape (T-1, dof)."""
    q = np.asarray(q, dtype=float).reshape(-1, q.shape[-1])
    if q.shape[0] < 2:
        return np.zeros((0, q.shape[1]), dtype=float)
    return np.abs(np.diff(q, axis=0))


def _arc_length_s(p: np.ndarray) -> np.ndarray:
    """Compute normalized arc-length parameter s in [0,1] for a polyline p(T,3)."""
    p = np.asarray(p, dtype=float).reshape(-1, 3)
    if p.shape[0] == 0:
        return np.zeros((0,), dtype=float)
    if p.shape[0] == 1:
        return np.array([0.0], dtype=float)
    ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    total = float(s[-1])
    if total <= 1e-12:
        return np.linspace(0.0, 1.0, num=p.shape[0], dtype=float)
    return (s / total).astype(float)


def build_report(*, mpc: RunData, ik: RunData, out_html: Path, title: str) -> None:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    # time alignment: shift each run so starts at t=0
    t_mpc = mpc.t0_aligned()
    t_ik = ik.t0_aligned()

    # 不做长度截断：mpc/ik 采样点数允许不同，只要各自从 t=0 开始。

    # 轨迹图：参考用各自的 xr_p（理论上是相同坐标系，但两次手动遥操不完全一致）
    fig_traj = go.Figure()

    fig_traj.add_trace(
        go.Scatter3d(
            x=mpc.xr_p[:, 0],
            y=mpc.xr_p[:, 1],
            z=mpc.xr_p[:, 2],
            mode="lines",
            name="XR_ref (MPC run)",
            line=dict(width=4, color="#888"),
            opacity=0.6,
        )
    )
    fig_traj.add_trace(
        go.Scatter3d(
            x=mpc.ee_p[:, 0],
            y=mpc.ee_p[:, 1],
            z=mpc.ee_p[:, 2],
            mode="lines",
            name="EE_meas (MPC)",
            line=dict(width=5, color="#1f77b4"),
        )
    )
    fig_traj.add_trace(
        go.Scatter3d(
            x=ik.xr_p[:, 0],
            y=ik.xr_p[:, 1],
            z=ik.xr_p[:, 2],
            mode="lines",
            name="XR_ref (IK run)",
            line=dict(width=4, color="#aaa"),
            opacity=0.6,
        )
    )
    fig_traj.add_trace(
        go.Scatter3d(
            x=ik.ee_p[:, 0],
            y=ik.ee_p[:, 1],
            z=ik.ee_p[:, 2],
            mode="lines",
            name="EE_meas (IK)",
            line=dict(width=5, color="#d62728"),
        )
    )

    fig_traj.update_layout(
        title="3D轨迹对比（XR参考 vs 实测EE）",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(orientation="h"),
        margin=dict(l=0, r=0, t=50, b=0),
    )

    # 误差时间序列：2x2
    fig_err = make_subplots(
        rows=2,
        cols=2,
        subplot_titles=(
            "位置误差：MPC(投影) vs IK(端点)（t从0对齐）",
            "姿态误差角距离 (deg)",
            "位置误差（对数视图）",
            "姿态误差（对数视图）",
        ),
    )

    # MPC position error: show projection only (fallback to err_pos)
    mpc_pos = mpc.err_proj if mpc.err_proj is not None else mpc.err_pos
    fig_err.add_trace(go.Scatter(x=t_mpc, y=mpc_pos, name="MPC err_proj", line=dict(color="#1f77b4")), row=1, col=1)
    fig_err.add_trace(go.Scatter(x=t_ik, y=ik.err_pos, name="IK err_end", line=dict(color="#d62728")), row=1, col=1)

    fig_err.add_trace(go.Scatter(x=t_mpc, y=np.rad2deg(mpc.err_ori), name="MPC", line=dict(color="#1f77b4"), showlegend=False), row=1, col=2)
    fig_err.add_trace(go.Scatter(x=t_ik, y=np.rad2deg(ik.err_ori), name="IK", line=dict(color="#d62728"), showlegend=False), row=1, col=2)

    fig_err.add_trace(go.Scatter(x=t_mpc, y=np.maximum(mpc_pos, 1e-9), name="MPC", line=dict(color="#1f77b4"), showlegend=False), row=2, col=1)
    fig_err.add_trace(go.Scatter(x=t_ik, y=np.maximum(ik.err_pos, 1e-9), name="IK", line=dict(color="#d62728"), showlegend=False), row=2, col=1)

    fig_err.add_trace(go.Scatter(x=t_mpc, y=np.maximum(np.rad2deg(mpc.err_ori), 1e-6), name="MPC", line=dict(color="#1f77b4"), showlegend=False), row=2, col=2)
    fig_err.add_trace(go.Scatter(x=t_ik, y=np.maximum(np.rad2deg(ik.err_ori), 1e-6), name="IK", line=dict(color="#d62728"), showlegend=False), row=2, col=2)

    fig_err.update_yaxes(title_text="位置误差 (m)", row=1, col=1)
    fig_err.update_yaxes(title_text="姿态误差 (deg)", row=1, col=2)
    fig_err.update_yaxes(title_text="位置误差 (m)", type="log", row=2, col=1)
    fig_err.update_yaxes(title_text="姿态误差 (deg)", type="log", row=2, col=2)
    fig_err.update_xaxes(title_text="t (s)")
    fig_err.update_layout(title="误差随时间对比", legend=dict(orientation="h"), height=700)

    # 分布对比：箱线图
    fig_dist = make_subplots(rows=1, cols=2, subplot_titles=("位置误差分布 (m)", "姿态误差分布 (deg)"))

    fig_dist.add_trace(go.Box(y=mpc.err_pos, name="MPC", marker_color="#1f77b4"), row=1, col=1)
    fig_dist.add_trace(go.Box(y=ik.err_pos, name="IK", marker_color="#d62728"), row=1, col=1)

    fig_dist.add_trace(go.Box(y=np.rad2deg(mpc.err_ori), name="MPC", marker_color="#1f77b4", showlegend=False), row=1, col=2)
    fig_dist.add_trace(go.Box(y=np.rad2deg(ik.err_ori), name="IK", marker_color="#d62728", showlegend=False), row=1, col=2)

    fig_dist.update_layout(title="误差分布对比", height=450)

    # 统计表
    mpc_pos_for_stats = mpc.err_des if mpc.err_des is not None else (mpc.err_proj if mpc.err_proj is not None else mpc.err_pos)
    ik_pos_for_stats = ik.err_pos
    mpc_pos_f = _finite(mpc_pos_for_stats)
    ik_pos_f = _finite(ik_pos_for_stats)
    mpc_ori_f = _finite(np.rad2deg(mpc.err_ori))
    ik_ori_f = _finite(np.rad2deg(ik.err_ori))

    stats = {
        "metric": ["RMSE", "MAE", "P95", "MAX"],
        "MPC_pos(m)": [
            _rmse(mpc_pos_f),
            _mae(mpc_pos_f),
            _p95(mpc_pos_f),
            float(np.max(mpc_pos_f)) if mpc_pos_f.size else float('nan'),
        ],
        "IK_pos(m)": [
            _rmse(ik_pos_f),
            _mae(ik_pos_f),
            _p95(ik_pos_f),
            float(np.max(ik_pos_f)) if ik_pos_f.size else float('nan'),
        ],
        "MPC_ori(deg)": [
            _rmse(mpc_ori_f),
            _mae(mpc_ori_f),
            _p95(mpc_ori_f),
            float(np.max(mpc_ori_f)) if mpc_ori_f.size else float('nan'),
        ],
        "IK_ori(deg)": [
            _rmse(ik_ori_f),
            _mae(ik_ori_f),
            _p95(ik_ori_f),
            float(np.max(ik_ori_f)) if ik_ori_f.size else float('nan'),
        ],
    }

    fig_table = go.Figure(
        data=[
            go.Table(
                header=dict(values=list(stats.keys()), fill_color="#eee", align="left"),
                cells=dict(values=list(stats.values()), align="left"),
            )
        ]
    )
    fig_table.update_layout(title="统计指标对比")

    # 关节抖动趋势：使用 q_meas8 的逐步差分幅值 |dq|（rad/step）
    fig_joint = None
    try:
        if (mpc.q_meas8 is not None) or (ik.q_meas8 is not None):
            joint_names = [f"J{i}" for i in range(8)]
            fig_joint = make_subplots(
                rows=8,
                cols=1,
                shared_xaxes=True,
                vertical_spacing=0.02,
                subplot_titles=[f"|Δq| per step (rad/step) - {jn}" for jn in joint_names],
            )

            if mpc.q_meas8 is not None:
                mpc_dq = _joint_jitter_series(mpc.q_meas8)
                t_mpc_dq = t_mpc[1 : 1 + mpc_dq.shape[0]] if t_mpc.size >= 2 else np.arange(mpc_dq.shape[0])
                for j in range(min(8, mpc_dq.shape[1])):
                    fig_joint.add_trace(
                        go.Scatter(
                            x=t_mpc_dq,
                            y=mpc_dq[:, j],
                            name=f"MPC {joint_names[j]}",
                            line=dict(color="#1f77b4"),
                            showlegend=(j == 0),
                        ),
                        row=j + 1,
                        col=1,
                    )

            if ik.q_meas8 is not None:
                ik_dq = _joint_jitter_series(ik.q_meas8)
                t_ik_dq = t_ik[1 : 1 + ik_dq.shape[0]] if t_ik.size >= 2 else np.arange(ik_dq.shape[0])
                for j in range(min(8, ik_dq.shape[1])):
                    fig_joint.add_trace(
                        go.Scatter(
                            x=t_ik_dq,
                            y=ik_dq[:, j],
                            name=f"IK {joint_names[j]}",
                            line=dict(color="#d62728"),
                            showlegend=(j == 0),
                        ),
                        row=j + 1,
                        col=1,
                    )

            fig_joint.update_xaxes(title_text="t (s)", row=8, col=1)
            for r in range(1, 9):
                fig_joint.update_yaxes(title_text="rad/step", row=r, col=1)
            fig_joint.update_layout(title="关节角变化/抖动趋势（|Δq|）", height=1400, legend=dict(orientation="h"))
    except Exception:
        fig_joint = None

    # s-parameterized alignment: x(s), y(s), z(s) for XR vs EE (per run)
    fig_s = None
    try:
        fig_s = make_subplots(
            rows=3,
            cols=1,
            shared_xaxes=False,
            subplot_titles=(
                "x(s) 对齐：MPC vs IK",
                "y(s) 对齐：MPC vs IK",
                "z(s) 对齐：MPC vs IK",
            ),
        )

        def _add_run(run: RunData, *, color_ref: str, color_ee: str, dash_ref: str = "dash") -> None:
            s_ref = _arc_length_s(run.xr_p)
            s_ee = _arc_length_s(run.ee_p)
            fig_s.add_trace(go.Scatter(x=s_ref, y=run.xr_p[:, 0], name=f"{run.name} XR", line=dict(color=color_ref, dash=dash_ref)), row=1, col=1)
            fig_s.add_trace(go.Scatter(x=s_ee, y=run.ee_p[:, 0], name=f"{run.name} EE", line=dict(color=color_ee)), row=1, col=1)
            fig_s.add_trace(go.Scatter(x=s_ref, y=run.xr_p[:, 1], name=f"{run.name} XR", line=dict(color=color_ref, dash=dash_ref), showlegend=False), row=2, col=1)
            fig_s.add_trace(go.Scatter(x=s_ee, y=run.ee_p[:, 1], name=f"{run.name} EE", line=dict(color=color_ee), showlegend=False), row=2, col=1)
            fig_s.add_trace(go.Scatter(x=s_ref, y=run.xr_p[:, 2], name=f"{run.name} XR", line=dict(color=color_ref, dash=dash_ref), showlegend=False), row=3, col=1)
            fig_s.add_trace(go.Scatter(x=s_ee, y=run.ee_p[:, 2], name=f"{run.name} EE", line=dict(color=color_ee), showlegend=False), row=3, col=1)

        # Put MPC and IK on the same axes for direct comparison
        _add_run(mpc, color_ref="#888888", color_ee="#1f77b4")
        _add_run(ik, color_ref="#aaaaaa", color_ee="#d62728")

        for r, axis_name in [(1, "x"), (2, "y"), (3, "z")]:
            fig_s.update_yaxes(title_text=f"{axis_name} (m)", row=r, col=1)
        fig_s.update_xaxes(title_text="s (normalized arc length)", row=3, col=1)
        fig_s.update_layout(title="基于弧长参数 s 的对齐对比（MPC vs IK）", height=900, legend=dict(orientation="h"))
    except Exception:
        fig_s = None

    html_parts = []
    html_parts.append(f"<h1>{title}</h1>")
    html_parts.append("<h2>实验元信息</h2>")
    html_parts.append("<pre>" + json.dumps({"mpc": mpc.meta, "ik": ik.meta}, ensure_ascii=False, indent=2) + "</pre>")

    html_parts.append("<h2>3D轨迹</h2>")
    html_parts.append(fig_traj.to_html(full_html=False, include_plotlyjs="cdn"))

    html_parts.append("<h2>误差时间序列</h2>")
    html_parts.append(fig_err.to_html(full_html=False, include_plotlyjs=False))

    html_parts.append("<h2>误差分布</h2>")
    html_parts.append(fig_dist.to_html(full_html=False, include_plotlyjs=False))

    html_parts.append("<h2>统计指标</h2>")
    html_parts.append(fig_table.to_html(full_html=False, include_plotlyjs=False))

    if fig_s is not None:
        html_parts.append("<h2>弧长参数对齐：x(s)/y(s)/z(s)</h2>")
        html_parts.append("<p>说明：各轨迹分别按自身弧长归一化得到 s∈[0,1]，用于在采样点数/速度不同的情况下比较轨迹形状相似度（XR参考 vs EE实测）。</p>")
        html_parts.append(fig_s.to_html(full_html=False, include_plotlyjs=False))

    if fig_joint is not None:
        html_parts.append("<h2>关节抖动趋势</h2>")
        html_parts.append("<p>说明：使用 q_meas8 计算逐步差分幅值 |Δq|（rad/step），用于观察关节角变化/抖动趋势。</p>")
        html_parts.append(fig_joint.to_html(full_html=False, include_plotlyjs=False))

    out_html.parent.mkdir(parents=True, exist_ok=True)
    out_html.write_text("\n".join(html_parts), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mpc", default="", help="MPC实验 npz 路径（可选；为空则自动选取 teleop_compare_logs 下最新的 mpc 文件）")
    ap.add_argument("--ik", default="", help="IK实验 npz 路径（可选；为空则自动选取 teleop_compare_logs 下最新的 ik 文件）")
    ap.add_argument("--log-dir", default="./teleop/teleop_compare_logs", help="默认日志目录（用于自动选择最新 mpc/ik npz）")
    ap.add_argument("--out", default="./teleop_compare_report.html", help="输出 HTML 路径")
    ap.add_argument("--title", default="Teleop MPC vs IK 对比报告", help="报告标题")
    args = ap.parse_args()

    def _pick_latest(pattern: str) -> str:
        log_dir = Path(str(args.log_dir))
        cands = sorted(log_dir.glob(pattern), key=lambda p: p.stat().st_mtime)
        return str(cands[-1]) if cands else ""

    mpc_path = str(args.mpc).strip() or _pick_latest("teleop_compare_mpc_*.npz")
    ik_path = str(args.ik).strip() or _pick_latest("teleop_compare_ik_*.npz")

    if not mpc_path:
        raise FileNotFoundError(f"未找到 MPC compare 文件：{args.log_dir}/teleop_compare_mpc_*.npz")
    if not ik_path:
        raise FileNotFoundError(f"未找到 IK compare 文件：{args.log_dir}/teleop_compare_ik_*.npz")

    mpc = _to_run(_load_npz(mpc_path), name="MPC")
    ik = _to_run(_load_npz(ik_path), name="IK")

    build_report(mpc=mpc, ik=ik, out_html=Path(args.out), title=str(args.title))


if __name__ == "__main__":
    main()
