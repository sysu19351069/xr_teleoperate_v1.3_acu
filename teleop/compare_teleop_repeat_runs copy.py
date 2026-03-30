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
    if "meta" in out:
        try:
            out["meta"] = out["meta"].item()
        except Exception:
            pass
    return out


def _rmse(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float).reshape(-1)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(x))))


def _mae(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float).reshape(-1)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.mean(np.abs(x)))


def _p95(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float).reshape(-1)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.quantile(x, 0.95))


def _finite(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float).reshape(-1)
    return x[np.isfinite(x)]


def _joint_jitter_series(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[0] < 2:
        return np.zeros((0, q.shape[1] if q.ndim == 2 else 8), dtype=float)
    return np.abs(np.diff(q, axis=0))


def _arc_length_s(p: np.ndarray) -> np.ndarray:
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


@dataclass
class RunData:
    name: str
    meta: dict[str, Any]
    t: np.ndarray
    ee_p: np.ndarray
    xr_p: np.ndarray
    err_pos: np.ndarray
    err_ori: np.ndarray
    q_meas8: np.ndarray | None = None
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

    q_meas8 = None
    if "q_meas8" in d:
        try:
            q_meas8 = np.asarray(d["q_meas8"], dtype=float).reshape(-1, 8)
        except Exception:
            q_meas8 = None

    def _opt_1d(k: str):
        if k not in d:
            return None
        try:
            return np.asarray(d[k], dtype=float).reshape(-1)
        except Exception:
            return None

    return RunData(
        name=name,
        meta=meta,
        t=np.asarray(d["t"], dtype=float).reshape(-1),
        ee_p=np.asarray(d["ee_p"], dtype=float).reshape(-1, 3),
        xr_p=np.asarray(d["xr_p"], dtype=float).reshape(-1, 3),
        err_pos=np.asarray(d["err_pos"], dtype=float).reshape(-1),
        err_ori=np.asarray(d["err_ori"], dtype=float).reshape(-1),
        q_meas8=q_meas8,
        err_proj=_opt_1d("err_proj"),
        err_des=_opt_1d("err_des"),
        err_end_pos=_opt_1d("err_end_pos"),
    )


def _trim_tail_by_z_stability(run: RunData, z_eps: float = 1e-5, win: int = 20) -> RunData:
    """
    可选：自动去掉尾部长时间静止段，避免结束后悬停/停机段拉长图像。
    默认比较保守；如果检测不到稳定尾段，就不裁剪。
    """
    if run.ee_p.shape[0] < 3 * win:
        return run

    z = run.ee_p[:, 2]
    dz = np.abs(np.diff(z))
    tail = dz[-win:]
    if tail.size == 0 or np.nanmean(tail) > z_eps:
        return run

    # 从尾部向前找第一个“重新开始明显变化”的位置
    cut_idx = len(z)
    for i in range(len(dz) - win - 1, 0, -1):
        if np.nanmean(dz[max(0, i - win):i]) > z_eps * 5:
            cut_idx = i + 1
            break

    if cut_idx >= len(z) - win:
        return run

    def _cut(arr, ndim=1):
        if arr is None:
            return None
        return arr[:cut_idx] if ndim == 1 else arr[:cut_idx, :]

    return RunData(
        name=run.name,
        meta=dict(run.meta),
        t=run.t[:cut_idx],
        ee_p=run.ee_p[:cut_idx, :],
        xr_p=run.xr_p[:cut_idx, :],
        err_pos=run.err_pos[:cut_idx],
        err_ori=run.err_ori[:cut_idx],
        q_meas8=_cut(run.q_meas8, ndim=2),
        err_proj=_cut(run.err_proj),
        err_des=_cut(run.err_des),
        err_end_pos=_cut(run.err_end_pos),
    )


def build_report(*, run1: RunData, run2: RunData, out_html: Path, title: str) -> None:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    t1 = run1.t0_aligned()
    t2 = run2.t0_aligned()

    # 位置误差：优先用 err_proj，其次 err_des / err_pos
    pos1 = run1.err_proj if run1.err_proj is not None else (run1.err_des if run1.err_des is not None else run1.err_pos)
    pos2 = run2.err_proj if run2.err_proj is not None else (run2.err_des if run2.err_des is not None else run2.err_pos)

    # 1) 3D轨迹
    fig_traj = go.Figure()
    fig_traj.add_trace(go.Scatter3d(
        x=run1.xr_p[:, 0], y=run1.xr_p[:, 1], z=run1.xr_p[:, 2],
        mode="lines", name="XR ref - Run 1",
        line=dict(width=4, color="#999999"), opacity=0.55
    ))
    fig_traj.add_trace(go.Scatter3d(
        x=run1.ee_p[:, 0], y=run1.ee_p[:, 1], z=run1.ee_p[:, 2],
        mode="lines", name="EE meas - Run 1",
        line=dict(width=5, color="#1f77b4")
    ))
    fig_traj.add_trace(go.Scatter3d(
        x=run2.xr_p[:, 0], y=run2.xr_p[:, 1], z=run2.xr_p[:, 2],
        mode="lines", name="XR ref - Run 2",
        line=dict(width=4, color="#bbbbbb"), opacity=0.55
    ))
    fig_traj.add_trace(go.Scatter3d(
        x=run2.ee_p[:, 0], y=run2.ee_p[:, 1], z=run2.ee_p[:, 2],
        mode="lines", name="EE meas - Run 2",
        line=dict(width=5, color="#d62728")
    ))
    fig_traj.update_layout(
        title="两次重复实验 3D 轨迹展示",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(orientation="h"),
        margin=dict(l=0, r=0, t=50, b=0),
    )

    # 2) 误差时间序列
    fig_err = make_subplots(
        rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.12,
        subplot_titles=("位置误差（t 从 0 对齐）", "姿态误差角距离 (deg)")
    )
    fig_err.add_trace(go.Scatter(x=t1, y=pos1, name="Run 1", line=dict(color="#1f77b4")), row=1, col=1)
    fig_err.add_trace(go.Scatter(x=t2, y=pos2, name="Run 2", line=dict(color="#d62728")), row=1, col=1)

    fig_err.add_trace(go.Scatter(x=t1, y=np.rad2deg(run1.err_ori), name="Run 1 ori", line=dict(color="#1f77b4"), showlegend=False), row=2, col=1)
    fig_err.add_trace(go.Scatter(x=t2, y=np.rad2deg(run2.err_ori), name="Run 2 ori", line=dict(color="#d62728"), showlegend=False), row=2, col=1)

    fig_err.update_yaxes(title_text="位置误差 (m)", row=1, col=1)
    fig_err.update_yaxes(title_text="姿态误差 (deg)", row=2, col=1)
    fig_err.update_xaxes(title_text="t (s)", row=2, col=1)
    fig_err.update_layout(title="两次重复实验误差时间历程", legend=dict(orientation="h"), height=650)

    # 3) 分布对比
    fig_dist = make_subplots(rows=1, cols=2, subplot_titles=("位置误差分布 (m)", "姿态误差分布 (deg)"))
    fig_dist.add_trace(go.Box(y=pos1, name="Run 1", marker_color="#1f77b4"), row=1, col=1)
    fig_dist.add_trace(go.Box(y=pos2, name="Run 2", marker_color="#d62728"), row=1, col=1)
    fig_dist.add_trace(go.Box(y=np.rad2deg(run1.err_ori), name="Run 1", marker_color="#1f77b4", showlegend=False), row=1, col=2)
    fig_dist.add_trace(go.Box(y=np.rad2deg(run2.err_ori), name="Run 2", marker_color="#d62728", showlegend=False), row=1, col=2)
    fig_dist.update_layout(title="两次重复实验误差分布", height=450)

    # 4) 统计表
    pos1_f = _finite(pos1)
    pos2_f = _finite(pos2)
    ori1_f = _finite(np.rad2deg(run1.err_ori))
    ori2_f = _finite(np.rad2deg(run2.err_ori))

    stats = {
        "metric": ["RMSE", "MAE", "P95", "MAX"],
        "Run1_pos(m)": [
            _rmse(pos1_f), _mae(pos1_f), _p95(pos1_f),
            float(np.max(pos1_f)) if pos1_f.size else float("nan")
        ],
        "Run2_pos(m)": [
            _rmse(pos2_f), _mae(pos2_f), _p95(pos2_f),
            float(np.max(pos2_f)) if pos2_f.size else float("nan")
        ],
        "Run1_ori(deg)": [
            _rmse(ori1_f), _mae(ori1_f), _p95(ori1_f),
            float(np.max(ori1_f)) if ori1_f.size else float("nan")
        ],
        "Run2_ori(deg)": [
            _rmse(ori2_f), _mae(ori2_f), _p95(ori2_f),
            float(np.max(ori2_f)) if ori2_f.size else float("nan")
        ],
    }

    fig_table = go.Figure(data=[go.Table(
        header=dict(values=list(stats.keys()), fill_color="#eeeeee", align="left"),
        cells=dict(values=list(stats.values()), align="left"),
    )])
    fig_table.update_layout(title="重复实验统计指标")

    # 5) 弧长参数对齐
    fig_s = make_subplots(
        rows=1, cols=3,
        subplot_titles=("x(s) 对齐", "y(s) 对齐", "z(s) 对齐")
    )

    def _add_run_s(run: RunData, ref_color: str, ee_color: str):
        s_ref = _arc_length_s(run.xr_p)
        s_ee = _arc_length_s(run.ee_p)

        fig_s.add_trace(go.Scatter(x=s_ref, y=run.xr_p[:, 0], name=f"{run.name} XR", line=dict(color=ref_color, dash="dash")), row=1, col=1)
        fig_s.add_trace(go.Scatter(x=s_ee, y=run.ee_p[:, 0], name=f"{run.name} EE", line=dict(color=ee_color)), row=1, col=1)

        fig_s.add_trace(go.Scatter(x=s_ref, y=run.xr_p[:, 1], name=f"{run.name} XR", line=dict(color=ref_color, dash="dash"), showlegend=False), row=1, col=2)
        fig_s.add_trace(go.Scatter(x=s_ee, y=run.ee_p[:, 1], name=f"{run.name} EE", line=dict(color=ee_color), showlegend=False), row=1, col=2)

        fig_s.add_trace(go.Scatter(x=s_ref, y=run.xr_p[:, 2], name=f"{run.name} XR", line=dict(color=ref_color, dash="dash"), showlegend=False), row=1, col=3)
        fig_s.add_trace(go.Scatter(x=s_ee, y=run.ee_p[:, 2], name=f"{run.name} EE", line=dict(color=ee_color), showlegend=False), row=1, col=3)

    _add_run_s(run1, ref_color="#999999", ee_color="#1f77b4")
    _add_run_s(run2, ref_color="#bbbbbb", ee_color="#d62728")

    fig_s.update_yaxes(title_text="x (m)", row=1, col=1)
    fig_s.update_yaxes(title_text="y (m)", row=1, col=2)
    fig_s.update_yaxes(title_text="z (m)", row=1, col=3)
    fig_s.update_xaxes(title_text="s", row=1, col=1)
    fig_s.update_xaxes(title_text="s", row=1, col=2)
    fig_s.update_xaxes(title_text="s", row=1, col=3)
    fig_s.update_layout(
        title="基于归一化弧长参数的轨迹形状对齐",
        height=420,
        legend=dict(orientation="h", x=0.0, y=-0.25, xanchor="left", yanchor="top"),
        margin=dict(b=110),
    )

    # 6) 关节抖动
    fig_joint = None
    try:
        if (run1.q_meas8 is not None) or (run2.q_meas8 is not None):
            joint_names = [f"J{i}" for i in range(8)]
            n_rows, n_cols = 4, 2
            subplot_titles = [f"|Δq| - {jn}" for jn in joint_names]

            fig_joint = make_subplots(
                rows=n_rows, cols=n_cols,
                shared_xaxes=True, vertical_spacing=0.06, horizontal_spacing=0.07,
                subplot_titles=subplot_titles,
            )

            def rc(j: int):
                return (j // n_cols) + 1, (j % n_cols) + 1

            if run1.q_meas8 is not None:
                dq1 = _joint_jitter_series(run1.q_meas8)
                tt1 = t1[1:1 + dq1.shape[0]] if t1.size >= 2 else np.arange(dq1.shape[0])
                for j in range(min(8, dq1.shape[1])):
                    r, c = rc(j)
                    fig_joint.add_trace(go.Scatter(
                        x=tt1, y=dq1[:, j], name=f"Run1 {joint_names[j]}",
                        line=dict(color="#1f77b4"), showlegend=(j == 0)
                    ), row=r, col=c)

            if run2.q_meas8 is not None:
                dq2 = _joint_jitter_series(run2.q_meas8)
                tt2 = t2[1:1 + dq2.shape[0]] if t2.size >= 2 else np.arange(dq2.shape[0])
                for j in range(min(8, dq2.shape[1])):
                    r, c = rc(j)
                    fig_joint.add_trace(go.Scatter(
                        x=tt2, y=dq2[:, j], name=f"Run2 {joint_names[j]}",
                        line=dict(color="#d62728"), showlegend=(j == 0)
                    ), row=r, col=c)

            for c in range(1, n_cols + 1):
                fig_joint.update_xaxes(title_text="t (s)", row=n_rows, col=c)
            for r in range(1, n_rows + 1):
                for c in range(1, n_cols + 1):
                    fig_joint.update_yaxes(title_text="rad/step", row=r, col=c)

            fig_joint.update_layout(
                title="两次重复实验关节变化/抖动趋势（|Δq|）",
                height=900,
                legend=dict(orientation="h"),
                margin=dict(t=80),
            )
    except Exception:
        fig_joint = None

    html_parts = []
    html_parts.append(
        """
<style>
  :root { --page-width: 210mm; }
  body { margin: 0; padding: 0; font-family: sans-serif; }
  .page { max-width: var(--page-width); margin: 0 auto; padding: 10mm 8mm; box-sizing: border-box; }
  pre { white-space: pre-wrap; word-break: break-word; }
  .plotly-graph-div { width: 100% !important; }
  @media print { .page { padding: 0; } }
</style>
        """.strip()
    )
    html_parts.append("<div class='page'>")
    html_parts.append(f"<h1>{title}</h1>")
    html_parts.append("<h2>实验元信息</h2>")
    html_parts.append("<pre>" + json.dumps({"run1": run1.meta, "run2": run2.meta}, ensure_ascii=False, indent=2) + "</pre>")
    html_parts.append("<h2>3D轨迹</h2>")
    html_parts.append(fig_traj.to_html(full_html=False, include_plotlyjs="cdn"))
    html_parts.append("<h2>弧长参数对齐</h2>")
    html_parts.append("<p>用于在两次实验采样点数与速度不一致时，对比轨迹整体形状一致性。</p>")
    html_parts.append(fig_s.to_html(full_html=False, include_plotlyjs=False))
    html_parts.append("<h2>误差时间序列</h2>")
    html_parts.append(fig_err.to_html(full_html=False, include_plotlyjs=False))
    html_parts.append("<h2>误差分布</h2>")
    html_parts.append(fig_dist.to_html(full_html=False, include_plotlyjs=False))
    html_parts.append("<h2>统计指标</h2>")
    html_parts.append(fig_table.to_html(full_html=False, include_plotlyjs=False))
    if fig_joint is not None:
        html_parts.append("<h2>关节抖动趋势</h2>")
        html_parts.append("<p>使用 q_meas8 的逐步差分幅值 |Δq|，用于展示两次实机重复实验的动作平滑性与稳定性。</p>")
        html_parts.append(fig_joint.to_html(full_html=False, include_plotlyjs=False))
    html_parts.append("</div>")

    out_html.parent.mkdir(parents=True, exist_ok=True)
    out_html.write_text("\n".join(html_parts), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--run1",
        type=str,
        default="./teleop_compare_logs_experiment/teleop_compare_mpc_20260311_225531.npz",
        help="第1次实验 npz 路径"
    )
    ap.add_argument(
        "--run2",
        type=str,
        default="./teleop_compare_logs_experiment/teleop_compare_mpc_20260311_210951.npz",
        help="第2次实验 npz 路径"
    )
    ap.add_argument("--log-dir", default="./teleop_logs_experiment_report", help="实验日志目录")
    ap.add_argument("--out", default="./teleop_logs_experiment_report/repeat_runs_report.html", help="输出 HTML 路径")
    ap.add_argument("--title", default="Preneedle Teleoperation Repeated Experiments Report", help="报告标题")
    ap.add_argument("--auto-trim-tail", action="store_true", help="自动裁剪末尾静止段")
    args = ap.parse_args()

    def _pick_two_latest():
        log_dir = Path(args.log_dir)
        cands = sorted(log_dir.glob("teleop_compare_*.npz"), key=lambda p: p.stat().st_mtime)
        if len(cands) < 2:
            raise FileNotFoundError(f"在 {log_dir} 下少于两个 teleop_compare_*.npz 文件")
        return str(cands[-2]), str(cands[-1])

    run1_path = str(args.run1).strip()
    run2_path = str(args.run2).strip()

    if not Path(run1_path).exists():
        raise FileNotFoundError(f"找不到 --run1 指定文件: {run1_path}")
    if not Path(run2_path).exists():
        raise FileNotFoundError(f"找不到 --run2 指定文件: {run2_path}")

    run1 = _to_run(_load_npz(run1_path), name="Run 1")
    run2 = _to_run(_load_npz(run2_path), name="Run 2")

    if args.auto_trim_tail:
        run1 = _trim_tail_by_z_stability(run1)
        run2 = _trim_tail_by_z_stability(run2)

    build_report(run1=run1, run2=run2, out_html=Path(args.out), title=str(args.title))
    print(f"[OK] report saved to: {args.out}")


if __name__ == "__main__":
    main()