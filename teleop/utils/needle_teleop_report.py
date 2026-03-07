"""针刺遥操（提插+捻转）自适应阻抗力控：CSV 离线可视化报告

读取 acu_adaptive_impedance.py 在运行结束时落盘的 CSV（默认与 summary PNG 同名），
生成可用浏览器打开的 HTML 报告（Plotly 离线）。

覆盖内容：
1) 自适应阻抗力控下的提插参考跟踪展示
2) 自适应门控参数变化（mode、hysteresis_current、|dF/dt|）
3) 捻转参考/速度展示

用法示例：
  python teleop/utils/needle_teleop_report.py \
    --csv teleop/acu_adaptive_impedance_retargeting_summary.csv \
    --out teleop/needle_teleop_report.html

也支持多个 --csv 输入，用于多次运行对比；每个运行将作为一组 traces 加入同一报告。
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np


def _as_float(x: Any, default: float = float('nan')) -> float:
    try:
        return float(x)
    except Exception:
        return float(default)


def _finite(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float).reshape(-1)
    return x[np.isfinite(x)]


def _rmse(x: np.ndarray) -> float:
    x = _finite(x)
    if x.size == 0:
        return float('nan')
    return float(np.sqrt(np.mean(np.square(x))))


def _mae(x: np.ndarray) -> float:
    x = _finite(x)
    if x.size == 0:
        return float('nan')
    return float(np.mean(np.abs(x)))


def _p95(x: np.ndarray) -> float:
    x = _finite(x)
    if x.size == 0:
        return float('nan')
    return float(np.quantile(x, 0.95))


@dataclass
class NeedleRun:
    name: str
    path: Path
    columns: List[str]
    data: Dict[str, np.ndarray]

    def t0(self) -> np.ndarray:
        t = self.data.get('t', np.zeros((0,), dtype=float))
        if t.size == 0:
            return t
        return t - float(t[0])


def load_csv(path: Path, *, name: Optional[str] = None) -> NeedleRun:
    rows: List[Dict[str, str]] = []
    with path.open('r', newline='') as f:
        r = csv.DictReader(f)
        cols = list(r.fieldnames or [])
        for row in r:
            rows.append(row)

    if not rows:
        return NeedleRun(name=name or path.stem, path=path, columns=cols, data={c: np.zeros((0,), dtype=float) for c in cols})

    data: Dict[str, np.ndarray] = {}
    for c in cols:
        data[c] = np.array([_as_float(rr.get(c)) for rr in rows], dtype=float)

    return NeedleRun(name=name or path.stem, path=path, columns=cols, data=data)


def _require(run: NeedleRun, key: str) -> np.ndarray:
    if key not in run.data:
        return np.full((run.t0().shape[0],), np.nan, dtype=float)
    return np.asarray(run.data[key], dtype=float).reshape(-1)


def build_report(*, runs: Sequence[NeedleRun], out_html: Path, title: str) -> None:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    # Unit conventions (based on acu_adaptive_impedance.py logging):
    #   - ins_ref/x_cmd/xm: insertion position after *1e3 => mm
    #   - ins_vel_ref: after *1e3 => mm/s
    #   - pm, f_safe, dF: N
    #   - dfdt_abs: N/s
    #   - ke_hat/be_hat/xe_hat: inferred in mm-domain (Ke in N/mm, Be in N*s/mm, xe in mm)
    #   - tws_ref: rad, tws_vel_ref: rad/s

    # ---- Figure 1: insertion tracking ----
    fig1 = go.Figure()
    for run in runs:
        t = run.t0()
        fig1.add_trace(go.Scatter(x=t, y=_require(run, 'ins_ref'), name=f"{run.name}: ins_ref"))
        fig1.add_trace(go.Scatter(x=t, y=_require(run, 'x_cmd'), name=f"{run.name}: x_cmd"))
        fig1.add_trace(go.Scatter(x=t, y=_require(run, 'xm'), name=f"{run.name}: x_m"))

    fig1.update_layout(
        title='提插参考跟踪（ins_ref / x_cmd / x_m）',
        xaxis_title='t (s)',
        yaxis_title='Insertion / joint pos (mm)',
        legend=dict(orientation='h'),
        height=520,
    )

    # ---- Figure 1b: insertion velocity ref + commanded velocity ----
    fig1b = go.Figure()
    for run in runs:
        t = run.t0()
        fig1b.add_trace(go.Scatter(x=t, y=_require(run, 'ins_vel_ref'), name=f"{run.name}: ins_vel_ref"))
        fig1b.add_trace(go.Scatter(x=t, y=_require(run, 'v_cmd'), name=f"{run.name}: v_cmd"))
    fig1b.update_layout(
        title='提插速度：参考与控制输出（ins_vel_ref / v_cmd）',
        xaxis_title='t (s)',
        yaxis_title='Velocity (mm/s)',
        legend=dict(orientation='h'),
        height=420,
    )

    # ---- Figure 2: force tracking ----
    fig_force = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.06,
                              subplot_titles=('接触力：pm 与 f_safe', '力误差：dF = F_r - F_e'))
    for run in runs:
        t = run.t0()
        pm = _require(run, 'pm')
        f_safe = _require(run, 'f_safe')
        dF = _require(run, 'dF')
        fig_force.add_trace(go.Scatter(x=t, y=pm, name=f"{run.name}: pm"), row=1, col=1)
        # f_safe is constant but logged per-sample; plot as line
        fig_force.add_trace(go.Scatter(x=t, y=f_safe, name=f"{run.name}: f_safe", line=dict(dash='dash')), row=1, col=1)
        fig_force.add_trace(go.Scatter(x=t, y=dF, name=f"{run.name}: dF"), row=2, col=1)

    fig_force.update_yaxes(title_text='Force (N)', row=1, col=1)
    fig_force.update_yaxes(title_text='Force error (N)', row=2, col=1)
    fig_force.update_xaxes(title_text='t (s)', row=2, col=1)
    fig_force.update_layout(title='力控相关曲线', legend=dict(orientation='h'), height=650)

    # ---- Figure 3: gating signals ----
    fig2 = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.03,
        subplot_titles=(
            'mode_impedance (0/1)',
            'hysteresis_current (N)',
            '|dF/dt| = dfdt_abs (N/s)',
        ),
    )
    for run in runs:
        t = run.t0()
        fig2.add_trace(go.Scatter(x=t, y=_require(run, 'mode_impedance'), name=f"{run.name}: mode"), row=1, col=1)
        fig2.add_trace(go.Scatter(x=t, y=_require(run, 'hysteresis_current'), name=f"{run.name}: hyst"), row=2, col=1)
        fig2.add_trace(go.Scatter(x=t, y=_require(run, 'dfdt_abs'), name=f"{run.name}: dfdt"), row=3, col=1)

    fig2.update_yaxes(title_text='-', row=1, col=1)
    fig2.update_yaxes(title_text='N', row=2, col=1)
    fig2.update_yaxes(title_text='N/s', row=3, col=1)
    fig2.update_xaxes(title_text='t (s)', row=3, col=1)
    fig2.update_layout(title='门控与自适应迟滞', legend=dict(orientation='h'), height=820)

    # ---- Figure 4: adaptive env params ----
    fig3 = make_subplots(rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.03,
                         subplot_titles=('K_e_hat (N/mm)', 'B_e_hat (N·s/mm)', 'x_e_hat (mm)'))
    for run in runs:
        t = run.t0()
        fig3.add_trace(go.Scatter(x=t, y=_require(run, 'ke_hat'), name=f"{run.name}: Ke"), row=1, col=1)
        fig3.add_trace(go.Scatter(x=t, y=_require(run, 'be_hat'), name=f"{run.name}: Be"), row=2, col=1)
        fig3.add_trace(go.Scatter(x=t, y=_require(run, 'xe_hat'), name=f"{run.name}: xe"), row=3, col=1)
    fig3.update_yaxes(title_text='N/mm', row=1, col=1)
    fig3.update_yaxes(title_text='N·s/mm', row=2, col=1)
    fig3.update_yaxes(title_text='mm', row=3, col=1)
    fig3.update_xaxes(title_text='t (s)', row=3, col=1)
    fig3.update_layout(title='环境参数自适应估计', legend=dict(orientation='h'), height=820)

    # ---- Figure 5: twist ref/vel + measured ----
    fig4 = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.03,
        subplot_titles=(
            'twist angle: ref vs meas (rad)',
            'twist velocity: ref vs meas (rad/s)',
        ),
    )
    for run in runs:
        t = run.t0()
        fig4.add_trace(go.Scatter(x=t, y=_require(run, 'tws_ref'), name=f"{run.name}: tws_ref"), row=1, col=1)
        fig4.add_trace(go.Scatter(x=t, y=_require(run, 'tws_meas'), name=f"{run.name}: tws_meas"), row=1, col=1)
        fig4.add_trace(go.Scatter(x=t, y=_require(run, 'tws_vel_ref'), name=f"{run.name}: tws_vel_ref"), row=2, col=1)
        fig4.add_trace(go.Scatter(x=t, y=_require(run, 'tws_meas_vel'), name=f"{run.name}: tws_meas_vel"), row=2, col=1)

    fig4.update_yaxes(title_text='rad', row=1, col=1)
    fig4.update_yaxes(title_text='rad/s', row=2, col=1)
    fig4.update_xaxes(title_text='t (s)', row=2, col=1)
    fig4.update_layout(title='捻转：参考-测量对比', legend=dict(orientation='h'), height=750)

    # ---- Table: key stats ----
    stats_rows: List[List[Any]] = []
    for run in runs:
        e = _require(run, 'x_cmd') - _require(run, 'ins_ref')
        pm = _require(run, 'pm')
        mode = _require(run, 'mode_impedance')
        stats_rows.append([
            run.name,
            str(run.path),
            int(np.sum(np.isfinite(mode) & (mode > 0.5))),
            float(np.nanmax(run.t0())) if run.t0().size else float('nan'),
            _rmse(e),
            _mae(e),
            _p95(np.abs(e)),
            float(np.nanmax(pm)) if pm.size else float('nan'),
        ])

    fig_table = go.Figure(
        data=[
            go.Table(
                header=dict(
                    values=[
                        'run', 'csv_path', 'mode_impedance_samples', 'duration_s',
                        'RMSE(x_cmd-ins_ref) [mm]', 'MAE(x_cmd-ins_ref) [mm]', 'P95(|x_cmd-ins_ref|) [mm]', 'max(pm) [N]',
                    ],
                    fill_color='#eee',
                    align='left',
                ),
                cells=dict(values=list(map(list, zip(*stats_rows))) if stats_rows else [], align='left'),
            )
        ]
    )
    fig_table.update_layout(title='统计指标（用于报告摘要）', height=380)

    # ---- HTML assembly ----
    meta = {
        'title': title,
        'unit_conventions': {
            't': 's',
            'ins_ref/x_cmd/xm': 'mm (logged after *1e3 in acu_adaptive_impedance runner)',
            'ins_vel_ref/v_cmd': 'mm/s (logged after *1e3 in acu_adaptive_impedance runner)',
            'pm/f_safe/dF': 'N',
            'dfdt_abs': 'N/s',
            'ke_hat': 'N/mm (inferred; controller uses mm-domain signals)',
            'be_hat': 'N·s/mm (inferred)',
            'xe_hat': 'mm (inferred)',
            'tws_ref': 'rad',
            'tws_vel_ref': 'rad/s',
            'tws_meas': 'rad (robot joint8 q[7])',
            'tws_meas_vel': 'rad/s (finite diff of tws_meas)',
        },
        'runs': [{'name': r.name, 'path': str(r.path), 'columns': r.columns} for r in runs],
    }

    parts: List[str] = []
    parts.append(f'<h1>{title}</h1>')
    parts.append('<h2>输入数据与单位约定</h2>')
    parts.append('<p>说明：单位约定根据 acu_adaptive_impedance.py 的日志实现推断；若你后续统一为 SI(m) 记录，可在本脚本中同步更新。</p>')
    parts.append('<pre>' + json.dumps(meta, ensure_ascii=False, indent=2) + '</pre>')

    parts.append('<h2>1) 提插参考跟踪（mm）</h2>')
    parts.append(fig1.to_html(full_html=False, include_plotlyjs='cdn'))

    parts.append('<h2>1b) 提插速度：参考与控制输出（mm/s）</h2>')
    parts.append(fig1b.to_html(full_html=False, include_plotlyjs=False))

    parts.append('<h2>2) 力控相关曲线（N）</h2>')
    parts.append(fig_force.to_html(full_html=False, include_plotlyjs=False))

    parts.append('<h2>3) 自适应门控参数（mode、迟滞、|dF/dt|）</h2>')
    parts.append(fig2.to_html(full_html=False, include_plotlyjs=False))

    parts.append('<h2>4) 环境自适应参数（K_e_hat / B_e_hat / x_e_hat）</h2>')
    parts.append(fig3.to_html(full_html=False, include_plotlyjs=False))

    parts.append('<h2>5) 捻转参考（rad）与测量（rad）/速度（rad/s）对比</h2>')
    parts.append('<p>说明：tws_meas/tws_meas_vel 来自机器人当前关节 q[7] 的测量与差分估计；tws_ref/tws_vel_ref 来自手部重定向输出。</p>')
    parts.append(fig4.to_html(full_html=False, include_plotlyjs=False))

    parts.append('<h2>6) 统计摘要</h2>')
    parts.append(fig_table.to_html(full_html=False, include_plotlyjs=False))

    out_html.parent.mkdir(parents=True, exist_ok=True)
    out_html.write_text('\n'.join(parts), encoding='utf-8')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', action='append', default=[], help='输入 CSV 路径（可多次指定）。若为空则自动寻找 teleop/acu_adaptive_impedance_retargeting_summary.csv')
    ap.add_argument('--out', default='./teleop/needle_teleop_report.html', help='输出 HTML 路径')
    ap.add_argument('--title', default='针刺遥操自适应阻抗控制离线报告', help='报告标题')
    ap.add_argument('--name', action='append', default=[], help='每个 CSV 对应的运行名称（可选，数量需与 --csv 一致）')
    args = ap.parse_args()

    csv_paths = [Path(p) for p in (args.csv or []) if str(p).strip()]
    if not csv_paths:
        default = Path('./teleop/acu_adaptive_impedance_retargeting_summary.csv')
        if default.exists():
            csv_paths = [default]
        else:
            # also allow CWD-generated default from acu_adaptive_impedance.py
            alt = Path('./acu_adaptive_impedance_retargeting_summary.csv')
            if alt.exists():
                csv_paths = [alt]

    if not csv_paths:
        raise FileNotFoundError('未提供 --csv，且未找到默认 CSV：teleop/acu_adaptive_impedance_retargeting_summary.csv')

    names = list(args.name or [])
    runs: List[NeedleRun] = []
    for i, p in enumerate(csv_paths):
        if not p.exists():
            raise FileNotFoundError(str(p))
        nm = names[i] if i < len(names) and str(names[i]).strip() else p.stem
        runs.append(load_csv(p, name=nm))

    build_report(runs=runs, out_html=Path(args.out), title=str(args.title))


if __name__ == '__main__':
    main()
