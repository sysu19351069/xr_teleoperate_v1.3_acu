#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D
from matplotlib.patches import Arc
import matplotlib.patheffects as pe

XR_HAND_JOINT_THUMB_TIP = 4
XR_HAND_JOINT_INDEX_PROXIMAL = 6
XR_HAND_JOINT_INDEX_TIP = 9


def setup_chinese_font() -> FontProperties | None:
    candidates = [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    ]
    for path in candidates:
        if Path(path).exists():
            fp = FontProperties(fname=path)
            plt.rcParams['font.family'] = fp.get_name()
            plt.rcParams['axes.unicode_minus'] = False
            return fp
    plt.rcParams['axes.unicode_minus'] = False
    return None


def load_episode(json_path: str) -> np.ndarray:
    with open(json_path, 'r', encoding='utf-8') as f:
        obj = json.load(f)
    frames = obj['data']
    right_ee = []
    for fr in frames:
        ee_q = fr['states']['right_ee']['qpos']
        if len(ee_q) != 75:
            raise ValueError(f'right_ee.qpos 长度应为 75，实际为 {len(ee_q)}')
        right_ee.append(ee_q)
    return np.asarray(right_ee, dtype=float).reshape(-1, 25, 3)


def extract_vectors(hand: np.ndarray):
    n = hand.shape[0]
    v_index = np.zeros((n, 3), dtype=float)
    rel_xy = np.zeros((n, 2), dtype=float)
    is_new_input = np.zeros(n, dtype=bool)
    prev_key = None
    for i in range(n):
        idx_prox = hand[i, XR_HAND_JOINT_INDEX_PROXIMAL]
        idx_tip = hand[i, XR_HAND_JOINT_INDEX_TIP]
        th_tip = hand[i, XR_HAND_JOINT_THUMB_TIP]
        key = np.concatenate([idx_prox, idx_tip, th_tip]).round(6).tobytes()
        if prev_key is None or key != prev_key:
            is_new_input[i] = True
        prev_key = key
        v_index[i] = idx_tip - idx_prox
        rel_xy[i] = (th_tip - idx_tip)[:2]
    return v_index, rel_xy, is_new_input


def auto_active_window(v_index: np.ndarray, rel_xy: np.ndarray, is_new_input: np.ndarray):
    dv = np.linalg.norm(np.diff(v_index, axis=0, prepend=v_index[[0]]), axis=1)
    dr = np.linalg.norm(np.diff(rel_xy, axis=0, prepend=rel_xy[[0]]), axis=1)
    activity = (800.0 * dv + 1200.0 * dr) * is_new_input.astype(float)
    if np.all(activity <= 0):
        return 0, len(activity) - 1
    kernel = np.ones(11, dtype=float) / 11.0
    activity_s = np.convolve(activity, kernel, mode='same')
    thr = max(np.percentile(activity_s, 72.0), 0.10 * float(np.max(activity_s)))
    idx = np.where(activity_s >= thr)[0]
    if len(idx) == 0:
        return 0, len(activity_s) - 1
    start = max(0, int(idx[0]) - 20)
    end = min(len(activity_s) - 1, int(idx[-1]) + 20)
    return start, end


def pick_samples(start: int, end: int, is_new_input: np.ndarray, num_samples: int):
    valid = np.where(is_new_input[start:end + 1])[0] + start
    if len(valid) == 0:
        return np.linspace(start, end, num_samples, dtype=int)
    if len(valid) < num_samples:
        return valid
    pos = np.linspace(0, len(valid) - 1, num_samples)
    return valid[np.round(pos).astype(int)]


def signed_angle_2d(a: np.ndarray, b: np.ndarray, eps: float = 1e-12) -> float:
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


def draw_arrow_2d(ax, end, color='C0', lw=2.0, ls='-', alpha=1.0, zorder=3):
    ax.annotate(
        '', xy=(float(end[0]), float(end[1])), xytext=(0.0, 0.0),
        arrowprops=dict(arrowstyle='->', lw=lw, linestyle=ls, color=color,
                        alpha=alpha, shrinkA=0.0, shrinkB=0.0),
        zorder=zorder,
    )


def draw_arrow_3d(ax, end, color='C0', lw=2.0, alpha=1.0):
    ax.quiver(0.0, 0.0, 0.0,
              float(end[0]), float(end[1]), float(end[2]),
              arrow_length_ratio=0.12, color=color, linewidth=lw,
              alpha=alpha, normalize=False)


def set_equal_3d(ax, pts: np.ndarray, pad_ratio: float = 0.16):
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.5 * float(np.max(maxs - mins))
    radius *= (1.0 + pad_ratio)
    radius = max(radius, 1e-3)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def ensure_equal_xy(ax, x: np.ndarray, y: np.ndarray, pad_ratio: float = 0.16):
    xmin, xmax = float(np.min(x)), float(np.max(x))
    ymin, ymax = float(np.min(y)), float(np.max(y))
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    radius = 0.5 * max(xmax - xmin, ymax - ymin)
    radius *= (1.0 + pad_ratio)
    radius = max(radius, 1e-3)
    ax.set_xlim(cx - radius, cx + radius)
    ax.set_ylim(cy - radius, cy + radius)


def annotate_point_2d(ax, x, y, text, color, font_prop=None, offset=(6, 6), ha='left', va='bottom'):
    ax.annotate(
        text, xy=(float(x), float(y)), xytext=offset, textcoords='offset points',
        fontsize=9.5, color=color, ha=ha, va=va, fontproperties=font_prop,
        bbox=dict(boxstyle='round,pad=0.14', facecolor='white', edgecolor='none', alpha=0.76),
        zorder=9,
    )


def draw_small_vertical_direction(ax, x: float, z: float, sign_z: float, color, scale: float):
    dy = scale if sign_z >= 0.0 else -scale
    ann = ax.annotate(
        '', xy=(x, z + dy), xytext=(x, z),
        arrowprops=dict(arrowstyle='-|>', lw=2.4, linestyle='-', color=color,
                        alpha=0.98, mutation_scale=17, shrinkA=0.0, shrinkB=0.0),
        zorder=8,
    )
    if hasattr(ann, 'arrow_patch') and ann.arrow_patch is not None:
        ann.arrow_patch.set_path_effects([pe.Stroke(linewidth=4.0, foreground='white'), pe.Normal()])


def draw_arc_arrow(ax, theta0: float, theta1: float, radius: float, color, lw: float = 1.4, alpha: float = 0.95):
    dtheta = signed_angle_2d([np.cos(theta0), np.sin(theta0)], [np.cos(theta1), np.sin(theta1)])
    if abs(dtheta) < 1e-9:
        return
    t1 = theta0
    t2 = theta0 + dtheta
    theta_deg1 = np.degrees(min(t1, t2))
    theta_deg2 = np.degrees(max(t1, t2))
    arc = Arc((0.0, 0.0), 2.0 * radius, 2.0 * radius, angle=0.0,
              theta1=theta_deg1, theta2=theta_deg2, lw=lw, color=color, alpha=alpha, zorder=1)
    ax.add_patch(arc)
    sgn = 1.0 if dtheta > 0 else -1.0
    tip = np.array([radius * np.cos(t2), radius * np.sin(t2)])
    tan = np.array([-np.sin(t2), np.cos(t2)]) * sgn
    start = tip - 0.22 * radius * tan
    ax.annotate('', xy=(tip[0], tip[1]), xytext=(start[0], start[1]),
                arrowprops=dict(arrowstyle='-|>', lw=lw, color=color,
                                alpha=alpha, mutation_scale=10, shrinkA=0.0, shrinkB=0.0),
                zorder=2)


def plot_method_ab(out_path: str, v_index: np.ndarray, rel_xy: np.ndarray, is_new_input: np.ndarray,
                   start: int, end: int, fps: float, num_samples: int = 7):
    font_prop = setup_chinese_font()

    nonzero = np.where(np.linalg.norm(v_index, axis=1) > 0)[0]
    if len(nonzero) == 0:
        raise RuntimeError('未找到有效的食指关键向量')
    i0 = int(nonzero[0])
    v0 = v_index[i0].copy()
    r0 = rel_xy[i0].copy()

    sample_ids = pick_samples(start, end, is_new_input, num_samples=num_samples)
    sample_colors = cm.viridis(np.linspace(0.05, 0.95, len(sample_ids)))
    v_samples = v_index[sample_ids]
    r_samples = rel_xy[sample_ids]
    z0 = float(v0[2])

    # 图中提插方向用“相邻采样向量”的 z 差分表示，与编号 1..N 对应更直观
    sample_dvz_sign = []
    for k, i in enumerate(sample_ids):
        j = i0 if k == 0 else int(sample_ids[k - 1])
        sample_dvz_sign.append(float(v_index[i, 2] - v_index[j, 2]))

    fig = plt.figure(figsize=(15.8, 6.1), dpi=180)
    gs = fig.add_gridspec(1, 3, width_ratios=[1.18, 1.10, 1.14],
                          left=0.04, right=0.985, top=0.79, bottom=0.20, wspace=0.36)
    ax3d = fig.add_subplot(gs[0, 0], projection='3d')
    ax_xz = fig.add_subplot(gs[0, 1])
    ax_xy = fig.add_subplot(gs[0, 2])

    # (a) 3D
    draw_arrow_3d(ax3d, v0, color='black', lw=3.0, alpha=0.98)
    ax3d.scatter([v0[0]], [v0[1]], [v0[2]], color='black', s=22, zorder=5)
    ax3d.text(v0[0], v0[1], v0[2], ' 初始', fontsize=10, color='black', fontproperties=font_prop)
    offsets_3d = {1:(0.0008,0.0005,0.0010),2:(0.0010,0.0004,-0.0012),3:(0.0010,0.0006,0.0010),4:(0.0012,0.0004,-0.0010),5:(0.0008,0.0007,0.0008),6:(0.0008,0.0007,0.0008),7:(0.0010,0.0006,0.0008)}
    for k, (i, c) in enumerate(zip(sample_ids, sample_colors), start=1):
        vt = v_index[i]
        draw_arrow_3d(ax3d, vt, color=c, lw=2.5, alpha=0.98)
        ax3d.plot([v0[0], vt[0]], [v0[1], vt[1]], [v0[2], vt[2]], '--', color=c, lw=1.05, alpha=0.90, zorder=1)
        ax3d.scatter([vt[0]], [vt[1]], [vt[2]], color=[c], s=18, zorder=5)
        dx,dy,dz = offsets_3d.get(k,(0.001,0.0006,0.0008))
        ax3d.text(vt[0]+dx, vt[1]+dy, vt[2]+dz, f'{k}', fontsize=9, color=c, fontproperties=font_prop)
    pts3d = np.vstack([np.zeros((1, 3)), v0[None, :], v_samples])
    set_equal_3d(ax3d, pts3d)
    ax3d.set_xlabel('x / m', labelpad=2)
    ax3d.set_ylabel('y / m', labelpad=2)
    ax3d.set_zlabel('z / m', labelpad=2)
    ax3d.view_init(elev=23, azim=-58)
    ax3d.set_title('(a) 提插重定位：食指关键向量的三维变化', fontsize=13, pad=12, fontproperties=font_prop)

    # (a) x-z 投影
    draw_arrow_2d(ax_xz, (v0[0], v0[2]), color='black', lw=3.0, alpha=0.98)
    ax_xz.scatter([v0[0]], [v0[2]], color='black', s=24, zorder=5)
    annotate_point_2d(ax_xz, v0[0], v0[2], '初始', color='black', font_prop=font_prop, offset=(6, 6))
    xz_all_x = [0.0, v0[0]]
    xz_all_z = [0.0, v0[2]]
    xz_offsets = {1:(5,6),2:(5,-8),3:(5,4),4:(6,-2),5:(6,4),6:(6,4),7:(6,2)}
    for k, (i, c, dvz) in enumerate(zip(sample_ids, sample_colors, sample_dvz_sign), start=1):
        vt = v_index[i]
        draw_arrow_2d(ax_xz, (vt[0], vt[2]), color=c, lw=2.5, alpha=0.98)
        ax_xz.plot([v0[0], vt[0]], [v0[2], vt[2]], '--', color=c, lw=1.05, alpha=0.90, zorder=1)
        ax_xz.scatter([vt[0]], [vt[2]], color=[c], s=18, zorder=5)
        annotate_point_2d(ax_xz, vt[0], vt[2], f'{k}', color=c, font_prop=font_prop, offset=xz_offsets.get(k,(5,4)))
        ax_xz.plot([vt[0], vt[0]], [z0, vt[2]], ':', color=c, lw=1.0, alpha=0.95)
        draw_small_vertical_direction(ax_xz, float(vt[0] + 0.0014), float(vt[2]), float(dvz), c, scale=0.0032)
        xz_all_x.extend([vt[0], vt[0] + 0.0014])
        xz_all_z.extend([vt[2], vt[2] + (0.0032 if dvz >= 0 else -0.0032)])
    ensure_equal_xy(ax_xz, np.asarray(xz_all_x), np.asarray(xz_all_z), pad_ratio=0.19)
    ax_xz.set_xlabel('x / m')
    ax_xz.set_ylabel('z / m')
    ax_xz.grid(True, alpha=0.28)
    ax_xz.set_title('x-z 平面投影：以 z 分量变化与相邻采样向量的 Δv_z 解释提插映射', fontsize=12, pad=12, fontproperties=font_prop)

    # (b) x-y 相对向量
    draw_arrow_2d(ax_xy, (r0[0], r0[1]), color='black', lw=3.0, alpha=0.98)
    ax_xy.scatter([r0[0]], [r0[1]], color='black', s=24, zorder=5)
    annotate_point_2d(ax_xy, r0[0], r0[1], '初始', color='black', font_prop=font_prop, offset=(10, -10), va='top')
    max_r = max(float(np.max(np.linalg.norm(r_samples, axis=1))), float(np.linalg.norm(r0)), 1e-6)
    base_radius = 0.18 * max_r
    radius_step = 0.040 * max_r
    xy_offsets = {1:(8,-6),2:(8,6),3:(8,4),4:(8,4),5:(6,4),6:(6,-2),7:(6,-2)}
    for k, (i, c) in enumerate(zip(sample_ids, sample_colors), start=1):
        rt = rel_xy[i]
        draw_arrow_2d(ax_xy, (rt[0], rt[1]), color=c, lw=2.5, alpha=0.98)
        ax_xy.plot([r0[0], rt[0]], [r0[1], rt[1]], '--', color=c, lw=1.05, alpha=0.90, zorder=1)
        ax_xy.scatter([rt[0]], [rt[1]], color=[c], s=18, zorder=5)
        annotate_point_2d(ax_xy, rt[0], rt[1], f'{k}', color=c, font_prop=font_prop, offset=xy_offsets.get(k,(6,4)))
        theta0 = float(np.arctan2(r0[1], r0[0]))
        thetat = float(np.arctan2(rt[1], rt[0]))
        draw_arc_arrow(ax_xy, theta0, thetat, radius=base_radius + (k - 1) * radius_step, color=c, lw=1.1)
    ensure_equal_xy(ax_xy, np.r_[0.0, r0[0], r_samples[:, 0]], np.r_[0.0, r0[1], r_samples[:, 1]], pad_ratio=0.20)
    ax_xy.axhline(0.0, color='0.75', lw=0.8)
    ax_xy.axvline(0.0, color='0.75', lw=0.8)
    ax_xy.set_xlabel(r'$r_x$ / m')
    ax_xy.set_ylabel(r'$r_y$ / m')
    ax_xy.grid(True, alpha=0.28)
    ax_xy.set_title('(b) 捻转重定位：拇指-食指相对向量在 x-y 平面的变化', fontsize=13, pad=12, fontproperties=font_prop)

    legend_handles = [
        Line2D([0], [0], color='black', lw=3.0, label='初始向量'),
        Line2D([0], [0], color=cm.viridis(0.72), lw=2.5, label='有效动作区间内按等时间间隔取样的当前向量'),
        Line2D([0], [0], color=cm.viridis(0.72), lw=1.1, ls='--', label='当前向量端点相对初始向量端点的偏移'),
        Line2D([0], [0], color=cm.viridis(0.72), lw=1.1, ls=':', label='相对初始的 z 方向分量变化（仅用于 a 的 x-z 投影）'),
        Line2D([0], [0], color=cm.viridis(0.72), lw=1.0, marker=r'$↑$', markersize=9, label='Δv_z 小箭头：相邻采样向量差分与 +z 的关系（用于提插判向）'),
        Line2D([0], [0], color=cm.viridis(0.72), lw=1.0, marker=r'$↺$', markersize=10, label='弧箭头：由 r₀ 与 r_t 的叉乘 z 分量决定的捻转方向'),
    ]
    fig.legend(handles=legend_handles, loc='upper center', bbox_to_anchor=(0.5, 0.995),
               ncol=3, frameon=False, prop=font_prop, fontsize=10.0,
               handlelength=2.2, columnspacing=1.6, labelspacing=0.85)

    norm = colors.Normalize(vmin=0, vmax=len(sample_ids) - 1)
    sm = cm.ScalarMappable(cmap=cm.viridis, norm=norm)
    sm.set_array([])
    cax = fig.add_axes([0.34, 0.076, 0.32, 0.045])
    cb = fig.colorbar(sm, cax=cax, orientation='horizontal')
    cb.set_ticks(np.arange(len(sample_ids)))
    cb.set_ticklabels([str(i) for i in range(1, len(sample_ids) + 1)])
    cb.set_label('时间推进（早 → 晚；在自动识别的有效动作区间内按等时间间隔取样）', fontsize=11, fontproperties=font_prop)

    duration_s = (end - start) / max(fps, 1e-9)
    fig.text(0.045, 0.10,
             f'                                             彩色编号 1–{len(sample_ids)} 为等时间间隔采样时刻。',
             fontsize=10.5, ha='left', va='center', fontproperties=font_prop)

    fig.savefig(out_path, dpi=320, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='论文用：手部关键向量到针灸执行器重定位方法的静态可视化')
    parser.add_argument('--json', type=str, required=True, help='data.json 路径')
    parser.add_argument('--out-dir', type=str, default='./retarget_vis', help='输出目录')
    parser.add_argument('--fps', type=float, default=30.0, help='录制频率，默认 30 Hz')
    parser.add_argument('--start', type=int, default=None, help='手动指定起始帧')
    parser.add_argument('--end', type=int, default=None, help='手动指定结束帧')
    parser.add_argument('--num-samples', type=int, default=7, help='有效动作区间内等时间间隔采样的向量数')
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    hand = load_episode(args.json)
    v_index, rel_xy, is_new_input = extract_vectors(hand)

    if args.start is not None and args.end is not None:
        start = max(0, int(args.start))
        end = min(len(v_index) - 1, int(args.end))
    else:
        start, end = auto_active_window(v_index, rel_xy, is_new_input)
    if end <= start:
        start, end = 0, len(v_index) - 1

    out_path = out_dir / 'retarget_method_ab.png'
    plot_method_ab(
        out_path=str(out_path),
        v_index=v_index,
        rel_xy=rel_xy,
        is_new_input=is_new_input,
        start=start,
        end=end,
        fps=args.fps,
        num_samples=max(3, int(args.num_samples)),
    )
    print(f'Saved: {out_path}')
    print(f'Active window: [{start}, {end}] / {len(v_index)} frames')


if __name__ == '__main__':
    plt.rcParams.update({
        'font.size': 10,
        'axes.titlesize': 12,
        'axes.labelsize': 11,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
    })
    main()
