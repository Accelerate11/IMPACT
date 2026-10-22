#!/usr/bin/env python3
"""Read-only evidence rendering of P10/P13. No source evidence is rewritten.

WSL reproduction:
  PYTHONNOUSERSITE=1 python3 render_mechanisms.py
Outputs are deliberately limited to 02/03 graphics and mechanism_sources.json.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.ticker import FormatStrFormatter
import numpy as np

BASE = '#738294'
OUR = '#007F78'
AUX = '#C78035'
INK = '#1D2D3A'
MUTED = '#5A6A77'
GRID = '#E4E9ED'


def configure(font):
    font_manager.fontManager.addfont(str(font))
    name = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({
        'font.family': name, 'font.size': 11, 'axes.titlesize': 14,
        'axes.labelsize': 11, 'axes.labelcolor': MUTED, 'text.color': INK,
        'xtick.color': INK, 'ytick.color': MUTED, 'axes.edgecolor': GRID,
        'axes.spines.top': False, 'axes.spines.right': False,
        'axes.unicode_minus': False, 'pdf.fonttype': 3, 'svg.fonttype': 'path',
        'figure.facecolor': 'white', 'axes.facecolor': 'white',
        'savefig.facecolor': 'white',
    })


def decorate(ax, title, ylabel):
    ax.set_title(title, loc='left', pad=20, fontweight='bold')
    ax.set_ylabel(ylabel, labelpad=10)
    ax.set_axisbelow(True)
    ax.grid(axis='y', color=GRID, linewidth=.8)
    ax.tick_params(axis='both', length=0, pad=9)
    ax.spines['left'].set_visible(False)


def values(ax, bars, fmt, offset, negative_offset=None):
    for bar in bars:
        value = bar.get_height()
        y = value + offset if value >= 0 else value - (negative_offset or offset)
        ax.text(bar.get_x() + bar.get_width()/2, y, fmt.format(value),
                ha='center', va='bottom' if value >= 0 else 'top',
                fontsize=12, fontweight='bold', color=bar.get_facecolor())


def header(fig, tag, title, sub):
    fig.text(.062, .948, tag, color=OUR, fontsize=11, fontweight='bold', va='top')
    fig.text(.062, .906, title, fontsize=24, fontweight='bold', va='top')
    fig.text(.062, .835, sub, fontsize=11.3, color=MUTED, va='top')


def save(fig, out, stem):
    for suffix in ['png', 'pdf', 'svg']:
        fig.savefig(out / (stem + '.' + suffix), dpi=250)
    plt.close(fig)


def load(repo, relative):
    return json.loads((repo / relative).read_text(encoding='utf-8-sig'))


def render_p10(repo, out):
    data = load(repo, 'evidence/P10/flight-gate-summary.json')
    arms = ['baseline', 'yaw_only', 'minimum_excitation']
    labels = ['名义轨迹', '仅偏航', '最小机动']
    colors = [BASE, AUX, OUR]
    margin = [data['actual_minimum_integrity_margins_m'][a] for a in arms]
    ate = [data['ate_rms_m'][a] for a in arms]
    path = [data['ground_truth_path_length_m'][a] for a in arms]
    extra = np.asarray(path) - path[0]
    reserve = data['thresholds']['margin_reserve_m']
    fig, axes = plt.subplots(1, 3, figsize=(13.6, 7.4))
    fig.subplots_adjust(left=.075, right=.965, top=.705, bottom=.28, wspace=.39)
    header(fig, 'IMPACT  /  已有机制验证  /  P10', '小幅侧向机动，在本案例中恢复完整性裕度',
           '三组采用相同场景、固定种子与校准；最小机动选择并执行右侧移。')
    x = np.arange(3)
    ax = axes[0]
    bars = ax.bar(x, margin, width=.55, color=colors, zorder=3)
    decorate(ax, 'A  最低完整性裕度', 'M = AL − PL（m）')
    ax.axhline(0, color=MUTED, linewidth=.9)
    ax.axhline(reserve, color=OUR, linestyle=(0, (4, 3)), linewidth=1.3)
    ax.text(-.45, reserve + .024, '门限 0.10 m', fontsize=10, color=OUR,
            bbox={'facecolor':'white', 'edgecolor':'none', 'pad':1})
    values(ax, bars, '{:+.3f}', .013, .014)
    ax.set_ylim(-.56, .25)
    ax.set_xticks(x, labels)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax = axes[1]
    bars = ax.bar(x, ate, width=.55, color=colors, zorder=3)
    decorate(ax, 'B  轨迹误差', 'ATE RMS（m，越低越好）')
    values(ax, bars, '{:.3f}', .004)
    ax.set_ylim(0, .16)
    ax.set_xticks(x, labels)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax = axes[2]
    bars = ax.bar(x, extra, width=.55, color=colors, zorder=3)
    decorate(ax, 'C  额外路径开销', '相对名义轨迹的额外路径（m）')
    values(ax, bars, '{:+.3f}', .007)
    ax.set_ylim(0, .26)
    ax.set_xticks(x, labels)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    gain = margin[2] - margin[0]
    fig.text(.062, .178,
             f'最低裕度增益  +{gain:.3f} m     ·     额外路径  +{extra[2]*100:.2f} cm     ·     任务时间  43.0 s（三组相同）',
             fontsize=12.5, color=OUR, fontweight='bold',
             bbox={'facecolor':'#ECF6F4', 'edgecolor':'none', 'pad':13})
    fig.text(.062, .092, '边界：固定单场景、固定种子 Gazebo SIL；无统计误差棒。M 是完整性预算裕度，非真值碰撞距离。',
             fontsize=10.2, color=MUTED)
    fig.text(.062, .057, '来源：evidence/P10/flight-gate-summary.json  ·  最小机动在本例并未取得最低 ATE。',
             fontsize=9.4, color=MUTED)
    save(fig, out, '02_p10_minimum_excitation')
    return {'arms': arms, 'minimum_integrity_margin_m': margin, 'ate_rms_m': ate,
            'extra_path_vs_baseline_m': extra.tolist(), 'reserve_m': reserve,
            'margin_gain_vs_baseline_m': gain,
            'scope': 'One fixed-scene/seed Gazebo SIL case. M=AL-PL, not GT collision distance. No statistical error bars.'}


def render_p13(repo, out):
    data = load(repo, 'evidence/P13/p13-gate-result.json')['comparison']
    trials = [load(repo, 'evidence/P13/low-50ms-trial-result.json'),
              load(repo, 'evidence/P13/high-200ms-trial-result.json')]
    labels = ['50 ms 负载', '200 ms 负载']
    p99 = [data['low_end_to_end_p99_ms'], data['high_end_to_end_p99_ms']]
    speed = [data['low_speed_limit_mps'], data['high_speed_limit_mps']]
    raw = [t['metrics']['final_unmitigated_integrity_margin_m'] for t in trials]
    safe = [t['metrics']['final_integrity_margin_m'] for t in trials]
    # Cross-check source units/semantics before rendering any claims.
    for i, trial in enumerate(trials):
        assert abs(p99[i] - trial['metrics']['end_to_end_p99_ms']) < 1e-10
        assert abs(speed[i] - trial['metrics']['final_speed_limit_mps']) < 1e-10
    fig, axes = plt.subplots(1, 3, figsize=(13.6, 7.4))
    fig.subplots_adjust(left=.075, right=.965, top=.705, bottom=.28, wspace=.39)
    header(fig, 'IMPACT  /  已有机制验证  /  P13', '处理时延上升后，以降速恢复完整性预算',
           '同一 Gazebo 场景，人工规划处理负载由 50 ms 增至 200 ms；两轮均完成约 24 m 任务。')
    x = np.arange(2)
    ax = axes[0]
    bars = ax.bar(x, p99, width=.48, color=[BASE, AUX], zorder=3)
    decorate(ax, 'A  实测端到端时延', '端到端 p99（ms）')
    values(ax, bars, '{:.2f}', 8)
    ax.set_ylim(0, 390)
    ax.set_xticks(x, labels)
    ax = axes[1]
    bars = ax.bar(x, speed, width=.48, color=[BASE, OUR], zorder=3)
    decorate(ax, 'B  应用的最终速度上限', '速度上限（m/s）')
    values(ax, bars, '{:.3f}', .012)
    ax.set_ylim(0, .52)
    ax.set_xticks(x, labels)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax = axes[2]
    width = .28
    b1 = ax.bar(x - width/2, raw, width=width, color=AUX,
                label='不降速预算', zorder=3)
    b2 = ax.bar(x + width/2, safe, width=width, color=OUR,
                label='应用降速后', zorder=3)
    decorate(ax, 'C  最终完整性预算裕度', '最终 M = AL − PL（m）')
    ax.axhline(0, color=MUTED, linewidth=.9)
    ax.axhline(.06, color=OUR, linestyle=(0, (4, 3)), linewidth=1.1)
    ax.text(-.45, .108, '要求 M ≥ 0.06 m', fontsize=9.7, color=OUR)
    values(ax, [b1[1]], '{:+.3f}', .004, .003)
    values(ax, [b2[1]], '{:+.3f}', .004, .003)
    ax.text(0, raw[0] + .005, '均为 +0.068', ha='center', va='bottom',
            fontsize=11.5, fontweight='bold', color=INK)
    ax.set_ylim(-.055, .125)
    ax.set_xticks(x, labels)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.legend(loc='lower left', bbox_to_anchor=(-.04, -.26), ncol=2,
              frameon=False, fontsize=9.5, handlelength=1.2, columnspacing=1)
    fig.text(.062, .178, '代价同时报告：任务时间  132.4 → 221.4 s（+67.2%）',
             fontsize=12.5, color=OUR, fontweight='bold',
             bbox={'facecolor':'#ECF6F4', 'edgecolor':'none', 'pad':13})
    fig.text(.062, .092, '边界：“不降速”是同轮预算反事实，非未缓解算法的实际飞行对照；软件注入负载，非 Orin 实测。',
             fontsize=10.1, color=MUTED)
    fig.text(.062, .057, '来源：evidence/P13/p13-gate-result.json 及两轮 trial-result.json  ·  单场景，非多次独立统计结论。',
             fontsize=9.4, color=MUTED)
    save(fig, out, '03_p13_latency_response')
    return {'planner_injected_load_ms':[50,200], 'end_to_end_p99_ms':p99,
            'final_speed_limit_mps':speed,
            'final_unmitigated_integrity_margin_m':raw, 'final_applied_integrity_margin_m':safe,
            'mission_elapsed_s':[data['low_mission_elapsed_s'],data['high_mission_elapsed_s']],
            'scope':'Single Gazebo scene, injected planner delay. Raw values are budget counterfactuals from the same trials, not unmitigated flight arms. Not Orin measurements.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--font', type=Path, default=Path('/mnt/c/Windows/Fonts/msyh.ttc'))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    configure(args.font)
    source_files = [
        'evidence/P10/flight-gate-summary.json',
        'evidence/P13/p13-gate-result.json',
        'evidence/P13/low-50ms-trial-result.json',
        'evidence/P13/high-200ms-trial-result.json',
        'docs/P10_MINIMUM_EXCITATION_ACCEPTANCE_REPORT.md',
        'docs/P13_LATENCY_AWARE_SAFETY_ACCEPTANCE_REPORT.md',
    ]
    fingerprints = {p: hashlib.sha256((args.repo_root/p).read_bytes()).hexdigest() for p in source_files}
    output = {'schema_version':1, 'render_library': 'matplotlib ' + matplotlib.__version__,
              'font':str(args.font), 'source_sha256':fingerprints,
              'P10':render_p10(args.repo_root,args.out),
              'P13':render_p13(args.repo_root,args.out)}
    assert fingerprints == {p:hashlib.sha256((args.repo_root/p).read_bytes()).hexdigest() for p in source_files}
    output['render_script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (args.out/'mechanism_sources.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'status':'PASS','figures':2,'formats':['png','pdf','svg'],'source_unchanged':True}))


if __name__ == '__main__':
    main()
