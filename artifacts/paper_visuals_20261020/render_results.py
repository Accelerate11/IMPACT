"""Render measured results only; do not modify the original evidence.

Run in WSL with system NumPy/Matplotlib and --root /mnt/d/CODEX/impact.
An existing sources/ snapshot makes this renderer independent of the WSL truth path.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
import matplotlib.pyplot as plt
import numpy as np

P15 = 'evidence/P15/comparison-result.json'
JOINT = 'evidence/RESEARCH_R2B3B0/r2b3b0_suite_20261020T094534Z/summary.json'
LIVO_DIR = 'evidence/RESEARCH_R2B2/r2b2_suite_20261019T141249Z'
TRUTH = Path('/home/accelerate/impact_research_ws/experiments/results/research_multisensor/r2b2_replay_20261019T140630Z_1564/truth.jsonl')
COLORS = ['#738294', '#007F78', '#C78035']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def style():
    candidates = [Path('/mnt/c/Windows/Fonts/msyh.ttc'), Path('C:/Windows/Fonts/msyh.ttc')]
    font = next(p for p in candidates if p.exists())
    font_manager.fontManager.addfont(str(font))
    family = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'font.family': family, 'font.size': 11,
                         'axes.unicode_minus': False, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.labelcolor': '#263444',
                         'text.color': '#263444', 'axes.titleweight': 'normal',
                         'axes.edgecolor': '#AAB4BF', 'xtick.color': '#405165',
                         'ytick.color': '#405165', 'figure.facecolor': 'white',
                         'savefig.facecolor': 'white', 'pdf.fonttype': 3,
                         'svg.fonttype': 'path'})


def header(fig, title, subtitle):
    fig.text(.06, .963, title, fontsize=20, weight='bold', va='top')
    fig.text(.06, .907, subtitle, fontsize=10.5, color='#586879', va='top')


def foot(fig, line1, line2):
    fig.text(.06, .069, line1, fontsize=9.5, color='#586879')
    fig.text(.06, .035, line2, fontsize=9, color='#586879')


def save(fig, output, stem):
    for ext in ('png', 'pdf', 'svg'):
        fig.savefig(output / (stem + '.' + ext), dpi=250)
    plt.close(fig)


def bars(ax, values, names, title, ylabel, ylim, formats, note=None):
    ax.bar(np.arange(len(values)), values, width=.49,
           color=COLORS[:len(values)], zorder=3)
    ax.set(xticks=np.arange(len(values)), xticklabels=names,
           ylabel=ylabel, ylim=ylim)
    ax.set_title(title, loc='left', pad=15, fontsize=12)
    ax.grid(axis='y', color='#DCE3EA', alpha=.75, zorder=0)
    for x, (v, label) in enumerate(zip(values, formats)):
        ax.text(x, v + .025 * (ylim[1]-ylim[0]), label,
                ha='center', va='bottom', fontsize=13, weight='bold')
    if note:
        ax.text(.99, .97, note, transform=ax.transAxes, ha='right', va='top',
                fontsize=9, color='#007F78')


def draw_p15(a, output):
    data = a['metrics']
    base, ours = data['information_only'], data['integrity_constrained']
    names = ['仅信息效用基线', '完整性约束规划']
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 8.0))
    fig.subplots_adjust(left=.085, right=.97, top=.80, bottom=.19, hspace=.60, wspace=.31)
    header(fig, '安全收益与代价：完整性约束规划',
           'P15 · 同一复杂仓库约24米任务的一次配对 Gazebo SIL；两臂仅完整性硬过滤不同')
    vals = [v['gt_minimum_realized_margin_m']*100 for v in (base, ours)]
    bars(axes[0, 0], vals, names, '(a) 最低真值误差评价裕度 ↑', '厘米', (0, 36),
         [f'{v:.2f}' for v in vals], '+25.94 厘米')
    vals = [v['availability_rate']*100 for v in (base, ours)]
    bars(axes[0, 1], vals, names, '(b) 完整性可用率 ↑', '%', (0, 124),
         [f'{v:.2f}%' for v in vals], '+19.29 个百分点')
    axes[0, 1].set_yticks([0, 25, 50, 75, 100])
    vals = [v['ground_truth_path_length_m'] for v in (base, ours)]
    bars(axes[1, 0], vals, names, '(c) 实际路径长度 ↓', '米', (0, 32),
         [f'{v:.3f}' for v in vals], '路径代价 +0.67%')
    vals = [v['ate_rms_m']*1000 for v in (base, ours)]
    bars(axes[1, 1], vals, names, '(d) 定位 ATE RMS ↓', '毫米', (0, 80),
         [f'{v:.3f}' for v in vals], 'ATE 增加 0.739 毫米')
    foot(fig, '评价裕度 = AL − |真实方向定位误差|，并非距障碍物的实际净空。本次路径略增、ATE略增，两臂真实安全违规均为0。',
         '数据：evidence/P15/comparison-result.json。单场景单次配对；未画重复试验误差棒，不代表多地图或实机结论。')
    save(fig, output, '01_p15_safety_tradeoff')


def draw_joint(summary, output):
    a = summary['joint_study']
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 6.2))
    fig.subplots_adjust(left=.085, right=.97, top=.76, bottom=.27, wspace=.29)
    header(fig, '多窗口联合状态：检查不确定性是否可信',
           'R2B3B0 · 36,000条独立合成轨迹，每条4个相关窗口；同一观测下比较依赖模型')
    methods = ['full_joint', 'reset_shared_nuisance', 'zero_clone_cross']
    labels = ['完整联合状态', '每窗重置共享尺度', '删除克隆交叉协方差']
    colors = ['#007F78', '#C78035', '#9A638D']
    x = np.arange(1, 5)
    for i, (method, label, color) in enumerate(zip(methods, labels, colors)):
        rows = [w['methods'][method] for w in a['cases']]
        coverage = np.array([r['coverage'] for r in rows])*100
        ci = np.array([r['wilson_95'] for r in rows])*100
        axes[0].errorbar(x+(i-1)*.06, coverage,
                         yerr=np.vstack([coverage-ci[:, 0], ci[:, 1]-coverage]),
                         marker=['o','s','^'][i], color=color, linewidth=1.8,
                         markersize=5, capsize=3, label=label)
        ratio = [r['empirical_covariance_xy'][0][0]/r['predicted_covariance_xy'][0][0]
                 for r in rows]
        axes[1].plot(x, ratio, marker=['o','s','^'][i], color=color,
                     linewidth=1.8, markersize=5, label=label)
    axes[0].axhline(95, color='#324454', linestyle='--', linewidth=1)
    axes[1].axhline(1, color='#324454', linestyle='--', linewidth=1)
    axes[0].set(title='(a) 速度 x 的95%边际区间覆盖率', ylabel='覆盖率（%）', ylim=(94, 96.5))
    axes[1].set(title='(b) 经验方差 / 预测方差', ylabel='比值（理想值为1）', ylim=(.85, 1.10))
    for ax in axes:
        ax.set(xlabel='轨迹内的观测窗口', xticks=x, xlim=(.75,4.25))
        ax.grid(color='#DCE3EA', alpha=.75)
        ax.legend(loc='lower left', bbox_to_anchor=(0, -.39), fontsize=9, frameon=False)
    foot(fig, '误差棒为各窗口 Wilson 95%区间；完整模型覆盖率94.875%—95.025%。窗口间相关，不能当成144,000次独立试验。',
         '线性高斯软件实验；后两项为依赖关系消融。未融合实际光流，不据微小差异宣称统计显著性、真实定位优势或整航程安全保证。')
    save(fig, output, '04_joint_state_consistency')


def draw_livo(a, trajectories, truth, output):
    reference_t = np.array([r['stamp_s'] for r in truth])
    reference_p = np.array([r['position'] for r in truth])
    assert np.all(np.diff(reference_t) >= 0)
    fig, axes = plt.subplots(1, 3, figsize=(13.4, 6.4), gridspec_kw={'width_ratios':[1, 1.2, .85]})
    fig.subplots_adjust(left=.07, right=.97, top=.76, bottom=.26, wspace=.35)
    header(fig, '融合链路成果：激光—视觉—惯性共同状态回放',
           'R2B2 · 相同录制仿真输入、相同前视历元；424个配对状态，422次有效视觉更新')
    axes[0].plot(reference_t-reference_t[0], reference_p[:,2], color='#243646',
                 linestyle='--', linewidth=1.2, label='仿真真值')
    provenance = []
    for arm, color, label in [('arm_no_visual', '#738294', '关闭视觉校正'),
                              ('arm_livo', '#007F78', '启用视觉校正')]:
        ts = np.array([r['stamp_s'] for r in trajectories[arm]])
        ps = np.array([r['position'] for r in trajectories[arm]])
        idx = np.searchsorted(reference_t, ts)
        idx = np.clip(idx, 0, len(reference_t)-1)
        prev = np.maximum(idx-1, 0)
        # The frozen evaluator breaks exactly equal distances toward the earlier index.
        idx = np.where(np.abs(reference_t[prev]-ts) <= np.abs(reference_t[idx]-ts), prev, idx)
        assert np.max(np.abs(reference_t[idx]-ts)) <= .02
        error = np.linalg.norm(ps-reference_p[idx], axis=1)
        rms = float(np.sqrt(np.mean(error**2)))
        assert abs(rms-a['arms'][arm]['ate_rms_m']) < 1e-12, (arm, rms)
        provenance.append({'arm':arm,'recomputed_ate_rms_m':rms,'pairs':len(ts),
                           'maximum_truth_time_offset_s':float(np.max(np.abs(reference_t[idx]-ts)))})
        axes[0].plot(ts-reference_t[0], ps[:,2], color=color, linewidth=1.2, label=label)
        axes[1].plot(ts-reference_t[0], error*1000, color=color, linewidth=1.25, label=label)
    axes[0].set(title='(a) 垂直运动与悬停', xlabel='相对真值首帧时间（秒）', ylabel='世界系高度（米）')
    axes[1].set(title='(b) 配对位置误差', xlabel='相对真值首帧时间（秒）', ylabel='误差（毫米）', ylim=(0,55))
    vals = [a['arms'][arm]['ate_rms_m']*1000 for arm in ['arm_no_visual','arm_livo']]
    bars(axes[2], vals, ['关闭视觉\n校正','启用视觉\n校正'], '(c) 定位 ATE RMS ↓', '毫米', (0,6.5), [f'{v:.3f}' for v in vals])
    for ax in axes[:2]:
        ax.grid(color='#DCE3EA', alpha=.75)
        ax.legend(loc='lower left', bbox_to_anchor=(0,-.39), fontsize=9, frameon=False)
    foot(fig, '仅首对状态作 SE(3) 对齐，不拟合轨迹尺度。两臂 ATE 接近；本图用于说明链路与校正生效，不宣称泛化精度优势。',
         '数据：R2B2 固定单段录制仿真。尚未包含独立下视光流状态更新、AP闭环、Orin或真实传感器实验。')
    save(fig, output, '05_livo_replay')
    return provenance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path('/mnt/d/CODEX/impact'))
    args = parser.parse_args()
    output = Path(__file__).resolve().parent
    snapshot = output/'sources'
    snapshot.mkdir(exist_ok=True)
    records = []

    def read_source(relative):
        source = args.root/relative
        copy = snapshot/relative
        if not copy.exists():
            copy.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, copy)
        assert sha(source) == sha(copy), relative
        records.append({'path':relative,'sha256':sha(source),'bytes':source.stat().st_size,
                        'snapshot':str(copy.relative_to(output))})
        return json.loads(copy.read_text())

    p15 = read_source(P15)
    joint = read_source(JOINT)
    livo = read_source(LIVO_DIR+'/summary.json')
    trajectories = {arm:read_source(LIVO_DIR+'/'+arm+'-aligned-trajectory.json')
                    for arm in ['arm_livo','arm_no_visual']}
    truth_copy = snapshot/'r2b2-truth.jsonl'
    if not truth_copy.exists():
        shutil.copy2(TRUTH, truth_copy)
    if TRUTH.exists():
        assert sha(TRUTH) == sha(truth_copy)
    records.append({'path':str(TRUTH),'sha256':sha(truth_copy),
                    'bytes':truth_copy.stat().st_size,'snapshot':str(truth_copy.relative_to(output))})
    truth = [json.loads(line) for line in truth_copy.read_text().splitlines()]
    style()
    draw_p15(p15, output)
    draw_joint(joint, output)
    checks = draw_livo(livo, trajectories, truth, output)
    for record in records:
        path = args.root/record['path'] if not Path(record['path']).is_absolute() else Path(record['path'])
        if path.exists():
            assert sha(path) == record['sha256']
    (output/'result_sources.json').write_text(json.dumps({'status':'PASS', 'sources':records,
            'livo_independent_recalculation':checks}, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status':'PASS','figures':3,'livo_recalculation':checks}, ensure_ascii=False))


if __name__ == '__main__':
    main()
