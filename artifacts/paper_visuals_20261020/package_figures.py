"""Assemble and validate the figure deliverable, preserving every original input."""
import hashlib
import html
import json
import shutil
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def main():
    figures = [
        {'stem':'01_p15_safety_tradeoff', 'title':'完整性约束规划的收益与代价',
         'caption':'固定仓库一次配对：最低真值误差评价裕度2.19→28.13厘米，可用率80.71%→100%；路径+0.67%，ATE略增0.739毫米。',
         'boundary':'裕度定义为AL−|真实方向定位误差|，不是障碍净空；不代表多场景统计或实机结论。',
         'sources':['evidence/P15/comparison-result.json'], 'evidence_type':'single_scene_paired_Gazebo_SIL'},
        {'stem':'02_p10_minimum_excitation', 'title':'小幅侧移恢复完整性预算',
         'caption':'名义轨迹/仅偏航/最小机动三组对照；最小机动最低AL−PL为+0.105米，额外路径6.59厘米。',
         'boundary':'固定单场景与种子；最小机动并非ATE最优。',
         'sources':['evidence/P10/flight-gate-summary.json'], 'evidence_type':'single_scene_three_arm_Gazebo_SIL'},
        {'stem':'03_p13_latency_response', 'title':'处理时延、速度与完整性预算联动',
         'caption':'人工规划负载50→200毫秒；端到端p99为150.51→301.29毫秒；限速0.420→0.145米/秒，任务132.4→221.4秒。',
         'boundary':'不降速预算是同轮反事实计算，不是未缓解算法的实际飞行试验；非Orin实测。',
         'sources':['evidence/P13/p13-gate-result.json','evidence/P13/low-50ms-trial-result.json','evidence/P13/high-200ms-trial-result.json'],
         'evidence_type':'single_scene_injected_delay_Gazebo_SIL'},
        {'stem':'04_joint_state_consistency', 'title':'多窗口联合状态一致性',
         'caption':'36,000条独立合成轨迹，每条4个相关窗口；完整模型各窗95%边际区间覆盖率94.875%—95.025%。',
         'boundary':'线性高斯软件实验；消融方法故意删除依赖，不是SOTA基线。窗口相关，不可当成144,000次独立试验。',
         'sources':['evidence/RESEARCH_R2B3B0/r2b3b0_suite_20261020T094534Z/summary.json'],
         'evidence_type':'synthetic_linear_Gaussian'},
        {'stem':'05_livo_replay', 'title':'激光—视觉—惯性融合回放',
         'caption':'同段仿真数据424个配对状态，422次有效视觉更新；两臂ATE为4.964/4.882毫米。',
         'boundary':'单bag回放与链路验证，微小差异不支持泛化精度优势；尚无独立下视光流状态更新、AP闭环或硬件验证。',
         'sources':['evidence/RESEARCH_R2B2/r2b2_suite_20261019T141249Z/summary.json'],
         'evidence_type':'recorded_single_simulation_replay'},
    ]
    mechanism = json.loads((HERE/'mechanism_sources.json').read_text())
    result = json.loads((HERE/'result_sources.json').read_text())
    source_hashes = {}
    for relative, digest in mechanism['source_sha256'].items():
        original = ROOT/relative
        assert sha(original) == digest, relative
        copy = HERE/'sources'/relative
        copy.parent.mkdir(parents=True, exist_ok=True)
        if not copy.exists():
            shutil.copy2(original, copy)
        assert sha(copy) == digest, relative
        source_hashes[relative] = digest
    for record in result['sources']:
        assert sha(HERE/record['snapshot']) == record['sha256'], record
        source_hashes[record['path']] = record['sha256']
    assert sha(HERE/'render_mechanisms.py') == mechanism['render_script_sha256']
    for figure in figures:
        for ext in ('png','pdf','svg'):
            path = HERE/(figure['stem']+'.'+ext)
            assert path.is_file() and path.stat().st_size > 1000, path
        with Image.open(HERE/(figure['stem']+'.png')) as image:
            image.verify()
        with Image.open(HERE/(figure['stem']+'.png')) as image:
            assert image.width >= 3000 and image.height >= 1500
            figure['png_dimensions'] = [image.width, image.height]
        assert (HERE/(figure['stem']+'.pdf')).read_bytes().startswith(b'%PDF-')
        assert '<svg' in (HERE/(figure['stem']+'.svg')).read_text()
    preservation = json.loads((HERE/'paper-preservation-check.json').read_text())
    assert preservation['status'] == 'PASS' and all(preservation['checks'].values())
    write_json(HERE/'figures.json', {'generated_at_utc':datetime.now(timezone.utc).isoformat(),
        'original_evidence_modified':False,'new_experiment_performed':False,
        'novelty_established':False,'hardware_established':False,
        'description':'Re-rendered existing measurements. Different validation stages must not be combined into one experiment.',
        'figures':figures, 'source_sha256':source_hashes})
    cards = []
    for i, figure in enumerate(figures, 1):
        stem = figure['stem']
        links = ' · '.join(f'<a href="{stem}.{ext}">{ext.upper()}</a>' for ext in ['png','pdf','svg'])
        cards.append(f'<article><h2>{i}. {html.escape(figure["title"])}</h2>'
                     f'<p>{html.escape(figure["caption"])}</p>'
                     f'<a href="{stem}.png"><img src="{stem}.png" alt="{html.escape(figure["title"])}"></a>'
                     f'<p class="boundary">{html.escape(figure["boundary"])}</p><p>{links}</p></article>')
    gallery = ('<!doctype html><html lang="zh-CN"><meta charset="utf-8">'
               '<meta name="viewport" content="width=device-width,initial-scale=1">'
               '<title>IMPACT 现有实验图集</title><style>'
               'body{font:16px/1.7 "Microsoft YaHei",sans-serif;color:#263444;background:#f2f5f7;margin:0}'
               'main{max-width:1160px;margin:36px auto;padding:0 22px}h1{font-size:30px}'
               'article{background:white;border-radius:12px;padding:24px;margin:28px 0}'
               'img{display:block;width:100%;height:auto}h2{font-size:22px}a{color:#007f78}'
               '.boundary{color:#586879;border-left:3px solid #c78035;padding-left:14px}'
               '</style><main><h1>IMPACT · 现有实验图集</h1>'
               '<p>2026-10-20 · 五张实测结果与软件验证图，保留原始数据来源、限制及代价。'
               '本次只重绘已有结果，没有新增实验或硬件测试。支持PNG高清图与PDF/SVG矢量图。</p>'
               + ''.join(cards) + '<p>数据出处与校验：<a href="figures.json">figures.json</a> · '
               '<a href="paper-preservation-check.json">论文证据保全检查</a> · '
               '<a href="reproduce.txt">重绘说明</a></p></main></html>')
    (HERE/'index.html').write_text(gallery, encoding='utf-8')
    (HERE/'reproduce.txt').write_text(
        '此包只重绘既有证据，不是新试验。数据副本在sources/；输入SHA在figures.json。\n'
        '环境：WSL Ubuntu22.04，系统Python3、NumPy1.21.5、Matplotlib3.5.1、Pillow。\n'
        '中文字体取本机Windows微软雅黑 /mnt/c/Windows/Fonts/msyh.ttc；包中不分发字体。\n'
        '在本目录执行：\n'
        'PYTHONNOUSERSITE=1 python3 render_results.py --root sources\n'
        'PYTHONNOUSERSITE=1 python3 render_mechanisms.py --repo-root sources --out .\n'
        '原始外置truth路径不存在时，render_results.py使用sources/r2b2-truth.jsonl副本。\n'
        '图04按轨迹内窗口显示边际覆盖率，窗口间相关，不提供整轨迹风险保证。\n'
        'PNG适合预览，PDF/SVG适合排版。期刊投稿前应统一为英文图注并补充必要的强基线和多场景统计。\n', encoding='utf-8')
    files = {str(p.relative_to(HERE)):{'bytes':p.stat().st_size,'sha256':sha(p)}
             for p in sorted(HERE.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
    write_json(HERE/'manifest.json', {'schema_version':1,'files':files,'self_excluded':True,
                                    'figure_count':len(figures),'formats':['png','pdf','svg']})
    archive = HERE.parent/'IMPACT_paper_visuals_20261020.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as bundle:
        for path in sorted(HERE.rglob('*')):
            if path.is_file():
                bundle.write(path, str(Path(HERE.name)/path.relative_to(HERE)))
    with zipfile.ZipFile(archive) as bundle:
        assert bundle.testzip() is None
        assert len(bundle.infolist()) == len(files)+1
        for name, record in files.items():
            assert hashlib.sha256(bundle.read(str(Path(HERE.name)/name))).hexdigest() == record['sha256']
    print(json.dumps({'status':'PASS','figures':5,'png_pdf_svg_files':15,
                      'bundle_files':len(files)+1,'zip_bytes':archive.stat().st_size,
                      'zip_sha256':sha(archive)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
