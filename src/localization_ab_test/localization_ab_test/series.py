"""Three explicitly confirmed alternating trials, without automatic retries."""
import argparse
import csv
import json
from pathlib import Path
import signal
import subprocess
import sys

PLAN=('forward','reverse','forward')


def make_plan(first):
    if first not in ('forward','reverse'):raise ValueError('Unknown initial direction')
    return (first,'reverse' if first=='forward' else 'forward',first)


def completed(code,manifest):
    return code==0 and manifest.get('status')=='completed' and manifest.get('final_state',{}).get('reason')=='target_reached'


def continuation_trials(root,group,plan=PLAN):
    previous=json.loads((root/'series_manifest.json').read_text())
    trials=previous.get('trials',[])
    if (previous.get('status')!='aborted' or previous.get('group')!=group or len(trials)!=2 or
            tuple(previous.get('plan',plan))!=tuple(plan)):
        raise ValueError('Continuation requires one completed trial and a preflight-blocked second trial')
    first,last=trials
    first_path=Path(first['directory']);last_path=Path(last['directory'])
    # A parked operator may have renamed the enclosing experiment directory.
    # Resolve only its exact child basename, then verify the real manifests;
    # never rewrite or move the original data to make old absolute paths fit.
    if not first_path.exists():first_path=root/first_path.name
    if not last_path.exists():last_path=root/last_path.name
    first_manifest=json.loads((first_path/'manifest.json').read_text())
    failed=json.loads((last_path/'manifest.json').read_text())
    if (not completed(0,first_manifest) or not (first_path/'report/summary.json').exists() or
            first.get('direction')!=plan[0] or last.get('direction')!=plan[1] or
            failed.get('status')!='aborted' or not failed.get('error','').startswith('Preflight blocked:') or
            failed.get('recorded_topics') or failed.get('final_state') or (last_path/'bag').exists()):
        raise ValueError('Refusing automatic continuation of any started/failed motion')
    return [dict(first,directory=str(first_path.resolve()),carried_forward_from_previous_series=True)]


def summarize(root,group,trials):
    rows=[]
    for i,path in enumerate(trials,1):
        report=path/'report/summary.json'
        if not report.exists():continue
        s=json.loads(report.read_text());m=json.loads((path/'manifest.json').read_text())
        x=s['all_motion_and_5s_stop']['xy_m']
        rows.append(dict(trial=i,direction=m.get('direction','forward'),status=m['status'],
                         xy_rmse_mm=x['rmse']*1000,p95_mm=x['p95_absolute']*1000,
                         max_mm=x['max_absolute']*1000,yaw_rmse_deg=s['all_motion_and_5s_stop']['yaw_deg']['rmse'],
                         endpoint_mm=s['endpoint']['xy_error_m']*1000 if s['endpoint']['xy_error_m'] is not None else None,
                         max_pose_jump_mm=s['max_adjacent_pose_translation_m']*1000,
                         max_error_jump_mm=s['max_adjacent_error_jump_m']*1000,
                         trigger_m=s['trigger_displacement_m'],parked_m=s['endpoint']['physical_displacement_m'],
                         coverage=s['coverage'],timing_verified=s['timing_verified'],
                         reference_independent=group!='C',
                         metric_label='same-mocap consistency' if group=='C' else 'start-aligned relative localization error',
                         directory=str(path.resolve())))
    if rows:
        with (root/'summary.csv').open('w',newline='',encoding='utf8') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (root/'summary.json').write_text(json.dumps(dict(group=group,trials=rows),ensure_ascii=False,indent=2)+'\n')
    directions=' / '.join(r['direction'] for r in rows) or '尚无已完成报告'
    lines=[f'# {group} 组三次试验'+('：同源动捕一致性' if group=='C' else ''),'',
           directions+'；按方向记录，不作为三次相同正向重复。',
           '每次绝对速度指令 0.15 m/s、位移 2 m 发零、续录 5 s；各自起始静止窗口固定对齐。',
           '动捕不反馈到 A/B 定位。时间偏移未独立核实，动态指标为初步相对误差。','',
           '| 次数 | 方向 | 状态 | XY RMSE mm | P95 mm | Max mm | yaw RMSE ° | 终点差 mm |',
           '|---|---|---|---:|---:|---:|---:|---:|']
    for r in rows:
        endpoint='未知' if r['endpoint_mm'] is None else f"{r['endpoint_mm']:.3f}"
        lines.append(f"| {r['trial']} | {r['direction']} | {r['status']} | {r['xy_rmse_mm']:.3f} | {r['p95_mm']:.3f} | {r['max_mm']:.3f} | {r['yaw_rmse_deg']:.4f} | {endpoint} |")
    lines.extend(['','任一失败停止整批，不自动重试；未完成的试次不得记为成功。',
                  ('C 的同一动捕流同时参与全局校正和对比，以下指标不是独立绝对定位精度；FAST-LIO 内部状态和已有 TF 不被改写。'
                   if group=='C' else 'B 组另保留每次 NDT 原始诊断，未修改 NDT 参数或接受条件。')])
    (root/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--group',choices=('A','B','C'),required=True)
    parser.add_argument('--first-direction',choices=('forward','reverse'),default='forward')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--route-clear',action='store_true')
    parser.add_argument('--continue-from',type=Path,help='New onsite confirmation; only continue a preflight-blocked, never-started second trial')
    args=parser.parse_args()
    plan=make_plan(args.first_direction)
    prefix=continuation_trials(args.continue_from,args.group,plan) if args.continue_from else []
    if not args.execute:
        print(json.dumps(dict(read_only=True,group=args.group,plan=plan[len(prefix):],reference_independent=args.group!='C')));return
    if not args.route_clear:parser.error('Bidirectional clear route and onsite physical stop confirmation required')
    if args.output.exists():parser.error('Original results will not be overwritten')
    args.output.mkdir(parents=True)
    record=dict(group=args.group,plan=plan,status='running',trials=prefix,route_clear_confirmed=True,
                safety_limits_relaxed=False,mocap_feedback_to_localization=args.group=='C',
                reference_independent=args.group!='C',
                timing_verified=False,fastlio_restart_between_trials=False)
    if prefix:
        record['continuation_of']=str(args.continue_from.resolve())
        record['operator_reconfirmed_route_after_stop']=True
    reports=[Path(x['directory']) for x in prefix];child=None
    def save():
        (args.output/'series_manifest.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    def launch(command,log):
        nonlocal child
        with log.open('w',encoding='utf8') as stream:
            child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            code=child.wait();child=None;return code
    def interrupt(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:
        save()
        for i,direction in enumerate(plan[len(prefix):],len(prefix)+1):
            path=args.output/f'{i:02d}_{direction}'
            item=dict(trial=i,direction=direction,directory=str(path.resolve()),status='running')
            record['trials'].append(item);save()
            print(f'{args.group} {i}/3 START {direction}',flush=True)
            code=launch([sys.executable,'-m','localization_ab_test.run','--group',args.group,
                         '--direction',direction,'--output',str(path),'--execute','--route-clear'],
                        args.output/f'{i:02d}_runner.log')
            m=json.loads((path/'manifest.json').read_text());item['status']=m['status'];save()
            if not completed(code,m):raise RuntimeError(f'Trial {i} stopped: '+m.get('error',m.get('final_state',{}).get('reason','unknown')))
            print(f'{args.group} {i}/3 STOPPED target_reached; 5 s tail complete',flush=True)
            code=launch([sys.executable,'-m','localization_ab_test.evaluate','--trial',str(path)],
                        args.output/f'{i:02d}_evaluation.log')
            if code:raise RuntimeError(f'Trial {i} evaluation failed; stop before next motion')
            reports.append(path);rows=summarize(args.output,args.group,reports)
            print(json.dumps(rows[-1],ensure_ascii=False),flush=True)
        record['status']='completed'
    except (Exception,KeyboardInterrupt) as error:
        record['status']='aborted';record['error']=str(error) or 'interrupted'
        print('SERIES STOP: '+record['error'],flush=True)
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:child.terminate()
        for item in record['trials']:
            path=Path(item['directory'])/'manifest.json'
            if path.exists():item['status']=json.loads(path.read_text()).get('status','unknown')
        save();summarize(args.output,args.group,reports)
    print(str(args.output/'report.md'),flush=True)
    if record['status']!='completed':raise SystemExit(2)


if __name__=='__main__':main()
