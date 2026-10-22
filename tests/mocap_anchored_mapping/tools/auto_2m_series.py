#!/usr/bin/env python3
"""Complete three forward measurements; reverse returns are separate, no automatic retry."""
import argparse
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--series',required=True)
    parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    series=Path(args.series).resolve();manifest=json.loads((series/'series.json').read_text())
    template=Path(manifest['first_completed_forward']);source=json.loads((template/'manifest.json').read_text())
    plan=[('return_before_02','reverse','reset_return'),('forward_02','forward','forward_measurement'),
          ('return_before_03','reverse','reset_return'),('forward_03','forward','forward_measurement')]
    if not args.execute:print(json.dumps(plan,ensure_ascii=False,indent=2));return
    if manifest['stage']!='prepared':raise ValueError('Series already started; never restart/retry automatically')
    current=None;interrupted=False
    def stop(*_):
        nonlocal interrupted
        interrupted=True
        if current is not None and current.poll() is None:current.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def save():(series/'series.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    manifest.update(stage='running',segments=[]);save()
    try:
        for name,direction,role in plan:
            if interrupted:raise RuntimeError('Operator interrupted series')
            trial=series/name;trial.mkdir(exist_ok=False)
            keys=['data_kind','run','localization_session','localization_launch_pid','target_m','route_clear_confirmed',
                  'manual_override_retained','world_transform_refit','timing_verified','protected_files']
            meta={k:source[k] for k in keys};meta.update(trial=str(trial),direction=direction,role=role,
                command_mps=.15 if direction=='forward' else -.15,stage='prepared',series=str(series))
            if role=='reset_return':
                first_result=json.loads((template/'results.json').read_text())
                first_start=next(e['start_base_pose'] for e in first_result['events'] if e['event']=='motion_start')
                meta['reset_target_mocap_position_m']=[first_start[k] for k in 'xyz']
            (trial/'manifest.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
            shutil.copy2(template/'watchdog_preparation.json',trial/'watchdog_preparation.json')
            shutil.copytree(template/'frozen_inputs',trial/'frozen_inputs')
            manifest['current_segment']=name;save()
            argv=[sys.executable,'-u',str(Path(__file__).with_name('auto_2m_trial.py')),'--trial',str(trial),'--execute']
            with (trial/'execution.log').open('x') as log:
                current=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                while current.poll() is None:time.sleep(.1)
            result=json.loads((trial/'manifest.json').read_text());segment=dict(path=str(trial),role=role,direction=direction,
                stage=result['stage'],exit_code=current.returncode,final_command_zero=result.get('final_command_zero'))
            manifest['segments'].append(segment);save()
            if current.returncode or result['stage']!='completed' or not result.get('final_command_zero'):
                raise RuntimeError('Segment failed; stop series without retry: '+name)
            if role=='forward_measurement':manifest['completed_forward_trials'].append(str(trial))
            save()
        manifest['stage']='completed'
    except Exception as error:manifest.update(stage='stopped_on_failure',error=str(error))
    finally:
        save();print(json.dumps(manifest,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
