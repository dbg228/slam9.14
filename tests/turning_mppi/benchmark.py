"""Reproducible offline closed-loop comparison; never publishes ROS commands.

Run from this directory: python3 benchmark.py --output results/pilot
The synthetic plant does NOT run FAST-LIO, NDT, Gazebo or real sensors.
"""
import argparse
from collections import deque
from dataclasses import asdict, replace
import hashlib
import json
import os
from pathlib import Path as FilePath
import sys
import time
import subprocess

import numpy as np

HERE = FilePath(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from controller import Config, GuidedMPPI, Path, clearance, smooth_step, wrap


def make_path(kind):
    ds = .01
    if kind == 'historical_route':
        source=ROOT/'robot_data/experiments/full_map_loop/20261018_relocated_01/group1_ndt/trial_01/frozen_inspection_path.json'
        xy=np.asarray(json.loads(source.read_text())['points_xyz_m'])[:,:2]
        xy=xy[np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-6]]
        segment=np.arctan2(np.diff(xy[:,1]),np.diff(xy[:,0]))
        unwrapped=np.unwrap(segment)
        yaw=np.r_[unwrapped[0],(unwrapped[:-1]+unwrapped[1:])*.5,unwrapped[-1]]
        return Path(np.c_[xy,yaw])
    if kind in ('obstacle', 'blocked', 'straight'):
        s = np.arange(0, 3.501, ds)
        return Path(np.c_[s,s*0,s*0])
    radius = .75 if kind == 's_bend' else .8
    ramp = .30
    bend_length = radius*np.pi/2+ramp
    total = 1.+bend_length+1.
    if kind == 's_bend':
        total += bend_length+.45
    s = np.arange(0,total+ds/2,ds)
    def bend(start, sign):
        z = s-start
        k = np.zeros(len(s))
        a=(z>=0)&(z<ramp)
        b=(z>=ramp)&(z<bend_length-ramp)
        d=(z>=bend_length-ramp)&(z<=bend_length)
        k[a]=.5*(1-np.cos(np.pi*z[a]/ramp))/radius
        k[b]=1/radius
        k[d]=.5*(1+np.cos(np.pi*(z[d]-bend_length+ramp)/ramp))/radius
        return sign*k
    k=bend(1.,-1 if kind == 'right' else 1)
    if kind == 's_bend':
        k+=bend(1.+bend_length+.45,-1)
    yaw=np.r_[0,np.cumsum((k[1:]+k[:-1])*.5*ds)]
    x=np.r_[0,np.cumsum(np.cos((yaw[1:]+yaw[:-1])*.5)*ds)]
    y=np.r_[0,np.cumsum(np.sin((yaw[1:]+yaw[:-1])*.5)*ds)]
    return Path(np.c_[x,y,yaw])


class Plant:
    """30Hz open-loop smoother + discrete transport + 1st-order body lag.

    Smoother is actually substepped at 30Hz independent of the 20Hz controller.
    Lag is assumed, not identified from real CAN. No wheel slip dynamics.
    """
    def __init__(self, cfg, tau_w=.25, delay=.10, tau_v=.12, bias=0.):
        self.cfg=cfg
        self.state=np.zeros(5)
        self.smooth=np.zeros(2)
        self.tau_w,self.tau_v,self.bias=tau_w,tau_v,bias
        self.delay_steps=round(delay/cfg.dt)
        self.queue=deque([np.zeros(2) for _ in range(self.delay_steps)])
        self.time=0.
        self.next_smooth=0.
        self.interventions=0

    def step(self,command):
        c=self.cfg
        command=np.asarray(command)
        self.interventions+=int(abs(command[1])>c.wmax+1e-7)
        # Event integration captures the 30:20Hz phase relationship.
        end=self.time+c.dt
        while self.next_smooth<end-1e-9:
            self.smooth=smooth_step(self.smooth,command,c,dt=1/30)
            self.next_smooth+=1/30
        self.queue.append(self.smooth.copy())
        delayed=self.queue.popleft()
        x,y,yaw,v,w=self.state
        # 5ms substeps reduce dependence on controller integration scheme.
        h=c.dt/10
        for _ in range(10):
            v+=(1-np.exp(-h/self.tau_v))*(delayed[0]-v)
            w+=(1-np.exp(-h/self.tau_w))*(delayed[1]+self.bias-w)
            x+=v*np.cos(yaw+.5*w*h)*h
            y+=v*np.sin(yaw+.5*w*h)*h
            yaw+=w*h
        self.state=np.array([x,y,yaw,v,w])
        self.time=end


def execute(method,scenario,seed,plant_kind,cfg,output):
    path=make_path(scenario)
    obstacles=([(1.8,.47,.13)] if scenario == 'obstacle' else
               [(1.8,0.,.35)] if scenario == 'blocked' else [])
    tau,delay,bias=(.25,.10,0.) if plant_kind == 'nominal' else (.38,.15,.008)
    plant=Plant(cfg,tau,delay,bias=bias)
    plant.state[:3]=path.points[0]
    rng=np.random.default_rng(seed+1900)
    # Same pre-generated localization perturbations for every controller.
    max_steps=int((path.length/cfg.speed*1.65+8)/cfg.dt)
    noise=rng.normal(size=(max_steps,5))*[.002,.002,.003,.002,.003]
    controller=None
    if method.startswith('native'):
        from native.native_mppi import NativeMPPI
        controller=NativeMPPI(str(ROOT/'robot_data/config/navigation.yaml'),'InspectionStrict',seed=seed)
        controller.set_path(path.points)
    else:
        controller=GuidedMPPI(path,cfg,seed,mode=method)
    rows=[]
    hint=0.
    complete=False
    failure=None
    true_min=float('inf')
    start=time.perf_counter()
    for step in range(max_steps):
        measured=plant.state+noise[step]
        began=time.perf_counter()
        try:
            if method.startswith('native'):
                cmd=controller.step(*map(float,measured))
            else:
                # Pending commands are reconstructible from own sent commands.
                # Controller assumes .10s delay even for held-out .15s plant.
                pending=list(plant.queue)[-round(cfg.delay/cfg.dt):]
                cmd=controller.command(measured,plant.smooth,pending,obstacles)
        except Exception as exc:
            failure=f'controller_exception: {exc}'
            break
        elapsed=time.perf_counter()-began
        plant.step(cmd)
        x,y,yaw,v,w=plant.state
        hint,e,tangent=path.project(x,y,hint)
        k=float(np.interp(hint,path.s,path.k))
        dist=float(clearance(np.array(x),np.array(y),np.array(yaw),obstacles,cfg))
        true_min=min(true_min,dist)
        status=getattr(controller,'last_diagnostics',{}).get('status','native')
        rows.append([plant.time,x,y,yaw,v,w,float(cmd[0]),float(cmd[1]),hint,e,float(wrap(yaw-tangent)),k,elapsed,dist,status])
        if dist<0:
            failure='footprint_collision'
            break
        if hint>=path.length-.035 and np.hypot(x-path.points[-1,0],y-path.points[-1,1])<.08:
            complete=True
            break
        if abs(e)>.8:
            failure='tracking_divergence'
            break
        if scenario == 'blocked' and step>120 and abs(v)<.005:
            # Stopped is the intended outcome for this separate blocking test.
            break
    if not complete and not failure:
        failure='blocked_stop' if scenario=='blocked' else 'timeout'
    if hasattr(controller,'close'):
        controller.close()
    a=np.array([r[:14] for r in rows],dtype=float)
    def metrics(mask):
        e=a[mask,9]
        h=a[mask,10]
        return {'samples':int(len(e)), 'rmse_m':float(np.sqrt(np.mean(e**2))) if len(e) else None,
                'p95_abs_m':float(np.percentile(abs(e),95)) if len(e) else None,
                'max_abs_m':float(max(abs(e))) if len(e) else None,
                'heading_rmse_rad':float(np.sqrt(np.mean(h**2))) if len(e) else None,
                'mean_speed_mps':float(np.mean(a[mask,4])) if len(e) else None}
    result={'method':method,'scenario':scenario,'seed':seed,'plant':plant_kind,
            'evidence':'synthetic_closed_loop_not_fastlio_ndt',
            'complete':complete,'failure':failure,'time_s':plant.time,'steps':len(rows),
            'distance_m':float(np.sum(a[:,4])*cfg.dt) if len(a) else 0.,
            'all':metrics(np.ones(len(a),dtype=bool)) if len(a) else {},
            'turn':metrics(abs(a[:,11])>.15) if len(a) else {},
            'compute_p95_ms':float(np.percentile(a[:,12],95)*1000) if len(a) else None,
            'compute_max_ms':float(max(a[:,12])*1000) if len(a) else None,
            'deadline_misses':int(np.sum(a[:,12]>cfg.dt)) if len(a) else 0,
            'min_true_footprint_clearance_m':true_min if np.isfinite(true_min) else None,
            'wall_time_s':time.perf_counter()-start,'angular_clip_steps':plant.interventions,
            'true_plant':{'tau_w':tau,'delay':delay,'yaw_bias':bias}}
    if len(a):
        dk=np.interp(a[:,8],path.s,path.dk)*np.sign(a[:,11])
        result['phases']={
            'entry':metrics((abs(a[:,11])>.15)&(dk>.4)),
            'apex':metrics((abs(a[:,11])>.15)&(abs(dk)<=.4)),
            'exit':metrics((abs(a[:,11])>.15)&(dk<-.4)),
            'line':metrics(abs(a[:,11])<=.15)}
    key=f'{scenario}_{plant_kind}_{method}_{seed}'
    # NPZ contains finite traces and JSON metrics never uses NaN or infinity.
    np.savez_compressed(output/(key+'.npz'),trace=a,path=path.points,obstacles=np.asarray(obstacles).reshape(-1,3),
                        columns=np.array(['t','x','y','yaw','v','w','cmd_v','cmd_w','s','e','e_yaw','k','compute_s','clearance']))
    (output/(key+'.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def plot_results(results,output):
    os.environ.setdefault('MPLCONFIGDIR','/tmp/turning_mppi_mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    keys=list(dict.fromkeys((r['scenario'],r['plant']) for r in results))
    for scenario,plant_kind in keys:
        fig,axes=plt.subplots(1,3,figsize=(14,4))
        plotted=False
        for r in results:
            if (r['scenario'],r['plant'])!=(scenario,plant_kind) or r['seed']!=results[0]['seed']:
                continue
            source=output/f"{scenario}_{plant_kind}_{r['method']}_{r['seed']}.npz"
            if not source.exists():
                continue
            data=np.load(source)
            a=data['trace']; p=data['path']
            if len(a)==0:
                continue
            plotted=True
            axes[0].plot(a[:,1],a[:,2],label=r['method'])
            axes[1].plot(a[:,8],a[:,9]*1000,label=r['method'])
            axes[2].plot(a[:,0],a[:,4],label=r['method'])
        if not plotted:
            plt.close(fig)
            continue
        axes[0].plot(p[:,0],p[:,1],'k--',label='reference')
        for ox,oy,rad in data['obstacles']:
            axes[0].add_patch(plt.Circle((ox,oy),rad,color='red',alpha=.5))
        axes[0].set(xlabel='x [m]',ylabel='y [m]'); axes[0].axis('equal')
        axes[1].set(xlabel='path progress [m]',ylabel='signed lateral error [mm]')
        axes[2].set(xlabel='time [s]',ylabel='body speed [m/s]')
        for ax in axes:
            ax.grid(alpha=.3);ax.legend(fontsize=7)
        fig.suptitle(f'SYNTHETIC plant: {scenario}, {plant_kind}; no FAST-LIO/NDT simulation')
        fig.tight_layout();fig.savefig(output/f'{scenario}_{plant_kind}.png',dpi=150);plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=FilePath,required=True)
    ap.add_argument('--methods',nargs='+',default=['native','tracker','ideal','full'])
    ap.add_argument('--scenarios',nargs='+',default=['left','right','s_bend'])
    ap.add_argument('--plants',nargs='+',default=['nominal','heldout'])
    ap.add_argument('--seeds',nargs='+',type=int,default=[0,1,2])
    ap.add_argument('--samples',type=int,default=1000)
    ap.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    cfg=replace(Config(),samples=args.samples)
    backend_info={}
    if os.environ.get('TURNING_MPPI_BACKEND','cpp') == 'cpp':
        from acceleration.fast_rollout import prepare
        backend_info=prepare()  # build/loading excluded from per-step measurements
    config_file=ROOT/'robot_data/config/navigation.yaml'
    manifest={'schema':1,'evidence':'synthetic_closed_loop_not_fastlio_ndt',
              'rollout_backend':os.environ.get('TURNING_MPPI_BACKEND','cpp'),
              'backend_info':backend_info,
              'production_navigation_sha256':hashlib.sha256(config_file.read_bytes()).hexdigest(),
              'controller':asdict(cfg),'arguments':{k:str(v) if isinstance(v,FilePath) else v for k,v in vars(args).items()},
              'source_sha256':{str(p.relative_to(HERE)):hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in sorted(HERE.rglob('*')) if p.suffix in ('.py','.cpp','.hpp') and 'build' not in p.parts},
              'limitations':['No lidar/IMU rendering, FAST-LIO or NDT execution',
                             'Lag/delay values assumed, not identified from hardware',
                             'Synthetic state perturbations, not actual localization error',
                             'Native engine seeded; long traces not bitwise reproducible; see native README',
                             'Native original obstacle critics disabled: obstacle experiments omit native',
                             'Guided sampling uses heuristic residual importance weights; no optimality theorem']}
    if not args.worker:
        (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    results=[]
    for scenario in args.scenarios:
        for plant_kind in args.plants:
            for seed in args.seeds:
                for method in args.methods:
                    if scenario in ('obstacle','blocked') and method.startswith('native'):
                        continue
                    if args.worker:
                        r=execute(method,scenario,seed,plant_kind,cfg,args.output)
                        print(json.dumps(r,allow_nan=False),flush=True)
                        return
                    # A fresh process avoids shared RNG/context/plugin state
                    # leaking between trials of the installed C++ controller.
                    command=[sys.executable,str(FilePath(__file__).resolve()),'--worker',
                             '--output',str(args.output.resolve()),'--methods',method,
                             '--scenarios',scenario,'--plants',plant_kind,'--seeds',str(seed),
                             '--samples',str(args.samples)]
                    try:
                        subprocess.run(command,check=True,timeout=240,stdout=subprocess.DEVNULL)
                        r=json.loads((args.output/f'{scenario}_{plant_kind}_{method}_{seed}.json').read_text())
                    except (subprocess.SubprocessError,OSError) as exc:
                        r={'method':method,'scenario':scenario,'seed':seed,'plant':plant_kind,
                           'evidence':'synthetic_closed_loop_not_fastlio_ndt','complete':False,
                           'failure':f'worker_failure: {exc}','all':{},'turn':{}}
                    results.append(r)
                    print(json.dumps(r,allow_nan=False),flush=True)
                    (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    plot_results(results,args.output)


if __name__=='__main__':
    main()
