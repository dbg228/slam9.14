"""Export the assumed models, local linearizations and paper parameter tables.

These models describe the synthetic experiment. They are not identified robot
models and the local guide linearization is not a stability proof for MPPI.
"""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import sys

import numpy as np
from scipy.linalg import expm
import scipy
import yaml

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from controller import Config


def export(output):
    output.mkdir(parents=True,exist_ok=True)
    cfg=Config(); c=asdict(cfg)
    production=HERE.parents[1]/'robot_data/config/navigation.yaml'
    original=yaml.safe_load(production.read_text())
    model={
        'evidence':'assumed_synthetic_models_not_hardware_identification',
        'controller_parameters':c,
        'sampling':{'residual_noise_std_v_mps':c['noise_v'],'residual_noise_std_w_radps':c['noise_w'],
                    'temporal_recursion':'epsilon[:,j] = 0.80 * epsilon[:,j-1] + 0.60 * innovation[:,j]',
                    'heuristic_weights':'exp(-(J-min(J))/temperature), normalized over feasible proposals',
                    'important_limitation':'No exact importance-sampling correction for the state-dependent guided proposal is derived.'},
        'objective_no_obstacles':{
            'stage':'dt*[140*(1+2*min(abs(kappa),2))*e_y^2 + 10*e_psi^2 + 12*(v-v_ref)^2 + .15*r_v^2 + .05*r_w^2]',
            'terminal':'35*e_y_last^2 + 3*e_psi_last^2 - 2*(s_final-s_initial)',
            'timing':'stage e_y/e_psi and terminal errors use their values before the final integration update, as implemented',
            'mean_acceptance':'re-evaluate weighted residual; accept if feasible and J_mean <= J_best + .05; otherwise select best feasible sample'},
        'guidance':{
            'yaw_command':'clip(v_ref*kappa(s_preview) - k_y*e_y - k_psi*wrap(psi-psi_ref) + tau_w*v_ref^2*kappa_prime(s_preview) + r_w, -wmax, wmax)',
            'preview':'s_preview = s + len(pending)*dt*max(v,.05)',
            'speed_reference':'min(speed, sqrt(2*.30*max(L-s,0)), wmax/max(abs(kappa(s_preview)),1e-5))',
            'speed_command':'clip(v_ref+r_v,0,vmax)'},
        'sensor_perturbation':{'distribution':'independent zero-mean Gaussian samples at 20Hz',
            'std_x_m':.002,'std_y_m':.002,'std_yaw_rad':.003,'std_v_mps':.002,'std_w_radps':.003,
            'generator':'numpy.default_rng(seed+1900)',
            'limitation':'Not a learned or measured FAST-LIO/NDT noise process; no lidar or IMU rendering.'},
        'plant_parameters':{
            'nominal':{'tau_v_s':.12,'tau_w_s':.25,'delay_s':.10,'yaw_bias_radps':0.},
            'heldout':{'tau_v_s':.12,'tau_w_s':.38,'delay_s':.15,'yaw_bias_radps':.008}},
        'plant_timing':{
            'controller_dt_s':.05,'smoother_dt_s':1/30,'body_integration_dt_s':.005,
            'smoother':'clip target and independently rate-limit v/w at each 30Hz event before the 20Hz interval end',
            'within_interval_approximation':'The final smoother snapshot is used for the entire body integration interval; this is not exact multirate event-time integration.',
            'transport':'append smoother snapshot, pop oldest from D=round(delay/dt) queue',
            'body':'10 substeps; z <- z+(1-exp(-h/tau))*(delayed_input+bias-z), then midpoint-yaw pose update',
            'recording':'Original traces store state at t_end; corresponding cmd_v/cmd_w were issued at t_end-dt.'},
        'original_baseline':{'plugin':'nav2_mppi_controller::MPPIController','parameter_section':'InspectionStrict',
            'version':'1.1.20','parameters':original['controller_server']['ros__parameters']['InspectionStrict'],
            'production_navigation_sha256':hashlib.sha256(production.read_bytes()).hexdigest(),
            'limitations':['Original pluginlib factory, empty in-memory costmap; not the full Nav2/FAST-LIO/NDT pipeline.',
                           'Long native traces are not bitwise reproducible after seeding; see native/smoke_test_results.json.',
                           'An upstream helper boundary defect has an independent ASan reproducer; causality in benchmark variance is unestablished.']},
        'scope':'fixed-route inspection tracking; obstacle costs intentionally absent',
        'environment':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,'platform':platform.platform()}}
    (output/'model_specification.json').write_text(json.dumps(model,indent=2,allow_nan=False)+'\n')
    rows=[]
    units={'dt':'s','steps':'steps','samples':'trajectories','speed':'m/s','vmax':'m/s','wmax':'rad/s',
           'accel_v':'m/s^2','accel_w':'rad/s^2','tau_v':'s','tau_w':'s','delay':'s',
           'noise_v':'m/s','noise_w':'rad/s','lateral_gain':'rad/(m*s)','heading_gain':'1/s',
           'half_length':'m','half_width':'m','clearance':'m','temperature':'composite cost units'}
    for key,value in c.items():
        rows.append({'component':'controller_assumption','parameter':key,'value':value,'unit':units[key],
                     'status':'fixed experimental setting; not hardware calibration'})
    for condition,params in model['plant_parameters'].items():
        for key,value in params.items():
            rows.append({'component':'synthetic_plant_'+condition,'parameter':key,'value':value,
                         'unit':'rad/s' if 'bias' in key else 's','status':'assumed plant truth'})
    with (output/'controller_model_parameters.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    # A local analytic reference for the GUIDE only, without delays/saturation.
    # State [e_y,e_psi,w], constant speed, u_w=-k_y e_y-k_psi e_psi.
    matrices=[]
    frequencies=np.geomspace(.01,5.,300)
    frequency_rows=[]
    for name,p in model['plant_parameters'].items():
        tau=p['tau_w_s'];v=c['speed'];ky=c['lateral_gain'];kh=c['heading_gain']
        a=np.array([[0,v,0],[0,0,1],[0,0,-1/tau]])
        b=np.array([[0],[0],[1/tau]])
        acl=a-b@np.array([[ky,kh,0]])
        aug=np.zeros((4,4));aug[:3,:3]=a;aug[:3,3:]=b
        e=expm(aug*c['dt'])
        eig=np.linalg.eigvals(acl)
        matrices.append({'condition':name,'state':['e_y_m','e_psi_rad','w_radps'],
            'A_continuous':a.tolist(),'B_continuous':b.tolist(),
            'E_curvature':[0,-v,0],'E_yaw_bias':[0,0,1/tau],
            'A_discrete_ZOH':e[:3,:3].tolist(),'B_discrete_ZOH':e[:3,3:].tolist(),
            'guide_A_closed_loop_delay_free':acl.tolist(),
            'guide_eigenvalues_real_imag':[[float(z.real),float(z.imag)]for z in eig],
            'scope':'Local fixed-speed straight-path tracking GUIDE only: no smoother, delay, residual optimizer, constraints or state noise. Not a full-controller stability result.'})
        for axis,tau_axis in [('v',p['tau_v_s']),('w',p['tau_w_s'])]:
            for freq in frequencies:
                omega=2*np.pi*freq
                magnitude=1/np.sqrt(1+(omega*tau_axis)**2)
                phase=-np.arctan(omega*tau_axis)-omega*p['delay_s']
                frequency_rows.append({'condition':name,'axis':axis,'frequency_Hz':freq,
                    'magnitude':magnitude,'magnitude_dB':20*np.log10(magnitude),
                    'unwrapped_phase_deg':phase*180/np.pi,'evidence':'analytic_assumed_linear_channel_excludes_nonlinear_smoother'})
    (output/'local_linearizations.json').write_text(json.dumps(matrices,indent=2)+'\n')
    with (output/'model_frequency_response.csv').open('w',newline='')as f:
        w=csv.DictWriter(f,fieldnames=list(frequency_rows[0]));w.writeheader();w.writerows(frequency_rows)
    print(output)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=HERE/'paper/data')
    export(ap.parse_args().output)
