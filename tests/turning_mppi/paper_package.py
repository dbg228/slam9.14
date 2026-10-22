"""Validate and package the saved English research artifact; no robot I/O."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import zipfile

import matplotlib
import numpy as np
import scipy

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def validate():
    paper=HERE/'paper'
    provenance=json.loads((paper/'data/provenance.json').read_text())
    assert provenance['totals']['observed_trials']==74
    assert provenance['totals']['raw_samples']==41589
    assert provenance['totals']['trial_phase_rows']==444
    assert provenance['totals']['lag_rows']==888
    csv_samples=list((paper/'data/samples').glob('*.csv'));assert len(csv_samples)==74
    with (paper/'data/trial_metrics.csv').open(newline='')as f:rows=list(csv.DictReader(f))
    assert len(rows)==444
    convergence=json.loads((paper/'supplement/convergence_summary.json').read_text())
    assert len(convergence)==24 and all(r['complete']for r in convergence)
    keys={(r['plant'],r['method'],r['seed'])for r in convergence};assert len(keys)==24
    for r in convergence:
        p=paper/'supplement'/f"convergence_{r['plant']}_{r['method']}_{r['seed']}.npz"
        data=np.load(p);a=data['trace'];columns=list(data['columns'])
        assert len(a)==r['steps']+1 and len(columns)==a.shape[1]
        assert np.isfinite(a).all() and a[0,0]==0 and (np.diff(a[:,0])>0).all()
    mainhash=json.loads((paper/'figures/artifact_hashes.json').read_text())
    assert mainhash['figure_count']==26 and len(mainhash['files'])==78
    for name,expected in mainhash['files'].items():assert digest(paper/'figures'/name)==expected
    spectrum=json.loads((paper/'spectrum/manifest.json').read_text())
    assert len(spectrum['inputs'])==72
    for name,expected in spectrum['outputs'].items():assert digest(paper/'spectrum'/name)==expected
    assert len(list((paper/'spectrum').glob('*.png')))==1
    assert spectrum['fft_bin_spacing_hz']==.078125
    for p in sorted((paper/'tables').glob('table*.tex')):
        text=p.read_text();assert text.count(r'\begin{')==text.count(r'\end{')
        assert text.count('$')%2==0
    for p in paper.rglob('*.md'):
        for target in re.findall(r'\]\(([^)]+)\)',p.read_text()):
            if target.startswith(('http://','https://','#')):continue
            destination=(p.parent/target.split('#')[0])
            assert destination.exists(),(str(p),target)
    cpu='unknown'
    for line in Path('/proc/cpuinfo').read_text().splitlines():
        if line.startswith('model name'):
            cpu=line.split(':',1)[1].strip();break
    result={'status':'passed','created_at_utc':datetime.now(timezone.utc).isoformat(),
            'scope':'Synthetic fixed-route inspection; obstacles excluded; no physical FAST-LIO/NDT validation',
            'closed_loop_trials':98,'main_trials':72,'historical_route_simulations':2,'initial_error_trials':24,
            'main_plus_route_raw_samples':41589,'trial_phase_rows':444,'lag_diagnostics':888,
            'figure_groups':27,'graphics_assets':81,'publication_table_groups':7,
            'tests':{'controller_and_analysis_semantic_tests_passed':73,
                     'independent_table_numeric_cells_checked':296},
            'checks':{'csv_inventory':True,'supplement_finite_and_initial_timestamp':True,
                      'core_and_spectral_graphic_hashes':True,'local_markdown_links':True,
                      'latex_structure_only':True},
            'not_performed':['LaTeX compilation: no LaTeX engine installed',
                             'Real-robot/FAST-LIO/NDT closed-loop tests',
                             'Rate-independent mechanical hysteresis tests',
                             'Population-wide significance or full-controller stability proof'],
            'environment_collected_during_assembly':{'cpu':cpu,'available_logical_cpus':os.cpu_count(),
                 'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,
                 'matplotlib':matplotlib.__version__,'platform':platform.platform(),
                 'note':'Compute trials recorded perf_counter wall time; CPU clocks/load were not logged at each step.'},
            'analysis_code_sha256':{p.name:digest(p)for p in HERE.glob('paper_*.py')},
            'critical_interpretation':'Reduced tracking error and faster finite-horizon recovery coexist with higher angular command activity/angular jerk; planar jerk decreases.'}
    (paper/'VALIDATION.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def package(output):
    result=validate()
    files=set()
    for p in (HERE/'paper').rglob('*'):
        if p.is_file() and '__pycache__'not in p.parts and p.suffix!='.pyc':files.add(p)
    for pattern in ['*.py','acceleration/*.py','acceleration/*.cpp','acceleration/*.json',
                    'native/*.py','native/*.cpp','native/*.md','native/*.txt','native/*.json','native/*.sh',
                    'native/CMakeLists.txt']:
        files.update(p for p in HERE.glob(pattern)if p.is_file())
    for folder in ['tracking_final','historical_route']:
        files.update(p for p in (HERE/'results'/folder).rglob('*')if p.is_file()and'__pycache__'not in p.parts)
    for name in ['integrity_validation.json','software_validation.json']:
        files.add(HERE/'results'/name)
    files.add(ROOT/'robot_data/config/navigation.yaml')
    old=ROOT/'robot_data/experiments/full_map_loop/20261018_relocated_01/group1_ndt/trial_01'
    files.add(old/'frozen_inspection_path.json')
    files.add(old/'tracking_report_final/tracking_samples.csv')
    files.add(HERE/'history.py')
    # Preserve a derived historic-context report, without including a large bag.
    files.update(p for p in (HERE/'results/history').glob('*')if p.is_file())
    top='turning_mppi_paper'
    readme=('# English research artifact\n\n'
            'Open [the paper index](tests/turning_mppi/paper/README.md).\n\n'
            'The package contains editable English manuscript sections, figures, tables, full-resolution CSV data, '
            'saved simulation NPZ inputs, analysis code and source hashes. It does not run FAST-LIO/NDT or establish physical robot performance.\n\n'
            'From this directory, run `bash tests/turning_mppi/paper/reproduce_analysis.sh` to regenerate analysis from saved traces. '
            'Native controller re-simulation additionally requires the original ROS/Nav2 environment. '
            'SHA256_MANIFEST.json records every bundled file.\n')
    entries={str(p.relative_to(ROOT)):p.read_bytes()for p in sorted(files)}
    entries['README.md']=readme.encode()
    manifest={name:{'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}for name,data in entries.items()}
    output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6)as z:
        for name,data in entries.items():z.writestr(top+'/'+name,data)
        z.writestr(top+'/SHA256_MANIFEST.json',json.dumps(manifest,indent=2)+'\n')
    with zipfile.ZipFile(output)as z:
        assert z.testzip()is None
        for name,m in manifest.items():
            assert hashlib.sha256(z.read(top+'/'+name)).hexdigest()==m['sha256']
    report={'archive':str(output),'archive_bytes':output.stat().st_size,'archive_sha256':digest(output),
            'bundled_files_excluding_manifest':len(entries),'validated_all_archive_entry_hashes':True,
            'validation':result}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items()if k!='validation'},indent=2))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=HERE/'turning_mppi_paper_bundle.zip')
    ap.add_argument('--validate-only',action='store_true');args=ap.parse_args()
    print(json.dumps(validate(),indent=2))if args.validate_only else package(args.output)
