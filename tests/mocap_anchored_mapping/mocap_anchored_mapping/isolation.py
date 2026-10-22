"""Keep native FAST-LIO's hardcoded ROOT_DIR debug/exit exports per run.

Only the mapper process gets a private mount namespace. No source/binary patch,
network namespace, ROS remapping change, or host-directory replacement is made.
"""
from pathlib import Path
import shlex
import shutil
import subprocess
import time


def native_prefix(run,mode,cfg,capture_root=None):
    bwrap=shutil.which('bwrap')
    if not bwrap:raise ValueError('bubblewrap is required to isolate native hardcoded Log/PCD paths')
    source=Path(cfg['fastlio_source']).resolve().parents[1]
    for child in ('Log','PCD'):
        if not (source/child).is_dir():raise ValueError('Audited FAST-LIO auxiliary directory missing:'+str(source/child))
    auxiliary=Path(run)/'native_aux' if mode=='mapping' else (Path(capture_root) if capture_root else Path(run)/'localizations'/str(time.time_ns()))/'native_aux'
    args=[bwrap,'--die-with-parent','--bind','/','/']
    for child in ('Log','PCD'):
        destination=auxiliary/child;destination.mkdir(parents=True,exist_ok=False)
        args += ['--bind',str(destination),str(source/child)]
    # Verify namespace availability before starting any mapping node.
    subprocess.run([*args,'--','/bin/true'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    return shlex.join([*args,'--'])
