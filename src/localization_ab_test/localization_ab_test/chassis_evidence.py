"""Read current hardware process log; this hardware clears homing_done after restore."""
import re
from pathlib import Path


def hardware_readiness(log_text):
    opened=log_text.rfind('Velocity command gate: OPEN')
    restore=log_text.rfind('=== Operation Group 3 complete ===')
    reset=log_text.rfind('=== Operation Group 1 start ===')
    # OPEN plus completed restore after the last reset is stronger evidence
    # than the transient homing_done pulse, which the driver clears normally.
    ready=opened>=0 and restore>opened and opened>reset
    if ready and '[ERROR]' in log_text[opened:]:ready=False
    return dict(ready=ready,last_open_offset=opened,last_restore_offset=restore,last_reset_offset=reset,
                interpretation='driver clears homing_done after successful restore; zero is not a persistent not-homed status')


def current_chassis_evidence():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:exe=(proc/'cmdline').read_bytes().split(b'\0')[0]
        except OSError:continue
        if exe==b'/opt/ros/humble/lib/controller_manager/ros2_control_node':
            candidates=list(Path('/home/zkam/.ros/log').glob('ros2_control_node_'+proc.name+'_*.log'))
            if not candidates:continue
            path=max(candidates,key=lambda p:p.stat().st_mtime)
            with path.open('rb') as stream:
                stream.seek(max(0,path.stat().st_size-4*1024*1024))
                content=stream.read().decode('utf8',errors='replace')
            return dict(process_pid=int(proc.name),log_file=str(path),**hardware_readiness(content))
    return dict(ready=False,reason='No current chassis process log with confirmed restore found')
