#!/usr/bin/env python3
"""Read-only preservation audit, including prefix check of pre-existing live logs."""
import hashlib
import json
from pathlib import Path
import sys

root=Path(__file__).resolve().parents[3]
before=json.loads((root/'docs/mocap_anchored_mapping/protected_before.json').read_text())
unchanged=[];appended=[];changed=[]
for name,expected in before.items():
    p=Path(name)
    if not p.is_file():changed.append(dict(path=name,reason='missing'));continue
    payload=p.read_bytes();actual=hashlib.sha256(payload).hexdigest()
    if actual==expected['sha256']:unchanged.append(name)
    elif p.suffix=='.log' and len(payload)>expected['bytes'] and hashlib.sha256(payload[:expected['bytes']]).hexdigest()==expected['sha256']:
        appended.append(dict(path=name,before_bytes=expected['bytes'],after_bytes=len(payload),
            original_prefix_sha256=expected['sha256'],current_sha256=actual,reason='existing runtime appended; original bytes identical'))
    else:changed.append(dict(path=name,reason='hash_changed',current_sha256=actual))
result=dict(protected_count=len(before),unchanged=unchanged,live_logs_appended=appended,changed_or_missing=changed,
    original_maps_transforms_algorithms_unchanged=not changed)
if len(sys.argv)>1:Path(sys.argv[1]).write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(dict(protected_count=len(before),unchanged_count=len(unchanged),live_logs_appended=appended,changed_or_missing=changed),indent=2,ensure_ascii=False))
sys.exit(bool(changed))
