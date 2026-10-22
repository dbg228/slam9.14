"""Known held-out bias must survive CSV reporting; excluded windows must not dilute it."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest
from mocap_anchored_mapping.core import record,save,sha,validation_errors

spec=importlib.util.spec_from_file_location('static_points_report',Path(__file__).resolve().parents[1]/'tools/static_points_report.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def batch_fixture(tmp_path):
    run=tmp_path/'run';batch=run/'batch';seed_dir=run/'restart'
    (batch/'frozen_inputs').mkdir(parents=True);seed_dir.mkdir()
    calibration={'T_map_mocap':record(np.eye(4),'map','mocap_world')}
    seed={'T_map_odom':record(np.eye(4),'map','odom'),'static_spread':{'T_M_B':{'mean_xyz_m':[0.,0.,0.]}}}
    for p in [run/'calibration.yaml',batch/'frozen_inputs/calibration.yaml']:save(p,calibration,True)
    for p in [seed_dir/'initialization.yaml',batch/'frozen_inputs/initialization.yaml']:save(p,seed,True)
    manifest={'run':str(run),'localization_session':str(seed_dir),'data_kind':'software_fixture',
              'planned_validation_points':1,'protected_inputs':{'calibration.yaml':{'sha256':sha(run/'calibration.yaml')}},'captures':[]}
    for label,role,offset in [('heldout','independent_validation',[.03,.04,0]),('repeat','excluded_initialization_site_repeat',[1.,0,0])]:
        path=batch/label;path.mkdir();pairs=[]
        for i in range(2):
            h=np.eye(4);h[:3,3]=offset
            pairs.append({'stamp_ns':10**12+i*10**8,'mocap_stamp_ns':10**12+i*10**8,'skew_s':0.,'T_M_B':np.eye(4).tolist(),'T_F_B':h.tolist()})
        save(path/'pairs.json',pairs);(path/'raw.jsonl').write_text('{"software_fixture":true}\n')
        summary=validation_errors(pairs,calibration,np.eye(4))
        summary.update(raw_sha256=sha(path/'raw.jsonl'),pairs_sha256=sha(path/'pairs.json'),
                       calibration_sha256=sha(run/'calibration.yaml'),raw_normalized_check={'coverage':1.})
        save(path/'summary.json',summary)
        manifest['captures'].append({'path':label,'label':label,'role':role})
    save(batch/'session.json',manifest)
    return batch


def test_known_bias_and_exclusion_preserved(tmp_path):
    batch=batch_fixture(tmp_path)
    result=module.report(batch)
    assert result['data_kind']=='software_fixture' and result['complete']
    assert result['pooled_sample_statistics']['error_3d_m']['n']==2
    assert result['pooled_sample_statistics']['error_3d_m']['rmse']==pytest.approx(.05)
    assert result['equal_weight_position_mean_3d_error_m']['rmse']==pytest.approx(.05)
    assert result['excluded_windows']==1 and not result['transform_updated']
    assert (batch/'samples.csv').read_text().count('\n')==5
    assert (batch/'positions.png').is_file() and (batch/'errors.png').is_file()


def test_changed_raw_data_rejected(tmp_path):
    batch=batch_fixture(tmp_path);(batch/'heldout/raw.jsonl').write_text('changed\n')
    with pytest.raises(ValueError,match='Capture hash mismatch'):module.report(batch)
