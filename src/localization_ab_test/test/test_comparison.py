import json
import pytest


def write_trial(tmp_path,group='B',direction='reverse'):
    (tmp_path/'report').mkdir(parents=True)
    (tmp_path/'manifest.json').write_text(json.dumps(dict(group=group,status='completed',direction=direction)))
    summary=dict(all_motion_and_5s_stop=dict(xy_m=dict(rmse=.012,p95_absolute=.02,max_absolute=.03),yaw_deg=dict(rmse=.2)),
                 endpoint=dict(xy_error_m=.008,physical_displacement_m=2.007),coverage=1.,timing_verified=False,
                 max_adjacent_pose_translation_m=.04,max_adjacent_error_jump_m=.02,trigger_displacement_m=2.001,
                 reference_independent=group!='C')
    (tmp_path/'report/summary.json').write_text(json.dumps(summary))
    (tmp_path/'report/frame_errors.csv').write_text('valid,phase,true_displacement_m,e_xy_m,yaw_error_deg\nTrue,moving,1.0,0.012,0.2\n')


def test_report_conversion_and_direction_identity(tmp_path):
    from localization_ab_test.comparison import load_trial
    write_trial(tmp_path)
    row,frames=load_trial(tmp_path,'B',2)
    assert row['xy_rmse_mm']==pytest.approx(12.) and row['endpoint_mm']==pytest.approx(8.)
    assert row['direction']=='reverse' and len(frames)==1
    with pytest.raises(ValueError):load_trial(tmp_path,'B',1)
    with pytest.raises(ValueError):load_trial(tmp_path,'A',2)


def test_c_retains_direction_and_reference_scope(tmp_path):
    from localization_ab_test.comparison import load_trial,summarize_rows
    write_trial(tmp_path,group='C',direction='reverse')
    row,_=load_trial(tmp_path,'C',1)
    assert not row['reference_independent']
    assert row['metric_label']=='same-mocap consistency'
    rows=[dict(row,trial=i,direction=d,xy_rmse_mm=v) for i,d,v in
          [(1,'reverse',1.),(2,'forward',2.),(3,'reverse',3.)]]
    group=summarize_rows(rows)['C']
    assert group['directions']==['reverse','forward','reverse']
    assert group['mean_per_trial_xy_rmse_mm']==pytest.approx(2.)
    assert group['sample_std_per_trial_xy_rmse_mm']==pytest.approx(1.)
    assert not group['reference_independent']
    summary=json.loads((tmp_path/'report/summary.json').read_text())
    summary['reference_independent']=True
    (tmp_path/'report/summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError,match='non-independent'):load_trial(tmp_path,'C',1)


def test_abc_cli_exports_figures_and_preserves_experiment_conditions(tmp_path,monkeypatch):
    from localization_ab_test.comparison import main
    argv=['compare_trials']
    for group,directions in [('A',['forward','reverse','forward']),
                             ('B',['forward','reverse','forward']),
                             ('C',['reverse','forward','reverse'])]:
        argv.append('--'+group.lower())
        for i,d in enumerate(directions,1):
            path=tmp_path/f'{group}{i}';write_trial(path,group,d);argv.append(str(path))
    output=tmp_path/'output';argv.extend(['--output',str(output)])
    monkeypatch.setattr('sys.argv',argv);main()
    for stem in ['abc_metrics','abc_comparison']:
        for ext in ['png','svg','pdf']:assert (output/f'{stem}.{ext}').stat().st_size>100
    report=json.loads((output/'comparison.json').read_text())
    assert len(report['trials'])==9
    assert report['groups']['C']['directions']==['reverse','forward','reverse']
    assert not report['groups']['C']['reference_independent']
    assert '同源动捕一致性' in (output/'report.md').read_text()
    with pytest.raises(SystemExit):main()  # Never overwrite the previous output.
