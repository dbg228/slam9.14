"""Publish the audit without changing observations, registration or original ZIP."""
from pathlib import Path
import json,hashlib,shutil,sys
from datetime import datetime,timezone
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/home/zkam/test_nav');p=Path(__file__).resolve().parent.parent;review=p/'review_01';out=p/'final_report'
old=ROOT/'robot_data/experiments/fixed_world_calibration/20261017T061341Z_ndt_3calib_1test_attempt02'
load=lambda f:json.loads(f.read_text())
def save(f,d):f.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
cal=p/'registration_retry_01';locked=sha(cal/'calibration.yaml')
original_summary=load(out/'summary.json');windows=load(review/'stationary_window_audit.json')['results'][1:]
pre=load(review/'predeparture_window_audit.json')['results'][0]
def metrics(label,session,capture):
 c=yaml.safe_load((session/'calibration.yaml').read_text());d=load(capture);h=np.array(c['map_from_mocap']['homogeneous_matrix']);pairs=d['matched_pairs']
 a=np.array([r['map_xyz_m'] for r in pairs]);b=np.array([r['mocap_xyz_m'] for r in pairs]);e=a-(b@h[:3,:3].T+h[:3,3]);errors=np.linalg.norm(e[:,:2],axis=1)
 return dict(label=label,calibration_sha256=sha(session/'calibration.yaml'),samples=len(pairs),sample_errors_mm=(errors*1000).tolist(),sample_xy_rmse_mm=float(np.sqrt(np.mean(errors**2))*1000),mean_point_xy_error_mm=float(np.linalg.norm(e.mean(axis=0)[:2])*1000),training_xy_rmse_mm=c['errors']['distance_xy_m']['rmse']*1000,nearest_training_m=float(np.linalg.norm(np.array([r['mocap_xyz_m'] for r in c['errors']['points']])-b.mean(axis=0),axis=1).min()),pose_field=c['map_provenance']['calib_01']['ndt_pose_field'],capture=str(capture))
comparison=[metrics('previous_fourth_point',old,old/'validation_retry_02/test_04/summary.json'),metrics('current_initial',cal,p/'initial_capture_06/initial/summary.json'),metrics('current_endpoint_short_window',cal,p/'endpoint_arrival_capture_02/endpoint/summary.json')]
assert abs(comparison[0]['mean_point_xy_error_mm']-53.691374852794425)<1e-8
assert abs(comparison[2]['sample_xy_rmse_mm']-original_summary['endpoint']['xy_m']['rmse']*1000)<1e-8
assert pre['quality_passed']
distance=load(review/'distance_audit.json')
distance['/odom']['meaning']='Recorded /odom twist integral; runtime publisher provenance not certified by bag; large speed spikes; do not use as physical total distance'
distance['/reference_velocity']['meaning']='Commanded-speed integral; not measured travel'
distance['policy']='Decimated polyline lengths are sensitivity diagnostics only; none is an independently measured total. No rate selected to replace the original result.'
save(review/'distance_audit.json',distance)
first=windows[0];stop=first['source_window_ns'][0]
canonical=load(p/'endpoint_arrival_capture_02/endpoint/summary.json')
span=[(v-stop)*1e-9 for v in canonical['source_window_ns']]
fig,axes=plt.subplots(2,1,figsize=(11,8),constrained_layout=True)
r=first['score']['rows'];times=[(v['stamp_ns']-stop)*1e-9 for v in r];values=[v['xy_m']*1000 for v in r]
axes[0].plot(times,values,'o-',color='tab:red',label='All 7 valid paired measurements in first 15 s')
axes[0].axvspan(*span,color='tab:green',alpha=.16,label='Original selected 5 s window (3 measurements)')
for x,y in zip(times,values):axes[0].annotate(f'{y:.1f}',(x,y),xytext=(3,6),textcoords='offset points')
axes[0].set(xlabel='Source time after final zero command [s]',ylabel='XY consistency error [mm]',title='Arrival window audit: frozen original calibration; no refit')
axes[0].grid(alpha=.25);axes[0].legend()
sens=distance['lio_decimation_sensitivity'];axes[1].plot([v['period_s'] for v in sens],[v['polyline_m'] for v in sens],'o-',label='Same LIO positions, first sample per time bin')
axes[1].axhline(distance['reference_length_m'],color='purple',linestyle='--',label='Executed reference length (not measured travel)')
axes[1].set(xlabel='Sampling period [s]',ylabel='Accumulated position polyline [m]',title='Distance sensitivity: does not identify physical ground truth')
axes[1].grid(alpha=.25);axes[1].legend()
for ext in ['png','svg','pdf']:fig.savefig(review/('measurement_audit.'+ext),dpi=180)
plt.close(fig)
gap=max(b['ndt_stamp_ns']-a['ndt_stamp_ns'] for a,b in zip(first['matched_pairs'],first['matched_pairs'][1:]))*1e-9
record=dict(reviewed_utc=datetime.now(timezone.utc).isoformat(),data_kind='offline_audit_of_real_recordings',no_hardware_actions=True,transform_refitted=False,calibration_sha256=locked,
 comparisons=comparison,supplemental_windows=[dict(label=w['label'],quality_passed=w['quality_passed'],failures=w['failures'],samples=w['pair_count'],xy_rmse_mm=w['score']['xy_m']['rmse']*1000,xy_max_mm=w['score']['xy_m']['max_abs']*1000) for w in windows],
 original_short_window_not_retracted_arithmetically=True,stable_baseline_accuracy_established=False,independent_absolute_accuracy_certified=False,
 physical_total_distance_m=None,original_raw_lio_polyline_m=original_summary['distance']['estimated_total_m'],reference_length_m=distance['reference_length_m'],same_lio_1s_sample_polyline_m=distance['lio_decimation_sensitivity'][3]['polyline_m'],
 comparison_explanation='Both sessions used NDT-assisted registration; different camera worlds, fitted transforms, test positions and time windows. Arithmetic and mean-versus-RMSE differences do not explain away the observed difference; no demonstrated algorithm accuracy improvement.',
 quality_note=f'Arrival 0-15/30/60 s windows have all matched samples and raw normalized coverage 1, but max NDT gap {gap:.9f} exceeds frozen 2.5 s by {(gap-2.5)*1000:.6f} ms. Diagnostics only; original quality threshold not relaxed.')
save(review/'review_summary.json',record)
md=f'''# 原 NDT 数据复核：为什么与之前第四点不同

复核结论：**11.85 / 12.99 mm 的逐帧计算没有单位或 RMSE 算术错误；它们只是本次固定配准下的短时窗口一致性，不能作为 NDT 稳定定位精度。63.74 m 是原始 LIO 位置折线累加，不能当作已测得的实际行驶距离。** 原汇总对这两项限制突出不足，本页补正，原始测量、配准、数值和旧压缩包全部保留。

## 与你指出的三点标定后第四点比较

已定位到之前的 [实测报告]({old/'report.md'})：第四点 XY 为 53.691 mm。

| 同一计算口径 | 之前第四点 | 本次终点短窗 |
|---|---:|---:|
| 实际 NDT 配对数 | 3 | 3 |
| 平均位置之间的 XY 距离（之前报告采用） | {comparison[0]['mean_point_xy_error_mm']:.3f} mm | {comparison[2]['mean_point_xy_error_mm']:.3f} mm |
| 各帧 XY 距离的 RMSE（本次报告采用） | {comparison[0]['sample_xy_rmse_mm']:.3f} mm | {comparison[2]['sample_xy_rmse_mm']:.3f} mm |
| 三点训练 XY 拟合 RMSE | {comparison[0]['training_xy_rmse_mm']:.3f} mm | {comparison[2]['training_xy_rmse_mm']:.3f} mm |
| 评价点距最近训练点 | {comparison[0]['nearest_training_m']:.3f} m | {comparison[2]['nearest_training_m']:.3f} m |

之前逐帧 XY 距离是 56.185、56.453、48.728 mm；这次短窗是 3.762、9.975、19.815 mm。独立使用矩阵乘法和 NumPy 复算，分别恢复 53.908 与 12.991 mm 的逐帧 RMSE。均值距离与逐帧 RMSE 的差别无法单独解释本次差异。

两次使用的旧 PCD、地图配置哈希、`base_link` 参考点、0.18 m 动捕安装外参和 `final_map_to_base_at_cloud` 字段一致。**两次都由 NDT 三点参与配准**，不能只用“这次吸收了固定偏差”解释区别。相机移动后重求了新变换，三点位置、评价位置、NDT 运行状态及评价时间也不同；新终点更靠近训练位置。各项影响无法从这两段 3 帧数据中唯一分离，也不能据此声称算法精度提升。

## 终点窗口对结果影响明显

原选窗源时刻为导航成功后 8.943～13.943 s，只有 3 帧。本次以录包中到站时最后非零指令之后的首条零指令为基准，另外固定检查到站后 0～15、0～30、0～60 s，所有区间在计算误差前声明；不重拟合、不按误差选择区间。

| 固定到站区间 | 配对数 | 逐帧 XY RMSE | 质量状态 |
|---|---:|---:|---|
'''
for w in windows:md+=f"|{w['label']}|{w['pair_count']}|{w['score']['xy_m']['rmse']*1000:.3f} mm|诊断，NDT 间隔超限|\n"
md+=f'''
前 15 秒 7 帧包含 41.350、61.067 mm 的较大误差，随后有较小误差；只突出后面的 3 帧会掩盖到站阶段的变化。

这些扩展区间动捕静止、单位、同源时间配对检查通过，原始/米制覆盖率为 100%；但 NDT 最大间隔 {gap:.9f} s，比冻结门槛 2.5 s 超出 {(gap-2.5)*1000:.6f} ms。因此仍标记为质量未通过、仅作诊断，未事后放宽门槛，也不以 31.735 mm 替换原正式短窗。原首个候选因至少 3 帧要求及间隔门槛失败后选择下一个质量通过窗口，虽未按误差筛选，确实得到了较晚的到站窗口。

原正式初始窗口末帧距出发约 199 s；另外按出发前 10 s（排除第一条非零指令）固定复核，5 帧 XY RMSE 为 {pre['score']['xy_m']['rmse']*1000:.3f} mm，质量通过。这个补充窗口不替代原初始记录，同样不能认证独立绝对精度。

![窗口和里程敏感性复核](measurement_audit.png)

## 路程的重新解释

| 同一 LIO 轨迹的取点间隔 | 原始位置折线长度 |
|---|---:|
'''
for v in distance['lio_decimation_sensitivity']:md+=f"|{v['period_s']:.1f} s|{v['polyline_m']:.3f} m|\n"
md+='''
原始位置高频变化显著抬高了累加值。降低取点频率也会丢掉细节及弯道长度，所以 **48.090 m 也不是独立测得的真实路程**，不能挑这个数替换 63.739 m。明确已知的是下发参考路线长度 47.800 m；本轮实际物理总路程没有独立真值。`/odom` 速度还存在大尖峰，其积分也不作为替代真值；底盘指令积分只表示命令，不能当成测量。

后续对比需要固定到站后的采样延时、窗口长度、时间及质量规则，保留全部试次；不能各组分别用被测定位结果重新拟合后，就据残差宣布精度提升。若要评价独立绝对精度，仍需独立地图控制点。当前数据能够保留实车运行、整圈地图估计跟踪误差，以及明确窗口和配准条件下的定位一致性结果。

[逐项复核 JSON](review_summary.json) · [固定到站窗口逐样本数据](stationary_window_audit.json) · [出发前补充窗口](predeparture_window_audit.json) · [路程敏感性](distance_audit.json)

本复核仅离线读正式录包与旧数据，未启动小车、修改雷达或定位参数，未改配准和原始 CSV。补充诊断不能冒充新增实车试次或通过质量门槛的正式成绩。
'''
(review/'review_report.md').write_text(md)
# Preserve the exact pre-review presentation and its inventory before annotations.
before=review/'before_review';before.mkdir(exist_ok=False)
for f in [out/'experiment_report.md',out/'experiment_summary.json',out/'artifact_manifest.json',p/'manifest.json']:
 shutil.copy2(f,before/f.name)
master=out/'experiment_report.md';text=master.read_text();heading,rest=text.split('\n',1)
text=heading+'\n\n**数据复核补正：本轮运行已完成；12.991 mm 仅为较晚 5 秒、3 帧的配准一致性，不能概括稳定定位精度。63.739 m 为原始 LIO 位置累加，受高频抖动明显影响，实际物理路程未独立测定。** [查看与之前第四点的逐项复核](../review_01/review_report.md)。到站后前 15 秒全部 7 帧的 XY RMSE 为 31.735 mm；NDT 最大间隔略超原门槛，保留为诊断。\n'+rest
text=text.replace('| 累计 LIO 估计行驶路程 |','| 原始 LIO 位置累加（诊断，非实际路程） |')
text=text.replace('## 初始、终点定位误差曲线','## 初始、终点短窗定位一致性曲线')
text=text.replace('[下载图表、CSV 与报告压缩包](ndt_baseline_results.zip)','[下载复核后的图表、CSV 与报告](ndt_baseline_reviewed_results.zip)；[原始版本压缩包（保留）](ndt_baseline_results.zip)')
master.write_text(text)
summary=load(out/'experiment_summary.json');summary.update(review='review_01/review_report.md',stable_baseline_accuracy_established=False,physical_total_distance_m=None,
 metric_interpretation_review='Short-window registered consistency only; raw LIO polyline is noise-sensitive and not an independently measured physical distance; see review_01',review_summary='../review_01/review_summary.json')
save(out/'experiment_summary.json',summary)
manifest=load(p/'manifest.json');manifest.update(stage='route_completed_accuracy_comparison_unvalidated',review_report='review_01/review_report.md',review_summary='review_01/review_summary.json',physical_total_distance_m=None,stable_baseline_accuracy_established=False)
save(p/'manifest.json',manifest)
planfile=p.parents[1]/'plan.json';plan=load(planfile);plan['groups'][0].update(stage='route_completed_accuracy_comparison_unvalidated',review_report=str(review/'review_report.md'));save(planfile,plan)
for f in [ROOT/'docs/full_map_loop/NDT_BASELINE.md',ROOT/'docs/full_map_loop/README.md']:
 text=f.read_text();head,body=text.split('\n',1)
 note='\n\n**数据复核补正：12.991 mm 仅代表本轮较晚的 5 秒、3 帧配准一致性，不能概括稳定定位精度；63.739 m 是噪声敏感的原始 LIO 位置累加，实际物理路程未独立测定。** [复核报告](../../robot_data/experiments/full_map_loop/20261018_relocated_01/group1_ndt/trial_01/review_01/review_report.md)。\n'
 text=head+note+body
 text=text.replace('| 累计 LIO 估计路程 |','| 原始 LIO 位置累加（诊断） |')
 f.write_text(text)
assert locked==sha(cal/'calibration.yaml')
print(json.dumps(dict(comparison=comparison,diagnostic_arrival_15s_rmse_mm=record['supplemental_windows'][0]['xy_rmse_mm'],physical_total_distance_m=None,review=str(review/'review_report.md')),ensure_ascii=False,indent=2))
