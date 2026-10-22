"""Summarize all attempts, using independent trials rather than pooled frames."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--tracking',type=Path,required=True)
    ap.add_argument('--obstacles',type=Path)
    ap.add_argument('--historical',type=Path)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    rows=json.loads((args.tracking/'summary.json').read_text())
    lines=['# 控制器实验结果','',
           '本轮范围：按用户要求，巡检阶段不考虑避障，仅验收固定轨迹跟踪。','',
           '**以下控制对比全部为合成闭环仿真，未运行 FAST-LIO、NDT 或实车。**',
           '原基线直接调用已安装 Nav2 MPPI 插件；统一仿真执行链、路径和同种子状态扰动。',
           '固定参数：20 Hz、50×0.05 s、MPPI各1000样本、速度上限0.20 m/s、角速度上限0.45 rad/s。',
           '名义执行滞后0.25 s、延迟0.10 s；留出条件为0.38 s、0.15 s及0.008 rad/s角速度偏置，控制器参数不随之修改。','',
           '每个数字先在单轮试次计算，再对左右弯、S弯及三个种子的试次取均值。失败保留分母。',
           '横向误差为有限路径线段投影点处的切线法向误差；“完成”为几何到达，不是停车精度。','',
           '|执行条件|方法|完成/尝试|弯道横向RMSE均值 mm|弯道航向RMSE均值 °|弯道平均速度 m/s|计算P95均值 ms|超50ms调用数|',
           '|---|---|---:|---:|---:|---:|---:|---:|']
    groups=[]
    for plant in dict.fromkeys(r['plant'] for r in rows):
        for method in dict.fromkeys(r['method'] for r in rows):
            rr=[r for r in rows if r['plant']==plant and r['method']==method]
            def mean(section,key,scale=1):
                values=[(r.get(section,{}) if section else r).get(key) for r in rr]
                a=[v*scale for v in values if v is not None]
                return float(np.mean(a)) if a else None
            item={'plant':plant,'method':method,'attempts':len(rr),
                  'completed':sum(r['complete'] for r in rr),
                  'turn_rmse_mm':mean('turn','rmse_m',1000),
                  'turn_heading_rmse_deg':mean('turn','heading_rmse_rad',180/np.pi),
                  'turn_mean_speed_mps':mean('turn','mean_speed_mps'),
                  'compute_p95_mean_ms':mean(None,'compute_p95_ms'),
                  'deadline_misses':sum(r.get('deadline_misses',0) for r in rr),
                  'failures':[r['failure'] for r in rr if r.get('failure')]}
            groups.append(item)
            def fmt(x,d=3):return f'{x:.{d}f}' if x is not None else '缺测'
            lines.append(f"|{plant}|{method}|{item['completed']}/{item['attempts']}|{fmt(item['turn_rmse_mm'])}|{fmt(item['turn_heading_rmse_deg'])}|{fmt(item['turn_mean_speed_mps'])}|{fmt(item['compute_p95_mean_ms'],2)}|{item['deadline_misses']}|")
    lines+=['','模式：`native`=原InspectionStrict插件；`tracker`=预瞄跟踪器和同一可行性检查；',
            '`ideal`=去掉执行模型及其补偿的残差采样；`full`=完整方法。',
            '应同时阅读纯跟踪器成绩：组合方案没有预设必须在每种场景优于纯跟踪器。',
            '计算耗时未回灌为额外仿真延迟，以上不能证明真实CPU负载下始终满足20 Hz。','']
    lines+=['原生插件已通过pluginlib工厂加载，未修改其源码或权重。采样引擎设置种子后，长序列仍观察到非逐位可重复，',
            '因此这里只给多个实际试次的描述统计；不能把三个种子视为严格统计显著性验证，详见native/README.md。','']
    obstacle_groups=[]
    if args.obstacles:
        obstacles=json.loads((args.obstacles/'summary.json').read_text())
        lines+=['## 单独的静态圆障碍测试','',
                '原配置障碍代价关闭，因此不把它作为自主避障基线。跟踪和避障结果不能混为一个成绩。',
                'obstacle是参考线侧面的圆障碍，需要小幅绕行；blocked是参考线上大圆，检验有限预瞄下的保守停车，周围并非封死的通道。','',
                '|场景|执行条件|方法|几何完成/尝试|碰撞次数|停车次数|最小实际矩形足迹间隙 m|',
                '|---|---|---|---:|---:|---:|---:|']
        for scenario in dict.fromkeys(r['scenario'] for r in obstacles):
            for plant in dict.fromkeys(r['plant'] for r in obstacles):
                for method in dict.fromkeys(r['method'] for r in obstacles):
                    rr=[r for r in obstacles if (r['scenario'],r['plant'],r['method'])==(scenario,plant,method)]
                    gaps=[r['min_true_footprint_clearance_m'] for r in rr if r.get('min_true_footprint_clearance_m') is not None]
                    item={'scenario':scenario,'plant':plant,'method':method,'attempts':len(rr),
                          'completed':sum(r['complete'] for r in rr),
                          'collisions':sum(r.get('failure')=='footprint_collision' for r in rr),
                          'blocked_stops':sum(r.get('failure')=='blocked_stop' for r in rr),
                          'min_clearance_m':min(gaps) if gaps else None}
                    obstacle_groups.append(item)
                    gap=f"{item['min_clearance_m']:.4f}" if gaps else '缺测'
                    lines.append(f"|{scenario}|{plant}|{method}|{item['completed']}/{len(rr)}|{item['collisions']}|{item['blocked_stops']}|{gap}|")
        lines+=['','障碍调参仅使用名义seed0开发场景；留出执行条件和seed1、2用于额外检查。',
                '初版停滞及调参候选结果保存在其他development目录，未当作最终成功试次。','']
    historical=[]
    if args.historical:
        historical=json.loads((args.historical/'summary.json').read_text())
        lines+=['## 同一旧参考路径的补充仿真','',
                '读取旧NDT实验冻结的47.7米紫线路径，位置点不改，仅按相邻线段构造切线。',
                '这里复用了旧路径；状态和执行过程仍是仿真，没有重新运行旧定位算法。每种方法仅一轮，不能与历史实测误差直接比较。','',
                '|方法|完成|全程横向RMSE mm|弯道横向RMSE mm|弯道平均速度 m/s|用时 s|',
                '|---|---:|---:|---:|---:|---:|']
        for r in historical:
            lines.append(f"|{r['method']}|{int(r['complete'])}|{r['all']['rmse_m']*1000:.3f}|{r['turn']['rmse_m']*1000:.3f}|{r['turn']['mean_speed_mps']:.3f}|{r['time_s']:.2f}|")
        lines+=['','![旧路径补充仿真](results/historical_route/historical_route_nominal.png)','']
    lines+=['## 证据边界','',
            '历史原FAST-LIO＋NDT数据的弯道横向RMSE约39.9 mm、直线约49.9 mm；弯道航向RMSE约7.0°、直线约3.4°。',
            '这些是对冻结紫线的地图估计误差，混合定位误差，不能与本表合成真值精度直接比较。',
            '模型延迟/滞后为假设，尚未完成实车辨识。巡检模式按要求不考虑避障。',
            '本实验不证明全球首次创新、稳定性或最优性。真实同FAST-LIO＋NDT对照仍需实车闭环与独立真值。','']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text('\n'.join(lines))
    args.output.with_suffix('.json').write_text(json.dumps({'tracking':groups,'obstacles':obstacle_groups,'historical':historical},indent=2)+'\n')
    print(args.output)


if __name__=='__main__':main()
