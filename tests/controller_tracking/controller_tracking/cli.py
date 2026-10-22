"""Preview -> freeze -> check -> record -> explicit manual execute -> analyze."""
import argparse
import json
from pathlib import Path
import yaml
import numpy as np
from .common import write_json
from .configuration import load, check_session
from .geometry import geometry, swept, site_check, endpoint_diagnostic


def preview(cfg,kind,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    path=geometry(cfg,kind);out=Path(output);out.mkdir(parents=True,exist_ok=False)
    production=yaml.safe_load(Path(cfg['production_nav']).read_text())
    hull=swept(path,cfg);xy=np.array(path['points']);s=path['local']['s'];curv=path['local']['curvature']
    fig,axes=plt.subplots(1,3,figsize=(14,4))
    axes[0].plot(xy[:,0],xy[:,1],label=kind)
    for box in hull[::max(1,len(hull)//20)]:axes[0].fill(box[[0,1,3,2,0],0],box[[0,1,3,2,0],1],alpha=.08)
    for key,color in [('stable_mocap_bounds','green'),('clear_floor_bounds','orange')]:
        b=cfg['site'][key]
        if b:axes[0].plot([b[0],b[1],b[1],b[0],b[0]],[b[2],b[2],b[3],b[3],b[2]],color=color,label=key)
    axes[0].set_aspect('equal');axes[0].legend(fontsize=8);axes[0].set_xlabel('X forward [m]');axes[0].set_ylabel('Y left [m]')
    axes[1].plot(s,np.unwrap(xy[:,2]));axes[1].set_ylabel('tangent yaw [rad]');axes[2].plot(s,curv);axes[2].set_ylabel('curvature [1/m]')
    for ax in axes:ax.grid()
    fig.tight_layout();fig.savefig(out/'preview.png',dpi=130);plt.close(fig)
    write_json(out/'path.json',path)
    bounds=hull.reshape(-1,2)
    report=dict(site_check=site_check(path,cfg),required_swept_bounds=[float(bounds[:,0].min()),float(bounds[:,0].max()),float(bounds[:,1].min()),float(bounds[:,1].max())],
        preview_dimensions_only=True,endpoint=endpoint_diagnostic(path,production),
        motion='No goal or velocity is sent by preview')
    write_json(out/'preview_check.json',report)
    return path,report


def _main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('preview');p.add_argument('--config',required=True);p.add_argument('--kind',choices=['line','left_arc','right_arc'],required=True);p.add_argument('--output',required=True);p.add_argument('--publish',action='store_true')
    p=sub.add_parser('freeze');p.add_argument('--config',required=True);p.add_argument('--output',required=True)
    p=sub.add_parser('check');p.add_argument('--session',required=True);p.add_argument('--output',required=True)
    p=sub.add_parser('record');p.add_argument('--session',required=True);p.add_argument('--run',required=True);p.add_argument('--kind',choices=['line','left_arc','right_arc'],required=True);p.add_argument('--seconds',type=float)
    p=sub.add_parser('execute',help='Explicit manual start of the real car; never called by a launch file');p.add_argument('--run',required=True)
    p=sub.add_parser('analyze');p.add_argument('--run',required=True);p.add_argument('--output')
    p=sub.add_parser('compare');group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--runs',nargs='+');group.add_argument('--batch')
    p.add_argument('--kind',choices=['line','left_arc','right_arc']);p.add_argument('--output',required=True)
    args=parser.parse_args(argv)
    if args.command=='preview':
        path,report=preview(load(args.config),args.kind,args.output);print(json.dumps(report,ensure_ascii=False,indent=2))
        if not args.publish:return
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from nav_msgs.msg import Path as RosPath
        from .experiment import path_message
        rclpy.init();node=Node('path_preview',namespace='tracking')
        pub=node.create_publisher(RosPath,'/tracking/reference_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        msg=path_message(path,node);node.create_timer(.5,lambda:pub.publish(msg))
        try:rclpy.spin(node)
        except KeyboardInterrupt:pass
        finally:node.destroy_node();rclpy.shutdown()
        return
    if args.command in ('analyze','compare'):
        from .analysis import analyze,compare,batch_runs
        result=analyze(args.run,args.output) if args.command=='analyze' else compare(batch_runs(args.batch,args.kind) if args.batch else args.runs,args.output)
        print(json.dumps({k:result.get(k) for k in ('success_rate','reference_data_coverage','lateral_m')},ensure_ascii=False));return
    import rclpy
    from .experiment import freeze,Inspector,record,execute
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        if args.command=='freeze':print(json.dumps(freeze(args.config,args.output),ensure_ascii=False))
        elif args.command=='check':
            cfg,_,_=check_session(args.session);node=Inspector(cfg)
            try:
                report=node.inspect();write_json(args.output,report);print(json.dumps(report['errors'],ensure_ascii=False))
            finally:node.destroy_node()
            if not report['ok']:raise SystemExit(1)
        elif args.command=='record':record(args.session,args.run,args.kind,args.seconds)
        elif args.command=='execute':print(json.dumps(execute(args.run),ensure_ascii=False,indent=2))
    finally:
        if rclpy.ok():rclpy.shutdown()


def main(argv=None):
    try:return _main(argv)
    except (ValueError,RuntimeError,OSError) as error:
        import sys
        print('tracking_test: '+str(error),file=sys.stderr)
        raise SystemExit(2)


if __name__=='__main__':main()
