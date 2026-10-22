# 本轮自动直行 2 m 定位实验

状态：completed；数据类别：real_2m_dynamic_localization_test；方向：forward；固定速度请求幅值 0.15m/s。

复用旧 MotionGate 的动捕 base_link 位移停止条件；保留手柄 STOP/MANUAL 接管，独立进程发布底盘命令，底盘看门狗临时配置为0.25s。
位移2m是发零触发距离，实际停车可能有过冲。位姿采用本次固定世界标定及已冻结的map→odom，不重新拟合，不用NDT。
动捕单位米；两边参考点base_link；原始消息见raw.jsonl与bag/。缺失参考不填零；仅在启用CSV导出时生成CSV。曝光延迟/时差未独立核实，动态指标为初步结果。

```json
{
  "data_kind": "real_2m_dynamic_localization_test",
  "state": {
    "group": "A",
    "direction": 1,
    "state": "DONE",
    "reason": "target_reached",
    "displacement_m": 2.0004765964556355,
    "path_length_m": 2.003051378161665,
    "mocap_stamp_ns": 1792232095055654561,
    "ros_stamp_ns": 1792232100110961314,
    "request_linear_x": 0.0,
    "request_angular_z": 0.0,
    "duplicates": 0
  },
  "guard_during_trial": {
    "armed": true,
    "moving": false,
    "ready": true,
    "latched_failure": "",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792232100108315912
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "recorder_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792232101148514385
  },
  "events": [
    {
      "event": "motion_start",
      "steady_s": 33775.664368448,
      "start_base_pose": {
        "stamp_ns": 1792232075326255848,
        "valid": true,
        "x": -1.1391893686746442,
        "y": -0.1385035995467459,
        "z": 0.2084922425608104,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.00982636156519032,
        "qw": 0.9999517201437228
      },
      "direction": 1,
      "group": "A",
      "ros_stamp_ns": 1792232075336289346,
      "event_id": 0
    },
    {
      "event": "stop_command",
      "steady_s": 33795.39067595,
      "reason": "target_reached",
      "displacement_m": 2.0004765964556355,
      "path_length_m": 2.003051378161665,
      "mocap_stamp_ns": 1792232095055654561,
      "group": "A",
      "ros_stamp_ns": 1792232095073683574,
      "event_id": 1
    }
  ],
  "motion_started": true,
  "reference_coverage": 0.9919028340080972,
  "XY_error_m": {
    "n": 245,
    "rmse": 0.006821003147745696,
    "p95_abs": 0.007974925810659917,
    "max_abs": 0.008482784629008769,
    "mean": 0.0067770687213065395,
    "std": 0.000772931748762624
  },
  "yaw_error_deg": {
    "n": 245,
    "rmse": 0.049876796800961284,
    "p95_abs": 0.08287663845688992,
    "max_abs": 0.11011819904278405,
    "mean": 0.03934932886648311,
    "std": 0.03064841230605169
  },
  "phases": {
    "pre_motion": {
      "total": 109,
      "XY_error_m": {
        "n": 103,
        "rmse": 0.008530114878699434,
        "p95_abs": 0.009543067064723975,
        "max_abs": 0.010583446348125981,
        "mean": 0.008504757601897667,
        "std": 0.0006572351000770552
      }
    },
    "motion": {
      "total": 197,
      "XY_error_m": {
        "n": 195,
        "rmse": 0.006885025366463905,
        "p95_abs": 0.007989620369082703,
        "max_abs": 0.008482784629008769,
        "mean": 0.006842030815345868,
        "std": 0.0007682373452970086
      }
    },
    "post_stop": {
      "total": 50,
      "XY_error_m": {
        "n": 50,
        "rmse": 0.006565353878954833,
        "p95_abs": 0.007706789626513555,
        "max_abs": 0.00836062730211703,
        "mean": 0.006523716554553161,
        "std": 0.0007382370024296411
      }
    },
    "cleanup": {
      "total": 19,
      "XY_error_m": {
        "n": 13,
        "rmse": 0.006477718180923859,
        "p95_abs": 0.007938513396902944,
        "max_abs": 0.008077822129543166,
        "mean": 0.006417840541161261,
        "std": 0.0008787238586145552
      }
    }
  },
  "evaluation_window": "motion_start through stop_command+5s; pre-motion and cleanup retained separately",
  "stale_native_samples_retained_for_offline_analysis": true,
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "a1b420a5cae9125fdd757f4e9b0900a588d21fa14b821ad2c627daffa45066fc",
  "motion_duration_s": 19.737394228000003,
  "post_stop_2_to_5s_XY_error_m": {
    "n": 30,
    "rmse": 0.006631159147560939,
    "p95_abs": 0.007789895137974294,
    "max_abs": 0.00836062730211703,
    "mean": 0.006590980456145278,
    "std": 0.0007288677980211
  },
  "stopped_displacement_m": 2.0080511789225404
}
```

停车独立复核见 `post_stop_verification.json`：872条最终命令均为零，自动速度发布者已退出。监督程序末尾的false标记按原样保留；正式完成时guard正常，子进程退出码均为0。
