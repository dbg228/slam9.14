# 本轮自动直行 2 m 定位实验

状态：aborted；数据类别：real_2m_dynamic_localization_test；方向：reverse；固定速度请求幅值 0.15m/s。

复用旧 MotionGate 的动捕 base_link 位移停止条件；保留手柄 STOP/MANUAL 接管，独立进程发布底盘命令，底盘看门狗临时配置为0.25s。
位移2m是发零触发距离，实际停车可能有过冲。位姿采用本次固定世界标定及已冻结的map→odom，不重新拟合，不用NDT。
动捕单位米；两边参考点base_link；原始消息见raw.jsonl与bag/，CSV缺失参考留空。曝光延迟/时差未独立核实，动态指标为初步结果。

```json
{
  "data_kind": "real_2m_dynamic_localization_test",
  "state": {
    "group": "A",
    "direction": -1,
    "state": "ABORTED",
    "reason": "velocity_guard_failure",
    "displacement_m": 0.0663588586278948,
    "path_length_m": 0.06828282381042301,
    "mocap_stamp_ns": 1792230754860475630,
    "ros_stamp_ns": 1792230759901112867,
    "request_linear_x": 0.0,
    "request_angular_z": 0.0,
    "duplicates": 0
  },
  "guard_during_trial": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "request_recorder_mocap_localization_or_mode_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792230754865582750
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "request_recorder_mocap_localization_or_mode_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792230760172364386
  },
  "events": [
    {
      "event": "motion_start",
      "steady_s": 32454.454164127,
      "start_base_pose": {
        "stamp_ns": 1792230754110496256,
        "valid": true,
        "x": 0.5374558304022243,
        "y": -0.0822804835777897,
        "z": 0.22192020419772898,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.03805074659729067,
        "qw": 0.999275808114751
      },
      "direction": -1,
      "group": "A",
      "ros_stamp_ns": 1792230754125603034,
      "event_id": 0
    },
    {
      "event": "stop_command",
      "steady_s": 32455.197449002,
      "reason": "velocity_guard_failure",
      "displacement_m": 0.0663588586278948,
      "path_length_m": 0.06828282381042301,
      "mocap_stamp_ns": 1792230754860475630,
      "group": "A",
      "ros_stamp_ns": 1792230754881165294,
      "event_id": 1
    }
  ],
  "motion_started": true,
  "reference_coverage": 1.0,
  "XY_error_m": {
    "n": 55,
    "rmse": 0.013294164155668961,
    "p95_abs": 0.014800395727205401,
    "max_abs": 0.019449211399636675,
    "mean": 0.013225372000524151,
    "std": 0.001350679845716635
  },
  "yaw_error_deg": {
    "n": 55,
    "rmse": 0.1075182852027841,
    "p95_abs": 0.13535757576215723,
    "max_abs": 0.14680455169059886,
    "mean": 0.10629331710318406,
    "std": 0.01618370761442461
  },
  "phases": {
    "pre_motion": {
      "total": 117,
      "XY_error_m": {
        "n": 110,
        "rmse": 0.013102977145545466,
        "p95_abs": 0.01424248523935428,
        "max_abs": 0.014960891704039756,
        "mean": 0.013083442245137812,
        "std": 0.0007152266038326938
      }
    },
    "motion": {
      "total": 8,
      "XY_error_m": {
        "n": 8,
        "rmse": 0.015157283743742242,
        "p95_abs": 0.019003739071957224,
        "max_abs": 0.019449211399636675,
        "mean": 0.014980874952266,
        "std": 0.0023057831972850295
      }
    },
    "post_stop": {
      "total": 47,
      "XY_error_m": {
        "n": 47,
        "rmse": 0.012950370099842897,
        "p95_abs": 0.014531833750393682,
        "max_abs": 0.01507858071521922,
        "mean": 0.012926562987461709,
        "std": 0.0007848917467332034
      }
    },
    "cleanup": {
      "total": 10,
      "XY_error_m": {
        "n": 4,
        "rmse": 0.012810674541494578,
        "p95_abs": 0.013204046694924814,
        "max_abs": 0.01324721201402627,
        "mean": 0.012806500530138099,
        "std": 0.0003269959933545358
      }
    }
  },
  "evaluation_window": "motion_start through stop_command+5s; pre-motion and cleanup retained separately",
  "stale_native_samples_retained_for_offline_analysis": true,
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "6324b7a5439685c35bac4a7a4c1864e9aa06d22fcf7ebe7ea6885f0d9212e80d",
  "motion_duration_s": 0.75556226,
  "post_stop_2_to_5s_XY_error_m": {
    "n": 27,
    "rmse": 0.012944648804622831,
    "p95_abs": 0.013801852052589223,
    "max_abs": 0.01394634588304779,
    "mean": 0.012936009836070616,
    "std": 0.00047284479071638407
  },
  "stopped_displacement_m": 0.07414444243374016
}
```
