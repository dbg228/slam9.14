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
    "displacement_m": 0.4340151311421837,
    "path_length_m": 0.4349930592446307,
    "mocap_stamp_ns": 1792230471878228454,
    "ros_stamp_ns": 1792230476919696512,
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
    "ros_stamp_ns": 1792230471882930945
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "request_recorder_mocap_localization_or_mode_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792230477171247275
  },
  "events": [
    {
      "event": "motion_start",
      "steady_s": 32168.072662493,
      "start_base_pose": {
        "stamp_ns": 1792230467718404142,
        "valid": true,
        "x": 0.9765219942178625,
        "y": -0.03568382913041401,
        "z": 0.22590361693909042,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.042414358882045006,
        "qw": 0.9991001061758652
      },
      "direction": -1,
      "group": "A",
      "ros_stamp_ns": 1792230467745049440,
      "event_id": 0
    },
    {
      "event": "stop_command",
      "steady_s": 32172.216353507,
      "reason": "velocity_guard_failure",
      "displacement_m": 0.4340151311421837,
      "path_length_m": 0.4349930592446307,
      "mocap_stamp_ns": 1792230471878228454,
      "group": "A",
      "ros_stamp_ns": 1792230471899788578,
      "event_id": 1
    }
  ],
  "motion_started": true,
  "reference_coverage": 0.9888888888888889,
  "XY_error_m": {
    "n": 89,
    "rmse": 0.013425892828278543,
    "p95_abs": 0.015758044983236512,
    "max_abs": 0.019630075726227342,
    "mean": 0.01334789435503965,
    "std": 0.0014451001775385322
  },
  "yaw_error_deg": {
    "n": 89,
    "rmse": 0.11600105661516563,
    "p95_abs": 0.14508049195955275,
    "max_abs": 0.38489749369590864,
    "mean": 0.10973403694320033,
    "std": 0.037612315429699415
  },
  "phases": {
    "pre_motion": {
      "total": 118,
      "XY_error_m": {
        "n": 114,
        "rmse": 0.012465770020694001,
        "p95_abs": 0.01326309271715088,
        "max_abs": 0.013806769454441113,
        "mean": 0.012453587661846198,
        "std": 0.000550977819467784
      }
    },
    "motion": {
      "total": 41,
      "XY_error_m": {
        "n": 41,
        "rmse": 0.0138134760897169,
        "p95_abs": 0.016742085594464926,
        "max_abs": 0.019630075726227342,
        "mean": 0.013684089131764755,
        "std": 0.0018862201131093507
      }
    },
    "post_stop": {
      "total": 49,
      "XY_error_m": {
        "n": 48,
        "rmse": 0.013085744672251402,
        "p95_abs": 0.014820123938508338,
        "max_abs": 0.0150473991282586,
        "mean": 0.01306072798325362,
        "std": 0.0008087633614425057
      }
    },
    "cleanup": {
      "total": 10,
      "XY_error_m": {
        "n": 4,
        "rmse": 0.012754547570139925,
        "p95_abs": 0.013180882997418179,
        "max_abs": 0.013285290892113862,
        "mean": 0.012750809861572163,
        "std": 0.00030875814644537776
      }
    }
  },
  "evaluation_window": "motion_start through stop_command+5s; pre-motion and cleanup retained separately",
  "stale_native_samples_retained_for_offline_analysis": true,
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "6324b7a5439685c35bac4a7a4c1864e9aa06d22fcf7ebe7ea6885f0d9212e80d",
  "motion_duration_s": 4.154739138,
  "post_stop_2_to_5s_XY_error_m": {
    "n": 29,
    "rmse": 0.012860550159764148,
    "p95_abs": 0.013375560293833834,
    "max_abs": 0.01384477376136422,
    "mean": 0.012853683593331923,
    "std": 0.0004202005405986501
  },
  "stopped_displacement_m": 0.44152810923548585
}
```
