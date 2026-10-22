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
    "displacement_m": 0.0,
    "path_length_m": 0.0,
    "mocap_stamp_ns": 1792230331422403150,
    "ros_stamp_ns": 1792230336451539816,
    "request_linear_x": 0.0,
    "request_angular_z": 0.0,
    "duplicates": 0
  },
  "guard_during_trial": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "old_or_invalid_request",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792230331454381912
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "old_or_invalid_request",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792230336753771424
  },
  "events": [
    {
      "event": "stop_command",
      "steady_s": 32031.776365965,
      "reason": "velocity_guard_failure",
      "displacement_m": 0.0,
      "path_length_m": 0.0,
      "mocap_stamp_ns": 1792230331422403150,
      "group": "A",
      "ros_stamp_ns": 1792230331451431762,
      "event_id": 0
    }
  ],
  "motion_started": false,
  "reference_coverage": null,
  "XY_error_m": {
    "n": 0,
    "rmse": null,
    "p95_abs": null,
    "max_abs": null,
    "mean": null,
    "std": null
  },
  "yaw_error_deg": {
    "n": 0,
    "rmse": null,
    "p95_abs": null,
    "max_abs": null,
    "mean": null,
    "std": null
  },
  "phases": {
    "pre_motion": {
      "total": 130,
      "XY_error_m": {
        "n": 118,
        "rmse": 0.011975584968346685,
        "p95_abs": 0.012930295095232866,
        "max_abs": 0.013290747390193277,
        "mean": 0.011956978743592563,
        "std": 0.000667304023190849
      }
    },
    "motion": {
      "total": 0,
      "XY_error_m": {
        "n": 0,
        "rmse": null,
        "p95_abs": null,
        "max_abs": null,
        "mean": null,
        "std": null
      }
    },
    "post_stop": {
      "total": 0,
      "XY_error_m": {
        "n": 0,
        "rmse": null,
        "p95_abs": null,
        "max_abs": null,
        "mean": null,
        "std": null
      }
    },
    "cleanup": {
      "total": 0,
      "XY_error_m": {
        "n": 0,
        "rmse": null,
        "p95_abs": null,
        "max_abs": null,
        "mean": null,
        "std": null
      }
    }
  },
  "evaluation_window": "motion_start through stop_command+5s; pre-motion and cleanup retained separately",
  "stale_native_samples_retained_for_offline_analysis": true,
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "6324b7a5439685c35bac4a7a4c1864e9aa06d22fcf7ebe7ea6885f0d9212e80d"
}
```
