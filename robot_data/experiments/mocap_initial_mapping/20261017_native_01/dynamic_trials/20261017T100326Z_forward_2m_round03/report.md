# 本轮自动直行 2 m 定位实验

状态：completed；数据类别：real_2m_dynamic_localization_test；方向：forward；固定速度请求幅值 0.15m/s。

复用旧 MotionGate 的动捕 base_link 位移停止条件；保留手柄 STOP/MANUAL 接管，独立进程发布底盘命令，底盘看门狗临时配置为0.25s。
位移2m是发零触发距离，实际停车可能有过冲。位姿采用本次固定世界标定及已冻结的map→odom，不重新拟合，不用NDT。
动捕单位米；两边参考点base_link；原始消息见raw.jsonl与bag/，CSV缺失参考留空。曝光延迟/时差未独立核实，动态指标为初步结果。

```json
{
  "data_kind": "real_2m_dynamic_localization_test",
  "state": {
    "group": "A",
    "direction": 1,
    "state": "DONE",
    "reason": "target_reached",
    "displacement_m": 2.0003861358576738,
    "path_length_m": 2.004077539663757,
    "mocap_stamp_ns": 1792231440362556262,
    "ros_stamp_ns": 1792231445401573346,
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
    "ros_stamp_ns": 1792231445405822662
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "recorder_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792231446446022441
  },
  "events": [
    {
      "event": "motion_start",
      "steady_s": 33120.934390277,
      "start_base_pose": {
        "stamp_ns": 1792231420582984436,
        "valid": true,
        "x": -1.1406710974089376,
        "y": -0.1894721099707711,
        "z": 0.20836420689645202,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.020318660178540022,
        "qw": 0.9997935547144465
      },
      "direction": 1,
      "group": "A",
      "ros_stamp_ns": 1792231420607281134,
      "event_id": 0
    },
    {
      "event": "stop_command",
      "steady_s": 33140.699013237,
      "reason": "target_reached",
      "displacement_m": 2.0003861358576738,
      "path_length_m": 2.004077539663757,
      "mocap_stamp_ns": 1792231440362556262,
      "group": "A",
      "ros_stamp_ns": 1792231440381261843,
      "event_id": 1
    }
  ],
  "motion_started": true,
  "reference_coverage": 0.9838709677419355,
  "XY_error_m": {
    "n": 244,
    "rmse": 0.005409137852668047,
    "p95_abs": 0.0065609160727721156,
    "max_abs": 0.00727961998185394,
    "mean": 0.0053665170586991735,
    "std": 0.0006776923843876774
  },
  "yaw_error_deg": {
    "n": 244,
    "rmse": 0.056267677636991115,
    "p95_abs": 0.08973300752675278,
    "max_abs": 0.1097995204163577,
    "mean": 0.04882048991547357,
    "std": 0.027975190992618658
  },
  "phases": {
    "pre_motion": {
      "total": 104,
      "XY_error_m": {
        "n": 100,
        "rmse": 0.006324732766068917,
        "p95_abs": 0.0071168551041178064,
        "max_abs": 0.0076498367416881685,
        "mean": 0.006304642807759141,
        "std": 0.0005037098656535401
      }
    },
    "motion": {
      "total": 198,
      "XY_error_m": {
        "n": 196,
        "rmse": 0.005459788828679496,
        "p95_abs": 0.006634325846533895,
        "max_abs": 0.00727961998185394,
        "mean": 0.005415410057924356,
        "std": 0.0006947142997701579
      }
    },
    "post_stop": {
      "total": 50,
      "XY_error_m": {
        "n": 48,
        "rmse": 0.0051971923688360015,
        "p95_abs": 0.005988515990419346,
        "max_abs": 0.006394760113974983,
        "mean": 0.0051668706451963465,
        "std": 0.0005605856352917556
      }
    },
    "cleanup": {
      "total": 19,
      "XY_error_m": {
        "n": 15,
        "rmse": 0.005112399639637846,
        "p95_abs": 0.005825784603059313,
        "max_abs": 0.00586085632903603,
        "mean": 0.005094537384861649,
        "std": 0.00042698818439883144
      }
    }
  },
  "evaluation_window": "motion_start through stop_command+5s; pre-motion and cleanup retained separately",
  "stale_native_samples_retained_for_offline_analysis": true,
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "a1b420a5cae9125fdd757f4e9b0900a588d21fa14b821ad2c627daffa45066fc",
  "motion_duration_s": 19.773980709,
  "post_stop_2_to_5s_XY_error_m": {
    "n": 30,
    "rmse": 0.005303142657230679,
    "p95_abs": 0.006020359844882629,
    "max_abs": 0.006394760113974983,
    "mean": 0.005271810537266039,
    "std": 0.0005756176700818129
  },
  "stopped_displacement_m": 2.008144285174462
}
```
