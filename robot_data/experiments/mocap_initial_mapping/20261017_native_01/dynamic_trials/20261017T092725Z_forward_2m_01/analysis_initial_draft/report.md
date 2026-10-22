# 本轮自动前进 2 m 定位实验

状态：aborted；真实传感器数据；固定速度请求 0.15m/s。

复用旧 MotionGate 的动捕 base_link 位移停止条件；保留手柄 STOP/MANUAL 接管，独立进程发布底盘命令，底盘看门狗临时配置为0.25s。
位移2m是发零触发距离，实际停车可能有过冲。位姿采用本次固定世界标定及已冻结的map→odom，不重新拟合，不用NDT。
动捕单位米；两边参考点base_link；原始消息见raw.jsonl与bag/，CSV缺失参考留空。曝光延迟/时差未独立核实，动态指标为初步结果。

```json
{
  "data_kind": "real_2m_dynamic_localization_test",
  "state": {
    "group": "A",
    "direction": 1,
    "state": "ABORTED",
    "reason": "mocap_stale",
    "displacement_m": 0.2012228057438651,
    "path_length_m": 0.20177131166073067,
    "mocap_stamp_ns": 1792229722101629186,
    "ros_stamp_ns": 1792229727341005550,
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
    "ros_stamp_ns": 1792229722418826437
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "request_recorder_mocap_localization_or_mode_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792229727458699906
  },
  "events": [
    {
      "event": "motion_start",
      "steady_s": 31420.35369699,
      "start_base_pose": {
        "stamp_ns": 1792229720011681136,
        "valid": true,
        "x": -1.1010060715011931,
        "y": -0.14539369097638025,
        "z": 0.20860183289841885,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.030646191288602704,
        "qw": 0.9995302951684368
      },
      "direction": 1,
      "group": "A",
      "ros_stamp_ns": 1792229720025211641,
      "event_id": 0
    },
    {
      "event": "stop_command",
      "steady_s": 31422.6537992,
      "reason": "mocap_stale",
      "displacement_m": 0.2012228057438651,
      "path_length_m": 0.20177131166073067,
      "mocap_stamp_ns": 1792229722101629186,
      "group": "A",
      "ros_stamp_ns": 1792229722320910912,
      "event_id": 1
    }
  ],
  "motion_started": true,
  "reference_coverage": 0.9125,
  "XY_error_m": {
    "n": 73,
    "rmse": 0.008727693169446382,
    "p95_abs": 0.009590278129853205,
    "max_abs": 0.02018450082816122,
    "mean": 0.008599086829094375,
    "std": 0.0014927604515449504
  },
  "yaw_error_deg": {
    "n": 73,
    "rmse": 0.040263295105344106,
    "p95_abs": 0.05462688215143923,
    "max_abs": 0.07111875704153636,
    "mean": 0.03822645227007274,
    "std": 0.012644021495706061
  },
  "phases": {
    "pre_motion": {
      "total": 79,
      "XY_error_m": {
        "n": 77,
        "rmse": 0.008110739563938894,
        "p95_abs": 0.008874314921783488,
        "max_abs": 0.009566324162359129,
        "mean": 0.008093132652187597,
        "std": 0.0005341349531145164
      }
    },
    "motion": {
      "total": 23,
      "XY_error_m": {
        "n": 21,
        "rmse": 0.00836085518176647,
        "p95_abs": 0.009631886439878229,
        "max_abs": 0.009681706655549231,
        "mean": 0.008322754980581474,
        "std": 0.0007972759269396722
      }
    },
    "post_stop": {
      "total": 57,
      "XY_error_m": {
        "n": 52,
        "rmse": 0.008871540454214967,
        "p95_abs": 0.009408404802961694,
        "max_abs": 0.02018450082816122,
        "mean": 0.00871068238330151,
        "std": 0.0016817379248900352
      }
    }
  },
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "6324b7a5439685c35bac4a7a4c1864e9aa06d22fcf7ebe7ea6885f0d9212e80d",
  "post_stop_2_to_5s_XY_error_m": {
    "n": 30,
    "rmse": 0.008378242192273429,
    "p95_abs": 0.00919518647348661,
    "max_abs": 0.009408288015452546,
    "mean": 0.008364412903014245,
    "std": 0.00048118501668224827
  },
  "stopped_displacement_m": 0.2368005528739258
}
```
