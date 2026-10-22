# 本轮自动前进 2 m 定位实验

状态：completed；真实传感器数据；固定速度请求 0.15m/s。

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
    "displacement_m": 2.0005859229910943,
    "path_length_m": 2.004502245338546,
    "mocap_stamp_ns": 1792229871646436622,
    "ros_stamp_ns": 1792229876689401284,
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
    "ros_stamp_ns": 1792229876693974191
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "recorder_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792229877694058875
  },
  "events": [
    {
      "event": "motion_start",
      "steady_s": 31552.182405736,
      "start_base_pose": {
        "stamp_ns": 1792229851847024100,
        "valid": true,
        "x": -1.0226985899126075,
        "y": -0.23775586362776277,
        "z": 0.20881046661979602,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.029590158826674127,
        "qw": 0.9995621153788353
      },
      "direction": 1,
      "group": "A",
      "ros_stamp_ns": 1792229851853235449,
      "event_id": 0
    },
    {
      "event": "stop_command",
      "steady_s": 31571.985703526,
      "reason": "target_reached",
      "displacement_m": 2.0005859229910943,
      "path_length_m": 2.004502245338546,
      "mocap_stamp_ns": 1792229871646436622,
      "group": "A",
      "ros_stamp_ns": 1792229871669602962,
      "event_id": 1
    }
  ],
  "motion_started": true,
  "reference_coverage": 0.9810606060606061,
  "XY_error_m": {
    "n": 259,
    "rmse": 0.010235799669736229,
    "p95_abs": 0.011430685099399748,
    "max_abs": 0.012381739627083053,
    "mean": 0.010208498170230904,
    "std": 0.00074710105565755
  },
  "yaw_error_deg": {
    "n": 259,
    "rmse": 0.09139045296614313,
    "p95_abs": 0.12959340097969602,
    "max_abs": 0.14062261007805968,
    "mean": 0.08677512114096624,
    "std": 0.02867565595287154
  },
  "phases": {
    "pre_motion": {
      "total": 114,
      "XY_error_m": {
        "n": 105,
        "rmse": 0.009533146595524322,
        "p95_abs": 0.010175240134089979,
        "max_abs": 0.010597854476703562,
        "mean": 0.009523840246669404,
        "std": 0.00042113058269026445
      }
    },
    "motion": {
      "total": 198,
      "XY_error_m": {
        "n": 196,
        "rmse": 0.0104394536001033,
        "p95_abs": 0.011475603768159683,
        "max_abs": 0.012381739627083053,
        "mean": 0.010417474346709697,
        "std": 0.0006770669866085114
      }
    },
    "post_stop": {
      "total": 66,
      "XY_error_m": {
        "n": 63,
        "rmse": 0.00957454756342218,
        "p95_abs": 0.01040609276362229,
        "max_abs": 0.010903445447787726,
        "mean": 0.009558350065630213,
        "std": 0.0005566911774948239
      }
    }
  },
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "6324b7a5439685c35bac4a7a4c1864e9aa06d22fcf7ebe7ea6885f0d9212e80d",
  "post_stop_2_to_5s_XY_error_m": {
    "n": 30,
    "rmse": 0.009575497276129684,
    "p95_abs": 0.010317006097747852,
    "max_abs": 0.010621765667029368,
    "mean": 0.009566034277215762,
    "std": 0.0004256010952759956
  },
  "stopped_displacement_m": 2.009395767213465
}
```
