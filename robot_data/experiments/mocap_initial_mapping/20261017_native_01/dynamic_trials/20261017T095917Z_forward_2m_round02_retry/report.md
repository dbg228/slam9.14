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
    "displacement_m": 2.000830549081907,
    "path_length_m": 2.006812877425923,
    "mocap_stamp_ns": 1792231235537781242,
    "ros_stamp_ns": 1792231240584720490,
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
    "ros_stamp_ns": 1792231240572218272
  },
  "guard_after_intentional_disarm": {
    "armed": true,
    "moving": false,
    "ready": false,
    "latched_failure": "recorder_not_ready",
    "mode": "EXTERNAL",
    "ros_stamp_ns": 1792231241612622693
  },
  "events": [
    {
      "event": "motion_start",
      "steady_s": 32916.117334416,
      "start_base_pose": {
        "stamp_ns": 1792231215728296747,
        "valid": true,
        "x": -1.1006133487642402,
        "y": -0.20799735136413525,
        "z": 0.20845247551032928,
        "qx": 0.0,
        "qy": 0.0,
        "qz": -0.019944132130352277,
        "qw": 0.999801096015386
      },
      "direction": 1,
      "group": "A",
      "ros_stamp_ns": 1792231215790069581,
      "event_id": 0
    },
    {
      "event": "stop_command",
      "steady_s": 32935.873311241,
      "reason": "target_reached",
      "displacement_m": 2.000830549081907,
      "path_length_m": 2.006812877425923,
      "mocap_stamp_ns": 1792231235537781242,
      "group": "A",
      "ros_stamp_ns": 1792231235544083961,
      "event_id": 1
    }
  ],
  "motion_started": true,
  "reference_coverage": 0.9879032258064516,
  "XY_error_m": {
    "n": 245,
    "rmse": 0.005052032812489003,
    "p95_abs": 0.00688999596579822,
    "max_abs": 0.00976066304681944,
    "mean": 0.004897068540917228,
    "std": 0.0012416743711715916
  },
  "yaw_error_deg": {
    "n": 245,
    "rmse": 0.0669023424450064,
    "p95_abs": 0.09499749959856511,
    "max_abs": 0.11732026786777813,
    "mean": 0.06251145688799692,
    "std": 0.02383780993231146
  },
  "phases": {
    "pre_motion": {
      "total": 113,
      "XY_error_m": {
        "n": 110,
        "rmse": 0.003104434183411763,
        "p95_abs": 0.003993906886075162,
        "max_abs": 0.004516367660012367,
        "mean": 0.003056656516167523,
        "std": 0.0005425518789075188
      }
    },
    "motion": {
      "total": 198,
      "XY_error_m": {
        "n": 195,
        "rmse": 0.005370492247916689,
        "p95_abs": 0.007052922393657872,
        "max_abs": 0.00976066304681944,
        "mean": 0.005254936712539586,
        "std": 0.0011080736131398902
      }
    },
    "post_stop": {
      "total": 50,
      "XY_error_m": {
        "n": 50,
        "rmse": 0.0035465962410798166,
        "p95_abs": 0.004439069940671578,
        "max_abs": 0.004878673564154378,
        "mean": 0.0035013826715900222,
        "std": 0.0005645035733550346
      }
    },
    "cleanup": {
      "total": 18,
      "XY_error_m": {
        "n": 14,
        "rmse": 0.00366973048832342,
        "p95_abs": 0.0043643801991100056,
        "max_abs": 0.004376388477815157,
        "mean": 0.0036329552321436973,
        "std": 0.000518225952814194
      }
    }
  },
  "evaluation_window": "motion_start through stop_command+5s; pre-motion and cleanup retained separately",
  "stale_native_samples_retained_for_offline_analysis": true,
  "timing_verified": false,
  "matching": "nearest within 25ms; no interpolation, extrapolation, zero-fill or alignment fit",
  "calibration_sha256": "8882e30413600bff0f61daf976a967eec171e5c360dd9ac73518c5a4999ecf82",
  "initialization_sha256": "a1b420a5cae9125fdd757f4e9b0900a588d21fa14b821ad2c627daffa45066fc",
  "motion_duration_s": 19.75401438,
  "post_stop_2_to_5s_XY_error_m": {
    "n": 30,
    "rmse": 0.0034295629896829855,
    "p95_abs": 0.004242335783044418,
    "max_abs": 0.004878673564154378,
    "mean": 0.0033898905378541722,
    "std": 0.0005201388675825377
  },
  "stopped_displacement_m": 2.009340909593165
}
```

## 运行条件与停车复核

本轮前静止重启局部 LIO 并关闭累积地图显示发布，使用新的冻结初始化；固定世界标定和原生 PCD 未改。不能把与上一组的误差差异直接归因为算法改进。

原监督程序末尾 `final_command_zero=false` 保留；独立复核见 `post_stop_verification.json`：481 条最终命令均为零，自动速度发布者已退出。
