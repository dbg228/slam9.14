# 本轮自动直行 2 m 定位实验

状态：aborted；数据类别：real_2m_dynamic_localization_test；方向：forward；固定速度请求幅值 0.15m/s。

复用旧 MotionGate 的动捕 base_link 位移停止条件；保留手柄 STOP/MANUAL 接管，独立进程发布底盘命令，底盘看门狗临时配置为0.25s。
位移2m是发零触发距离，实际停车可能有过冲。位姿采用本次固定世界标定及已冻结的map→odom，不重新拟合，不用NDT。
动捕单位米；两边参考点base_link；原始消息见raw.jsonl与bag/。缺失参考不填零；仅在启用CSV导出时生成CSV。曝光延迟/时差未独立核实，动态指标为初步结果。

```json
{
  "data_kind": "real_2m_dynamic_localization_test",
  "state": null,
  "guard_during_trial": null,
  "guard_after_intentional_disarm": null,
  "events": [],
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
      "total": 48,
      "XY_error_m": {
        "n": 20,
        "rmse": 0.00842286212128586,
        "p95_abs": 0.009288583860888792,
        "max_abs": 0.009468644499686958,
        "mean": 0.008408691714007271,
        "std": 0.0004883748283619947
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
  "initialization_sha256": "a1b420a5cae9125fdd757f4e9b0900a588d21fa14b821ad2c627daffa45066fc"
}
```
