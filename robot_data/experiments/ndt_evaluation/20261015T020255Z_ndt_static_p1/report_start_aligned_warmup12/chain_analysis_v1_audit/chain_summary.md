# Localization-chain analysis

All errors are against the same base_link mocap reference and same fixed start alignment.
The comparison timestamp is the NDT input-cloud timestamp, not matching completion or pose-publication time.

## Absolute error
```json
{
  "initial": {
    "samples": 0,
    "metrics_available": false,
    "reason": "time-matched FAST-LIO initial prediction is not recorded in this report"
  },
  "raw": {
    "samples": 38,
    "metrics_available": true,
    "ex_m": {
      "mean": -0.0010462041754963202,
      "std_sample": 0.016480084452394943,
      "rmse": 0.016295414621723917,
      "p95_absolute": 0.027924145143113872,
      "max_absolute": 0.03671951457292688
    },
    "ey_m": {
      "mean": -0.003555960826703847,
      "std_sample": 0.015285588883822218,
      "rmse": 0.015496626118095474,
      "p95_absolute": 0.02904523751464604,
      "max_absolute": 0.03923100558116355
    },
    "xy_m": {
      "rmse": 0.022487462256509348,
      "p95": 0.045166876628862755,
      "max": 0.047834452689861355,
      "mean": 0.018876316997179954
    },
    "yaw_deg": {
      "mean": 0.03977844206792372,
      "std_sample": 0.09906465591900808,
      "rmse": 0.10553611918056005,
      "p95_absolute": 0.18834297668447358,
      "max_absolute": 0.26249105201125983
    },
    "weighting": "one pose per NDT input-cloud timestamp"
  },
  "final": {
    "samples": 38,
    "metrics_available": true,
    "ex_m": {
      "mean": -0.0010462041754963202,
      "std_sample": 0.016480084452394943,
      "rmse": 0.016295414621723917,
      "p95_absolute": 0.027924145143113872,
      "max_absolute": 0.03671951457292688
    },
    "ey_m": {
      "mean": -0.003555960826703847,
      "std_sample": 0.015285588883822218,
      "rmse": 0.015496626118095474,
      "p95_absolute": 0.02904523751464604,
      "max_absolute": 0.03923100558116355
    },
    "xy_m": {
      "rmse": 0.022487462256509348,
      "p95": 0.045166876628862755,
      "max": 0.047834452689861355,
      "mean": 0.018876316997179954
    },
    "yaw_deg": {
      "mean": 0.03977844206792372,
      "std_sample": 0.09906465591900808,
      "rmse": 0.10553611918056005,
      "p95_absolute": 0.18834297668447358,
      "max_absolute": 0.26249105201125983
    },
    "weighting": "one pose per NDT input-cloud timestamp"
  }
}
```

## Adjacent-frame relative motion
```json
{
  "initial": {
    "pairs": 0,
    "metrics_available": false,
    "reason": "fewer than two time-adjacent FAST-LIO initial prediction poses"
  },
  "raw": {
    "pairs": 37,
    "skipped_long_or_nonpositive_gap_pairs": 0,
    "max_gap_seconds": 5.0,
    "estimate_delta_position": {
      "rmse_m": 0.02589388600970997,
      "p95_m": 0.04917210789533774,
      "max_m": 0.08037191404093898
    },
    "reference_delta_position": {
      "rmse_m": 0.0001528381983090037,
      "p95_m": 0.00027769584979344617,
      "max_m": 0.00030086246579059565
    },
    "relative_delta_position_error": {
      "rmse_m": 0.025870265524448464,
      "p95_m": 0.04916019718359162,
      "max_m": 0.0803160651404944
    },
    "relative_delta_yaw_error_deg": {
      "mean": 0.001598428431563885,
      "std_sample": 0.09497135189907642,
      "rmse": 0.09369280028773967,
      "p95_absolute": 0.19981755878223414,
      "max_absolute": 0.2376504674244296
    }
  },
  "final": {
    "pairs": 37,
    "skipped_long_or_nonpositive_gap_pairs": 0,
    "max_gap_seconds": 5.0,
    "estimate_delta_position": {
      "rmse_m": 0.02589388600970997,
      "p95_m": 0.04917210789533774,
      "max_m": 0.08037191404093898
    },
    "reference_delta_position": {
      "rmse_m": 0.0001528381983090037,
      "p95_m": 0.00027769584979344617,
      "max_m": 0.00030086246579059565
    },
    "relative_delta_position_error": {
      "rmse_m": 0.025870265524448464,
      "p95_m": 0.04916019718359162,
      "max_m": 0.0803160651404944
    },
    "relative_delta_yaw_error_deg": {
      "mean": 0.001598428431563885,
      "std_sample": 0.09497135189907642,
      "rmse": 0.09369280028773967,
      "p95_absolute": 0.19981755878223414,
      "max_absolute": 0.2376504674244296
    }
  }
}
```

## NDT effect from initial prediction to raw result
```json
{
  "samples": 0,
  "available": false,
  "reason": "T_init at cloud time absent; V1 callback-time initial pose is intentionally not substituted"
}
```

Case context for [225, 226, 241]: `case_context.csv` (±4 records).
