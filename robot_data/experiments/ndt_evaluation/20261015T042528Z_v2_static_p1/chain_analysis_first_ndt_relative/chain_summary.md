# Localization-chain analysis

All errors are against the same base_link mocap reference and same fixed start alignment.
The comparison timestamp is the NDT input-cloud timestamp, not matching completion or pose-publication time.

## Start-aligned relative error
```json
{
  "initial": {
    "samples": 67,
    "metrics_available": true,
    "ex_m": {
      "mean": 0.006766200406343428,
      "std_sample": 0.014293300775951309,
      "rmse": 0.015717209817817587,
      "p95_absolute": 0.03683589456318232,
      "max_absolute": 0.04447944219483668
    },
    "ey_m": {
      "mean": 0.011637947644315146,
      "std_sample": 0.013892140674653391,
      "rmse": 0.01804308528673721,
      "p95_absolute": 0.031690632157384664,
      "max_absolute": 0.04821457423560793
    },
    "xy_m": {
      "rmse": 0.02392871937905943,
      "p95": 0.04100068047394965,
      "max": 0.049989373180090925,
      "mean": 0.02156680503581973
    },
    "yaw_deg": {
      "mean": -0.2185793027864783,
      "std_sample": 0.10927282293857044,
      "rmse": 0.2440066480198871,
      "p95_absolute": 0.4088938892313869,
      "max_absolute": 0.4794350562682686
    },
    "weighting": "one pose per NDT input-cloud timestamp"
  },
  "raw": {
    "samples": 67,
    "metrics_available": true,
    "ex_m": {
      "mean": 0.007196830620522167,
      "std_sample": 0.014894915917820653,
      "rmse": 0.016442067083625264,
      "p95_absolute": 0.03770901435226532,
      "max_absolute": 0.046751598747951206
    },
    "ey_m": {
      "mean": 0.011687602761520866,
      "std_sample": 0.013895353774273448,
      "rmse": 0.01807758587297499,
      "p95_absolute": 0.03131463618988728,
      "max_absolute": 0.04865836119722711
    },
    "xy_m": {
      "rmse": 0.024436462120716623,
      "p95": 0.04173143716093537,
      "max": 0.05105400474998928,
      "mean": 0.0220594625249033
    },
    "yaw_deg": {
      "mean": -0.22457761780827812,
      "std_sample": 0.10924389181759184,
      "rmse": 0.2493816583380287,
      "p95_absolute": 0.4204653239468021,
      "max_absolute": 0.4887274651899437
    },
    "weighting": "one pose per NDT input-cloud timestamp"
  },
  "final": {
    "samples": 67,
    "metrics_available": true,
    "ex_m": {
      "mean": 0.007196830620522167,
      "std_sample": 0.014894915917820653,
      "rmse": 0.016442067083625264,
      "p95_absolute": 0.03770901435226532,
      "max_absolute": 0.046751598747951206
    },
    "ey_m": {
      "mean": 0.011687602761520866,
      "std_sample": 0.013895353774273448,
      "rmse": 0.01807758587297499,
      "p95_absolute": 0.03131463618988728,
      "max_absolute": 0.04865836119722711
    },
    "xy_m": {
      "rmse": 0.024436462120716623,
      "p95": 0.04173143716093537,
      "max": 0.05105400474998928,
      "mean": 0.0220594625249033
    },
    "yaw_deg": {
      "mean": -0.22457761780827812,
      "std_sample": 0.10924389181759184,
      "rmse": 0.2493816583380287,
      "p95_absolute": 0.4204653239468021,
      "max_absolute": 0.4887274651899437
    },
    "weighting": "one pose per NDT input-cloud timestamp"
  }
}
```

## Adjacent-frame relative motion
```json
{
  "initial": {
    "pairs": 66,
    "skipped_long_or_nonpositive_gap_pairs": 0,
    "max_gap_seconds": 5.0,
    "estimate_delta_position": {
      "rmse_m": 0.025595084425904996,
      "p95_m": 0.046420188802645354,
      "max_m": 0.05721635208988187
    },
    "reference_delta_position": {
      "rmse_m": 0.00015021975226648358,
      "p95_m": 0.0002668796697482277,
      "max_m": 0.0002999458749510667
    },
    "relative_delta_position_error": {
      "rmse_m": 0.025571703883126753,
      "p95_m": 0.0464448894013581,
      "max_m": 0.05732965379024754
    },
    "relative_delta_yaw_error_deg": {
      "mean": -0.003103514016867266,
      "std_sample": 0.0996513751539231,
      "rmse": 0.09894224484631868,
      "p95_absolute": 0.1728587713646306,
      "max_absolute": 0.31858497212860415
    }
  },
  "raw": {
    "pairs": 66,
    "skipped_long_or_nonpositive_gap_pairs": 0,
    "max_gap_seconds": 5.0,
    "estimate_delta_position": {
      "rmse_m": 0.026531387469998325,
      "p95_m": 0.0470741043342811,
      "max_m": 0.058196191016595906
    },
    "reference_delta_position": {
      "rmse_m": 0.00015021975226648358,
      "p95_m": 0.0002668796697482277,
      "max_m": 0.0002999458749510667
    },
    "relative_delta_position_error": {
      "rmse_m": 0.026546411403983106,
      "p95_m": 0.04694857123732358,
      "max_m": 0.058226281090602065
    },
    "relative_delta_yaw_error_deg": {
      "mean": -0.00659602443430115,
      "std_sample": 0.10397033833472688,
      "rmse": 0.10339029678724058,
      "p95_absolute": 0.19446400026991567,
      "max_absolute": 0.3291812144733178
    }
  },
  "final": {
    "pairs": 66,
    "skipped_long_or_nonpositive_gap_pairs": 0,
    "max_gap_seconds": 5.0,
    "estimate_delta_position": {
      "rmse_m": 0.026531387469998325,
      "p95_m": 0.0470741043342811,
      "max_m": 0.058196191016595906
    },
    "reference_delta_position": {
      "rmse_m": 0.00015021975226648358,
      "p95_m": 0.0002668796697482277,
      "max_m": 0.0002999458749510667
    },
    "relative_delta_position_error": {
      "rmse_m": 0.026546411403983106,
      "p95_m": 0.04694857123732358,
      "max_m": 0.058226281090602065
    },
    "relative_delta_yaw_error_deg": {
      "mean": -0.00659602443430115,
      "std_sample": 0.10397033833472688,
      "rmse": 0.10339029678724058,
      "p95_absolute": 0.19446400026991567,
      "max_absolute": 0.3291812144733178
    }
  }
}
```

## NDT effect from initial prediction to raw result
```json
{
  "samples": 67,
  "available": true,
  "raw_minus_initial_xy_error_m": {
    "mean": 0.000492657489083567,
    "std_sample": 0.014747199116291897,
    "rmse": 0.01464502047770507,
    "p95_absolute": 0.0307794616451171,
    "max_absolute": 0.03664373682205255
  },
  "raw_improves_xy_error_frames": 34,
  "raw_worsens_xy_error_frames": 33,
  "interpretation": "negative raw-minus-initial means NDT reduced that frame's XY error"
}
```

Case context for [155, 180, 197]: `case_context.csv` (±4 records).
