# Installed Nav2 MPPI offline baseline

This adapter loads `nav2_mppi_controller::MPPIController` using the installed
pluginlib factory, then calls `configure`, `setPlan`, and
`computeVelocityCommands` through the original controller interface. Using the
original factory also preserves the binary's private xtensor/xsimd allocation
and construction ABI. Its optimizer,
critics, sampling, path pruning, and command generation come from the original
shared libraries. None of those algorithms are reimplemented here. Source
production YAML and the installed ROS libraries are not modified.

The tested package is `ros-humble-nav2-mppi-controller`
`1.1.20-1jammy.20260607.083249`. Library hashes and adapter checks are recorded
in `smoke_test_results.json`.

## Build and call

From the repository root:

```bash
bash tests/turning_mppi/native/build.sh
python3 tests/turning_mppi/native/smoke_test.py \
  --output tests/turning_mppi/native/smoke_test_results.json
```

```python
from native.native_mppi import NativeMPPI

with NativeMPPI("robot_data/config/navigation.yaml", "InspectionStrict", seed=7) as controller:
    controller.set_path(path_xy_yaw)  # float Nx3 array, radians
    v, w = controller.step(x, y, yaw, measured_v, measured_w)
```

The wrapper is intended for `tests/turning_mppi/benchmark.py`; the installed
library must be built on this machine before running it. Native C ABI functions
are `mppi_create(yaml_path, plugin_name, uint_seed)`, `mppi_set_path`, `mppi_step`,
`mppi_reset`, `mppi_destroy`, and `mppi_last_error`. Arrays are contiguous doubles;
step returns `(v,w)`. A failed API call returns `-1` and stores its error text.

## Configuration and reproducibility

The selected plugin's complete YAML subtree and `controller_frequency` are
loaded without changing values. `InspectionStrict` is the default; `FollowPath`
can also be tested. The goal tolerance, local costmap dimensions, resolution,
and padded robot footprint are read from the same file. For comparisons the
caller should point to its frozen copy of the production YAML and record that
file's SHA-256.

The adapter calls `xt::random::seed(seed)` before configuring each plugin. In
the installed library, its exported xtensor default engine is shared with this
adapter. `regenerate_noises: false` in the production YAML caches its sampled
noise. Changing this seed changes the first native command. This is an adapter
mechanism, rather than a Nav2 controller parameter.

**The complete original baseline is not guaranteed bitwise reproducible.**
Repeated 512-command input sequences exposed differing commands in
`InspectionStrict` with the same sampling seed, including fresh-process
trials. Short prefixes often match exactly. The smoke report records observed
repeat differences; it does not hide or treat them as successful deterministic
reproduction. Use multiple baseline trials and report this limitation.

Calls are serialized because the installed sampling engine is process global.
Use a fresh process per trial and pass the trial's seed explicitly. `reset()`
resets the installed controller and does not reseed the global engine. Original
wall clock based `reset_period` behavior remains in the plugin. Original
Savitzky-Golay output filtering can slightly overshoot the optimizer's internal
speed limits; raw original outputs are preserved, and the benchmark's separate
velocity smoother clips them.

## Installed path-distance boundary diagnostic

The installed `tools/utils.hpp:674` helper `findClosestPathPt` dereferences
`lower_bound` without checking for the `end()` result. A query beyond the final
distance can therefore read beyond its vector. The original
[Nav2 1.1.20 PathAlignCritic source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/path_align_critic.cpp)
uses this helper on trajectory integrated distances. This is a plausible source
of native variability, but **its causal role in the full baseline trial has not
been established**. No fix or changed critic is applied to this baseline.

`path_bounds_repro.cpp` invokes the actual installed inline helper under
AddressSanitizer. It is a separate diagnostic, excluded from normal builds:

```bash
cmake --build tests/turning_mppi/native/.build-stage --target path_bounds_repro
ASAN_OPTIONS=detect_leaks=0 tests/turning_mppi/native/.build-stage/path_bounds_repro
```

The expected diagnostic exits unsuccessfully on the out-of-bounds read. It
does not run the robot or alter any installed library.

## Offline environment and scope

The adapter creates an in-memory rolling **empty costmap**, with the original
dimensions and footprint. It moves that map's origin to the supplied robot
position before each command. It places both supplied path and state in the
same `odom` frame, so there is no localization/TF transform uncertainty in the
native adapter. The Python benchmark supplies synthetic state perturbations
and a separate plant model.

No executor, costmap update loop, sensor layers, subscriptions to robot state,
velocity publisher, or robot action client is started. The middleware is forced
to FastDDS with shared-memory-only transports on private domain 231. This
profile disables UDP transports. ROS initialization still attempts interface
enumeration under this sandbox; its diagnostics are retained in
`/tmp/turning_mppi_native_logs/middleware.log`. The Python wrapper redirects
native initialization stdout/stderr to that file, then restores both streams.
Runtime command exceptions are propagated to Python.

The adapter does not run FAST-LIO, NDT/DNP, lidar/IMU simulation, the Nav2
controller server, its goal/progress termination logic, the real velocity
smoother, or hardware. The benchmark models its smoother and body dynamics
separately. This is a real **original-controller** baseline in an offline
synthetic closed loop, and must not be described as a full FAST-LIO navigation
or physical robot experiment. Both production plugins currently omit obstacle
critics, so this baseline does not validate obstacle avoidance.
