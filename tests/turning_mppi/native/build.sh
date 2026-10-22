#!/usr/bin/env bash
set -euo pipefail
native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Select the interpreter shipped with ROS, independent of the shell's venv.
cmake -S "$native_dir" -B "$native_dir/.build-stage" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/humble \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build "$native_dir/.build-stage" --target turning_mppi_native --parallel 2
# Never rewrite a shared object that an active benchmark may have mapped.
mkdir -p "$native_dir/build"
staged_library="$native_dir/build/.libturning_mppi_native.so.new.$$"
cp "$native_dir/.build-stage/libturning_mppi_native.so" "$staged_library"
mv -f "$staged_library" "$native_dir/build/libturning_mppi_native.so"
