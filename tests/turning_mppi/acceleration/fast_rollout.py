"""ctypes bridge to the numerically equivalent, pure C++ rollout evaluator.

Public API: rollout(controller, state, smooth, pending, residuals, obstacles,
                    guide_only=False), matching GuidedMPPI.rollout.
No changes to sampling, costs, models, constraints, or selection are made.
"""
import ctypes
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import threading

import numpy as np

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "rollout.cpp"
LIBRARY = HERE / "libguided_rollout.so"
_LOCK = threading.Lock()
_FUNCTION = None
_PARAMETERS = ("dt", "speed", "vmax", "wmax", "accel_v", "accel_w", "tau_v", "tau_w",
               "delay", "lateral_gain", "heading_gain", "half_length", "half_width", "clearance")


def build():
    """Build in this directory. Deliberately omit fast-math and contraction."""
    # Existing benchmark processes can still map the old inode. Never truncate
    # a loaded .so: publish a complete replacement atomically on the same FS.
    descriptor, temporary = tempfile.mkstemp(prefix=".libguided_rollout-", suffix=".so", dir=HERE)
    os.close(descriptor)
    try:
        subprocess.run(["g++", "-O3", "-std=c++17", "-shared", "-fPIC", "-ffp-contract=off",
                        str(SOURCE), "-o", temporary], check=True)
        os.replace(temporary, LIBRARY)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return LIBRARY


def _function():
    global _FUNCTION
    if _FUNCTION is not None:
        return _FUNCTION
    with _LOCK:
        if _FUNCTION is None:
            if not LIBRARY.exists() or LIBRARY.stat().st_mtime_ns < SOURCE.stat().st_mtime_ns:
                build()
            function = ctypes.CDLL(str(LIBRARY)).guided_mppi_rollout
            array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
            mask = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
            function.argtypes = [ctypes.c_int] * 7 + [array] * 7 + [ctypes.c_double] + [array] * 6 + [mask] + [array] * 2
            function.restype = ctypes.c_int
            _FUNCTION = function
    return _FUNCTION


def prepare():
    """Load/build before timing control calls, and return reproducible metadata."""
    _function()
    return {"backend": "cpp_equivalent_rollout_ctypes", "ros": False,
            "source": str(SOURCE), "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            "library": str(LIBRARY), "library_sha256": hashlib.sha256(LIBRARY.read_bytes()).hexdigest(),
            "compiler_flags": ["-O3", "-std=c++17", "-shared", "-fPIC", "-ffp-contract=off"]}


def rollout(controller, state, smooth, pending, residuals, obstacles, guide_only=False):
    """Evaluate original controller rollout equations on contiguous doubles."""
    c, path = controller.cfg, controller.path
    residuals = np.ascontiguousarray(residuals, dtype=np.float64)
    if residuals.ndim != 3 or residuals.shape[1:] != (c.steps, 2):
        raise ValueError("residuals must have shape (count, cfg.steps, 2)")
    state = np.ascontiguousarray(state, dtype=np.float64)
    smooth = np.ascontiguousarray(smooth, dtype=np.float64)
    pending = np.ascontiguousarray(pending, dtype=np.float64).reshape(-1, 2)
    obstacles = np.ascontiguousarray(obstacles, dtype=np.float64).reshape(-1, 3)
    if state.shape != (5,) or smooth.shape != (2,):
        raise ValueError("state and smooth must have shapes (5,) and (2,)")
    count = len(residuals)
    costs = np.empty(count, dtype=np.float64)
    feasible = np.empty(count, dtype=np.uint8)
    commands = np.empty_like(residuals)
    minimum = np.empty(count, dtype=np.float64)
    # Cache only immutable path geometry. A new path object creates a new cache.
    cached = getattr(controller, "_fast_rollout_path", None)
    if cached is None or cached[0] is not path:
        arrays = tuple(np.ascontiguousarray(a, dtype=np.float64) for a in
                       (path.s, path.points[:, 0], path.points[:, 1], path.yaw, path.k, path.dk))
        cached = controller._fast_rollout_path = (path, arrays)
    cfg = np.ascontiguousarray([getattr(c, key) for key in _PARAMETERS], dtype=np.float64)
    result = _function()(count, c.steps, len(path.s), len(pending), len(obstacles),
                         int(controller.mode == "ideal"), int(guide_only),
                         *cached[1], cfg, float(controller.s), state, smooth, pending, residuals,
                         obstacles, costs, feasible, commands, minimum)
    if result:
        raise ValueError("C++ rollout rejected array dimensions")
    return costs, feasible.astype(bool), commands, minimum


if __name__ == "__main__":
    print(build())
