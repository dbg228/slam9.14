"""ctypes bridge to the installed original Nav2 MPPI, without robot commands."""
from __future__ import annotations

import ctypes
from contextlib import contextmanager
import os
from pathlib import Path
import sys
import threading
import numpy as np

_initialization_lock = threading.Lock()


@contextmanager
def _initialization_log():
    """Preserve native middleware diagnostics without polluting JSON output."""
    # This is used only around node construction. Step failures remain visible
    # through native exceptions, and their diagnostics are not redirected.
    logdir = Path("/tmp/turning_mppi_native_logs")
    logdir.mkdir(exist_ok=True)
    with _initialization_lock, (logdir / "middleware.log").open("ab", buffering=0) as log:
        sys.stdout.flush()
        sys.stderr.flush()
        old = [os.dup(1), os.dup(2)]
        try:
            ctypes.CDLL(None).fflush(None)
            os.dup2(log.fileno(), 1)
            os.dup2(log.fileno(), 2)
            yield
        finally:
            ctypes.CDLL(None).fflush(None)
            for fd, saved in zip((1, 2), old):
                os.dup2(saved, fd)
                os.close(saved)


class NativeMPPI:
    def __init__(self, yaml_path: str | Path, plugin: str = "InspectionStrict", seed: int = 1):
        path = Path(__file__).parent / "build/libturning_mppi_native.so"
        self.lib = ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
        self.lib.mppi_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]
        self.lib.mppi_create.restype = ctypes.c_void_p
        self.lib.mppi_last_error.restype = ctypes.c_char_p
        arr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        self.lib.mppi_set_path.argtypes = [ctypes.c_void_p, arr, ctypes.c_int]
        self.lib.mppi_step.argtypes = [ctypes.c_void_p] + [ctypes.c_double] * 5 + [arr]
        self.lib.mppi_reset.argtypes = [ctypes.c_void_p]
        self.lib.mppi_destroy.argtypes = [ctypes.c_void_p]
        with _initialization_log():
            self.handle = self.lib.mppi_create(str(yaml_path).encode(), plugin.encode(), seed)
        if not self.handle:
            self._raise()

    def _raise(self):
        raise RuntimeError(self.lib.mppi_last_error().decode())

    def set_path(self, xy_yaw):
        path = np.ascontiguousarray(xy_yaw, dtype=np.float64)
        if path.ndim != 2 or path.shape[1] != 3:
            raise ValueError("Path must be Nx3 (x,y,yaw)")
        if not np.isfinite(path).all():
            raise ValueError("Path must contain finite values")
        if self.lib.mppi_set_path(self.handle, path, len(path)):
            self._raise()

    def step(self, x: float, y: float, yaw: float, vx: float, wz: float):
        if not np.isfinite([x, y, yaw, vx, wz]).all():
            raise ValueError("State must contain finite values")
        out = np.zeros(2, dtype=np.float64)
        if self.lib.mppi_step(self.handle, x, y, yaw, vx, wz, out):
            self._raise()
        return out

    def reset(self):
        if self.lib.mppi_reset(self.handle):
            self._raise()

    def close(self):
        if getattr(self, "handle", None):
            self.lib.mppi_destroy(self.handle)
            self.handle = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        self.close()
