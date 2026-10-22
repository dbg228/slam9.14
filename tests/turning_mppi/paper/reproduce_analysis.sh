#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR="${TMPDIR:-/tmp}/turning_mppi_paper_mpl"
python3 "$task_dir/paper_models.py"
python3 "$task_dir/paper_metrics.py"
python3 "$task_dir/paper_spectrum.py"
python3 "$task_dir/paper_figures.py"
python3 "$task_dir/paper_writeup.py"
