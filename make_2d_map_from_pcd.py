#!/usr/bin/env python3
"""
Convert a FAST-LIO PCD map into a Nav2-compatible 2D occupancy map.

This is a local fallback for the installed pcd2pgm node.  It intentionally
does not depend on ROS at runtime; it parses common PCD v0.7 ASCII/binary files
directly, applies conservative PCL-style preprocessing, then projects to a 2D
Nav2 occupancy map:

  1. height filter
  2. PCL VoxelGrid-style centroid downsample
  3. PCL RadiusOutlierRemoval-style light outlier filter
  4. direct XY projection
  5. per-cell point-count threshold
  6. small-gap morphological closing (wall continuity)
  7. 8-neighbour connected component cleanup

  robot_data/maps/map.pgm
  robot_data/maps/map.yaml

PGM convention used by Nav2/map_server:
  0   = occupied
  254 = free
"""

from __future__ import annotations

import argparse
import math
import shutil
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import cv2
from PIL import Image
from scipy.ndimage import binary_closing
from scipy.spatial import cKDTree


def _parse_header(pcd_path: Path) -> Tuple[Dict[str, str], int]:
    header: Dict[str, str] = {}
    offset = 0
    with pcd_path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{pcd_path} has no DATA line")
            offset += len(line)
            text = line.decode("latin1").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            header[parts[0].upper()] = " ".join(parts[1:])
            if parts[0].upper() == "DATA":
                break
    return header, offset


def _numpy_type(pcd_type: str, size: int) -> str:
    key = (pcd_type.upper(), size)
    table = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    if key not in table:
        raise ValueError(f"Unsupported PCD field type/size: {pcd_type}{size}")
    return table[key]


def load_xyz_from_pcd(pcd_path: Path) -> np.ndarray:
    header, data_offset = _parse_header(pcd_path)

    fields = header["FIELDS"].split()
    sizes = [int(x) for x in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(x) for x in header.get("COUNT", " ".join(["1"] * len(fields))).split()]
    points = int(header.get("POINTS", header.get("WIDTH", "0")))
    data_kind = header["DATA"].lower()

    for required in ("x", "y", "z"):
        if required not in fields:
            raise ValueError(f"{pcd_path} is missing field '{required}'")

    if data_kind == "binary":
        dtype_fields: List[Tuple[str, str]] = []
        for name, size, typ, count in zip(fields, sizes, types, counts):
            base = _numpy_type(typ, size)
            if count == 1:
                dtype_fields.append((name, base))
            else:
                for i in range(count):
                    dtype_fields.append((f"{name}_{i}", base))
        dtype = np.dtype(dtype_fields)
        with pcd_path.open("rb") as f:
            f.seek(data_offset)
            arr = np.frombuffer(f.read(points * dtype.itemsize), dtype=dtype, count=points)
        xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32, copy=False)
    elif data_kind == "ascii":
        field_index = {name: i for i, name in enumerate(fields)}
        usecols = [field_index["x"], field_index["y"], field_index["z"]]
        xyz = np.loadtxt(pcd_path, comments="#", skiprows=len(_header_lines(pcd_path)), usecols=usecols)
        xyz = xyz.astype(np.float32, copy=False)
    else:
        raise ValueError(f"Unsupported PCD DATA kind: {data_kind}")

    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


def _header_lines(pcd_path: Path) -> List[str]:
    lines: List[str] = []
    with pcd_path.open("r", encoding="latin1") as f:
        for line in f:
            lines.append(line)
            if line.upper().startswith("DATA"):
                break
    return lines


def dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask
    out = mask.copy()
    ys, xs = np.nonzero(mask)
    height, width = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            y0 = np.clip(ys + dy, 0, height - 1)
            x0 = np.clip(xs + dx, 0, width - 1)
            out[y0, x0] = True
    return out


def disk_structure(radius: int) -> np.ndarray:
    """Return a circular grid kernel with a radius measured in map cells."""
    if radius < 1:
        return np.ones((1, 1), dtype=bool)
    ys, xs = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    return (xs * xs + ys * ys) <= radius * radius


def close_small_gaps(mask: np.ndarray, radius: int, resolution: float) -> np.ndarray:
    """Bridge small breaks and fill small holes without inflating all walls.

    Closing is dilation followed by erosion with the same circular kernel.  It
    is deliberately different from ``--inflate-cells``: occupied walls retain
    approximately their original thickness, while gaps smaller than the kernel
    are connected.
    """
    if radius <= 0:
        return mask
    closed = binary_closing(mask, structure=disk_structure(radius), border_value=0)
    print(
        f"[closing] radius_cells={radius}, radius_m={radius * resolution:.3f}, "
        f"added_cells={int(closed.sum()) - int(mask.sum())}"
    )
    return closed


def voxel_grid_filter(points: np.ndarray, leaf_size: float) -> np.ndarray:
    """PCL VoxelGrid-style centroid downsample."""
    if len(points) == 0 or leaf_size <= 0:
        return points

    origin = points.min(axis=0)
    voxel_idx = np.floor((points - origin) / leaf_size).astype(np.int64)
    _, inverse = np.unique(voxel_idx, axis=0, return_inverse=True)
    counts = np.bincount(inverse).astype(np.float32)
    filtered = np.empty((len(counts), 3), dtype=np.float32)
    for dim in range(3):
        filtered[:, dim] = np.bincount(inverse, weights=points[:, dim]) / counts
    return filtered


def radius_outlier_filter(points: np.ndarray, radius: float, min_neighbors: int) -> np.ndarray:
    """PCL RadiusOutlierRemoval-style filter.

    min_neighbors is interpreted as neighbouring points excluding the query
    point itself, matching the practical expectation for outlier removal.
    """
    if len(points) == 0 or radius <= 0 or min_neighbors <= 0:
        return points

    tree = cKDTree(points)
    neighbour_counts = tree.query_ball_point(points, r=radius, return_length=True)
    keep = (neighbour_counts - 1) >= min_neighbors
    return points[keep]


def project_to_counts(
    points: np.ndarray,
    resolution: float,
    padding: float,
) -> Tuple[np.ndarray, float, float]:
    if len(points) == 0:
        raise SystemExit("No points left for 2D projection")

    min_x = math.floor((float(points[:, 0].min()) - padding) / resolution) * resolution
    min_y = math.floor((float(points[:, 1].min()) - padding) / resolution) * resolution
    max_x = math.ceil((float(points[:, 0].max()) + padding) / resolution) * resolution
    max_y = math.ceil((float(points[:, 1].max()) + padding) / resolution) * resolution

    width = int(math.ceil((max_x - min_x) / resolution)) + 1
    height = int(math.ceil((max_y - min_y) / resolution)) + 1
    print(f"[map] size={width}x{height}, resolution={resolution}, origin=[{min_x:.3f}, {min_y:.3f}, 0]")

    cols = np.floor((points[:, 0] - min_x) / resolution).astype(np.int64)
    rows_from_bottom = np.floor((points[:, 1] - min_y) / resolution).astype(np.int64)
    valid = (cols >= 0) & (cols < width) & (rows_from_bottom >= 0) & (rows_from_bottom < height)
    cols = cols[valid]
    rows = height - 1 - rows_from_bottom[valid]

    counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(counts, (rows, cols), 1)
    return counts, min_x, min_y


def project_to_existing_grid_counts(
    points: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    shape: Tuple[int, int],
) -> np.ndarray:
    """Project points into an existing map grid without changing its origin."""
    height, width = shape
    counts = np.zeros(shape, dtype=np.uint16)
    if len(points) == 0:
        return counts
    cols = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    rows_from_bottom = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    valid = (cols >= 0) & (cols < width) & (rows_from_bottom >= 0) & (rows_from_bottom < height)
    rows = height - 1 - rows_from_bottom[valid]
    np.add.at(counts, (rows, cols[valid]), 1)
    return counts


def parse_height_layers(value: str) -> List[Tuple[float, float]]:
    """Parse ``low:high,low:high`` height-layer syntax."""
    layers: List[Tuple[float, float]] = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            low_text, high_text = item.split(":", maxsplit=1)
            low, high = float(low_text), float(high_text)
        except ValueError as exc:
            raise ValueError(f"Invalid height layer {item!r}; expected low:high") from exc
        if high <= low:
            raise ValueError(f"Height layer upper bound must exceed lower bound: {item!r}")
        layers.append((low, high))
    if not layers:
        raise ValueError("At least one height layer is required")
    return layers


def multi_height_wall_evidence(
    points: np.ndarray,
    layers: List[Tuple[float, float]],
    min_layers_per_cell: int,
    min_points_per_layer: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    shape: Tuple[int, int],
) -> np.ndarray:
    """Return cells supported by endpoints from several independent heights.

    A floor speckle or a table edge normally appears in only one horizontal
    slice. A wall normally has endpoints in more than one slice. This makes
    high/low point evidence useful without projecting every high object as a
    2-D obstacle.
    """
    support = np.zeros(shape, dtype=np.uint8)
    for low, high in layers:
        layer_points = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
        counts = project_to_existing_grid_counts(
            layer_points, resolution, origin_x, origin_y, shape)
        support += (counts >= max(1, min_points_per_layer)).astype(np.uint8)
    evidence = support >= max(1, min_layers_per_cell)
    print(
        f"[multi-height] layers={','.join(f'{low:.2f}:{high:.2f}' for low, high in layers)}, "
        f"min_layers={min_layers_per_cell}, evidence_cells={int(evidence.sum())}"
    )
    return evidence


def bridge_collinear_wall_gaps(
    evidence_mask: np.ndarray,
    output_mask: np.ndarray,
    resolution: float,
    vote_threshold: int,
    min_length_m: float,
    max_gap_m: float,
    min_support_ratio: float,
) -> Tuple[np.ndarray, int]:
    """Bridge only short gaps in long, evidence-backed Hough line segments."""
    if min_length_m <= 0.0 or max_gap_m <= 0.0:
        return output_mask, 0
    minimum_length = max(2, int(round(min_length_m / resolution)))
    maximum_gap = max(1, int(round(max_gap_m / resolution)))
    source = (evidence_mask.astype(np.uint8) * 255)
    lines = cv2.HoughLinesP(
        source,
        rho=1,
        theta=np.pi / 180.0,
        threshold=max(1, vote_threshold),
        minLineLength=minimum_length,
        maxLineGap=maximum_gap,
    )
    if lines is None:
        print("[line-bridge] no eligible Hough line segments")
        return output_mask, 0

    bridged = output_mask.copy()
    accepted = 0
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        line = np.zeros(evidence_mask.shape, dtype=np.uint8)
        cv2.line(line, (int(x1), int(y1)), (int(x2), int(y2)), color=1, thickness=1)
        pixels = line.astype(bool)
        if not pixels.any() or float(evidence_mask[pixels].mean()) < min_support_ratio:
            continue
        bridged[pixels] = True
        accepted += 1
    added = int(bridged.sum()) - int(output_mask.sum())
    print(
        f"[line-bridge] accepted_lines={accepted}, min_length_m={min_length_m:.2f}, "
        f"max_gap_m={max_gap_m:.2f}, min_support={min_support_ratio:.2f}, added_cells={added}"
    )
    return bridged, accepted


def vertical_structure_filter(
    points: np.ndarray,
    base_mask: np.ndarray,
    counts: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    z_bin_size: float,
    min_z_bins: int,
    min_z_span: float,
    keep_dense_cells: int,
) -> Tuple[np.ndarray, int]:
    """Keep cells that look like vertical structure, not flat ground speckles.

    Walls, table legs, door frames and shelf edges usually occupy multiple
    height bins in the same XY cell.  Ground reflections / floor speckles are
    often thin in Z.  Dense cells are kept as a safety valve so compact real
    obstacles are not removed merely because they are horizontally flat.
    """
    if len(points) == 0 or z_bin_size <= 0:
        return base_mask, 0

    height, width = base_mask.shape
    cols = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    rows_from_bottom = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    valid = (cols >= 0) & (cols < width) & (rows_from_bottom >= 0) & (rows_from_bottom < height)
    cols = cols[valid]
    rows = height - 1 - rows_from_bottom[valid]
    zs = points[:, 2][valid]

    min_z = np.full((height, width), np.inf, dtype=np.float32)
    max_z = np.full((height, width), -np.inf, dtype=np.float32)
    np.minimum.at(min_z, (rows, cols), zs)
    np.maximum.at(max_z, (rows, cols), zs)
    z_span = max_z - min_z

    z_bins = np.floor((zs - float(zs.min())) / z_bin_size).astype(np.int64)
    row_col_z = np.column_stack((rows, cols, z_bins))
    unique_row_col_z = np.unique(row_col_z, axis=0)
    z_bin_counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(z_bin_counts, (unique_row_col_z[:, 0], unique_row_col_z[:, 1]), 1)

    vertical = (z_bin_counts >= min_z_bins) | (z_span >= min_z_span)
    dense = counts >= keep_dense_cells
    filtered = base_mask & (vertical | dense)
    removed = int(base_mask.sum() - filtered.sum())
    print(
        f"[vertical] z_bin_size={z_bin_size}, min_z_bins={min_z_bins}, "
        f"min_z_span={min_z_span}, keep_dense_cells={keep_dense_cells}, "
        f"removed_cells={removed}, kept_cells={int(filtered.sum())}"
    )
    return filtered, removed


def connected_component_filter(
    mask: np.ndarray,
    min_component_size: int,
    connectivity: int,
) -> Tuple[np.ndarray, int, int, int]:
    """Remove small connected occupied components.

    This deliberately does not use neighbour-count pruning. A 1-pixel-wide wall
    is kept as long as the whole connected wall segment has enough pixels.
    """
    if min_component_size <= 1:
        total = int(count_connected_components(mask, connectivity))
        return mask, total, 0, 0

    height, width = mask.shape
    visited = np.zeros(mask.shape, dtype=bool)
    kept = np.zeros(mask.shape, dtype=bool)
    total_components = 0
    removed_components = 0
    removed_cells = 0

    if connectivity == 4:
        neighbours = ((-1, 0), (0, -1), (0, 1), (1, 0))
    elif connectivity == 8:
        neighbours = (
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        )
    else:
        raise ValueError(f"connectivity must be 4 or 8, got {connectivity}")

    ys, xs = np.nonzero(mask)
    for start_y, start_x in zip(ys, xs):
        if visited[start_y, start_x]:
            continue

        total_components += 1
        q = deque([(int(start_y), int(start_x))])
        visited[start_y, start_x] = True
        cells: List[Tuple[int, int]] = []

        while q:
            y, x = q.popleft()
            cells.append((y, x))

            for dy, dx in neighbours:
                ny = y + dy
                nx = x + dx
                if ny < 0 or ny >= height or nx < 0 or nx >= width:
                    continue
                if visited[ny, nx] or not mask[ny, nx]:
                    continue
                visited[ny, nx] = True
                q.append((ny, nx))

        if len(cells) >= min_component_size:
            cy, cx = zip(*cells)
            kept[cy, cx] = True
        else:
            removed_components += 1
            removed_cells += len(cells)

    print(
        f"[components] connectivity={connectivity}, min_component_size={min_component_size}, "
        f"total_components={total_components}, removed_components={removed_components}, "
        f"removed_cells={removed_cells}"
    )
    return kept, total_components, removed_components, removed_cells


def count_connected_components(mask: np.ndarray, connectivity: int) -> int:
    return connected_component_filter(mask, 2, connectivity)[1]


def backup_if_exists(path: Path) -> None:
    if path.exists():
        stamp = time.strftime("%Y%m%d_%H%M%S")
        backup = path.with_name(f"{path.name}.bak_{stamp}")
        shutil.copy2(path, backup)
        print(f"[backup] {path} -> {backup}")


def write_pgm(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(f"P5\n# Created by make_2d_map_from_pcd.py\n{image.shape[1]} {image.shape[0]}\n255\n".encode())
        f.write(image.astype(np.uint8).tobytes())


def write_debug_png(path: Path, occupied: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    image = np.full(occupied.shape, 255, dtype=np.uint8)
    image[occupied] = 0
    Image.fromarray(image, mode="L").save(path)


def write_pcd_ascii(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    points = points.astype(np.float32, copy=False)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA ascii\n"
    )
    with path.open("w", encoding="ascii") as f:
        f.write(header)
        np.savetxt(f, points, fmt="%.6f %.6f %.6f")


def write_yaml(path: Path, image_name: str, resolution: float, origin_x: float, origin_y: float) -> None:
    text = (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    path.write_text(text, encoding="utf-8")


def str2bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in ("1", "true", "yes", "y", "on"):
        return True
    if lowered in ("0", "false", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Expected boolean value, got {value!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", default="robot_data/maps/data/scans.pcd", help="input PCD path")
    parser.add_argument("--output-prefix", default="robot_data/maps/map", help="output prefix without extension")
    parser.add_argument("--z-min", type=float, default=-0.10, help="minimum z kept for projection")
    parser.add_argument("--z-max", type=float, default=0.35, help="maximum z kept for projection")
    parser.add_argument(
        "--enable-voxel-filter",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="enable PCL VoxelGrid-style light downsample",
    )
    parser.add_argument("--voxel-leaf-size", type=float, default=0.03, help="VoxelGrid leaf size in meters")
    parser.add_argument(
        "--enable-radius-filter",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="enable PCL RadiusOutlierRemoval-style filter",
    )
    parser.add_argument("--radius-search", type=float, default=0.10, help="RadiusOutlierRemoval search radius in meters")
    parser.add_argument(
        "--radius-min-neighbors",
        type=int,
        default=2,
        help="minimum neighbours inside radius, excluding the query point itself",
    )
    parser.add_argument(
        "--map-resolution",
        "--resolution",
        dest="map_resolution",
        type=float,
        default=0.05,
        help="map resolution in meters/cell",
    )
    parser.add_argument(
        "--min-points-per-cell",
        "--min-points",
        dest="min_points_per_cell",
        type=int,
        default=2,
        help="minimum 3D points in a 2D cell before marking occupied",
    )
    parser.add_argument(
        "--enable-component-filter",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="enable 2D connected-component cleanup",
    )
    parser.add_argument(
        "--enable-vertical-filter",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="require per-cell vertical evidence to suppress floor speckles",
    )
    parser.add_argument("--z-bin-size", type=float, default=0.05, help="vertical bin size in meters")
    parser.add_argument(
        "--min-z-bins-per-cell",
        type=int,
        default=2,
        help="minimum occupied z bins in a 2D cell to keep it as vertical structure",
    )
    parser.add_argument(
        "--min-z-span-per-cell",
        type=float,
        default=0.08,
        help="minimum z span in a 2D cell to keep it as vertical structure",
    )
    parser.add_argument(
        "--vertical-keep-dense-cells",
        type=int,
        default=6,
        help="always keep cells with at least this many points even if z span is small",
    )
    parser.add_argument(
        "--enable-multi-height-evidence",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="add only cells supported by endpoints from multiple height layers",
    )
    parser.add_argument(
        "--height-layers",
        default="-0.15:0.05,0.05:0.35,0.35:0.60",
        help="comma-separated low:high endpoint layers for multi-height wall evidence",
    )
    parser.add_argument(
        "--multi-height-min-layers-per-cell",
        type=int,
        default=2,
        help="number of independent height layers required to add a cell",
    )
    parser.add_argument(
        "--multi-height-min-points-per-layer",
        type=int,
        default=1,
        help="minimum endpoints in one layer before it contributes support",
    )
    parser.add_argument(
        "--enable-line-bridge",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="use Hough line segments to bridge short, collinear wall gaps",
    )
    parser.add_argument("--line-vote-threshold", type=int, default=28, help="minimum Hough votes for a wall line")
    parser.add_argument(
        "--line-min-length-m", type=float, default=1.5,
        help="minimum evidence-backed line length in meters",
    )
    parser.add_argument(
        "--line-max-gap-m", type=float, default=0.40,
        help="maximum collinear gap in meters that may be bridged",
    )
    parser.add_argument(
        "--line-min-support-ratio", type=float, default=0.65,
        help="minimum pre-existing wall evidence fraction along a fitted line",
    )
    parser.add_argument("--connectivity", type=int, choices=(4, 8), default=8, help="connected component connectivity")
    parser.add_argument(
        "--min-component-size",
        "--min-component-cells",
        dest="min_component_size",
        type=int,
        default=3,
        help="remove connected occupied components smaller than this many cells",
    )
    parser.add_argument("--padding", type=float, default=1.0, help="free border around point cloud, meters")
    parser.add_argument("--inflate-cells", type=int, default=0, help="optional occupied-cell dilation radius; default disabled")
    parser.add_argument(
        "--enable-morphological-close",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="bridge small occupied-wall gaps with a closing operation",
    )
    parser.add_argument(
        "--closing-radius-cells",
        type=int,
        default=1,
        help="closing kernel radius in map cells; 1 equals 5 cm at 5 cm resolution",
    )
    parser.add_argument("--debug-dir", default="", help="directory for intermediate PCD/PNG debug outputs")
    parser.add_argument(
        "--enable-debug-output",
        type=str2bool,
        nargs="?",
        const=True,
        default=True,
        help="write intermediate PCD/PNG debug files",
    )
    parser.add_argument("--no-backup", action="store_true", help="do not backup existing map files")
    args = parser.parse_args()

    pcd_path = Path(args.pcd).expanduser().resolve()
    output_prefix = Path(args.output_prefix).expanduser()
    if not output_prefix.is_absolute():
        output_prefix = (Path.cwd() / output_prefix).resolve()
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    debug_dir = Path(args.debug_dir).expanduser() if args.debug_dir else output_prefix.parent / "pcd_to_2d_debug"
    if not debug_dir.is_absolute():
        debug_dir = (Path.cwd() / debug_dir).resolve()

    xyz = load_xyz_from_pcd(pcd_path)
    raw_points = len(xyz)
    print(f"[pcd] input={pcd_path}")
    print(f"[stats] raw_points={raw_points}")

    z_mask = (xyz[:, 2] >= args.z_min) & (xyz[:, 2] <= args.z_max)
    height_points = xyz[z_mask]
    if len(height_points) == 0:
        raise SystemExit(f"No points left after z filter [{args.z_min}, {args.z_max}]")
    print(f"[stats] height_filtered_points={len(height_points)} z=[{args.z_min}, {args.z_max}]")

    if args.enable_debug_output:
        write_pcd_ascii(debug_dir / "01_height_filtered.pcd", height_points)

    if args.enable_voxel_filter:
        voxel_points = voxel_grid_filter(height_points, args.voxel_leaf_size)
        print(f"[stats] voxel_filtered_points={len(voxel_points)} leaf_size={args.voxel_leaf_size}")
    else:
        voxel_points = height_points
        print(f"[stats] voxel_filtered_points={len(voxel_points)} disabled")

    if args.enable_debug_output:
        write_pcd_ascii(debug_dir / "02_voxel_filtered.pcd", voxel_points)

    if args.enable_radius_filter:
        radius_points = radius_outlier_filter(voxel_points, args.radius_search, args.radius_min_neighbors)
        print(
            f"[stats] radius_filtered_points={len(radius_points)} "
            f"radius_search={args.radius_search} radius_min_neighbors={args.radius_min_neighbors}"
        )
    else:
        radius_points = voxel_points
        print(f"[stats] radius_filtered_points={len(radius_points)} disabled")

    if args.enable_debug_output:
        write_pcd_ascii(debug_dir / "03_radius_filtered.pcd", radius_points)

    counts, min_x, min_y = project_to_counts(
        radius_points,
        args.map_resolution,
        args.padding,
    )
    raw_projection = counts > 0
    point_count_filtered = counts >= max(1, args.min_points_per_cell)
    raw_occupied_cells = int(raw_projection.sum())
    point_count_cells = int(point_count_filtered.sum())
    point_count_removed = raw_occupied_cells - point_count_cells
    print(f"[stats] raw_2d_occupied_cells={raw_occupied_cells}")
    print(
        f"[stats] min_points_per_cell={args.min_points_per_cell} "
        f"removed_cells={point_count_removed} kept_cells={point_count_cells}"
    )

    if args.enable_debug_output:
        write_debug_png(debug_dir / "04_raw_projection.png", raw_projection)
        write_debug_png(debug_dir / "05_point_count_filtered.png", point_count_filtered)

    if args.enable_vertical_filter:
        vertical_filtered, vertical_removed = vertical_structure_filter(
            radius_points,
            point_count_filtered,
            counts,
            args.map_resolution,
            min_x,
            min_y,
            args.z_bin_size,
            args.min_z_bins_per_cell,
            args.min_z_span_per_cell,
            args.vertical_keep_dense_cells,
        )
    else:
        vertical_filtered = point_count_filtered
        vertical_removed = 0
        print("[vertical] disabled")

    if args.enable_debug_output:
        write_debug_png(debug_dir / "05b_vertical_filtered.png", vertical_filtered)

    if args.enable_multi_height_evidence:
        layers = parse_height_layers(args.height_layers)
        multi_height = multi_height_wall_evidence(
            xyz,
            layers,
            args.multi_height_min_layers_per_cell,
            args.multi_height_min_points_per_layer,
            args.map_resolution,
            min_x,
            min_y,
            vertical_filtered.shape,
        )
        line_evidence = vertical_filtered | multi_height
        print(
            f"[multi-height] occupied_before={int(vertical_filtered.sum())}, "
            f"line_evidence_after={int(line_evidence.sum())}"
        )
    else:
        line_evidence = vertical_filtered
        print("[multi-height] disabled")

    if args.enable_debug_output:
        write_debug_png(debug_dir / "05c_multi_height_evidence.png", line_evidence)

    if args.enable_line_bridge:
        line_bridged, accepted_lines = bridge_collinear_wall_gaps(
            line_evidence,
            vertical_filtered,
            args.map_resolution,
            args.line_vote_threshold,
            args.line_min_length_m,
            args.line_max_gap_m,
            args.line_min_support_ratio,
        )
    else:
        line_bridged = vertical_filtered
        accepted_lines = 0
        print("[line-bridge] disabled")

    if args.enable_debug_output:
        write_debug_png(debug_dir / "05d_line_bridged.png", line_bridged)

    if args.enable_morphological_close:
        before_closing = int(line_bridged.sum())
        continuity_filtered = close_small_gaps(
            line_bridged, args.closing_radius_cells, args.map_resolution)
        print(f"[closing] occupied_before={before_closing}, occupied_after={int(continuity_filtered.sum())}")
    else:
        continuity_filtered = vertical_filtered
        print("[closing] disabled")

    if args.enable_debug_output:
        write_debug_png(debug_dir / "05e_closed.png", continuity_filtered)

    if args.enable_component_filter:
        occupied, total_components, removed_components, removed_component_cells = connected_component_filter(
            continuity_filtered,
            args.min_component_size,
            args.connectivity,
        )
    else:
        occupied = continuity_filtered
        total_components = 0
        removed_components = 0
        removed_component_cells = 0
        print("[components] disabled")

    if args.inflate_cells > 0:
        before_inflate = int(occupied.sum())
        occupied = dilate(occupied, args.inflate_cells)
        print(f"[inflate] radius_cells={args.inflate_cells} added_cells={int(occupied.sum()) - before_inflate}")

    if args.enable_debug_output:
        write_debug_png(debug_dir / "06_component_filtered.png", occupied)

    image = np.full(occupied.shape, 254, dtype=np.uint8)
    image[occupied] = 0
    occ_ratio = float(occupied.mean())
    print(f"[stats] component_total_count={total_components}")
    print(f"[stats] vertical_removed_cells={vertical_removed}")
    print(f"[stats] hough_line_count={accepted_lines}")
    print(f"[stats] component_removed_count={removed_components}")
    print(f"[stats] component_removed_cells={removed_component_cells}")
    print(f"[stats] final_occupied_cells={int(occupied.sum())} ({occ_ratio:.3%})")

    if not args.no_backup:
        backup_if_exists(pgm_path)
        backup_if_exists(yaml_path)

    write_pgm(pgm_path, image)
    write_yaml(yaml_path, pgm_path.name, args.map_resolution, min_x, min_y)
    print(f"[done] wrote {pgm_path}")
    print(f"[done] wrote {yaml_path}")
    if args.enable_debug_output:
        print(f"[done] debug outputs in {debug_dir}")


if __name__ == "__main__":
    main()
