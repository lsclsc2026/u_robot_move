#!/usr/bin/env python3
"""Clear explicitly selected polygons in a ROS occupancy-map PGM.

This tool is intentionally offline: it never publishes ROS data and it never
modifies the source map. Pixel coordinates use image convention (x right, y
down). Each selected polygon is filled with the map's free-cell value (254).
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import cv2
import numpy as np


def parse_polygon(text: str) -> np.ndarray:
    points = []
    for encoded_point in text.split(";"):
        fields = encoded_point.split(",")
        if len(fields) != 2:
            raise argparse.ArgumentTypeError(
                "polygon points must use x,y;x,y;... syntax"
            )
        try:
            points.append((int(fields[0]), int(fields[1])))
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                f"non-integer polygon point: {encoded_point}"
            ) from error
    if len(points) < 3:
        raise argparse.ArgumentTypeError("a polygon requires at least three points")
    return np.asarray(points, dtype=np.int32)


def yaml_scalar(text: str, key: str) -> str:
    match = re.search(rf"(?m)^\s*{re.escape(key)}\s*:\s*([^#\r\n]+)", text)
    if not match:
        raise RuntimeError(f"map YAML is missing '{key}'")
    return match.group(1).strip().strip("'\"")


def parse_origin(text: str) -> tuple[float, float, float]:
    raw = yaml_scalar(text, "origin")
    fields = [field.strip() for field in raw.strip("[]").split(",")]
    if len(fields) != 3:
        raise RuntimeError("map YAML origin must contain x, y, and yaw")
    return tuple(float(field) for field in fields)


def pixel_to_map(
    x: float,
    y: float,
    *,
    height: int,
    resolution: float,
    origin: tuple[float, float, float],
) -> tuple[float, float]:
    local_x = (x + 0.5) * resolution
    local_y = (height - y - 0.5) * resolution
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (
        origin[0] + cosine * local_x - sine * local_y,
        origin[1] + sine * local_x + cosine * local_y,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-yaml", required=True, type=Path)
    parser.add_argument(
        "--output-prefix",
        required=True,
        type=Path,
        help="Output path without .yaml/.pgm suffix.",
    )
    parser.add_argument(
        "--free-polygon-pixels",
        action="append",
        required=True,
        type=parse_polygon,
        help="Repeatable x,y;x,y;... polygon in original PGM pixel coordinates.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace an existing output pair; the source is never replaced.",
    )
    arguments = parser.parse_args()

    source_yaml = arguments.source_yaml.resolve()
    output_prefix = arguments.output_prefix.resolve()
    output_yaml = output_prefix.with_suffix(".yaml")
    output_pgm = output_prefix.with_suffix(".pgm")
    if output_yaml == source_yaml:
        raise RuntimeError("output prefix must not replace the source YAML")
    if not arguments.force and (output_yaml.exists() or output_pgm.exists()):
        raise RuntimeError("output exists; choose another prefix or pass --force")

    yaml_text = source_yaml.read_text(encoding="utf-8")
    source_image_value = yaml_scalar(yaml_text, "image")
    source_pgm = (source_yaml.parent / source_image_value).resolve()
    if output_pgm == source_pgm:
        raise RuntimeError("output prefix must not replace the source PGM")

    image = cv2.imread(str(source_pgm), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"could not read occupancy PGM: {source_pgm}")
    height, width = image.shape
    mask = np.zeros_like(image, dtype=np.uint8)
    for polygon in arguments.free_polygon_pixels:
        if (
            np.any(polygon[:, 0] < 0)
            or np.any(polygon[:, 0] >= width)
            or np.any(polygon[:, 1] < 0)
            or np.any(polygon[:, 1] >= height)
        ):
            raise RuntimeError(f"polygon is outside the {width}x{height} image")
        cv2.fillPoly(mask, [polygon], 255)

    selected = mask != 0
    changed = selected & (image != 254)
    occupied_cleared = selected & (image <= 89)
    unknown_cleared = selected & (image > 89) & (image < 250)
    cleaned = image.copy()
    cleaned[selected] = 254

    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_pgm), cleaned):
        raise RuntimeError(f"could not write {output_pgm}")
    output_yaml_text, replacements = re.subn(
        r"(?m)^(\s*image\s*:\s*)[^#\r\n]+",
        rf"\g<1>{output_pgm.name}",
        yaml_text,
        count=1,
    )
    if replacements != 1:
        output_pgm.unlink(missing_ok=True)
        raise RuntimeError("could not rewrite image entry in map YAML")
    output_yaml.write_text(output_yaml_text, encoding="utf-8")

    resolution = float(yaml_scalar(yaml_text, "resolution"))
    origin = parse_origin(yaml_text)
    ys, xs = np.nonzero(selected)
    lower_left = pixel_to_map(
        float(xs.min()),
        float(ys.max()),
        height=height,
        resolution=resolution,
        origin=origin,
    )
    upper_right = pixel_to_map(
        float(xs.max()),
        float(ys.min()),
        height=height,
        resolution=resolution,
        origin=origin,
    )
    print(f"source: {source_yaml}")
    print(f"output: {output_yaml}")
    print(f"selected cells: {int(selected.sum())}")
    print(f"changed cells: {int(changed.sum())}")
    print(f"occupied cells cleared: {int(occupied_cleared.sum())}")
    print(f"unknown/intermediate cells cleared: {int(unknown_cleared.sum())}")
    print(
        "combined map-frame bounds: "
        f"({lower_left[0]:.3f}, {lower_left[1]:.3f}) to "
        f"({upper_right[0]:.3f}, {upper_right[1]:.3f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
