#!/usr/bin/env python3
"""Conservatively remove thin free-space scan streaks from a ROS PGM map.

Occupied and unknown cells are never changed to free. Free cells removed by a
3x3 morphological opening become unknown, so this operation cannot erase a
wall or invent traversable space. The source map is never overwritten.
"""

import argparse
import re
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-yaml", required=True, type=Path)
    parser.add_argument("--output-prefix", required=True, type=Path)
    args = parser.parse_args()

    source_yaml = args.source_yaml.resolve()
    output_prefix = args.output_prefix.resolve()
    output_yaml = output_prefix.with_suffix(".yaml")
    output_pgm = output_prefix.with_suffix(".pgm")
    if output_yaml == source_yaml:
        raise RuntimeError("source map must not be overwritten")
    if output_yaml.exists() or output_pgm.exists():
        raise RuntimeError("output already exists")

    text = source_yaml.read_text(encoding="utf-8")
    match = re.search(r"(?m)^\s*image\s*:\s*([^#\r\n]+)", text)
    if not match:
        raise RuntimeError("map YAML has no image entry")
    source_pgm = (source_yaml.parent / match.group(1).strip().strip("'\"")).resolve()
    image = cv2.imread(str(source_pgm), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"cannot read {source_pgm}")

    free = (image >= 250).astype(np.uint8) * 255
    kernel = np.ones((3, 3), dtype=np.uint8)
    stable_free = cv2.morphologyEx(free, cv2.MORPH_OPEN, kernel, iterations=1) != 0
    removed = (free != 0) & ~stable_free

    cleaned = image.copy()
    cleaned[removed] = 205  # unknown; never turn any cell into free
    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_pgm), cleaned):
        raise RuntimeError(f"cannot write {output_pgm}")
    output_text = re.sub(
        r"(?m)^(\s*image\s*:\s*)[^#\r\n]+",
        rf"\g<1>{output_pgm.name}", text, count=1)
    output_yaml.write_text(output_text, encoding="utf-8")
    print(f"source: {source_yaml}")
    print(f"output: {output_yaml}")
    print(f"free cells changed to unknown: {int(removed.sum())}")
    print("occupied cells changed: 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
