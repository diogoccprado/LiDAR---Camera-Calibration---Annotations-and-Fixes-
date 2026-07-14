#!/usr/bin/env python3
"""Generate one folder of automatic SAM2 masks per calibration image."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", type=Path, help="directory containing calibration images")
    parser.add_argument("output", type=Path, help="output mask root")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--model-config",
        required=True,
        help="SAM2 config, for example configs/sam2.1/sam2.1_hiera_s.yaml",
    )
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--points-per-side", type=int, default=32)
    parser.add_argument("--pred-iou-thresh", type=float, default=0.80)
    parser.add_argument("--stability-score-thresh", type=float, default=0.88)
    parser.add_argument("--min-mask-region-area", type=int, default=150)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="regenerate image folders that already contain PNG masks",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.images.is_dir():
        raise SystemExit(f"Image directory not found: {args.images}")
    if not args.checkpoint.is_file():
        raise SystemExit(f"Checkpoint not found: {args.checkpoint}")

    try:
        import cv2
        import numpy as np
        import torch
        from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
        from sam2.build_sam import build_sam2
    except ImportError as error:
        raise SystemExit(
            "Install SAM2, PyTorch, NumPy, and OpenCV in the active environment."
        ) from error

    device = (
        "cuda" if args.device == "auto" and torch.cuda.is_available() else args.device
    )
    if device == "auto":
        device = "cpu"
    model = build_sam2(
        args.model_config,
        str(args.checkpoint),
        device=device,
        apply_postprocessing=False,
    )
    generator = SAM2AutomaticMaskGenerator(
        model,
        points_per_side=args.points_per_side,
        pred_iou_thresh=args.pred_iou_thresh,
        stability_score_thresh=args.stability_score_thresh,
        min_mask_region_area=args.min_mask_region_area,
    )

    image_paths = sorted(args.images.glob("*.png"))
    if not image_paths:
        raise SystemExit(f"No PNG images found in {args.images}")
    args.output.mkdir(parents=True, exist_ok=True)

    for index, image_path in enumerate(image_paths, start=1):
        output_dir = args.output / image_path.stem
        output_dir.mkdir(parents=True, exist_ok=True)
        existing = list(output_dir.glob("*.png"))
        if existing and not args.overwrite:
            print(f"[{index}/{len(image_paths)}] skip {image_path.stem}")
            continue
        if args.overwrite:
            for old_mask in existing:
                old_mask.unlink()

        bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f"[{index}/{len(image_paths)}] unreadable: {image_path}")
            continue
        masks = generator.generate(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        masks.sort(key=lambda mask: mask.get("area", 0), reverse=True)
        written = 0
        for mask in masks:
            segmentation = mask["segmentation"].astype(np.uint8) * 255
            area = int(mask.get("area", np.count_nonzero(segmentation)))
            if area < args.min_mask_region_area:
                continue
            mask_path = output_dir / f"{written:03d}.png"
            if not cv2.imwrite(str(mask_path), segmentation):
                raise RuntimeError(f"Failed to write {mask_path}")
            written += 1
        print(f"[{index}/{len(image_paths)}] {image_path.stem}: {written} masks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
