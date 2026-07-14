#!/usr/bin/env python3
"""Optimize CalibAnything projection images and build labeled comparison montages."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FILES = {
    "init_proj.png": "Initial projection",
    "refined_proj.png": "Refined projection",
    "init_proj_seg.png": "Initial segmented projection",
    "refined_proj_seg.png": "Refined segmented projection",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="directory containing four projection PNGs")
    parser.add_argument("output_dir", type=Path, help="destination for optimized images")
    parser.add_argument(
        "--source-max-width",
        type=int,
        default=1280,
        help="maximum width of copied source images (default: 1280)",
    )
    parser.add_argument(
        "--panel-width",
        type=int,
        default=800,
        help="width of each montage panel (default: 800)",
    )
    return parser.parse_args()


def font(size: int) -> ImageFont.ImageFont:
    candidates = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    )
    for candidate in candidates:
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


def resize_to_width(image: Image.Image, width: int) -> Image.Image:
    if image.width == width:
        return image.copy()
    height = round(image.height * width / image.width)
    return image.resize((width, height), Image.Resampling.LANCZOS)


def labeled_panel(image: Image.Image, label: str, width: int) -> Image.Image:
    resized = resize_to_width(image, width)
    label_height = max(42, round(width * 0.055))
    panel = Image.new("RGB", (width, resized.height + label_height), "#111827")
    panel.paste(resized, (0, label_height))
    draw = ImageDraw.Draw(panel)
    text_font = font(max(18, round(width * 0.027)))
    box = draw.textbbox((0, 0), label, font=text_font)
    text_width = box[2] - box[0]
    text_height = box[3] - box[1]
    draw.text(
        ((width - text_width) / 2, (label_height - text_height) / 2 - box[1]),
        label,
        fill="white",
        font=text_font,
    )
    return panel


def save_png(image: Image.Image, path: Path) -> None:
    image.save(path, format="PNG", optimize=True, compress_level=9)


def main() -> int:
    args = parse_args()
    if args.source_max_width < 1 or args.panel_width < 1:
        raise SystemExit("Image widths must be positive")

    missing = [name for name in FILES if not (args.input_dir / name).is_file()]
    if missing:
        raise SystemExit(f"Missing projection images: {', '.join(missing)}")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    images: dict[str, Image.Image] = {}
    for name in FILES:
        with Image.open(args.input_dir / name) as source:
            image = source.convert("RGB")
        if image.width > args.source_max_width:
            image = resize_to_width(image, args.source_max_width)
        images[name] = image
        save_png(image, args.output_dir / name)

    before = labeled_panel(images["init_proj.png"], FILES["init_proj.png"], args.panel_width)
    after = labeled_panel(
        images["refined_proj.png"], FILES["refined_proj.png"], args.panel_width
    )
    before_after = Image.new(
        "RGB", (before.width + after.width, max(before.height, after.height)), "white"
    )
    before_after.paste(before, (0, 0))
    before_after.paste(after, (before.width, 0))
    save_png(before_after, args.output_dir / "calibration_before_after.png")

    panels = [
        labeled_panel(images[name], label, args.panel_width)
        for name, label in FILES.items()
    ]
    panel_height = max(panel.height for panel in panels)
    grid = Image.new("RGB", (args.panel_width * 2, panel_height * 2), "white")
    for index, panel in enumerate(panels):
        grid.paste(
            panel,
            ((index % 2) * args.panel_width, (index // 2) * panel_height),
        )
    save_png(grid, args.output_dir / "calibration_projection_grid.png")

    for path in sorted(args.output_dir.glob("*.png")):
        print(f"{path.name}: {path.stat().st_size / (1024 * 1024):.2f} MiB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
