import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def build_output_path(input_path: Path, output_path: str | None) -> Path:
    if output_path:
        return Path(output_path).expanduser().resolve()
    return input_path.with_suffix(".mp4")


def convert_mov_to_mp4(input_path: Path, output_path: Path) -> int:
    ffmpeg_path = shutil.which("ffmpeg")
    if ffmpeg_path is None:
        print("Error: ffmpeg is not installed or not available in PATH.")
        return 1

    if not input_path.exists():
        print(f"Error: input file not found: {input_path}")
        return 1

    if input_path.suffix.lower() != ".mov":
        print(f"Error: input file must be a .mov file: {input_path}")
        return 1

    output_path.parent.mkdir(parents=True, exist_ok=True)

    command = [
        ffmpeg_path,
        "-y",
        "-i",
        str(input_path),
        "-vcodec",
        "libx264",
        "-acodec",
        "aac",
        str(output_path),
    ]

    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as exc:
        print(f"Error: conversion failed with exit code {exc.returncode}.")
        return exc.returncode or 1

    print(f"Conversion completed: {output_path}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a .mov video file to .mp4 using ffmpeg."
    )
    parser.add_argument("input", help="Path to the input .mov file")
    parser.add_argument(
        "-o",
        "--output",
        help="Path to the output .mp4 file. If omitted, the script uses the input name with .mp4 extension.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = build_output_path(input_path, args.output)
    return convert_mov_to_mp4(input_path, output_path)


if __name__ == "__main__":
    sys.exit(main())
