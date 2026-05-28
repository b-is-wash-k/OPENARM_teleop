#!/usr/bin/env python3
"""
Download LeRobot datasets from HuggingFace using direct HTTP (bypasses xet pointer issues).
Run this on the workstation before training.

Usage:
    # Pass repo IDs directly:
    python download_datasets.py 20-wasa/openarm-cube-pickup-20260528
    python download_datasets.py 20-wasa/openarm-cube-pickup-20260528 20-wasa/openarm-cube-pickup-right-20260528

    # Custom output root:
    python download_datasets.py 20-wasa/openarm-cube-pickup-20260528 --root ~/my_datasets

    # No args — falls back to the hardcoded DEFAULTS list:
    python download_datasets.py
"""

import argparse
import requests
from pathlib import Path

HF_BASE = "https://huggingface.co/datasets"

# Fallback list used when no repo IDs are passed on the command line.
DEFAULTS = [
    "20-wasa/openarm-cube-pickup-20260528",
    "20-wasa/openarm-cube-pickup-right-20260528",
]

DEFAULT_ROOT_WORKSTATION = Path.home() / "Biswash" / "lerobot_datasets"
DEFAULT_ROOT_LAPTOP      = Path.home() / "lerobot_datasets"


def download_file(repo_id: str, repo_path: str, local_path: Path) -> None:
    url = f"{HF_BASE}/{repo_id}/resolve/main/{repo_path}"
    local_path.parent.mkdir(parents=True, exist_ok=True)
    resp = requests.get(url, timeout=300, stream=True)
    resp.raise_for_status()
    with open(local_path, "wb") as f:
        for chunk in resp.iter_content(chunk_size=1 << 20):
            f.write(chunk)
    size_kb = local_path.stat().st_size // 1024
    print(f"  OK  {repo_path}  ({size_kb} KB)")


def get_file_list(repo_id: str) -> list[str]:
    api_url = f"https://huggingface.co/api/datasets/{repo_id}"
    resp = requests.get(api_url, timeout=30)
    resp.raise_for_status()
    return [s["rfilename"] for s in resp.json().get("siblings", [])]


def download_dataset(repo_id: str, output_root: Path) -> None:
    print(f"\n{'='*60}")
    print(f"  Downloading: {repo_id}")
    print(f"{'='*60}")

    local_root = output_root / repo_id
    local_root.mkdir(parents=True, exist_ok=True)

    files = get_file_list(repo_id)
    print(f"  {len(files)} files found\n")

    for fpath in files:
        if fpath.startswith("."):
            continue
        local = local_root / fpath
        if local.exists() and local.stat().st_size > 100:
            print(f"  skip {fpath}  (already exists)")
            continue
        download_file(repo_id, fpath, local)

    ep_parquet = local_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    if ep_parquet.exists():
        magic = open(ep_parquet, "rb").read(4)
        status = "OK (PAR1)" if magic == b"PAR1" else f"CORRUPT ({magic})"
        print(f"\n  episodes parquet: {status}")

    print(f"\n  Saved to: {local_root}")
    print(f"  Train with: --dataset.root={local_root}")


def main() -> None:
    # Pick the default root based on which path exists
    default_root = (
        DEFAULT_ROOT_WORKSTATION
        if DEFAULT_ROOT_WORKSTATION.parent.exists()
        else DEFAULT_ROOT_LAPTOP
    )

    parser = argparse.ArgumentParser(
        description="Download LeRobot datasets from HuggingFace (bypasses xet).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "repos",
        nargs="*",
        metavar="ORG/REPO",
        help="One or more HuggingFace dataset repo IDs (e.g. 20-wasa/openarm-cube-pickup-20260528). "
             f"Defaults to: {DEFAULTS}",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=default_root,
        help=f"Local directory to save datasets under. Default: {default_root}",
    )
    args = parser.parse_args()

    repos = args.repos if args.repos else DEFAULTS
    print(f"Output root : {args.root}")
    print(f"Datasets    : {repos}")

    for repo_id in repos:
        download_dataset(repo_id, args.root)

    print("\nAll done.")


if __name__ == "__main__":
    main()
