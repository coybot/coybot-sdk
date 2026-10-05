#!/usr/bin/env python3
"""Prepare a licensed Open Images subset for Create ML object detection."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import random
import shutil
import subprocess
import urllib.request
from collections import defaultdict


POSITIVE_LABELS = {"/m/02522", "/m/07c52"}
HARD_NEGATIVE_LABELS = {
    "/m/01c648",
    "/m/01n5jq",
    "/m/02d9qx",
    "/m/02dgv",
    "/m/06z37_",
    "/m/0bh9flk",
    "/m/0bt_c3",
    "/m/0d4v4",
}
ACCEPTED_LICENSE_PREFIXES = (
    "http://creativecommons.org/licenses/by/2.0",
    "https://creativecommons.org/licenses/by/2.0",
)


def create_ml_coordinates(row, width, height):
    xmin = float(row["XMin"]) * width
    xmax = float(row["XMax"]) * width
    ymin = float(row["YMin"]) * height
    ymax = float(row["YMax"]) * height
    return {
        "x": (xmin + xmax) / 2,
        "y": (ymin + ymax) / 2,
        "width": xmax - xmin,
        "height": ymax - ymin,
    }


def build_examples(rows, dimensions):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["ImageID"]].append(row)

    examples = []
    for image_id in sorted(grouped):
        image_rows = grouped[image_id]
        positives = [row for row in image_rows if row["LabelName"] in POSITIVE_LABELS]
        is_hard_negative = any(row["LabelName"] in HARD_NEGATIVE_LABELS for row in image_rows)
        if not positives and not is_hard_negative:
            continue
        if image_id not in dimensions:
            raise ValueError(f"Missing image dimensions for {image_id}")
        width, height = dimensions[image_id]
        annotations = [
            {
                "label": "screen",
                "coordinates": create_ml_coordinates(row, width, height),
            }
            for row in positives
        ]
        examples.append({"imagefilename": f"{image_id}.jpg", "annotation": annotations})
    return examples


def partition_image_ids(image_ids, seed, validation_fraction=0.15, test_fraction=0.15):
    ordered = sorted(set(image_ids))
    random.Random(seed).shuffle(ordered)
    validation_count = round(len(ordered) * validation_fraction)
    test_count = round(len(ordered) * test_fraction)
    validation = sorted(ordered[:validation_count])
    test = sorted(ordered[validation_count : validation_count + test_count])
    train = sorted(ordered[validation_count + test_count :])
    return train, validation, test


def read_partition_manifests(paths, available_image_ids):
    partitions = []
    seen = set()
    available = set(available_image_ids)
    for split in ("train", "validation", "test"):
        values = [
            line.strip()
            for line in paths[split].read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        duplicates = seen.intersection(values)
        if duplicates:
            raise ValueError(f"Partition manifests overlap: {sorted(duplicates)}")
        unknown = set(values) - available
        if unknown:
            raise ValueError(f"Partition manifest contains unavailable images: {sorted(unknown)}")
        seen.update(values)
        partitions.append(values)
    return tuple(partitions)


def sha256_file(path):
    digest = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_annotations(paths):
    rows = []
    for path in paths:
        split = "validation" if "validation" in path.name else "test"
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row["LabelName"] in POSITIVE_LABELS | HARD_NEGATIVE_LABELS:
                    row["_source_split"] = split
                    rows.append(row)
    return rows


def read_metadata(paths):
    metadata = {}
    for path in paths:
        split = "validation" if "validation" in path.name else "test"
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                license_url = row.get("License", "")
                if license_url.startswith(ACCEPTED_LICENSE_PREFIXES):
                    metadata[row["ImageID"]] = {
                        "license": license_url,
                        "original_url": row.get("OriginalURL", ""),
                        "source_split": split,
                    }
    return metadata


def choose_rows(rows, metadata, max_positive, max_negative, seed):
    grouped = defaultdict(list)
    for row in rows:
        if row["ImageID"] in metadata:
            grouped[row["ImageID"]].append(row)

    positives = sorted(
        image_id
        for image_id, image_rows in grouped.items()
        if any(row["LabelName"] in POSITIVE_LABELS for row in image_rows)
    )
    negatives = sorted(
        image_id
        for image_id, image_rows in grouped.items()
        if image_id not in positives
        and any(row["LabelName"] in HARD_NEGATIVE_LABELS for row in image_rows)
    )
    rng = random.Random(seed)
    rng.shuffle(positives)
    rng.shuffle(negatives)
    selected = set(positives[:max_positive] + negatives[:max_negative])
    return [row for row in rows if row["ImageID"] in selected]


def image_dimensions(path):
    result = subprocess.run(
        ["/usr/bin/sips", "-g", "pixelWidth", "-g", "pixelHeight", str(path)],
        check=True,
        capture_output=True,
        text=True,
    )
    values = {}
    for line in result.stdout.splitlines():
        if ":" in line:
            key, value = line.strip().split(":", 1)
            values[key] = value.strip()
    return int(values["pixelWidth"]), int(values["pixelHeight"])


def download_image(image_id, source_split, destination):
    url = f"https://open-images-dataset.s3.amazonaws.com/{source_split}/{image_id}.jpg"
    if destination.is_file() and destination.stat().st_size > 0:
        return url
    with urllib.request.urlopen(url, timeout=60) as response, destination.open("wb") as output:
        shutil.copyfileobj(response, output)
    return url


def write_dataset(rows, metadata, output, seed, partitions=None):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["ImageID"]].append(row)
    partitions = partitions or partition_image_ids(grouped, seed=seed)
    provenance = []

    for split_name, image_ids in zip(("train", "validation", "test"), partitions):
        split_dir = output / split_name
        split_dir.mkdir(parents=True, exist_ok=True)
        split_rows = []
        dimensions = {}
        for image_id in image_ids:
            info = metadata[image_id]
            destination = split_dir / f"{image_id}.jpg"
            source_url = download_image(image_id, info["source_split"], destination)
            dimensions[image_id] = image_dimensions(destination)
            split_rows.extend(grouped[image_id])
            provenance.append({
                "image_id": image_id,
                "dataset_split": split_name,
                "source_split": info["source_split"],
                "source_url": source_url,
                "original_url": info["original_url"],
                "license": info["license"],
                "sha256": sha256_file(destination),
            })
        (split_dir / "annotations.json").write_text(
            json.dumps(build_examples(split_rows, dimensions), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (output / f"{split_name}.txt").write_text("\n".join(image_ids) + "\n", encoding="utf-8")

    (output / "provenance.json").write_text(
        json.dumps(sorted(provenance, key=lambda item: item["image_id"]), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--annotations", action="append", type=pathlib.Path, required=True)
    parser.add_argument("--metadata", action="append", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--max-positive", type=int, default=400)
    parser.add_argument("--max-negative", type=int, default=400)
    parser.add_argument("--seed", type=int, default=20260810)
    parser.add_argument("--train-manifest", type=pathlib.Path)
    parser.add_argument("--validation-manifest", type=pathlib.Path)
    parser.add_argument("--test-manifest", type=pathlib.Path)
    return parser.parse_args()


def main():
    args = parse_args()
    metadata = read_metadata(args.metadata)
    all_rows = read_annotations(args.annotations)
    manifest_arguments = (
        args.train_manifest,
        args.validation_manifest,
        args.test_manifest,
    )
    if any(manifest_arguments) and not all(manifest_arguments):
        raise SystemExit("Provide all three partition manifests or none")

    partitions = None
    if all(manifest_arguments):
        available_ids = {row["ImageID"] for row in all_rows if row["ImageID"] in metadata}
        partitions = read_partition_manifests(
            {
                "train": args.train_manifest,
                "validation": args.validation_manifest,
                "test": args.test_manifest,
            },
            available_ids,
        )
        selected_ids = set().union(*partitions)
        rows = [row for row in all_rows if row["ImageID"] in selected_ids]
    else:
        rows = choose_rows(
            all_rows,
            metadata,
            max_positive=args.max_positive,
            max_negative=args.max_negative,
            seed=args.seed,
        )
    if not rows:
        raise SystemExit("No licensed screen or hard-negative annotations selected")
    write_dataset(rows, metadata, args.output, args.seed, partitions=partitions)


if __name__ == "__main__":
    main()
