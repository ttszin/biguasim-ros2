"""Splits a collect_boat_dataset.py + label_boat_dataset.py output (flat
boat_XXXX.png + boat_XXXX.txt pairs) into the images/{train,val} +
labels/{train,val} directory layout Ultralytics YOLO expects (its default
label lookup replaces "images" with "labels" in the path), and writes a
data.yaml pointing at it.

Usage:
    python3 prepare_yolo_dataset.py --source-dir dataset_raw --dest-dir dataset_yolo
"""

from __future__ import annotations

import argparse
import os
import random
import shutil

CLASS_NAMES = ["blueboat"]


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare a YOLO train/val split from a labeled dataset")
    here = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--source-dir", default=os.path.join(here, "dataset_raw"))
    parser.add_argument("--dest-dir", default=os.path.join(here, "dataset_yolo"))
    parser.add_argument("--val-fraction", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    images = sorted(f for f in os.listdir(args.source_dir) if f.endswith(".png"))
    pairs = []
    for img in images:
        label = os.path.splitext(img)[0] + ".txt"
        label_path = os.path.join(args.source_dir, label)
        if not os.path.isfile(label_path):
            print(f"  skipping {img}: no matching label file (run label_boat_dataset.py first?)")
            continue
        pairs.append((img, label))

    if not pairs:
        raise SystemExit(f"No labeled image/label pairs found in '{args.source_dir}'.")

    random.seed(args.seed)
    random.shuffle(pairs)
    n_val = max(1, round(len(pairs) * args.val_fraction))
    val_pairs = pairs[:n_val]
    train_pairs = pairs[n_val:]

    for split, split_pairs in (("train", train_pairs), ("val", val_pairs)):
        img_dir = os.path.join(args.dest_dir, "images", split)
        label_dir = os.path.join(args.dest_dir, "labels", split)
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(label_dir, exist_ok=True)
        for img, label in split_pairs:
            shutil.copy2(os.path.join(args.source_dir, img), os.path.join(img_dir, img))
            shutil.copy2(os.path.join(args.source_dir, label), os.path.join(label_dir, label))

    data_yaml_path = os.path.join(args.dest_dir, "data.yaml")
    with open(data_yaml_path, "w") as f:
        f.write(f"path: {os.path.abspath(args.dest_dir)}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"nc: {len(CLASS_NAMES)}\n")
        f.write(f"names: {CLASS_NAMES}\n")

    print(f"Train: {len(train_pairs)} images, Val: {len(val_pairs)} images.")
    print(f"Dataset ready at '{args.dest_dir}'.")
    print(f"data.yaml written to '{data_yaml_path}'.")


if __name__ == "__main__":
    main()
