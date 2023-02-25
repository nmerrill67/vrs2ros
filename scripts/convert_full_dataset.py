#!/usr/bin/env python3

import os
from time import time
from tqdm import tqdm
from multiprocessing import Pool
from argparse import ArgumentParser

# Finds all vrs files in a directory tree recursively and converts them all to ros1 bag.
# /path/to/recording.vrs is converted to /path/to/recording.bag and calib json and yaml will be placed next to it
# Make sure to have devel/setup.bash sourced

def convert_single(vrs_path):
    rosbag_path = os.path.splitext(vrs_path)[0] + ".bag"
    cmd = f"rosrun vrs2ros vrs2rosbag _vrs_file:={vrs_path} _rosbag:={rosbag_path}"
    print("Running command:", cmd)
    os.system(cmd)
    # Convert json calib to yaml Kalibr format
    outdir = os.path.dirname(vrs_path)
    cmd = f"python3 scripts/calib_json_to_kalibr.py --json_path /tmp/calib.json --outdir {outdir}"
    print("Running command:", cmd)
    os.system(cmd)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--indir", default="/media/nate/Elements/aria_pilot/everyday_activities/", help="Root directory containing VRS files somewhere in the tree")
    args = parser.parse_args()
    vrs_paths = []
    for root, dirs, files in tqdm(os.walk(args.indir), desc=f"Finding VRS files in {args.indir}"):
        for f in files:
            if f.endswith(".vrs"):
                 vrs_paths.append(os.path.join(root, f))


    t0 = time()
    with Pool(1) as p:
      r = list(tqdm(p.imap(convert_single, vrs_paths), total=len(vrs_paths), desc="Converting VRS files to rosbag..."))
    t1 = time()

    print("Converted all VRS files in {(t1-t0)/60/60} hours")
