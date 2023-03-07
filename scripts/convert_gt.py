#!/usr/bin/env python3

import os
import numpy as np
from argparse import ArgumentParser

# Convert the VRS trajectory to OpenVINS format

def convert(args):
    data = np.loadtxt(args.gt_path, delimiter=',')

    # Each line is 
    # timestamp_unix_ns, x_t_world_left_imu, y_t_world_left_imu, z_t_world_left_imu, qw_R_world_left_imu, qx_R_world_left_imu, qy_R_world_left_imu, qz_R_world_left_imu
    # Need to convert to # timestamp(s) tx ty tz qx qy qz qw
    data_new = np.concatenate([
        data[:,0:1] * 1e-9,
        data[:,1:4],
        data[:,5:8],
        data[:,4:5],
    ], axis=1)
    np.savetxt(args.out_path, data_new, fmt='%.6f')

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--gt_path", default="/media/nate/Elements/aria_pilot/everyday_activities/location_1_indoor/script_1/seq_1/recording_1/location/trajectory.csv", help="Root directory containing VRS files somewhere in the tree")
    parser.add_argument("--out_path", default="/media/nate/Elements/aria_pilot/everyday_activities/location_1_indoor/script_1/seq_1/recording_1/trajectory.txt", help="Root directory containing VRS files somewhere in the tree")
    args = parser.parse_args()

    convert(args)
