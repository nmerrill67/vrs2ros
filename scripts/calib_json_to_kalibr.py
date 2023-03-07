#!/usr/bin/env python3

import os
import json
import yaml
import numpy as np
from pprint import pprint
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser

def write_yaml_opencv_style(yaml_path, calib_yaml):
    print(f"Writing calib yaml to {yaml_path}")
    with open(yaml_path, 'w') as f:
        f.write("%YAML:1.0\n\n")
        yaml.dump(calib_yaml, f)
    # Fix up the format for OpenCV reading in C++ (remove all ' characters from the arrays)
    with open(yaml_path, 'r') as f:
        s = f.read()
        s = [c for c in s if c != "'"]
        # Fix indent
        for i in range(len(s)):
            if s[i] == '-' and s[i+1] == ' ':
                s[i] = "  -"
        s = ''.join(s)
    with open(yaml_path, 'w') as f:
        f.write(s)


def convert(args):
    with open(args.json_path, 'r') as f:
        calib_json = json.load(f)

    pprint(calib_json)

    imu_calib_yaml = {}
    imucam_calib_yaml = {}
    T_Device_Imu = np.eye(4)
    T_Device_Imu_left = np.eye(4)
    T_Device_Camera = np.eye(4)

    for calib in calib_json["ImuCalibrations"]:
        if calib["Label"] == "imu-left":
            T_Device_Imu_left[:3,3] = calib["T_Device_Imu"]["Translation"]
            qxyzw_Device_Imu_left = (
                calib["T_Device_Imu"]["UnitQuaternion"][1] 
                + [calib["T_Device_Imu"]["UnitQuaternion"][0]]
            )
            T_Device_Imu_left[:3,:3] = Rotation.from_quat(qxyzw_Device_Imu).as_matrix()

        # Only grab imu-right for now (IMU0)
        if calib["Label"] == "imu-right":
            T_Device_Imu[:3,3] = calib["T_Device_Imu"]["Translation"]
            qxyzw_Device_Imu = (
                calib["T_Device_Imu"]["UnitQuaternion"][1] 
                + [calib["T_Device_Imu"]["UnitQuaternion"][0]]
            )
            T_Device_Imu[:3,:3] = Rotation.from_quat(qxyzw_Device_Imu).as_matrix()

            # NOTE just took from TUM-VI
            imu_calib_yaml["imu0"] = {
                "T_i_b": [str(x) for x in np.eye(4).tolist()], # Make imu0 the body frame
                "accelerometer_noise_density": 0.0028,  # [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
                "accelerometer_random_walk": 0.00086,   # [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
                "gyroscope_noise_density": 0.00016,     # [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
                "gyroscope_random_walk": 2.2e-05,       # [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
                "model": "calibrated",
                "rostopic": "/imu0",
                "time_offset": 0.0, # Note just init this to zero even though we have one
                "update_rate": 1000.0,
            }

    for calib in calib_json["CameraCalibrations"]:
        if calib["Label"] == "camera-slam-right":
            cam_id = 1
        elif calib["Label"] == "camera-slam-left":
            cam_id = 0
        #elif calib["Label"] == "camera-rgb":
        #    cam_id = "2_rgb"
        else:
            continue

        topic = f"/cam{cam_id}"
        cam_name = f"cam{cam_id}"

        T_Device_Camera[:3,3] = calib["T_Device_Camera"]["Translation"]
        qxyzw_Device_Camera = (
            calib["T_Device_Camera"]["UnitQuaternion"][1] 
            + [calib["T_Device_Camera"]["UnitQuaternion"][0]]
        )
        T_Device_Camera[:3,:3] = Rotation.from_quat(qxyzw_Device_Camera).as_matrix()
        
        T_Camera_Device = np.eye(4)
        T_Camera_Device[:3,:3] = T_Device_Camera[:3,:3].T
        T_Camera_Device[:3,3:4] = -T_Device_Camera[:3,:3].T @ T_Device_Camera[:3,3:4]

        T_Camera_Imu = T_Camera_Device @ T_Device_Imu
        # NOTE kinda guessing about this camera model, seems like f, cx, cy, k1, k2, k3, k4, ....
        # A lot of distortion parameters
        # NOTE the width and height is hardcoded. Not sure how to deal with that otherwise.
        assert calib["Projection"]["Name"] == "FisheyeRadTanThinPrism"
        d = calib["Projection"]["Params"]
        imucam_calib_yaml[cam_name] = {
            "T_cam_imu": [str(x) for x in T_Camera_Imu.tolist()],
            "cam_overlaps": str([abs(cam_id - 1)]),
            "camera_model": "pinhole",
            "distortion_coeffs": str([d[3], d[4], d[5], d[6]]),
            "distortion_model": "equidistant",
            "intrinsics": str([d[0], d[0], d[1], d[2]]),
            "resolution": str(
                [1408, 1408] if calib["Label"] == "camera-rgb" 
                else [640, 480]
            ), # [width, height]
            "rostopic": topic,
        }

    # Write using OpenCV YAML writer for compatability with C++
    # We write two files, one for IMU, one for IMU-CAM
    write_yaml_opencv_style(os.path.join(args.outdir, "kalibr_imu_chain.yaml"), imu_calib_yaml)
    write_yaml_opencv_style(os.path.join(args.outdir, "kalibr_imucam_chain.yaml"), imucam_calib_yaml)

    #print(T_Device_Imu)
    #print(T_Device_Imu_left)
    #print("T_Imu_right_left", np.linalg.inv(T_Device_Imu) @ T_Device_Imu_left)


if __name__ == "__main__":
    
    parser = ArgumentParser("Convert calib json from vrs file to Kalibr yaml format")
    parser.add_argument("--json_path", default="/tmp/calib.json")
    parser.add_argument(
        "--outdir", default=None, 
        help="output directory for the yaml files, if not set place in same directory as input json"
    )

    args = parser.parse_args()
    if args.outdir is None:
        args.outdir = os.path.dirname(args.json_path)
    convert(args)
