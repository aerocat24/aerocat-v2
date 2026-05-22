#!/usr/bin/env python3

import numpy as np
import yaml

# Load calibration results
calibration = np.load('/home/marcus/catkin_ws/src/tcc_marcus_albano/tcc_tello/config/calibration/camera_calibration.npz')
mtx = calibration['mtx']
dist = calibration['dist']

# Create YAML file
calibration_yaml = {
    'camera_matrix': {
        'rows': mtx.shape[0],
        'cols': mtx.shape[1],
        'data': mtx.flatten().tolist()
    },
    'distortion_coefficients': {
        'rows': dist.shape[0],
        'cols': dist.shape[1],
        'data': dist.flatten().tolist()
    }
}

with open('camera_calibration.yaml', 'w') as f:
    yaml.dump(calibration_yaml, f, default_flow_style=False)
