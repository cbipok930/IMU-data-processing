import os
from ReaderBNO08x import Reader
import time
import datetime
import threading as th
import json
import ahrs
import math
import pyrealsense2.pyrealsense2  as rs
import numpy as np
from scipy.spatial.transform import Rotation as R

def q2angles(Q):
        rm = ahrs.common.orientation.q2R(Q)
        roll = math.atan2(rm[2][1],rm[2][2]);#-math.asin(rm[0][2])
        pitch = math.atan2(-rm[2][0], math.sqrt(rm[2][1]**2 + rm[2][2]**2))#math.atan2(-rm[1][2], rm[2][2])
        yaw = math.atan2(rm[1][0], rm[0][0])
        return roll, pitch, yaw

t265_2_bnoR = R.from_euler('zyx', [0, 0, -90], degrees=True)
"""
x:pi/2
x=-x
"""
def t265_to_bno(vec):
    vec = t265_2_bnoR.apply(vec)
    vec[0] = -vec[0]
    return vec

import pyrealsense2 as rs2
ctx = rs2.context()
list = ctx.query_devices()
for dev in list:
      serial = dev.query_sensors()[0].get_info(rs2.camera_info.serial_number)
      # compare to desired SN
      print(dev)
      dev.hardware_reset()



pipeline = rs.pipeline()
config = rs.config()
pipeline.start(config)

        # self.filename = filename
    
    # def __read_data_common(self, data):
        # t1 = time.time()
        # a = self.get_data()
import time
while True:
    frames = pipeline.wait_for_frames(1500)
    dof6 = rs.pose_frame(frames[4]).get_pose_data()
    pose = {"x": dof6.translation.x, "y": dof6.translation.y, "z": dof6.translation.z}
    rot = {"x": dof6.rotation.x, "y": dof6.rotation.y, "z": dof6.rotation.z, "w": dof6.rotation.w}

            # t2 = time.time()
            # data.append(((t1 + t2)/2, a, pose, rot))
    x, y, z = pose["x"], pose["y"], pose["z"]
    x, y, z = t265_to_bno(np.array([x, y, z]))
    print(f"{x:.1f}  {y:.1f}   {z:.1f}")
#     print(f"{rot['x']:.1f}  {rot['y']:.1f}   {rot['z']:.1f} {rot['w']:.1f}")
    x, y, z, w = (rot["x"], rot["y"], rot["z"], rot["w"])
    r, p, y = q2angles(np.array([w, x, y, z]))
    # print(f"roll={math.degrees(r):.2f}  pitch={math.degrees(p):.2f}, yaw={math.degrees(y):.2f}")
#     time.sleep(0.5)
    