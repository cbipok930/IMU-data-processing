import time
from fake_mpu import fake_mpu6050
from utils.EKF import EKF
from ahrs.filters import EKF as ahrsEKF
import math

def show_angles(offsetsa, offsetesg):
    fake_device = fake_mpu6050("data/vibr01.11.2023.17_47_41.json")
    ekf = EKF()
    while True:
        accel_data, gyro_data = fake_device.get_all_data()
        accel_data = {"x": accel_data["x"] - offsetsa[0], "y": accel_data["y"] - offsetsa[1], "z": accel_data["z"] - offsetsa[2]}
        gyro_data = {"x": gyro_data["x"] - offsetesg[0], "y": gyro_data["y"] - offsetesg[1], "z": gyro_data["z"] - offsetesg[2]}
        res = ekf.apply(accel_data, gyro_data)
        print(res)

def show_raw_data():
    # device = Imu(0x68, 5)
    # fake_device = fake_mpu6050("data/40s_move31.10.2023.18_27_35.json")
    fake_device = fake_mpu6050("data/vibr01.11.2023.17_47_41.json")
    while True:
        # accel_data = device.get_accel_data(g=True)
        # gyro_data = device.get_gyro_data()
        accel_data, gyro_data = fake_device.get_all_data()
        print(f"Gyro: {accel_data}\tAccel: {gyro_data}")
        time.sleep(0.1)

if __name__ == "__main__":
    device = fake_mpu6050("data/vibr01.11.2023.17_47_41.json", True)
    while True:
        f_roll, f_pitch, f_yaw = device.get_ekf()
        print(f"F_RPY:\t\t{math.degrees(f_roll)} {math.degrees(f_pitch)} {math.degrees(f_yaw)}")

