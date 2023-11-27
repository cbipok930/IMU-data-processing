import math
import os
import time
from datetime import datetime

from mpu6050 import mpu6050

import numpy as np
import ahrs
from ahrs.filters import EKF as ahrsEKF

import math

class Imu(mpu6050):
    SIMPLE_COMPL_COEF = 0.02
    ACC_ANGLS_COEF = 0.1
    SUM_COMPL_COEF = 0.5
                                   # 1000              10000              100000
    OFFSET_GX = 1.4002653435114352 # 1.407948091603049 1.414287786259538  1.4002653435114352
    OFFSET_GY = 2.3267035877862683 # 2.311277099236632 2.316225190839689  2.3267035877862683
    OFFSET_GZ = 1.056425038167964  # 1.127225954198467 1.1091442748091613 1.056425038167964
                                        # 10000                   100000
    OFFSET_AX = 0.00010532714843736036  # -5.5810546875020254e-05 0.00010532714843736036
    OFFSET_AY = -0.00015212158203138355 # -7.409667968748131e-05  -0.00015212158203138355
    OFFSET_AZ = -5.152587890640912e-05  # 0.00025358886718756477  -5.152587890640912e-05

    def __init__(self, address, bus=1):
        self._gyro_pitch = 0
        self._gyro_roll = 0
        self._gyro_yaw = 0

        self._accel_pitch = 0
        self._accel_roll = 0
        self._accel_yaw = 0

        self._filt_pitch = 0
        self._filt_roll = 0
        self._filt_yaw = 0

        self._last_timestamp = time.time()

        #ekf fields
        self.ekf_module = None
        self.ekf_Q = None
        self.ekf_last_timestamp = self._last_timestamp
        #/ekf fields

        super().__init__(address, bus)

    def get_accel_data(self, g=False, swap_axes=False):
        accel_data = super().get_accel_data(g)
        accel_data['x'] -= self.OFFSET_AX
        accel_data['y'] -= self.OFFSET_AY
        accel_data['z'] -= self.OFFSET_AZ
        if swap_axes:
            accel_data.update(x=accel_data['z'], z=accel_data['x'])
        return accel_data

    def get_gyro_data(self, swap_axes=False):
        gyro_data = super().get_gyro_data()
        gyro_data['x'] -= self.OFFSET_GX
        gyro_data['y'] -= self.OFFSET_GY
        gyro_data['z'] -= self.OFFSET_GZ
        if swap_axes:
            gyro_data.update(x=gyro_data['z'], z=gyro_data['x'])
        return gyro_data

    def get_filtered_data(self, swap_axes=False):
        # Get time for calc read data time
        check_time = time.time()

        # Get accelerometer data
        accelerometer_data = self.get_accel_data(g=True, swap_axes=swap_axes)

        # Get gyroscope data
        gyroscope_data = self.get_gyro_data(swap_axes=swap_axes)

        # Get diff time
        diff_time = time.time() - check_time

        # Calc accelerometer angles
        accelerometer_pitch = math.atan2(
            accelerometer_data['x'],
            math.sqrt(accelerometer_data['y']**2 + accelerometer_data['z']**2)
        )
        accelerometer_roll = math.atan2(
            accelerometer_data['y'],
            math.sqrt(accelerometer_data['x']**2 + accelerometer_data['z']**2)
        )
        accelerometer_yaw = math.atan2(
            accelerometer_data['z'],
            math.sqrt(accelerometer_data['x']**2 + accelerometer_data['y']**2)
        )

        # Calc gyroscope angles
        gyroscope_pitch = accelerometer_pitch + gyroscope_data['x'] * diff_time
        gyroscope_roll = accelerometer_roll + gyroscope_data['y'] * diff_time
        gyroscope_yaw = accelerometer_yaw + gyroscope_data['z'] * diff_time

        # Calc filtered data
        filtered_pitch =\
            (1 - self.SIMPLE_COMPL_COEF) * gyroscope_pitch +\
            self.SIMPLE_COMPL_COEF * accelerometer_data['x']
        filtered_roll =\
            (1 - self.SIMPLE_COMPL_COEF) * gyroscope_roll +\
            self.SIMPLE_COMPL_COEF * accelerometer_data['y']
        filtered_yaw =\
            (1 - self.SIMPLE_COMPL_COEF) * gyroscope_yaw +\
            self.SIMPLE_COMPL_COEF * accelerometer_data['z']

        return (filtered_roll, filtered_pitch, filtered_yaw)

    def get_rpy(self, swap_axes=False):
        # Get accelerometer data
        accelerometer_data = self.get_accel_data(g=True, swap_axes=swap_axes)

        # Get gyroscope data
        gyroscope_data = self.get_gyro_data(swap_axes=swap_axes)

        # Calculating accelerometer angles
        accel_pitch = math.atan2(
            accelerometer_data['x'],
            math.sqrt(accelerometer_data['y']**2 + accelerometer_data['z']**2)
        )
        accel_roll = math.atan2(
            accelerometer_data['y'],
            math.sqrt(accelerometer_data['x']**2 + accelerometer_data['z']**2)
        )
        accel_yaw = math.atan2(
            accelerometer_data['z'],
            math.sqrt(accelerometer_data['x']**2 + accelerometer_data['y']**2)
        )

        # get diff time
        diff_time = time.time() - self._last_timestamp
        self._last_timestamp = time.time()

        # Calculating gyroscope angles
        self._gyro_pitch = self._gyro_pitch + gyroscope_data['x'] * diff_time
        self._gyro_roll = self._gyro_roll + gyroscope_data['y'] * diff_time
        self._gyro_yaw = self._gyro_yaw + gyroscope_data['z'] * diff_time

        # Calculating filtered angles
        filtered_pitch =\
            (1 - self.SIMPLE_COMPL_COEF) * self._gyro_pitch +\
            self.SIMPLE_COMPL_COEF * accel_pitch
        filtered_roll =\
            (1 - self.SIMPLE_COMPL_COEF) * self._gyro_roll +\
            self.SIMPLE_COMPL_COEF * accel_roll
        filtered_yaw =\
            (1 - self.SIMPLE_COMPL_COEF) * self._gyro_yaw +\
            self.SIMPLE_COMPL_COEF * accel_yaw

        return (filtered_roll, filtered_pitch, filtered_yaw)

    def get_angles(self, swap_axes=False):
        # Get accelerometer data
        accelerometer_data = self.get_accel_data(g=True, swap_axes=swap_axes)

        # Get gyroscope data
        gyroscope_data = self.get_gyro_data(swap_axes=swap_axes)

        # Calculating accelerometer angles
        self._accel_pitch = (1 - self.ACC_ANGLS_COEF) * self._accel_pitch +\
            self.ACC_ANGLS_COEF * math.atan2(
                accelerometer_data['x'],
                math.sqrt(accelerometer_data['y']**2 + accelerometer_data['z']**2)
            )
        self._accel_roll = (1 - self.ACC_ANGLS_COEF) * self._accel_roll +\
            self.ACC_ANGLS_COEF * math.atan2(
                accelerometer_data['y'],
                math.sqrt(accelerometer_data['x']**2 + accelerometer_data['z']**2)
            )
        self._accel_yaw = (1 - self.ACC_ANGLS_COEF) * self._accel_yaw +\
            self.ACC_ANGLS_COEF * math.atan2(
                accelerometer_data['z'],
                math.sqrt(accelerometer_data['x']**2 + accelerometer_data['y']**2)
            )

        # get diff time
        diff_time = time.time() - self._last_timestamp
        self._last_timestamp = time.time()

        # Calculating gyroscope angles
        self._gyro_pitch = self._gyro_pitch + gyroscope_data['x'] * diff_time
        self._gyro_roll = self._gyro_roll + gyroscope_data['y'] * diff_time
        self._gyro_yaw = self._gyro_yaw + gyroscope_data['z'] * diff_time

        # Calculating filtered angles
        self._filt_pitch =\
            (1 - self.SUM_COMPL_COEF) * (self._filt_pitch + self._gyro_pitch) +\
            self.SUM_COMPL_COEF * self._accel_pitch
        self._filt_roll =\
            (1 - self.SUM_COMPL_COEF) * (self._filt_roll + self._gyro_roll) +\
            self.SUM_COMPL_COEF * self._accel_roll
        self._filt_yaw =\
            (1 - self.SUM_COMPL_COEF) * (self._filt_yaw + self._gyro_yaw) +\
            self.SUM_COMPL_COEF * self._accel_yaw

        return (self._filt_pitch, self._filt_roll, self._filt_yaw)
    
    def get_offsets(self, count):
        from progress.bar import IncrementalBar
        first_bar = IncrementalBar("Calculating gyro offset and mean accel", max=count)
        second_bar = IncrementalBar("Calculating accel offset", max=count)

        # device = Imu(0x68, 5)

        offset_gx, offset_gy, offset_gz = 0, 0, 0
        mean_ax, mean_ay, mean_az = 0, 0, 0
        offset_ax, offset_ay, offset_az = 0, 0, 0

        for _ in range(count):
            gx, gy, gz= super().get_gyro_data().values()
            offset_gx += gx
            offset_gy += gy
            offset_gz += gz
            ax, ay, az= super().get_accel_data().values()
            offset_ax += ax
            offset_ay += ay
            offset_az += az
            first_bar.next()
        first_bar.finish()

        offset_gx /= count
        offset_gy /= count
        offset_gz /= count

        offset_ax /= count
        offset_ay /= count
        offset_az /= count

        offset_ax /= count
        offset_ay /= count
        offset_az /= count

        print("Gyro offset")
        print(f"X: {offset_gx} Y: {offset_gy} Z: {offset_gz}")
        print("Accel offset")
        print(f"X: {offset_ax} Y: {offset_ay} Z: {offset_az}")
        return [offset_ax, offset_ay, offset_az], [offset_gx, offset_gy, offset_gz]
    
    def set_offsets(self, count):
        o_a, o_g = self.get_offsets(count)
        self.OFFSET_AX = o_a[0]
        self.OFFSET_AY = o_a[1]
        self.OFFSET_AZ = o_a[2]
        self.OFFSET_GX = o_g[0]
        self.OFFSET_GY = o_g[1]
        self.OFFSET_GZ = o_g[2]
        self._last_timestamp = time.time()
    
    def euler_to_quaternion(self):
        (yaw, pitch, roll) = (self._filt_yaw, self._filt_pitch, self._filt_roll)
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qw, qx, qy, qz]

    def init_ekf(self, frame = "ENU", noises = [0.3**2, 0.5**2, 0.8**2]):
        self.ekf_module = ahrsEKF(frame= frame, noises = noises)
        self.ekf_Q = np.tile(self.euler_to_quaternion(), (1))
        self.ekf_last_timestamp = self._last_timestamp
    
    def get_ekf_angles(self):

        if self.ekf_module == None or self.ekf_last_timestamp != self._last_timestamp:
            self.init_ekf()
        # Get accelerometer data
        accelerometer_data = self.get_accel_data()

        # Get gyroscope data
        gyroscope_data = self.get_gyro_data()

        end = time.time()
        self.ekf_module.Dt =  end - self._last_timestamp
        self.ekf_module.frequency = self.ekf_module.Dt ** (-1)
        self._last_timestamp = end
        self.ekf_last_timestamp = end
        
        def deg2rad(dat):
            return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
        def dict2arr(dat):
            return [dat['x'], dat['y'], dat['z']]
        self.ekf_Q = self.ekf_module.update(self.ekf_Q,
                                             gyr=dict2arr(deg2rad(gyroscope_data)),
                                               acc=dict2arr(accelerometer_data))
        
        rm = ahrs.common.orientation.q2R(self.ekf_Q)
        self._filt_roll = math.atan2(rm[2][1],rm[2][2]);#-math.asin(rm[0][2])
        self._filt_pitch = math.atan2(-rm[2][0], math.sqrt(rm[2][1]**2 + rm[2][2]**2))#math.atan2(-rm[1][2], rm[2][2])

        return (self._filt_pitch, self._filt_roll, self._filt_yaw)
    
def get_offsets(count):
    from progress.bar import IncrementalBar
    first_bar = IncrementalBar("Calculating gyro offset and mean accel", max=count)
    second_bar = IncrementalBar("Calculating accel offset", max=count)

    device = Imu(0x68, 5)

    offset_gx, offset_gy, offset_gz = 0, 0, 0
    mean_ax, mean_ay, mean_az = 0, 0, 0
    offset_ax, offset_ay, offset_az = 0, 0, 0

    for _ in range(count):
        gx, gy, gz= device.get_gyro_data().values()
        offset_gx += gx
        offset_gy += gy
        offset_gz += gz
        ax, ay, az= device.get_accel_data(g=True).values()
        mean_ax += ax
        mean_ay += ay
        mean_az += az
        first_bar.next()
    first_bar.finish()

    offset_gx /= count
    offset_gy /= count
    offset_gz /= count

    mean_ax /= count
    mean_ay /= count
    mean_az /= count

    for _ in range(count):
        ax, ay, az= device.get_accel_data(g=True).values()
        offset_ax +=  mean_ax - ax
        offset_ay += mean_ay - ay
        offset_az += mean_az - az
        second_bar.next()
    second_bar.finish()

    offset_ax /= count
    offset_ay /= count
    offset_az /= count

    print("Gyro offset")
    print(f"X: {offset_gx} Y: {offset_gy} Z: {offset_gz}")
    print("Accel offset")
    print(f"X: {offset_ax} Y: {offset_ay} Z: {offset_az}")

def write_data(count):
    from progress.bar import IncrementalBar
    bar = IncrementalBar('Writing data', max=count)

    data_file = os.path.join(
        os.path.dirname(__file__),
        "datasets", f"data_{datetime.now().strftime('%H:%M:%S_%d.%m.%Y')}.csv"
    )

    device = Imu(0x68, 5)

    start_reading = time.time()
    with open(data_file, "w+") as f:
        f.write("Packet number,Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g)\n")
        for _ in range(count):
            accel_x, accel_y, accel_z = device.get_accel_data(g=True).values()
            gyro_x, gyro_y, gyro_z = device.get_gyro_data().values()

            accel_x -= device.OFFSET_AX
            accel_y -= device.OFFSET_AY
            accel_z -= device.OFFSET_AZ

            gyro_x -= device.OFFSET_GX
            gyro_y -= device.OFFSET_GY
            gyro_z -= device.OFFSET_GZ

            f.write(f"{_},{gyro_x},{gyro_y},{gyro_z},{accel_x},{accel_y},{accel_z}\n")
            bar.next()
    bar.finish()


def show_raw_data():
    device = Imu(0x68, 5)
    while True:
        accel_data = device.get_accel_data(g=True)
        gyro_data = device.get_gyro_data()

        print(f"Gyro: {accel_data}\tAccel: {gyro_data}")
        time.sleep(0.1)

def show_rpy_and_filtered_data():
    device = Imu(0x68, 5)
    device.set_offsets(500)
    while True:
        # filtered_roll, filtered_pitch, filtered_yaw = device.get_filtered_data(True)
        # filtered_roll = math.degrees(filtered_roll)
        # filtered_pitch = math.degrees(filtered_pitch)
        # filtered_yaw = math.degrees(filtered_yaw)
        # print(f"Filtered:\t{filtered_roll} {filtered_pitch} {filtered_yaw}")

        # roll, pitch, yaw = device.get_rpy()
        # print(f"RPY:\t\t{roll} {pitch} {yaw}")

        # f_roll, f_pitch, f_yaw = device.get_angles(True)
        f_roll, f_pitch, f_yaw = device.get_ekf_angles()

        print(f"F_RPY:\t\t{f_roll} {f_pitch} {f_yaw}")
        time.sleep(0.1)


if __name__ == "__main__":
    show_rpy_and_filtered_data()