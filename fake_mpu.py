import json
from time import sleep
import time
import math
from ahrs.filters import EKF as ahrsEKF
import numpy as np
class fake_mpu6050:
    OFFSET_GX = 1.4002653435114352 # 1.407948091603049 1.414287786259538  1.4002653435114352
    OFFSET_GY = 2.3267035877862683 # 2.311277099236632 2.316225190839689  2.3267035877862683
    OFFSET_GZ = 1.056425038167964  # 1.127225954198467 1.1091442748091613 1.056425038167964
                                        # 10000                   100000
    OFFSET_AX = 0.00010532714843736036  # -5.5810546875020254e-05 0.00010532714843736036
    OFFSET_AY = -0.00015212158203138355 # -7.409667968748131e-05  -0.00015212158203138355
    OFFSET_AZ = -5.152587890640912e-05  # 0.00025358886718756477  -5.152587890640912e-05
    data = None
    def __init__(self, fp: str, set_offset = False):
        with open(fp) as file:
            self.data = json.load(file)
        self.idx = 0
        self.len = len(self.data)
        if set_offset:
            self.set_offsets(100)
        self._last_timestamp = time.time()

        self._filt_pitch = 0
        self._filt_roll = 0
        self._filt_yaw = 0

        self.ekf_module = None
        self.ekf_Q = None   
    def init_ekf(self, frame = "ENU", noises = [0.3**2, 0.5**2, 0.8**2]):
        self.ekf_module = ahrsEKF(frame= frame, noises = noises)
        self.ekf_Q = np.tile([1., 0.,0.,0.], (1))

    def get_all_data(self):
        last_time = self.data[self.idx][0]
        acc_data = {"x": self.data[self.idx][1]["a"]["x"], "y": self.data[self.idx][1]["a"]["y"], "z": self.data[self.idx][1]["a"]["z"]}
        gyro_data = {"x": self.data[self.idx][1]["g"]["x"], "y": self.data[self.idx][1]["g"]["y"], "z": self.data[self.idx][1]["g"]["z"]}
        acc_data["x"]-=self.OFFSET_AX
        acc_data["y"]-=self.OFFSET_AY
        acc_data["z"]-=self.OFFSET_AZ
        gyro_data["x"]-=self.OFFSET_GX
        gyro_data["y"]-=self.OFFSET_GY
        gyro_data["z"]-=self.OFFSET_GZ
        self.idx =  (self.idx + 1) % self.len
        if (self.idx == 0):
            sleep(self.data[1][0] - self.data[0][0])
        else:
            sleep(self.data[self.idx][0] - last_time)
        return acc_data, gyro_data
    
    def get_offests(self, count):
        from progress.bar import IncrementalBar
        first_bar = IncrementalBar("Calculating gyro offset and mean accel", max=count)
        second_bar = IncrementalBar("Calculating accel offset", max=count)

        # device = Imu(0x68, 5)
        fake_device = fake_mpu6050("data/10000raw30.10.2023.14_32_10.json")
        
        offset_gx, offset_gy, offset_gz = 0, 0, 0
        mean_ax, mean_ay, mean_az = 0, 0, 0
        offset_ax, offset_ay, offset_az = 0, 0, 0

        for _ in range(count):
            # gx, gy, gz= device.get_gyro_data().values()
            acc, gyro = fake_device.get_all_data()
            gx, gy, gz, = gyro.values()
            offset_gx += gx
            offset_gy += gy
            offset_gz += gz
            # ax, ay, az= device.get_accel_data(g=True).values()
            ax, ay, az = acc.values()
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
            acc, _ = fake_device.get_all_data()
            # ax, ay, az= device.get_accel_data(g=True).values()
            ax, ay, az = acc.values()
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
        print(f"acc means: x={mean_ax} y={mean_ay} z={mean_az}")

        return [offset_ax, offset_ay, offset_az], [offset_gx, offset_gy, offset_gz]
    
    def set_offsets(self, count):
        o_a, o_g = self.get_offests(count)
        self.OFFSET_AX = o_a[0]
        self.OFFSET_AY = o_a[1]
        self.OFFSET_AZ = o_a[2]
        self.OFFSET_GX = o_g[0]
        self.OFFSET_GY = o_g[1]
        self.OFFSET_GZ = o_g[2]
    
    def get_ekf(self, swap_axes=False):

        if self.ekf_module == None:
            self.init_ekf()
        # Get accelerometer data
        # accelerometer_data = self.get_accel_data(g=True, swap_axes=swap_axes)

        # Get gyroscope data
        # gyroscope_data = self.get_gyro_data(swap_axes=swap_axes)
        accelerometer_data, gyroscope_data = self.get_all_data()
        end = time.time()
        self.ekf_module.Dt =  end - self._last_timestamp
        self.ekf_module.frequency = self.ekf_module.Dt ** (-1)
        self._last_timestamp = end
        
        def deg2rad(dat):
            return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
        def dict2arr(dat):
            return [dat['x'], dat['y'], dat['z']]
        self.ekf_Q = self.ekf_module.update(self.ekf_Q,
                                             gyr=dict2arr(deg2rad(gyroscope_data)),
                                               acc=dict2arr(accelerometer_data))
        
        ww = self.ekf_Q[0]
        xx = self.ekf_Q[1]
        yy = self.ekf_Q[2]
        zz = self.ekf_Q[3]
        t0 = +2.0 * (ww * xx + yy * zz)
        t1 = +1.0 - 2.0 * (xx * xx + yy * yy)
        self._filt_roll = math.atan2(t0, t1)
    
        t0 = math.sqrt(1 + 2 * (ww * yy - xx * zz))
        t1 = math.sqrt(1 - 2 * (ww * yy - xx * zz))
        self._filt_pitch = 2 * math.atan2(t0, t1) - math.pi / 2

        return (self._filt_pitch, self._filt_roll, self._filt_yaw)