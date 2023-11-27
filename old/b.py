import numpy as np
from mpu6050 import mpu6050
import time
import angles

# import smbus2 as sm
# bus = sm.SMBus(5)
# data = bus.read_byte(0x4b)
# print("data: ", data)

# sensor = mpu6050(0x68)
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER



i2c = busio.I2C((1, 14), (1, 15))
bno = BNO08X_I2C(i2c, address=0x4b)
bno.initialize()
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.begin_calibration()
# print(bno.calibration_status)

while True:
    acc = bno.acceleration
    gyro = bno.gyro
    mag = bno.magnetic
    print("acc: ", acc)
    print("gyro: ", gyro)
    print("mag: ", mag)
    time.sleep(0.1)

# while True:
#     data_g = sensor.get_gyro_data()
#     data_a = sensor.get_accel_data()
#     t_1 = time.time() - start_1
#     angl = angles.estimate_orientation([mess2list(data_a_old), mess2list(data_a)], [mess2list(deg2rad(data_g_old)), mess2list(deg2rad(data_g))],
#                                 [t_0, t_1])
#     data_g_old = data_g
#     data_a_old = data_a
#     print(angl)
#     t_1 = t_0
#     start_1 = time.time()