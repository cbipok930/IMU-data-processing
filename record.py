import os
import sys
import exceptiongroup as exc
import datetime
import argparse
# import gpiod


from mpu6050 import mpu6050

import busio
from imu_bno import Imu
# from adafruit_bno08x.i2c import _BNO08X_DEFAULT_ADDRESS, BNO08X_I2C
# from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_GRAVITY, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR, BNO_REPORT_ROTATION_VECTOR

# from adafruit_bno08x.i2c import BNO08X_I2C
# from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER

DIRNAME = os.path.abspath(os.path.dirname(__file__))
parent = os.path.dirname(DIRNAME)
# print(DIRNAME, parent)
module_abspath = os.path.join(parent)
sys.path.append(module_abspath)
print(sys.path)
try:
    sys.path.index(module_abspath)
    # from utils.Dumper import Dumper
    from DumperDroneEKFgrav import Dumper as DumperBNO08x
    # from utils.AnglesComp import AnglesComp
    from PostprocDefault import PostprocDefault
except ValueError:
    raise ValueError(f'No such folder: {module_abspath}')

parser = argparse.ArgumentParser(description='Recording IMU data')
parser.add_argument("sensor", choices=["mpu", "bn"])
parser.add_argument("path")
parser.add_argument("record_type", choices=["num", "duration"])
parser.add_argument("-n", "--num_samples", type=int)
parser.add_argument("-d", "--duration_sec", type=float)
parser.add_argument("--date", action='store_true')
parser.add_argument("-p", "--postproc", choices=["comp", "none"], default="none")
args = parser.parse_args()
if args.record_type == "num" and args.num_samples is None:
    raise TypeError("the following arguments are required: -n/--num_samples")
elif args.record_type == "duration" and args.duration_sec is None:
    raise TypeError("the following arguments are required: -d/--duration_sec")


if args.postproc == "none":
    postproc = PostprocDefault()
elif args.postproc == "comp":
    postproc = None#AnglesComp()

sensor = None
dumper = None
if args.sensor == "mpu":
    sensor = mpu6050(0x68)
    dumper = None#Dumper(sensor=sensor, postproc=postproc)
else:
    i2c = busio.I2C((1, 14), (1, 15))
    sensor = Imu(i2c, address=0x4b)
        
    # sensor = BNO08X_I2C(i2c, address=0x4b)
    # sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
    # sensor.enable_feature(BNO_REPORT_MAGNETOMETER)
    # sensor.enable_feature(BNO_REPORT_GYROSCOPE)
    # # sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    dumper = DumperBNO08x(sensor=sensor, postproc=postproc)

if args.date:
    postfix = datetime.datetime.now().strftime("%d.%m.%Y.%H_%M_%S")
else:
    postfix = ""
if args.record_type == "num":
    dumper.record_data(filename= args.path + postfix + ".json", blocking = True, n_samp=args.num_samples)
else:
    dumper.record_data(filename= args.path + postfix + ".json", blocking= True, duration=args.duration_sec)


