import os
import sys
import exceptiongroup as exc
import datetime
import argparse
from mpu6050 import mpu6050

DIRNAME = os.path.dirname(__file__)
parent = os.path.dirname(DIRNAME)
sys.path.append(os.path.join(parent, "utils"))
try:
    sys.path.index("IMU-data-processing/utils")
    from Dumper import Dumper
except ValueError:
    raise ValueError('No such folder: IMU-data-processing/utils')

parser = argparse.ArgumentParser(description='Recording IMU data')
parser.add_argument("path")
parser.add_argument("record_type", choices=["num", "duration"])
parser.add_argument("-n", "--num_samples", type=int)
parser.add_argument("-d", "--duration_sec", type=float)
args = parser.parse_args()
if args.record_type == "num" and args.num_samples is None:
    raise TypeError("the following arguments are required: -n/--num_samples")
elif args.record_type == "duration" and args.duration_sec is None:
    raise TypeError("the following arguments are required: -d/--duration_sec")

sensor = mpu6050(0x68)

dumper = Dumper(sensor=sensor)

if args.record_type == "num":
    dumper.record_data(filename= args.path + ".json", blocking = True, n_samp=args.num_samples)
else:
    dumper.record_data(filename= args.path + ".json", blocking= True, duration=args.duration_sec)


