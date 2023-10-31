import os
import sys
import exceptiongroup as exc
import datetime
import argparse
from mpu6050 import mpu6050

DIRNAME = os.path.abspath(os.path.dirname(__file__))
parent = os.path.dirname(DIRNAME)
# print(DIRNAME, parent)
module_abspath = os.path.join(parent)
sys.path.append(module_abspath)
print(sys.path)
try:
    sys.path.index(module_abspath)
    from utils.Dumper import Dumper
    from utils.AnglesComp import AnglesComp
    from utils.PostprocDefault import PostprocDefault
except ValueError:
    raise ValueError(f'No such folder: {module_abspath}')

parser = argparse.ArgumentParser(description='Recording IMU data')
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

sensor = mpu6050(0x68)
if args.postproc == "none":
    postproc = PostprocDefault()
elif args.postproc == "comp":
    postproc = AnglesComp()
dumper = Dumper(sensor=sensor, postproc=postproc)

if args.date:
    postfix = datetime.datetime.now().strftime("%d.%m.%Y.%H_%M_%S")
else:
    postfix = ""
if args.record_type == "num":
    dumper.record_data(filename= args.path + postfix + ".json", blocking = True, n_samp=args.num_samples)
else:
    dumper.record_data(filename= args.path + postfix + ".json", blocking= True, duration=args.duration_sec)


