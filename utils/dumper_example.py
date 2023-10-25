from Dumper import Dumper
from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)

dumper = Dumper(sensor=sensor)
dumper.record_data(n_samp=1000)
print("Doing something in thread...")
dumper.wait()

dumper.record_data(duration=5)
print("Doing something in thread...")
dumper.wait()

print("Blocking...")
dumper.record_data(duration=3., blocking=True)