from imu_bno import Imu, frequency
import queue
import threading as th

from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_GRAVITY
import busio
import time


import multiprocessing as mp



def read(queekf: mp.Queue):
    i2c = busio.I2C((1, 14), (1, 15))
    dev = Imu(i2c, address=0x4b)
    while True:
        ax, ay, az = dev.acceleration
        gx, gy, gz = dev.gyro
        mx, my, mz = dev.magnetic
        dat_r = [[ax, ay, az], [gx, gy, gz], [mx, my, mz]]
        dat =  dev.get_ekf_gravity(dat_r)
        try:
            queekf.put([dat_r, dat], block=True, timeout=0.01)
        except Exception:
            try:
                queekf.get(block=False)
            except Exception:
                pass

def recive(queekf: mp.Queue):
    freqs = []
    t1 = time.time()
    for i in range(1000):
        item = None
        while queekf.empty():
            pass
        item =  queekf.get(block=True)
        print("item: ", item[1])
        end = time.time()
        #60Hz
        while ((end - t1) < 0.0166666667):
            end = time.time()
            pass
        freqs.append((end - t1)**-1)
        t1 = end
    print(freqs)
    print("mean: ", (sum(freqs)/len(freqs)))
    print("min: ", min(freqs))
    print("max: ", max(freqs))



def main():
    # QueEKF = queue.Queue(20)
    # QueRAW = queue.Queue(3)
    queEKF = mp.Queue(5)

    reader = mp.Process(target=read, args=(queEKF,), daemon=True)
    reciver = mp.Process(target=recive, args=(queEKF,), daemon=True)
    reader.start()
    reciver.start()

if __name__ == "__main__":    
    main()
    while True:
        pass