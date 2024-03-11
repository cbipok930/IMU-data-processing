import os
import time
import datetime
import json

from imu_bno import Imu
import multiprocessing as mp

import queue


"""

|measurements----> 1. acc gyro magnet ->  2. ekf gravity. ti = splits mess by halves|

xi = [t, ax, ay, az, gx, gy, gz, magx, magy, magx, ekfx, ekfy, ekfz]

"""

"""
new way
xi = [t (for debug), ax, ay, az, gx, gy, gz, magx, magy, magx, ekfx, ekfy, ekfz], regular freq=60 Hz
"""
DIRNAME = os.path.dirname(__file__)
class Dumper():
    def __init__(self, sensor:Imu, filename = os.path.join(DIRNAME, datetime.datetime.now().strftime("%d.%m.%Y.%H_%M_%S.json")), *args, **kwargs) -> None:

        self.filename = filename
        self.sensor = sensor


        self.__reader = None
        self.__recorder = None
        self.__shareData: mp.Queue = None
        self.last_ts = None

        # self.read = True

    def read_process(self):
        while True:
            accX, accY, accZ = self.sensor.acceleration
            gyroX, gyroY, gyroZ = self.sensor.gyro
            magnX, magnY, magnZ = self.sensor.magnetic

            dat_r = [[accX, accY, accZ], [gyroX, gyroY, gyroZ], [magnX, magnY, magnZ]]
            # dat =  self.get_ekf_gravity(dat_r)
            gX, gY, gZ = self.sensor.get_ekf_gravity(dat_r)
            try:
                self.__shareData.put([accX, accY, accZ, gyroX, gyroY, gyroZ,magnX, magnY, magnZ, gX, gY, gZ], block=True, timeout=0.01)
            except queue.Full:
                try:
                    self.__shareData.get(block=False)
                except queue.Empty:
                    pass
    
    def record_process(self, duration=None, n_samp=None):
        try:
            with open(self.filename, 'x') as f:

                self.sensor.init_ekf()
                self.last_ts = time.time()

                if duration is None:
                    data = self.__read_and_write_nsamp(n_samp)
                else:
                    data = self.__read_and_write_duration(duration)
                json.dump(data, f)
        except FileNotFoundError:
            raise FileNotFoundError(f"No such directory: {os.path.dirname(self.filename)}")
    
    def __read_data_common(self, data):
        item = None
        while self.__shareData.empty():
            pass
        item =  self.__shareData.get(block=True)
        end = time.time()
        #60Hz
        while ((end - self.last_ts) < 0.0166666667):
            end = time.time()
            pass
        item.insert(0, (end + self.last_ts)/2)
        self.last_ts = end
        data.append(item)
        return data
    
    def __read_and_write_duration(self, duration):
        data = []
        start = time.time()
        end = time.time()
        while ((end-start) < duration):
            data = self.__read_data_common(data)
            end = time.time()
        return data
        
    def __read_and_write_nsamp(self, n_samp):
        data = []
        for i in range(n_samp):
            # t1 = time.time()
            # a = self.get_data()
            # t2 = time.time()    
            # data.append(((t1 + t2)/2, a))
            data = self.__read_data_common(data)
        return data

    def record_data(self, filename = None, duration=None, blocking=False, n_samp=10):
        if filename is None:
            self.filename = os.path.join(DIRNAME, datetime.datetime.now().strftime("%d.%m.%Y.%H_%M_%S.json"))
        else:
            self.filename = filename
        # if not blocking:
        #     self.__read_process = th.Thread(target=self.__dump, args=(duration, n_samp))
        #     self.__read_process.start()
        # else:
        self.__shareData = mp.Queue(3)

        self.__reader = mp.Process(target=self.read_process, daemon=True)
        self.__recorder = mp.Process(target=self.record_process, args=(duration, n_samp), daemon=True)

        self.__reader.start()
        self.__recorder.start()

        self.__recorder.join()
        self.__reader.terminate()
        self.__reader.kill()
    # def wait(self):
    #     self.__read_process.join()
    #     self.__read_process = None