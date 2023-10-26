import mpu6050
import queue
import threading as th
from PostprocDefault import PostprocDefault

class Reader():
    __isread = False
    __read_process = None
    __queue = None
    def __init__(self, sensor: mpu6050.mpu6050, postproc=PostprocDefault(), bias={'a': [0,0,0], 'g': [0,0,0]}) -> None:
        self.sensor = sensor
        self.postproc = postproc
    
    def __read(self):
        while (self.__isread):
            data_g = self.sensor.get_gyro_data()
            data_a = self.sensor.get_accel_data()
            try:
                self.__queue.put(self.postproc.apply(data_a, data_g))
            except queue.Full:
                self.__queue.get()

    def start(self):
        self.__isread = True
        self.__queue = queue.Queue(3)
        self.__read_process = th.Thread(target=self.__read)
        self.__read_process.start()

    def stop(self):
        self.__isread = False
        self.__read_process.join()
    
    def get_data(self):
        if self.__isread:
            item = None
            while self.__queue.empty():
                pass
            item =  self.__queue.get()
            return item
        else:
            data_g = self.sensor.get_gyro_data()
            data_a = self.sensor.get_accel_data()
            return self.postproc.apply(data_a, data_g)



