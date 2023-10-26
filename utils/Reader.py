import mpu6050
import queue
import threading as th
from utils.PostprocDefault import PostprocDefault

class Reader():
    __isread = False
    __read_process = None
    __queue = None
    def __init__(self, sensor: mpu6050.mpu6050, postproc=PostprocDefault(), bias={'a': [0,0,0], 'g': [0,0,0]}) -> None:
        self.sensor = sensor
        self.postproc = postproc
        self.bias = bias
    
    def apply_bias(self, a, g):
        return {'x': a['x'] + self.bias['a'][0], 'y': a['y'] + self.bias['a'][1], 'z': a['z'] + self.bias['a'][2]},{'x': g['x'] + self.bias['g'][0], 'y': g['y'] + self.bias['g'][1], 'z': g['z'] + self.bias['g'][2]}
    def __read(self):
        while (self.__isread):
            data_g = self.sensor.get_gyro_data()
            data_a = self.sensor.get_accel_data()
            data_a, data_g = self.apply_bias(data_a, data_g)
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
            data_a, data_g = self.apply_bias(data_a, data_g)
            data_g = self.sensor.get_gyro_data()
            data_a = self.sensor.get_accel_data()
            return self.postproc.apply(data_a, data_g)



