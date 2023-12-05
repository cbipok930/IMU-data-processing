from adafruit_bno08x.i2c import BNO08X_I2C
import queue
import threading as th
from utils.PostprocDefault import PostprocDefault
import math

class Reader():
    __isread = False
    __read_process = None
    __queue = None
    def __init__(self, sensor: BNO08X_I2C, postproc=PostprocDefault(), bias={'a': [0,0,0], 'g': [0,0,0]}) -> None:
        self.sensor = sensor
        self.postproc = postproc
        self.bias = bias
    
    def apply_bias(self, a, g, m):
        return ({'x': a['x'] + self.bias['a'][0], 'y': a['y'] + self.bias['a'][1], 'z': a['z'] + self.bias['a'][2]},
    {'x': g['x'] + self.bias['g'][0], 'y': g['y'] + self.bias['g'][1], 'z': g['z'] + self.bias['g'][2]}, m)
    def __read(self):
        while (self.__isread):
            acc_x, acc_y, acc_z = self.sensor.acceleration
            g_x, g_y, g_z = self.sensor.gyro
            m_x, m_y, m_z = self.sensor.magnetic
            data_a = {"x": acc_x, "y": acc_y, "z": acc_z}
            data_g = {"x": math.degrees(g_x), "y": math.degrees(g_y), "z": math.degrees(g_z)}
            data_m = {"x": m_x, "y": m_y, "z": m_z}
            data_a, data_g, data_m = self.apply_bias(data_a, data_g, data_m)
            # print(data_a, data_g)
            try:
                # print(data_a, data_g)
                dat = self.postproc.apply(data_a, data_g, data_m)
                self.__queue.put(dat, block=False)
            except queue.Full:
                self.__queue.get(block=False)

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
            acc_x, acc_y, acc_z = self.sensor.acceleration
            g_x, g_y, g_z = self.sensor.gyro
            m_x, m_y, m_z = self.sensor.magnetic
            data_a = {"x": acc_x, "y": acc_y, "z": acc_z}
            data_g = {"x": g_x, "y": g_y, "z": g_z}
            data_m = {"x": m_x, "y": m_y, "z": m_z}
            data_a, data_g, data_m = self.apply_bias(data_a, data_g, data_m)
            dat = self.postproc.apply(data_a, data_g, data_m)
            return dat