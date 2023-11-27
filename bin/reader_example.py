from utils.Reader import Reader
from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)

reader = Reader(sensor)
"""Запуск чтения данных с imu в отдельном потоке"""
reader.start()
k = []
start = time.time()
for i in range(1000):
    """
    Если чтение происходит в отдельном потоке, получаем данные из разделенной
    между потоками очереди.
    Иначе получаем данные напрямую с устройства через mpu6050
    """
    a = reader.get_data()
    end = time.time()
    t = end - start
    # print(t)
    k.append(t)
    start = end
"""Завершить поток (вызывать после Reader::start())"""
reader.stop()
"""
                min                   max                 avg
напрямую        0.006262779235839844  0.00964975357055664 0.007218189239501953
буфер в потоке  0.0051767826080322266 0.03937220573425293 0.010970677137374879
"""
print(min(k), max(k), sum(k)/len(k))