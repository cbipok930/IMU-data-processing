import os
from utils.ReaderBNO08x import Reader
import time
import datetime
import threading as th
import json
DIRNAME = os.path.dirname(__file__)
class Dumper(Reader):
    def __init__(self, sensor, filename = os.path.join(DIRNAME, datetime.datetime.now().strftime("%d.%m.%Y.%H_%M_%S.json")), *args, **kwargs) -> None:
        Reader.__init__(self, sensor, *args, **kwargs)
        self.filename = filename
    
    def __read_and_write_duration(self, duration):
        data = []
        start = time.time()
        end = time.time()
        while ((end-start) < duration):
            t1 = time.time()
            a = self.get_data()
            t2 = time.time()
            data.append(((t1 + t2)/2, a))
            end = time.time()
        return data
        
    def __read_and_write_nsamp(self, n_samp):
        data = []
        for i in range(n_samp):
            t1 = time.time()
            a = self.get_data()
            t2 = time.time()    
            data.append(((t1 + t2)/2, a))
        return data

    def __dump(self, duration, n_samp):
        try:
            with open(self.filename, 'x') as f:
                if duration is None:
                    data = self.__read_and_write_nsamp(n_samp)
                else:
                    data = self.__read_and_write_duration(duration)
                json.dump(data, f, indent=2)
        except FileNotFoundError:
            raise FileNotFoundError(f"No such directory: {os.path.dirname(self.filename)}")
    def record_data(self, filename = None, duration=None, blocking=False, n_samp=10):
        if filename is None:
            self.filename = os.path.join(DIRNAME, datetime.datetime.now().strftime("%d.%m.%Y.%H_%M_%S.json"))
        else:
            self.filename = filename
        if not blocking:
            self.__read_process = th.Thread(target=self.__dump, args=(duration, n_samp))
            self.__read_process.start()
        else:
            try:
                with open(self.filename, 'x') as f:
                    if duration is None:
                        data = self.__read_and_write_nsamp(n_samp)
                    else:
                        data = self.__read_and_write_duration(duration)
                    json.dump(data, f, indent=2)
            except FileNotFoundError:
                raise FileNotFoundError(f"No such directory: {os.path.dirname(self.filename)}")
    def wait(self):
        self.__read_process.join()
        self.__read_process = None