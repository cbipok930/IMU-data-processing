import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
from functools import partial

import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GRAVITY, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ACCELEROMETER
from other.imu_bno import Imu

import queue
import threading as th

class Buffer():
    def __init__(self, sensor:Imu, method):
        self.sensor = sensor
        if method == "ekf":
            def get_gravity(self):
                return self.sensor.get_ekf_gravity()
        elif method == "bltin":
            def get_gravity(self):
                return self.sensor.gravity
        else:
            def get_gravity(self):
                ekf = self.sensor.get_ekf_gravity()
                gb = self.sensor.gravity
                return [{"ekf": ekf[0], "b": gb[0]}, {"ekf": ekf[1], "b": gb[1]}, {"ekf": ekf[2], "b": gb[2]}]
        setattr(Buffer, 'get_gravity', get_gravity)
        self.__isread = True
        self.__queue = queue.Queue(50)
        self.__read_process = th.Thread(target=self.__read)
        self.__read_process.start()
    def __read(self):
        while (self.__isread):
            dat = self.get_gravity()
            try:
                self.__queue.put({"x": dat[0], "y": dat[1], "z": dat[2]}, block=False)
                print(dat)
                
            except queue.Full:
                self.__queue.get(block=False)

    def get_data(self, n = 1):
        if self.__isread:
            items = []
            for _ in range(n):
                while self.__queue.empty():
                    pass
                item =  self.__queue.get()
                items.append(item)
            return items
        else:
            return None
            
    def stop(self):
        self.__isread = False
        self.__read_process.join()



class S(BaseHTTPRequestHandler):
    def __init__(self, buffer: Buffer, *args, **kwargs):
        # self.sensor = sensor
        self.buffer = buffer
        super().__init__(*args, **kwargs)
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        # logging.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        resp = self.buffer.get_data(25)
        self._set_response()
        # self.wfile.write("GET request for {}:".format(self.path).encode('utf-8'))
        self.wfile.write(bytes(str(resp).encode('utf-8')))

    # def do_POST(self):
    #     content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
    #     post_data = self.rfile.read(content_length) # <--- Gets the data itself
    #     logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
    #             str(self.path), str(self.headers), post_data.decode('utf-8'))

    #     self._set_response()
    #     self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))

def run(buffer, server_class=HTTPServer, handler_class=S, port=8080):
    # print("ASD")
    logging.basicConfig(level=logging.INFO)
    server_address = ('', port)
    handler = partial(handler_class, buffer)
    httpd = server_class(server_address, handler)
    logging.info('Starting httpd...\n')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    # sensor.stop()
    httpd.server_close()
    logging.info('Stopping httpd...\n')

if __name__ == '__main__':
    # print("dsf")
    from sys import argv
    parser = argparse.ArgumentParser(description='Plotting Server')
    parser.add_argument("port")
    parser.add_argument("method", choices=["bltin", "ekf", "both"])
    args = parser.parse_args()
    i2c = busio.I2C((1, 14), (1, 15))
    if args.method == "bltin":
        sensor = Imu(i2c, address=0x4b,features=[BNO_REPORT_GRAVITY])
    elif args.method == "ekf":
        sensor = Imu(i2c, address=0x4b, features=[BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER])
    else:
        sensor = Imu(i2c, address=0x4b, features=[BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_GRAVITY])
    buffer = Buffer(sensor, args.method)
    # reader.postproc = algs[args.alg]()
    # reader.start()
    run(buffer, port=int(args.port))
    buffer.stop()
    # print(reader.bias)