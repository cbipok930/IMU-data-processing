import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
from functools import partial

import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GRAVITY
from other.imu_bno import Imu

class S(BaseHTTPRequestHandler):
    def __init__(self, sensor, *args, **kwargs):
        self.sensor = sensor
        super().__init__(*args, **kwargs)
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        # logging.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        self._set_response()
        dat = self.sensor.gravity
        # self.wfile.write("GET request for {}:".format(self.path).encode('utf-8'))
        self.wfile.write(bytes(str({"x": dat[0], "y": dat[1], "z": dat[2]}).encode('utf-8')))

    # def do_POST(self):
    #     content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
    #     post_data = self.rfile.read(content_length) # <--- Gets the data itself
    #     logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
    #             str(self.path), str(self.headers), post_data.decode('utf-8'))

    #     self._set_response()
    #     self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))

def run(reader, server_class=HTTPServer, handler_class=S, port=8080):
    # print("ASD")
    logging.basicConfig(level=logging.INFO)
    server_address = ('', port)
    handler = partial(handler_class, reader)
    httpd = server_class(server_address, handler)
    logging.info('Starting httpd...\n')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    reader.stop()
    httpd.server_close()
    logging.info('Stopping httpd...\n')

if __name__ == '__main__':
    # print("dsf")
    from sys import argv
    parser = argparse.ArgumentParser(description='Plotting Server')
    # parser.add_argument("sensor", choices=["mpu", "bno"])
    # parser.add_argument("alg", choices=["ekf", "comp", "aqua", "madgwick"])
    parser.add_argument("port")
    args = parser.parse_args()
    i2c = busio.I2C((1, 14), (1, 15))
    sensor = Imu(i2c, address=0x4b,features=[BNO_REPORT_GRAVITY])

    # reader.postproc = algs[args.alg]()
    # reader.start()
    run(sensor, port=int(args.port))
    # print(reader.bias)