from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
import utils.AnglesComp as AnglesComp
import utils.Madgwick as Madgwick
import utils.AQUA as AQUA
import utils.EKF as EKF
import utils.Reader as Reader
from mpu6050 import mpu6050
from functools import partial

def calib(reader: Reader.Reader, n_s=100):
    samples = []
    for _ in range(n_s):
        samples.append(reader.get_data())
    bias = {'a':{'x':0., 'y':0., 'z':sensor.GRAVITIY_MS2}, 'g':{'x':0., 'y':0., 'z':0.}}
    for device in ['a', 'g']:
        for k in ['x', 'y', 'z']:
            bias[device][k] -= sum([el[device][k] for el in samples])/n_s
    reader.bias = {'a': [bias['a'][k] for k in ['x', 'y', 'z']], 'g': [bias['g'][k] for k in ['x', 'y', 'z']]}
    

class S(BaseHTTPRequestHandler):
    def __init__(self, reader, *args, **kwargs):
        self.reader = reader
        super().__init__(*args, **kwargs)
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        # logging.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        self._set_response()
        dat = self.reader.get_data()
        # self.wfile.write("GET request for {}:".format(self.path).encode('utf-8'))
        self.wfile.write(bytes(str(dat).encode('utf-8')))

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
    sensor = mpu6050(0x68)
    reader = Reader.Reader(sensor)
    calib(reader, 500)
    # reader.postproc = Madgwick.Madgwick(0.033)
    # reader.postproc = AnglesComp.AnglesComp()
    reader.postproc = AQUA.AQUA()
    # reader.postproc = EKF.EKF()
    #bias {'a': [-0.4877683100219731, -0.04, -0.42], 'g':[4.97, 2.48857, 2.1388]}
    # reader = Reader.Reader(sensor, Madgwick.Madgwick(gain=0.033), {'a': [-0.4877683100219731, -0.04, -0.42], 'g':[4.97, 2.48857, 2.1388]})
    # reader = Reader.Reader(sensor, AnglesComp.AnglesComp(alpha=0.1), {'a': [-0.4877683100219731, -0.04, -0.42], 'g':[4.97, 2.48857, 2.1388]})
    reader.start()
    if len(argv) == 2:
        run(reader, port=int(argv[1]))
    else:
        run(reader)