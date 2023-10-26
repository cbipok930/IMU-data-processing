from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
import utils.AnglesComp as AnglesComp
import utils.Reader as Reader
from mpu6050 import mpu6050
from functools import partial

class S(BaseHTTPRequestHandler):
    def __init__(self, reader, *args, **kwargs):
        self.reader = reader
        super().__init__(*args, **kwargs)
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        logging.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        self._set_response()
        dat = self.reader.get_data()
        # self.wfile.write("GET request for {}:".format(self.path).encode('utf-8'))
        self.wfile.write(bytes(str(dat).encode('utf-8')))

    def do_POST(self):
        content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
        post_data = self.rfile.read(content_length) # <--- Gets the data itself
        logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
                str(self.path), str(self.headers), post_data.decode('utf-8'))

        self._set_response()
        self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))

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
    reader = Reader.Reader(sensor, AnglesComp.AnglesComp(), {'a': [-0.4877683100219731, -0.04, -0.42], 'g':[4.97, 2.886, 2.1388]})
    reader.start()
    if len(argv) == 2:
        run(reader, port=int(argv[1]))
    else:
        run(reader)