from time import time

from pymavlink import mavutil


class PixData():
    def __init__(self, port='udp:127.0.0.1:14551'):
        # set connection with PixHawk
        self._master = mavutil.mavlink_connection(port)
        self._master.wait_heartbeat()
        print(f'got heartbeat on {port}')

    def _recv(self, types):
        msg = {}
        for type in types:
            msg[type] = self._master.recv_match(type=type, blocking=True).to_dict()

        return msg
        
    def run(self, stop_event, outque, types):
        time_out = time()
        print('start polling pixhawk data')
        while not stop_event.is_set():
            msg = self._recv(types)
            outque.put(msg)
            #print(time() - time_out, end = '        \r')
            if outque.full():
                msg_out = outque.get()
                            
        print()
        print(f'pixhawk polling stopped',)
