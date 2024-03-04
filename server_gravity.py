from imu_bno import Imu
import busio
import numpy as np
import cv2


import socket


def server_program():
    host = socket.gethostname()
    host = "127.0.0.1"
    port = 5000  

    server_socket = socket.socket()  
    server_socket.bind((host, port))  

    server_socket.listen(2)
    while True:
        conn, address = server_socket.accept() 
        print("Connection from: " + str(address))



        i2c = busio.I2C((1, 14), (1, 15))
        device = Imu(i2c, address=0x4b)
        X = 200
        Y = 100
        G = 9.80665
        # image = np.zeros((X, Y))
        while True:
            data = conn.recv(1024).decode()
            if not data:
                # if data is not received break
                break
            g1, g2, g3 = device.get_ekf_gravity()
            x = int(((g1 + G) / (2 * G)) * X) - 1
            y = int(((-g2 + G) / (2 * G)) * Y) - 1
            data = f"{X};{Y};{x};{y}"
            conn.send(data.encode())  # send data to the client
        conn.close()  # close the connection


if __name__ == '__main__':
    server_program()



# while True:
#     image[x][y] = 1.
#     cv2.imshow("image", image)
#     if cv2.waitKey(1) == ord('q'):
#             break
#     # print(image)
#     image = np.zeros((X, Y))
