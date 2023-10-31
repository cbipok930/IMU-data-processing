import matplotlib.pyplot as plt
import matplotlib.animation as animation
import http.client as cl
import time
import datetime as dt
import random
import numpy as np
# from mpu6050 import mpu6050
import time
import math
import json

# plt.plot([1,2], [4,6])
c = cl.HTTPConnection("192.168.1.80", 8080)
# c = cl.HTTPConnection("192.168.0.100", 8080)

# start = time.time()
# while(True):
#     c.request("GET", "/")
#     end = time.time()
#     t1 = end - start
#     start = end
#     r1 = c.getresponse()
#     # print(r1.status, r1.reason)
#     data1 = r1.read()
#     print(str(data1), t1)


# Create figure for plotting
fig, ax = plt.subplots(1, 2)
# ax = fig.add_subplot(1, 1, 1)
xs = []
ys_r = []
ys_p = []

# start_t = time.time()

# Initialize communication with TMP102

# This function is called periodically from FuncAnimation
def animate(i, xs, ys_r, ys_p, conection):

    # Read temperature (Celsius) from TMP102
    conection.request("GET", "/")
    r1 = conection.getresponse()
    # print(r1.status, r1.reason)
    data1 = (r1.read().decode("utf-8")).replace("\'", "\"")
    print(data1)
    data1 = json.loads(data1)

    pitch = data1['pitch']
    roll = data1['roll']
    
    # Add x and y to lists
    datenow = dt.datetime.now()
    xs.append("%s:%s.%s" % (datenow.minute, datenow.second, str(datenow.microsecond)[:2]))
    # xs.append("%.2lf" % (time.time() - start_t))
    ys_p.append(pitch)
    ys_r.append(roll)

    # Limit x and y lists to 20 items
    xs = xs[-50:]
    ys_r = ys_r[-50:]
    ys_p = ys_p[-50:]

    # Draw x and y lists
    ax[0].clear()
    ax[0].plot(xs, ys_p)
    ax[0].tick_params(axis='x', rotation=85)
    ax[0].set_ylim([-100, 100])
    ax[1].clear()
    ax[1].plot(xs, ys_r)
    ax[1].set_ylim([-100, 100])
    ax[1].tick_params(axis='x', rotation=85)

    # Format plot
    # ax[0].xticks(rotation=45, ha='right')
    # ax[1].xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    # plt.text(-50, 5, s=str(round(pitch)))
    # ax[0].text(0, 25, s="%.1lf" % (pitch))
    # ax[1].text(0, 25, s="%.1lf" % (roll))
    ax[0].set_title("Pitch = %.2lf" % pitch)
    ax[1].set_title("Roll = %.2lf" % roll)


# Set up plot to call animate() function periodically
# anim = MyFuncAnim()
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys_r, ys_p, c), interval=0)
plt.show()