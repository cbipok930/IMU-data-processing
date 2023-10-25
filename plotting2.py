import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# import random
import ahrs
import numpy as np
from mpu6050 import mpu6050
import time
import math


def ToEulerAngles(q):
    angles = [0, 0, 0]

    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    angles[0] = math.atan2(sinr_cosp, cosr_cosp)

    sinp = math.sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]))
    cosp = math.sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]))
    angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

    return angles

def mess2list(dat):
    return [dat['x'], dat['y'], dat['z']]

def deg2rad(dat):
    return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}

def output(angles):
    angles = [round(math.degrees(el)) for el in angles]
    print(f"roll = {angles[0]} pitch = {angles[1]}")


class RD:
    def __init__(self, sensor) -> None:
        start = time.time()
        self.pitch_old = 0
        self.roll_old = 0
        self.data_g_old = sensor.get_gyro_data()
        self.data_a_old = sensor.get_accel_data()
        self.t_1 = time.time() - start
        self.start = time.time()


sensor = mpu6050(0x68)

# Create figure for plotting
fig, ax = plt.subplots(1, 2)
# ax = fig.add_subplot(1, 1, 1)
xs = []
ys_r = []
ys_p = []


# Initialize communication with TMP102

# This function is called periodically from FuncAnimation
def animate(i, xs, ys_r, ys_p, sensor, past_d):

    # Read temperature (Celsius) from TMP102
    data_g_2 = sensor.get_gyro_data()
    data_a_2 = sensor.get_accel_data()
    end = time.time()
    t_2 = end - past_d.start

    Q = ahrs.filters.Complementary(np.array([np.array(mess2list(deg2rad(past_d.data_g_old))), np.array(mess2list(deg2rad(data_g_2)))]),
                               np.array([np.array(mess2list(past_d.data_a_old)), np.array(mess2list(data_a_2))]),
                                 Dt = (t_2 + past_d.t_1)/2,
                                 w0=np.array([past_d.roll_old, past_d.pitch_old, 0]))

    q = Q.Q
    # a1 = ToEulerAngles(q[0])
    a2 = ToEulerAngles(q[1])
    # output(a1)
    output(a2)
    past_d.roll_old = a2[0]
    past_d.pitch_old = a2[1]
    pitch =  a2[1]
    roll = a2[0]
    # t_1 = t_0
    past_d.t_1 = t_2
    past_d.data_a_old = data_a_2
    past_d.data_g_old = data_g_2
    
    # Add x and y to lists
    datenow = dt.datetime.now()
    xs.append("%s.%s" % (datenow.second, str(datenow.microsecond)[:2]))
    ys_p.append(math.degrees(pitch))
    ys_r.append(math.degrees(roll))

    # Limit x and y lists to 20 items
    xs = xs[-50:]
    ys_r = ys_r[-50:]
    ys_p = ys_p[-50:]

    # Draw x and y lists
    ax[0].clear()
    ax[0].plot(xs, ys_p)
    ax[0].set_ylim([-100, 100])
    ax[1].clear()
    ax[1].plot(xs, ys_r)
    ax[1].set_ylim([-100, 100])

    # Format plot
    # ax[0].xticks(rotation=45, ha='right')
    # ax[1].xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    ax[0].set_title("Pitch")
    ax[1].set_title("Roll")
    past_d.start = time.time()


# Set up plot to call animate() function periodically
# anim = MyFuncAnim()
past_d = RD(sensor)
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys_r, ys_p, sensor, past_d), interval=0)
plt.show()
