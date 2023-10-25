import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import numpy as np
from mpu6050 import mpu6050
import time
import math



def deg2rad(dat):
    return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}


class RD:
    def __init__(self) -> None:
        self.pitch_old = 0
        self.roll_old = 0

        self.start = time.time()


sensor = mpu6050(0x68)

# Create figure for plotting
fig, ax = plt.subplots(1, 2)
# ax = fig.add_subplot(1, 1, 1)
xs = []
ys_r = []
ys_p = []


pitch_old = 0
pitch = 0
roll_old = 0
roll = 0

acc_coef = 0.8
gyro_coef = 1 - acc_coef


start = 0

# Initialize communication with TMP102

# This function is called periodically from FuncAnimation
def animate(i, xs, ys_r, ys_p, sensor, acc_coef, gyro_coef, past_d):

    # Read temperature (Celsius) from TMP102
    data_g = sensor.get_gyro_data()
    data_a = sensor.get_accel_data()
    end = time.time()
    t_1 = end - past_d.start

    pitch = (past_d.pitch_old + deg2rad(data_g)['y'] * t_1) * gyro_coef + acc_coef * (math.atan(-data_a['x'] / math.sqrt(data_a['y'] ** 2 + data_a['z'] ** 2)))
    past_d.pitch_old = pitch
    roll = (past_d.roll_old + deg2rad(data_g)['x'] * t_1) * gyro_coef + acc_coef * (math.atan(data_a['y'] / math.sqrt(data_a['x'] ** 2 + data_a['z'] ** 2)))
    past_d.roll_old = roll
    print(round(math.degrees(pitch)), round(math.degrees(roll)))
    
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
    past_d.start = end


# Set up plot to call animate() function periodically
# anim = MyFuncAnim()
past_d = RD()
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys_r, ys_p, sensor, acc_coef, gyro_coef, past_d), interval=0)
plt.show()
