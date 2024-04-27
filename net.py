from imu_bno import Imu
import busio
import torch
from torch import nn
import time
import numpy as np


# class TrajRegression(nn.Module):
#   def __init__(self, *args, **kwargs) -> None:
#     super().__init__(*args, **kwargs)
#     self.input_layer = nn.Linear(13, 7)
#     self.hidden = nn.Linear(7, 5)
#     self.out_layer = nn.Linear(5, 3)

#     self.lrelu = nn.LeakyReLU()
#     self.loss_val = None

#   def forward(self, input, target=None):
#     if (target is None):
#       self.requires_grad_(False)
#     else:
#       self.requires_grad_(True)

#     x = self.lrelu(self.input_layer(input))
#     x = self.lrelu(self.hidden(x))
#     output = self.out_layer(x)

#     return output

class TrajRegression(nn.Module):
  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.input_layer = nn.Linear(13, 11)
    self.hidden = nn.Linear(11, 9)
    self.hidden2 = nn.Linear(9, 7)
    self.hidden3 = nn.Linear(7, 5)
    self.out_layer = nn.Linear(5, 3)

    self.lrelu = nn.LeakyReLU()
    self.loss_val = None

  def forward(self, input, target=None):
    if (target is None):
      self.requires_grad_(False)
    else:
      self.requires_grad_(True)

    x = self.lrelu(self.input_layer(input))
    x = self.lrelu(self.hidden(x))
    x = self.lrelu(self.hidden2(x))
    x = self.lrelu(self.hidden3(x))
    output = self.out_layer(x)

    return output

path = "entire_model_bsx4.pt"
model = torch.load(path)
print(model.eval())

i2c = busio.I2C((1, 14), (1, 15))
device = Imu(i2c, address=0x4b)
        
g0 = [0., 0., -9.8]
t0 = time.time()

while True:
    t1 = time.time()
    accX, accY, accZ = device.acceleration
    gyroX, gyroY, gyroZ = device.gyro
    magnX, magnY, magnZ = device.magnetic
    a = device.get_ekf_gravity()
    t2 = time.time()

    ti = (t2 + t1)/2  
    F = (ti - t0) ** (-1)
    input = [g0[0], g0[1], g0[2], F, accX, accY, accZ, gyroX, gyroY, gyroZ, magnX, magnY, magnZ]
    # print("input=", input)
    out = model(torch.Tensor([g0[0], g0[1], g0[2], F, accX, accY, accZ, gyroX, gyroY, gyroZ, magnX, magnY, magnZ]))
    out = torch.Tensor.numpy(out)
    print(out)
    t0 = ti
    g0 = out
t0 = time.time()

while True:
    qq = []
    n = 30
    for i in range(n):
        t1 = time.time()
        accX, accY, accZ = device.acceleration
        gyroX, gyroY, gyroZ = device.gyro
        magnX, magnY, magnZ = device.magnetic
        # time.sleep(200**-1)
        device.get_ekf_angles()
        t2 = time.time()

        ti = (t2 + t1)/2
        qq.append([ti, accX, accY, accZ, gyroX, gyroY, gyroZ, magnX, magnY, magnZ])
    qq = np.array(qq)
    freqs = np.array([(((qq[:, 0][i+1] - qq[:, 0][i])/2) ** -1) for i in list(range(0, n-1))])
    freqs = np.insert(freqs, 0, freqs[0])
    for i in range(n):
       input = [g0[0], g0[1], g0[2], freqs[i], qq[i][1], qq[i][2], qq[i][3], qq[i][4], qq[i][5], qq[i][6], qq[i][7], qq[i][8], qq[i][9]]
       out = model(torch.Tensor(input))
       out = torch.Tensor.numpy(out)
       g0 = out
    print(out)
    continue