import json
import h5py
import numpy as np

def dict2arr(el):
    return np.array([el["x"], el["y"], el["z"]], dtype=np.float64)

with open("hdf5/15.02.2024.17_00_18.json") as fp:
    jsobj = json.load(fp)
    ll = len(jsobj)
    h5f = h5py.File("hdf5/mytestfile.hdf5", "w")
    # dset = h5f.create_group("")
    accel_bias = h5f.create_dataset("accel_bias", shape=(3,), dtype=np.float64)
    accel_bias[...] = np.zeros((3,))

    accel_calib = h5f.create_dataset("accel_calib", shape=(ll,3), dtype=np.float64)
    # print(np.array([dict2arr(jsobj[i][1]["a"]) for i in list(range(ll))]))
    accel_calib[...] = np.array([dict2arr(jsobj[i][1]["a"]) for i in list(range(ll))])

    accel_raw = h5f.create_dataset("accel_raw", shape=(ll,3), dtype=np.float64)
    accel_raw[...] = np.array([dict2arr(jsobj[i][1]["a"]) for i in list(range(ll))])

    gyro_bias = h5f.create_dataset("gyro_bias", shape=(3,), dtype=np.float64)
    gyro_bias[...] = np.zeros((3,))

    gyro_calib = h5f.create_dataset("gyro_calib", shape=(ll,3), dtype=np.float64)
    gyro_calib[...] = np.array([dict2arr(jsobj[i][1]["g"]) for i in list(range(ll))])

    gyro_raw = h5f.create_dataset("gyro_raw", shape=(ll,3), dtype=np.float64)
    gyro_raw[...] = np.array([dict2arr(jsobj[i][1]["g"]) for i in list(range(ll))])



