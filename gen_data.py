import math
import time
import traceback
from multiprocessing import Queue

from log import DefaultLogger
from utils import create_pose_dict
from utils.peripherals.camera import Camera
from utils.peripherals.imu import Imu
from utils.peripherals.pixhawk import PixHawk

logger = DefaultLogger("generate_data")

PREV_PACKET = dict()


def generate_data(
        camera: Camera,
        # imu: Imu,
        pixhawk: PixHawk,
        q_data: Queue
):
    global PREV_PACKET
    try:
        while True:
            # Get frame from camera
            timestamp = time.time_ns()
            frame, frame_id = camera.record()

            # Get data from pixhawk
            pixhawk_packet = pixhawk.get_packet()

            # # Get data from IMU
            # accel_data = imu.get_accel_data(g=True)
            # gyro_data = imu.get_gyro_data()

            # # From degrees to radians
            # gyro_data['x'] = math.radians(gyro_data['x'])
            # gyro_data['y'] = math.radians(gyro_data['y'])
            # gyro_data['z'] = math.radians(gyro_data['z'])

            # # Create imu packet
            # imu_packet = {
            #     'acc_x_IMU': accel_data['x'],
            #     'acc_y_IMU': accel_data['y'],
            #     'acc_z_IMU': accel_data['z'],
            #     'gyro_x_IMU': gyro_data['x'],
            #     'gyro_y_IMU': gyro_data['y'],
            #     'gyro_z_IMU': gyro_data['z'],
            # }

            # Combining packages into metadata
            metadata = {
                "PixHawk_data": pixhawk_packet,
                # "IMU_data": imu_packet
            }

            # Save frame with metadata
            camera.save_frame_with_metadata(frame, metadata, timestamp)

            while True:
                try:
                    # Generate pose dict
                    pose = create_pose_dict(frame, pixhawk_packet)
                    PREV_PACKET = pixhawk_packet
                    break
                except KeyError as e:
                    logger.warning(f"Take old values of {e.args[0]} for vio")
                    pixhawk_packet.update({e.args[0]:PREV_PACKET[e.args[0]]})
                    continue

            if q_data.full():
                continue
            # Put data to queue
            q_data.put((frame, frame_id, pose))
    except KeyboardInterrupt:
        logger.warning("Generating data stopped manualy")
        raise KeyboardInterrupt
    except Exception as e:
        logger.error(
            "Generating data stopped by {}: {}.\n{}".format(
                type(e).__name__, e, traceback.format_exc()
            )
        )
        raise SystemError
