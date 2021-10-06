import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages import gimbal
from olympe.messages.camera import (
    recording_progress,
    set_camera_mode,
    set_recording_mode,
    set_streaming_mode,
    start_recording,
    stop_recording
    )
from olympe.enums.camera import (
    camera_mode,
    streaming_mode,
    recording_mode,
    resolution,
    framerate,
    hyperlapse_value,
    recording_result
    )


ANAFI_IP = "192.168.42.1"


def takeoff(drone):
    drone(TakeOff()).wait().success()


def land(drone):
    drone(Landing()).wait().success()


def set_gimbal(drone):
     drone(gimbal.set_target(
    gimbal_id=0,
    control_mode="position",
    yaw_frame_of_reference="none",
    yaw=0.0,
    pitch_frame_of_reference="absolute",
    pitch=30.0,
    roll_frame_of_reference="none",
    roll=0.0,
)).wait()


def setup_movie_mode(drone):
    drone(set_camera_mode(cam_id=0, value="recording")).wait()
    drone(set_recording_mode(
            cam_id=0,
            mode="standard",
            resolution="res_1080p",
            framerate="fps_30",
            hyperlapse="ratio_60",
        )
    ).wait()


"""drone(set_streaming_mode(
        cam_id=0,
        mode=streaming_mode.LOW_LATENCY
    )
)"""


def start_movie(drone):
    drone(start_recording(cam_id=0)).wait().success()


def stop_movie(drone):
    drone(stop_recording(cam_id=0)).wait().success()


def main(drone):
    drone.connection()
    """takeoff(drone)"""
    set_gimbal(drone)
    time.sleep(10)
    setup_movie_mode(drone)
    start_movie(drone)
    time.sleep(10)
    stop_movie(drone)
    """land(drone)"""
    drone.disconnection()


if __name__ == "__main__":
    with olympe.Drone(ANAFI_IP) as drone:
        main(drone)