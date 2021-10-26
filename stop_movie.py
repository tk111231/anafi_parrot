import olympe
import time
from olympe.messages import gimbal
from olympe.messages.camera import stop_recording


ANAFI_IP = "192.168.42.1"


def stop_movie(drone):
    drone(stop_recording(cam_id=0)).wait().success()


def main(drone):
    stop_movie(drone)
    drone.disconnection()


if __name__ == "__main__":
    with olympe.Drone(ANAFI_IP) as drone:
        main(drone)