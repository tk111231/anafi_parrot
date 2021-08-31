# -*- coding: UTF-8 -*-
#!/usr/bin/python3

import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing

DRONE_IP = "192.168.42.1"
def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
def camera():
    pass


if __name__ == "__main__":
    main()
