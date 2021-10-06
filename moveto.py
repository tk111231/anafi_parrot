# -*- coding: UTF-8 -*-
#!/usr/bin/python3

import olympe
import os, csv, time, tempfile
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged,moveToChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status as status
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode


DRONE_IP = "192.168.42.1"
drone = olympe.Drone(DRONE_IP)


def takeoff(drone):
    drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()


def moveto(drone):
    drone(moveTo(
        lat,
        log,
        2.0,
        MoveTo_Orientation_mode.TO_TARGET,
        0.0
        )
        #>> FlyingStateChanged(state="hovering", _timeout=10)
        >> moveToChanged(status=status.DONE,_timeout=10)
        #>> FlyingStateChanged(state="hovering", _timeout=10)
    ).wait()


def land(drone):
    drone(Landing()).wait().success()


def main(drone):
    drone.connection()
    takeoff(drone)
    moveto(drone)
    land(drone)
    drone.disconnection()


if __name__ == "__main__":
    with olympe.Drone(DRONE_IP) as drone:
        main(drone)
