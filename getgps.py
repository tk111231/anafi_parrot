# -*- coding: UTF-8 -*-

from __future__ import print_function  # python2/3 compatibility for the print function
import olympe
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.Piloting import TakeOff
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged

# Connection
drone = olympe.Drone("192.168.42.1")
drone.connection()

# Wait for GPS fix
drone(GPSFixStateChanged(_policy = 'wait'))

print("GPS position before take-off :", drone.get_state(HomeChanged))
    

# Take-off
drone(TakeOff()).wait()

print("GPS position after take-off : ", drone.get_state(PositionChanged))

drone.disconnection()
