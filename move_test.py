# -*- coding: utf-8 -*-
from olympe.messages.camera import (
    set_camera_mode,
    set_photo_mode,
    take_photo,
    photo_progress,
)
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.move import extended_move_by
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, GpsLocationChanged
from olympe.messages import gimbal
from math import *
import olympe
import os
import re
import csv
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
import time
import math



DRONE_IP = "192.168.42.1"
drone = olympe.Drone(DRONE_IP)

ANAFI_URL = "http://{}/".format(DRONE_IP)

ANAFI_MEDIA_API_URL = ANAFI_URL + "api/v1/media/medias/"

XMP_TAGS_OF_INTEREST = (
    "CameraRollDegree",
    "CameraPitchDegree",
    "CameraYawDegree",
    "CaptureTsUs",
    # NOTE: GPS metadata is only present if the drone has a GPS fix
    # (i.e. they won't be present indoor)
    "GPSLatitude",
    "GPSLongitude",
    "GPSAltitude",
)

# 楕円体
ELLIPSOID_GRS80 = 1 # GRS80
ELLIPSOID_WGS84 = 2 # WGS84

# 楕円体ごとの長軸半径と扁平率
GEODETIC_DATUM = {
    ELLIPSOID_GRS80: [
        6378137.0,         # [GRS80]長軸半径
        1 / 298.257222101, # [GRS80]扁平率
    ],
    ELLIPSOID_WGS84: [
        6378137.0,         # [WGS84]長軸半径
        1 / 298.257223563, # [WGS84]扁平率
    ],
}

# 反復計算の上限回数
ITERATION_LIMIT = 1000


lat1= 35.7096795
lon1= 139.52309333333332


def takeoff(drone):
    drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()


def set_gimbal(drone):
     drone(gimbal.set_target(
    gimbal_id=0,
    control_mode="position",
    yaw_frame_of_reference="none",   # None instead of absolute
    yaw=0.0,
    pitch_frame_of_reference="absolute",
    pitch=-45.0,
    roll_frame_of_reference="none",     # None instead of absolute
    roll=0.0,
    )).wait()


def setup_photo_mode(drone):
    drone(set_camera_mode(cam_id=0, value="photo")).wait()
    # For the file_format: jpeg is the only available option
    # dng is not supported in burst mode
    drone(
        set_photo_mode(
            cam_id=0,
            mode="single",
            format="rectilinear",
            file_format="jpeg",
            burst="burst_4_over_1s",
            bracketing="preset_1ev",
            capture_interval=0.0,
        )
    ).wait()


def take_photo_single(drone):
    # take a photo burst and get the associated media_id
    photo_saved = drone(photo_progress(result="photo_saved", _policy="wait"))
    drone(take_photo(cam_id=0)).wait()
    photo_saved.wait()
    media_id = photo_saved.received_events().last().args["media_id"]

    # download the photos associated with this media id
    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()
    download_dir = tempfile.mkdtemp()
    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])
        image_response.raise_for_status()
        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)

        # parse the xmp metadata
        with open(download_path, "rb") as image_file:
            image_data = image_file.read()
            image_xmp_start = image_data.find(b"<x:xmpmeta")
            image_xmp_end = image_data.find(b"</x:xmpmeta")
            image_xmp = ET.fromstring(image_data[image_xmp_start : image_xmp_end + 12])
            for image_meta in image_xmp[0][0]:
                xmp_tag = re.sub(r"{[^}]*}", "", image_meta.tag)
                xmp_value = image_meta.text
                # only print the XMP tags we are interested in
                if xmp_tag in XMP_TAGS_OF_INTEREST:
                    print(resource["resource_id"], xmp_tag, xmp_value)


def get_now_gps(drone):
    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy='wait'))
    return drone.get_state(HomeChanged)


def vincenty_inverse(drone,ellipsoid=None):
    gps=get_now_gps(drone)
    start_lat,start_lon,drone_alt=gps['latitude'],gps['longitude'],gps['altitude']

    # 差異が無ければ0を返す
    if abs(start_lat-lat1)<0.00001 and abs(start_lon-lon1)<0.00001:
        s, α1, α2=0, 0, 0
        return (s, α1, α2)
    else:
        # 計算時に必要な長軸半径(a)と扁平率(ƒ)を定数から取得し、短軸半径(b)を算出する
        # 楕円体が未指定の場合はGRS80の値を用いる
        a, ƒ = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
        b = (1 - ƒ) * a

        φ1 = radians(start_lat)
        φ2 = radians(lat1)
        λ1 = radians(start_lon)
        λ2 = radians(lon1)

        # 更成緯度(補助球上の緯度)
        U1 = atan((1 - ƒ) * tan(φ1))
        U2 = atan((1 - ƒ) * tan(φ2))

        sinU1 = sin(U1)
        sinU2 = sin(U2)
        cosU1 = cos(U1)
        cosU2 = cos(U2)

        # 2点間の経度差
        L = λ2 - λ1

        # λをLで初期化
        λ = L

        # 以下の計算をλが収束するまで反復する
        # 地点によっては収束しないことがあり得るため、反復回数に上限を設ける
        for i in range(ITERATION_LIMIT):
            sinλ = sin(λ)
            cosλ = cos(λ)
            sinσ = sqrt((cosU2 * sinλ) ** 2 + (cosU1 * sinU2 - sinU1 * cosU2 * cosλ) ** 2)
            cosσ = sinU1 * sinU2 + cosU1 * cosU2 * cosλ
            σ = atan2(sinσ, cosσ)
            sinα = cosU1 * cosU2 * sinλ / sinσ
            cos2α = 1 - sinα ** 2
            cos2σm = cosσ - 2 * sinU1 * sinU2 / cos2α
            C = ƒ / 16 * cos2α * (4 + ƒ * (4 - 3 * cos2α))
            λʹ = λ
            λ = L + (1 - C) * ƒ * sinα * (σ + C * sinσ * (cos2σm + C * cosσ * (-1 + 2 * cos2σm ** 2)))

            # 偏差が.000000000001以下ならbreak
            if abs(λ - λʹ) <= 1e-12:
                break
        else:
            # 計算が収束しなかった場合はNoneを返す
            return None

        # λが所望の精度まで収束したら以下の計算を行う
        u2 = cos2α * (a ** 2 - b ** 2) / (b ** 2)
        A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
        B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
        Δσ = B * sinσ * (cos2σm + B / 4 * (cosσ * (-1 + 2 * cos2σm ** 2) - B / 6 * cos2σm * (-3 + 4 * sinσ ** 2) * (-3 + 4 * cos2σm ** 2)))

        # 2点間の楕円体上の距離
        s = b * A * (σ - Δσ)

        # 各点における方位角
        α1 = atan2(cosU2 * sinλ, cosU1 * sinU2 - sinU1 * cosU2 * cosλ)
        α2 = atan2(cosU1 * sinλ, -sinU1 * cosU2 + cosU1 * sinU2 * cosλ) + pi

        if α1 < 0:
            α1 = α1 + pi * 2

        return (s, α1, α2)

        """return {
        'distance': s,           # 距離
        'azimuth1': degrees(α1), # 方位角(始点→終点)
        'azimuth2': degrees(α2), # 方位角(終点→始点)
        }"""


def moveby_dpi(drone):
    s, α1, α2 = vincenty_inverse(drone,ellipsoid=1)
    #result=vincenty_inverse(drone,ellipsoid=1)
    β1, β2=degrees(α1), degrees(α2)
    #β1, β2=degrees(result['azimuth1']), degrees(result['azimuth2'])
    azimuth_rad = radians(β1+90)
    drone(
        moveBy(0, 0, -1, azimuth_rad)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()


def moveby_dy(drone):
    s, α1, α2 = vincenty_inverse(drone,ellipsoid=1)
    #result = vincenty_inverse(drone,ellipsoid=1)
    distance=1
    take_photo_single(drone)
    for x in range(100):
        drone(
            extended_move_by(0, -1, 0, 0, 1, 0, 0)
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        take_photo_single(drone)
        distance+=1

        if s<=distance:
            s0=s-(distance-1)
            drone(
                extended_move_by(0, -s0, 0, 0, 1, 0, 0)
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            take_photo_single(drone)
            break


def landing(drone):
    drone(Landing()).wait().success()


def main():
    drone.connect()
    takeoff(drone)
    set_gimbal(drone)
    setup_photo_mode(drone)
    moveby_dpi(drone)
    moveby_dy(drone)
    landing(drone) 
    drone.disconnect()

if __name__ == "__main__":
    main()
    