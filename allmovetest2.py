# -*- coding: utf-8 -*-
from olympe.messages.camera import (
    set_camera_mode,
    set_photo_mode,
    take_photo,
    photo_progress,
)
from olympe.messages.ardrone3.PilotingSettings import MaxTilt, MaxAltitude
from olympe.messages.ardrone3.PilotingSettingsState import MaxAltitudeChanged
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

# 緯度経度は時計回りに記入するとする
lat0= 35.709744
lon0= 139.523317

lat1= 35.709619
lon1= 139.523321

lat2= 35.709622
lon2= 139.523093

lat3= 35.709659
lon3= 139.523097


# 横にずれる幅[m] (distance)
x= 2

# ドローンの撮影間隔[m] (distance0)
y= 2

#ドローンの高度[m]
z= 5

# 横にずれた距離カウント[m]
distance1= x
distance2= x

# 縦にずれた距離カウント[m]
distance0= y
distance00= y
distance000= y
distance0000= y
distance9= y
distance99= y

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
    pitch=-90.0,
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


"""def get_now_gps(drone):
    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy='wait'))
    return drone.get_state(HomeChanged)"""


def vincenty_inverse(p0,p1,p2,p3,ellipsoid=None):
    """gps=get_now_gps(drone)
    lat0,lon0,alt0=gps['latitude'],gps['longitude'],gps['altitude']
    print('---------------latitude is '+str(gps['latitude'])+' ---------------')
    print('---------------longitude is '+str(gps['longitude'])+' ---------------')
    print('---------------altitude is '+str(gps['altitude'])+' ---------------')"""

    # 差異が無ければ0を返す
    if abs(p0-p1)<0.000001 and abs(p2-p3)<0.000001:
        s, α1, α2=0, 0, 0
        return (s, α1, α2)
    else:
        # 計算時に必要な長軸半径(a)と扁平率(ƒ)を定数から取得し、短軸半径(b)を算出する
        # 楕円体が未指定の場合はGRS80の値を用いる
        a, ƒ = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
        b = (1 - ƒ) * a

        φ1 = radians(p0)
        φ2 = radians(p1)
        λ1 = radians(p2)
        λ2 = radians(p3)

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

        return (s, α1)


def moveby0(drone):
    s1, α1= vincenty_inverse(lat0,lat1,lon0,lon1,ellipsoid=1)
    z0=-z+1
    print('------------------------------altitude is '+str(z)+' ------------------------------')
    print('------------------------------radians α1 is '+str(degrees(α1))+' ------------------------------')
    drone(
        moveBy(0, 0, z0, α1) # 高度、角度調整
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()


def moveby_roop1(drone):
    s1, α1= vincenty_inverse(lat0,lat1,lon0,lon1,ellipsoid=1)
    s2, α2= vincenty_inverse(lat1,lat2,lon1,lon2,ellipsoid=1)
    s4, α4= vincenty_inverse(lat0,lat3,lon0,lon3,ellipsoid=1)
    global distance1, distance0, distance00, distance000, distance0000
    β1=α2-α1
    β2=α4-α1
    θ1=β2
    θ2=pi-β1
    
    for i in range(100):
        d1=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 行きの移動距離
        distance1+=x
        print('------------------------------distance of main1 is '+str(d1)+' -----------------------------')
        take_photo_single(drone)
        for i in range(100):
            drone(
                extended_move_by(y, 0, 0, 0, 1, 0, 0)
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            take_photo_single(drone)
            distance0+=y
            if d1<=distance0:
                d0=d1-(distance0-y)
                drone(
                    extended_move_by(d0, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                break
        print('------------------------------radians of turn1 is '+str(degrees(β1))+' ------------------------------')
        drone(
            moveBy(0, 0, 0, β1) # ターン角度1
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        d2=x/sin(θ2) # 行きの移動幅
        print('------------------------------distance of turn1 is '+str(d2)+' ------------------------------')
        drone(
            extended_move_by(d2, 0, 0, 0, 1, 0, 0)
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        take_photo_single(drone)
        print('------------------------------radians of turn2 is '+str(degrees(θ2))+' ------------------------------')
        drone(
            moveBy(0, 0, 0, θ2) # ターン角度2
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        d3=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 帰りの移動距離
        distance1+=x
        print('------------------------------distance of main2 is '+str(d3)+' -----------------------------')
        for i in range(100):
            drone(
                extended_move_by(y, 0, 0, 0, 1, 0, 0)
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            take_photo_single(drone)
            distance00+=y
            if d3<=distance00:
                d00=d3-(distance00-y)
                drone(
                    extended_move_by(d00, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                break
        print('------------------------------radians of turn3 is '+str(pi-θ1)+' ------------------------------')
        drone(
            moveBy(0, 0, 0, -(pi-θ1)) # ターン角度3
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        d4=x/sin(θ1) # 帰りの移動幅
        print('------------------------------distance of turn2 is '+str(d4)+' ------------------------------')
        drone(
            extended_move_by(d4, 0, 0, 0, 1, 0, 0)
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        take_photo_single(drone)
        print('------------------------------radians of turn4 is '+str(θ1)+' ------------------------------')
        drone(
            moveBy(0, 0, 0, -θ1) # ターン角度4
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()

        # 移動した距離が上の頂点を越えそうになった場合の処理
        if distance1>s2*sin(θ2) and s4*sin(θ1)>s2*sin(θ2):
            d5=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 行きの移動距離ラスト
            print('------------------------------distance of mainlast is '+str(d5)+' -----------------------------')
            take_photo_single(drone)
            for i in range(100):
                drone(
                    extended_move_by(y, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                distance000+=y
                if d5<=distance000:
                    d000=d5-(distance000-y)
                    drone(
                        extended_move_by(d000, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                take_photo_single(drone)
                break
            print('------------------------------radians of turn1last is '+str(β1)+' -----------------------------')
            drone(
                moveBy(0, 0, 0, β1) # ターン角度1ラスト
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            s0=s2*sin(θ2)-(distance1-x) # 残りの幅
            s=s0/sin(θ2) # 行きの移動幅ラスト
            print('------------------------------distance of turn1last is '+str(s)+' -----------------------------')
            drone(
                extended_move_by(s, 0, 0, 0, 1, 0, 0)
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            take_photo_single(drone)
            print('------------------------------radians of turn2last is '+str(θ2)+' -----------------------------')
            drone(
                moveBy(0, 0, 0, θ2) # ターン角度2ラスト
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            d6=s1-(s2*sin(θ2)/tan(θ2))-(s2*sin(θ2)/tan(θ1)) # 帰りの移動距離ラスト
            print('------------------------------distance of main2last is '+str(d6)+' -----------------------------')
            for i in range(100):
                drone(
                    extended_move_by(y, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                distance0000+=y
                if d6<=distance0000:
                    d0000=d6-(distance0000-y)
                    drone(
                        extended_move_by(d0000, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    break
            break

        # 移動した距離が下の頂点を越えそうになった場合の処理
        elif distance1>s4*sin(θ1) and s2*sin(θ2)>s4*sin(θ1):
            d5=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 行きの移動距離ラスト
            print('------------------------------distance of main1last is '+str(d5)+' -----------------------------')
            take_photo_single(drone)
            for i in range(100):
                drone(
                    extended_move_by(y, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                distance000+=y
                if d5<=distance000:
                    d000=d5-(distance000-y)
                    drone(
                        extended_move_by(d000, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                take_photo_single(drone)
                break
            print('------------------------------radians of turn1last is '+str(β1)+' -----------------------------')
            drone(
                moveBy(0, 0, 0, β1) # ターン角度1ラスト
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            s0=s4*sin(θ1)-(distance1-x) # 残りの幅
            s=s0/sin(θ1) # 行きの移動距離ラスト
            print('------------------------------distance of turn1last is '+str(s)+' -----------------------------')
            drone(
                extended_move_by(s, 0, 0, 0, 1, 0, 0)
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            take_photo_single(drone)
            print('------------------------------radians of turn2last is '+str(degrees(θ2))+' ------------------------------')
            drone(
                moveBy(0, 0, 0, θ2) # ターン角度2ラスト
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()
            d6=s1-(s4*sin(θ1)/tan(θ2))-(s4*sin(θ1)/tan(θ1)) # 帰りの移動距離ラスト
            print('------------------------------distance of main2last is '+str(d6)+' -----------------------------')
            for i in range(100):
                drone(
                    extended_move_by(y, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                distance0000+=y
                if d6<=distance0000:
                    d0000=d6-(distance0000-y)
                    drone(
                        extended_move_by(d0000, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    break
            break
        distance0=y
        distance00=y
        distance000=y
        distance0000=y
    
    print('----------------------------------------')
    print('roop1 finish')
    print('----------------------------------------')


def moveby_roop2(drone):
    s1, α1= vincenty_inverse(lat0,lat1,lon0,lon1,ellipsoid=1)
    s2, α2= vincenty_inverse(lat1,lat2,lon1,lon2,ellipsoid=1)
    s31, α31= vincenty_inverse(lat2,lat3,lon2,lon3,ellipsoid=1)
    s32, α32= vincenty_inverse(lat3,lat2,lon3,lon2,ellipsoid=1)
    s4, α4= vincenty_inverse(lat0,lat3,lon0,lon3,ellipsoid=1)
    global distance2, distance9, distance99
    β1=α2-α1
    β2=α31-α1
    β3=α4-α1
    γ1=pi-β2
    γ2=α32-α1
    θ1=β3
    θ2=pi-β1
    
    # 差異がほぼない（面積が小さい）場合、フライト終了
    if s31*sin(γ1)<x or s31*sin(γ2)<x:
        print('distance s31*sin(γ1) is '+str(s31*sin(γ1)))
        print('distance s31*sin(γ2) is '+str(s31*sin(γ2)))
        print('一方がxの値より小さいため終了')
        pass
    else:
        # 上の頂点の方が近かった場合の処理
        if s2*sin(θ2)<s4*sin(θ1):
            print('上の頂点の方が下の頂点よりs1から近い')
            for i in range(100):
                print('------------------------------radians of turn1 is '+str(pi-θ1)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, -(pi-θ1)) # ターン角度1
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d1=x/sin(θ1) # 行きの移動幅
                print('------------------------------distance of turn1 is '+str(d1)+' -----------------------------')
                drone(
                    extended_move_by(d1, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                print('------------------------------radians of turn2 is '+str(θ1)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, -θ1) # ターン角度2
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d2=((s31*sin(γ1)-distance2)/tan(γ1))+((s31*sin(γ1)-distance2)/tan(θ1)) # 行きの移動距離
                distance2+=x
                print('------------------------------distance of main1 is '+str(d2)+' ------------------------------')
                for i in range(100):
                    drone(
                        extended_move_by(y, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    distance9+=y
                    if d2<=distance9:
                        d0=d2-(distance9-y)
                        drone(
                            extended_move_by(d0, 0, 0, 0, 1, 0, 0)
                            >> FlyingStateChanged(state="hovering", _timeout=5)
                        ).wait().success()
                        take_photo_single(drone)
                        break
                print('------------------------------radians of turn3 is '+str(β2)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, β2) # ターン角度3
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d3=x/sin(γ1) # 帰りの移動幅
                print('------------------------------distance of turn2 is '+str(d3)+' -----------------------------')
                drone(
                    extended_move_by(d3, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                print('------------------------------radians of turn4 is '+str(γ1)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, γ1) # ターン角度4
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d4=((s31*sin(γ1)-distance2)/tan(γ1))+((s31*sin(γ1)-distance2)/tan(θ1)) # 帰りの移動距離
                distance2+=x
                print('------------------------------distance of main2 is '+str(d4)+' ------------------------------')
                for i in range(100):
                    drone(
                        extended_move_by(y, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    distance99+=y
                    if d4<=distance99:
                        d00=d4-(distance99-y)
                        drone(
                            extended_move_by(d00, 0, 0, 0, 1, 0, 0)
                            >> FlyingStateChanged(state="hovering", _timeout=5)
                        ).wait().success()
                        take_photo_single(drone)
                        break
                distance9=y
                distance99=y

                # 最後の頂点を越えそうになった場合の処理
                if distance2>s31*sin(γ1):
                    print('------------------------------radians of turn1last is '+str(pi-θ1)+' -----------------------------')
                    drone(
                        moveBy(0, 0, 0, -(pi-θ1)) # ターン角度ラスト
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    d000=s31*sin(γ1)-(distance2-x) # 残りの幅
                    d5=d000/sin(θ1) # 行きの移動幅ラスト
                    print('------------------------------distance of turn1last is '+str(d5)+' -----------------------------')
                    drone(
                        extended_move_by(d5, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    break

        # 下の頂点の方が近かった場合の処理
        else:
            print('下の頂点の方が上の頂点よりs1から近い')
            for i in range(100):
                print('------------------------------radians of turn1 is '+str(pi-γ2)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, -(pi-γ2)) # ターン角度1
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d1=x/sin(γ2) # 行きの移動幅
                print('------------------------------distance of turn1 is '+str(d1)+' -----------------------------')
                drone(
                    extended_move_by(d1, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                print('------------------------------radians of turn2 is '+str(γ2)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, -γ2) # ターン角度2
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d2=((s31*sin(γ2)-distance2)/tan(γ2))+((s31*sin(γ2)-distance2)/tan(θ2)) # 行きの移動距離
                distance2+=x
                print('------------------------------distance of main1 is '+str(d2)+' ------------------------------')
                for i in range(100):
                    drone(
                        extended_move_by(y, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    distance9+=y
                    if d2<=distance9:
                        d00=d2-(distance9-y)
                        drone(
                            extended_move_by(d00, 0, 0, 0, 1, 0, 0)
                            >> FlyingStateChanged(state="hovering", _timeout=5)
                        ).wait().success()
                        take_photo_single(drone)
                        break
                print('------------------------------radians of turn3 is '+str(β1)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, β1) # ターン角度3
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d3=x/sin(θ2) # 帰りの移動幅
                print('------------------------------distance of turn2 is '+str(d3)+' -----------------------------')
                drone(
                    extended_move_by(d3, 0, 0, 0, 1, 0, 0)
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                take_photo_single(drone)
                print('------------------------------radians of turn4 is '+str(θ2)+' -----------------------------')
                drone(
                    moveBy(0, 0, 0, θ2) # ターン角度4
                    >> FlyingStateChanged(state="hovering", _timeout=5)
                ).wait().success()
                d4=((s31*sin(γ2)-distance2)/tan(γ2))+((s31*sin(γ2)-distance2)/tan(θ2)) # 帰りの移動距離
                distance2+=x
                print('------------------------------distance of main2 is '+str(d4)+' ------------------------------')
                for i in range(100):
                    drone(
                        extended_move_by(y, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    distance99+=y
                    if d4<=distance99:
                        d0=d4-(distance99-y)
                        drone(
                            extended_move_by(d0, 0, 0, 0, 1, 0, 0)
                            >> FlyingStateChanged(state="hovering", _timeout=5)
                        ).wait().success()
                        take_photo_single(drone)
                        break
                distance9=y
                distance99=y    

                # 最後の頂点を越えそうになった場合の処理
                if distance1>s31*sin(γ2):
                    print('------------------------------radians of turn1last is '+str(β1)+' -----------------------------')
                    drone(
                        moveBy(0, 0, 0, β1) # ターン角度ラスト
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    d000=s31*sin(γ2)-(distance1-x) # 残りの幅
                    d5=d000/sin(θ2) # 行きの移動幅
                    print('------------------------------distance of turn1last is '+str(d5)+' -----------------------------')
                    drone(
                        extended_move_by(d5, 0, 0, 0, 1, 0, 0)
                        >> FlyingStateChanged(state="hovering", _timeout=5)
                    ).wait().success()
                    take_photo_single(drone)
                    break    
    
    print('----------------------------------------')
    print('roop2 finish')
    print('----------------------------------------')   
        

def landing(drone):
    drone(Landing()).wait().success()


def main():
    drone.connect()
    #drone(MaxAltitude(2.5)).wait()
    takeoff(drone)
    set_gimbal(drone)
    setup_photo_mode(drone)
    moveby0(drone)
    moveby_roop1(drone)
    moveby_roop2(drone)
    landing(drone) 
    drone.disconnect()

if __name__ == "__main__":
    main()
    
