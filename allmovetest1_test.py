# -*- coding: utf-8 -*-
from math import *
import math



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

# スタート地点以外の緯度経度
start_lat= 35.709759
start_lon= 139.523113

lat1= 35.709919
lon1= 139.523137

lat2= 35.709924
lon2= 139.523309

lat3= 35.709766
lon3= 139.523317

# ドローンの撮影間隔[m] (distance0)
y= 1

# 高度
z= 5


"""
def get_now_gps(drone):
    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy='wait'))
    return drone.get_state(HomeChanged)
"""


def vincenty_inverse(lat,lat0,lon,lon0,ellipsoid=None):
    # 差異が無ければ0を返す
    if abs(lat-lat0)<0.000001 and abs(lon-lon0)<0.000001:
        s, α1, α2=0, 0, 0
        return (s, α1, α2)
    else:
        # 計算時に必要な長軸半径(a)と扁平率(ƒ)を定数から取得し、短軸半径(b)を算出する
        # 楕円体が未指定の場合はGRS80の値を用いる
        a, ƒ = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
        b = (1 - ƒ) * a

        φ1 = radians(lat)
        φ2 = radians(lat0)
        λ1 = radians(lon)
        λ2 = radians(lon0)

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


def dpi1_calculate():
    s, α1, α2 = vincenty_inverse(start_lat,lat1,start_lon,lon1,ellipsoid=1)
    azimuth_rad1 = α1+(pi/2)
    return azimuth_rad1


def moveby_dpi1():
    #gps=get_now_gps(drone)
    #start_lat,start_lon,drone_alt=gps['latitude'],gps['longitude'],gps['altitude']
    s, α1, α2 = vincenty_inverse(start_lat,lat1,start_lon,lon1,ellipsoid=1)
    z0=-z+1
    print('z is '+str(z))
    print('α1 is '+str(degrees(α1)))
    azimuth_rad1 = α1+(pi/2)
    print('azimuth_rad1 is '+str(degrees(azimuth_rad1)))


def moveby_dy1():
    #gps=get_now_gps(drone)
    #start_lat,start_lon,drone_alt=gps['latitude'],gps['longitude'],gps['altitude']
    s, α1, α2 = vincenty_inverse(start_lat,lat1,start_lon,lon1,ellipsoid=1)
    distance=y
    print('s1 is '+str(s))
    for x in range(100):
        distance+=y
        print('distance is '+str(distance))
        if s<=distance:
            s0=s-(distance-y)
            print('s0 is '+str(s0))
            break
        

def moveby_dpi2():
    azimuth_rad1 = dpi1_calculate()
    s, α1, α2 = vincenty_inverse(lat1,lat2,lon1,lon2,ellipsoid=1)
    print('α2 is '+str(degrees(α1)))
    azimuth_rad = α1+(pi/2)
    print('azimuth_rad is '+str(degrees(azimuth_rad)))
    # 1回目の角度の方が大きかった場合の処理
    if azimuth_rad<azimuth_rad1:
        print('a')
        azimuth_rad2=azimuth_rad+(2*pi-azimuth_rad1)
    else:
        print('b')
        azimuth_rad2 = azimuth_rad-azimuth_rad1
    print('azimuth_rad2 is '+str(degrees(azimuth_rad2)))


def dpi2_calculate():
    s, α1, α2 = vincenty_inverse(lat1,lat2,lon1,lon2,ellipsoid=1)
    azimuth_rad = α1+(pi/2)
    return azimuth_rad



def moveby_dy2():
    s, α1, α2 = vincenty_inverse(lat1,lat2,lon1,lon2,ellipsoid=1)
    distance=y
    print('s2 is '+str(s))
    for x in range(100):
        distance+=y
        print('distance is '+str(distance))
        if s<=distance:
            s0=s-(distance-y)
            print('s0 is '+str(s0))
            break


def moveby_dpi3():
    azimuth_rad2 = dpi2_calculate()
    s, α1, α2 = vincenty_inverse(lat2,lat3,lon2,lon3,ellipsoid=1)
    print('α3 is '+str(degrees(α1)))
    azimuth_rad = α1+(pi/2)
    print('azimuth_rad is '+str(degrees(azimuth_rad)))
    # 1回目の角度の方が大きかった場合の処理
    if azimuth_rad<azimuth_rad2:
        print('a')
        azimuth_rad3=azimuth_rad+(2*pi-azimuth_rad2)
    else:
        print('b')
        azimuth_rad3 = azimuth_rad-azimuth_rad2
    print('azimuth_rad3 is '+str(degrees(azimuth_rad3)))


def dpi3_calculate():
    s, α1, α2 = vincenty_inverse(lat2,lat3,lon2,lon3,ellipsoid=1)
    azimuth_rad = α1+(pi/2)
    return azimuth_rad


def moveby_dy3():
    s, α1, α2 = vincenty_inverse(lat2,lat3,lon2,lon3,ellipsoid=1)
    print('s3 is '+str(s))
    distance=y
    for x in range(100):
        distance+=y
        print('distance is '+str(distance))
        if s<=distance:
            s0=s-(distance-y)
            print('s0 is '+str(s0))
            break


def moveby_dpi4():
    azimuth_rad3 = dpi3_calculate()
    s, α1, α2 = vincenty_inverse(lat3,start_lat,lon3,start_lon,ellipsoid=1)
    print('α4 is '+str(degrees(α1)))
    azimuth_rad = α1+(pi/2)
    print('azimuth_rad is '+str(degrees(azimuth_rad)))
    # 1回目の角度の方が大きかった場合の処理
    if azimuth_rad<azimuth_rad3:
        print('a')
        azimuth_rad4=azimuth_rad+(2*pi-azimuth_rad3)
    else:
        print('b')
        azimuth_rad4 = azimuth_rad-azimuth_rad3
    print('azimuth_rad4 is '+str(degrees(azimuth_rad4)))


def moveby_dy4():
    s, α1, α2 = vincenty_inverse(lat3,start_lat,lon3,start_lon,ellipsoid=1)
    distance=y
    print('s4 is '+str(s))
    for x in range(100):
        distance+=y
        print('distance is '+str(distance))
        if s<=distance:
            s0=s-(distance-y)
            print('s0 is '+str(s0))
            break



def main():
    moveby_dpi1()
    moveby_dy1()
    moveby_dpi2()
    moveby_dy2()
    moveby_dpi3()
    moveby_dy3()
    moveby_dpi4()
    moveby_dy4()

if __name__ == "__main__":
    main()
    
