# -*- coding: utf-8 -*-

from math import *
import os
import re
import csv
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
import time
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
"""
distance999= y
distance9999= y
"""


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


def moveby0():
    s1, α1= vincenty_inverse(lat0,lat1,lon0,lon1,ellipsoid=1)
    z0=-z+1
    #print('altitude is '+str(z))
    #print('altitude z0 is '+str(z0))
    print('distance s1 is '+str(s1))
    print('radians α1 is '+str(degrees(α1)))


def moveby_roop1():
    s1, α1= vincenty_inverse(lat0,lat1,lon0,lon1,ellipsoid=1)
    s2, α2= vincenty_inverse(lat1,lat2,lon1,lon2,ellipsoid=1)
    s4, α4= vincenty_inverse(lat0,lat3,lon0,lon3,ellipsoid=1)
    global distance1, distance0, distance00, distance000, distance0000
    β1=α2-α1
    print('β1 is '+str(degrees(β1)))
    β2=α4-α1
    print('β2 is '+str(degrees(β2)))
    θ1=β2
    print('θ1 is '+str(degrees(θ1)))
    θ2=pi-β1
    print('θ2 is '+str(degrees(θ2)))

    """
    # 縦にずれた距離カウント[m]
    distance0= y
    distance00= y
    distance000= y
    distance0000= y
    """
    
    for i in range(100):
        d1=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 行きの移動距離
        distance1+=x
        print('distance of main1 is '+str(d1))
        for i in range(100):
            distance0+=y
            print('distance0 is '+str(distance0))
            if d1<=distance0:
                d0=d1-(distance0-y)
                print('distance d0 is '+str(d0))
                break
        print('radians of turn1 is '+str(degrees(β1)))
        d2=x/sin(θ2) # 行きの移動幅
        print('distance of turn1 is '+str(d2))
        print('radians of turn2 is '+str(degrees(θ2)))
        d3=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 帰りの移動距離
        distance1+=x
        print('distance of main2 is '+str(d3))
        for i in range(100):
            distance00+=y
            if d3<=distance00:
                d00=d3-(distance00-y)
                print('distance d00 is '+str(d00))
                break
        print('radians of turn3 is '+str(degrees(pi-θ1)))
        d4=x/sin(θ1) # 帰りの移動幅
        print('distance of turn2 is '+str(d4))
        print('radians of turn4 is '+str(degrees(θ1)))

        # 移動した距離が上の頂点を越えそうになった場合の処理
        if distance1>s2*sin(θ2) and s4*sin(θ1)>s2*sin(θ2):
            print('distance1 is '+str(distance1))
            print('distance s2*sin(θ2) is '+str(s2*sin(θ2)))
            d5=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 行きの移動距離ラスト
            print('distance of mainlast is '+str(d5))
            for i in range(100):
                distance000+=y
                if d5<=distance000:
                    d000=d5-(distance000-y)
                    print('distance d000 is '+str(d000))
                break
            s0=s2*sin(θ2)-(distance1-x) # 残りの幅
            s=s0/sin(θ2) # 行きの移動幅ラスト
            print('distance s0 is '+str(s0))
            print('distance s is '+str(s))
            d6=s1-(s2*sin(θ2)/tan(θ2))-(s2*sin(θ2)/tan(θ1)) # 帰りの移動距離ラスト
            print('distance of main1 is '+str(d6))
            for i in range(100):
                distance0000+=y
                if d6<=distance0000:
                    d0000=d6-(distance0000-y)
                    print('distance d0000 is '+str(d0000))
                    break
            break

        # 移動した距離が下の頂点を越えそうになった場合の処理
        elif distance1>s4*sin(θ1) and s2*sin(θ2)>s4*sin(θ1):
            print('distance1 is '+str(distance1))
            print('distance s4*sin(θ1) is '+str(s4*sin(θ1)))
            d5=s1-((distance1-x)/tan(θ2))-((distance1-x)/tan(θ1)) # 行きの移動距離ラスト
            print('distance of mainlast is '+str(d5))
            for i in range(100):
                distance000+=y
                if d5<=distance000:
                    d000=d5-(distance000-y)
                    print('distance d000 is '+str(d000))
                break
            s0=s4*sin(θ1)-(distance1-x) # 残りの幅
            s=s0/sin(θ1) # 行きの移動距離ラスト
            print('distance s0 is '+str(s0))
            print('distance s is '+str(s))
            d6=s1-(s4*sin(θ1)/tan(θ2))-(s4*sin(θ1)/tan(θ1)) # 帰りの移動距離ラスト
            print('distance of main1 is '+str(d6))
            for i in range(100):
                distance0000+=y
                if d6<=distance0000:
                    d0000=d6-(distance0000-y)
                    print('distance d0000 is '+str(d0000))
                    break
            break
        distance0=y
        distance00=y
        distance000=y
        distance0000=y
    
    print('----------------------------------------')
    print('roop1 finish')
    print('----------------------------------------')


def moveby_roop2():
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

    """
    # 縦にずれた距離カウント[m]
    distance9= y
    distance99= y
    distance999= y
    distance9999= y
    """
    
    # 差異がほぼない（面積が小さい）場合、フライト終了
    if s31*sin(γ1)<x or s31*sin(γ2)<x:
        print('distance s31*sin(γ1) is '+str(s31*sin(γ1)))
        print('distance s31*sin(γ2) is '+str(s31*sin(γ2)))
        """
        print('s4 is '+str(s4))
        print('sin(θ1) is '+str(sin(θ1)))
        print('distance s2*sin(θ2) is '+str(s2*sin(θ2)))
        print('distance s4*sin(θ1) is '+str(s4*sin(θ1)))
        print('distance s2*sin(θ2)-s4*sin(θ1) is '+str(abs(s2*sin(θ2)-s4*sin(θ1))))
        print('α32 is '+str(degrees(α32)))
        print('γ1 is '+str(degrees(γ1)))
        print('distance2 is '+str(distance2))
        print('s31 is '+str(s31))
        print('s31*sin(γ2) is '+str(s31*sin(γ2)))
        print('tan(γ2) is '+str(tan(γ2)))
        print('tan(θ2) is '+str(tan(θ2)))
        """
        pass
    else:
        print('s4 is '+str(s4))
        print('sin(θ1) is '+str(sin(θ1)))
        print('distance s2*sin(θ2) is '+str(s2*sin(θ2)))
        print('distance s4*sin(θ1) is '+str(s4*sin(θ1)))
        print('distance s2*sin(θ2)-s4*sin(θ1) is '+str(abs(s2*sin(θ2)-s4*sin(θ1))))
        print('α32 is '+str(degrees(α32)))
        print('γ2 is '+str(degrees(γ2)))
        print('distance2 is '+str(distance2))
        print('s31 is '+str(s31))
        print('s31*sin(γ2) is '+str(s31*sin(γ2)))
        print('tan(γ2) is '+str(tan(γ2)))
        print('tan(θ2) is '+str(tan(θ2)))

        # 上の頂点の方が近かった場合の処理
        if s2*sin(θ2)<s4*sin(θ1):
            print('上')
            for i in range(100):
                d1=x/sin(θ1) # 行きの移動幅
                print('radians of turn1 is '+str(degrees(pi-θ1)))
                print('distance of turn1 is '+str(d1))
                print('radians of turn2 is '+str(degrees(θ1)))
                d2=((s31*sin(γ1)-distance2)/tan(γ1))+((s31*sin(γ1)-distance2)/tan(θ1)) # 行きの移動距離
                distance2+=x
                print('distance of main1 is '+str(d2))
                for i in range(100):
                    distance9+=y
                    if d2<=distance9:
                        d0=d2-(distance9-y)
                        print('distance d0 is '+str(d0))
                        break
                d3=x/sin(γ1) # 帰りの移動幅
                print('radians of turn3 is '+str(degrees(β2)))
                print('distance of turn2 is '+str(d3))
                print('radians of turn4 is '+str(degrees(γ1)))
                d4=((s31*sin(γ1)-distance2)/tan(γ1))+((s31*sin(γ1)-distance2)/tan(θ1)) # 帰りの移動距離
                distance2+=x
                print('distance of main2 is '+str(d4))
                for i in range(100):
                    distance99+=y
                    if d4<=distance99:
                        d00=d2-(distance99-y)
                        print('distance d00 is '+str(d00))
                        break
                distance9=y
                distance99=y

                # 最後の頂点を越えそうになった場合の処理
                if distance2>s31*sin(γ1):
                    print('distance2 is '+str(distance2))
                    print('distance s31*sin(γ1) is '+str(s31*sin(γ1)))
                    print('radians of turnlast is '+str(degrees(pi-θ1)))
                    d000=s31*sin(γ1)-(distance2-x) # 残りの幅
                    d5=d000/sin(θ1) # 行きの移動幅ラスト
                    print('distance d000 is '+str(d000))
                    print('distance d5 is '+str(d5))
                    break

        # 下の頂点の方が近かった場合の処理
        else:
            print('下')
            for i in range(100):
                d1=x/sin(γ2) # 行きの移動幅
                print('radians of turn1 is '+str(degrees(pi-γ2)))
                print('distance of turn1 is '+str(d1))
                print('radians of turn2 is '+str(degrees(γ2)))
                d2=((s31*sin(γ2)-distance2)/tan(γ2))+((s31*sin(γ2)-distance2)/tan(θ2)) # 行きの移動距離
                distance2+=x
                print('distance of main1 is '+str(d2))
                for i in range(100):
                    distance9+=y
                    if d2<=distance9:
                        d0=d2-(distance9-y)
                        print('distance d0 is '+str(d0))
                        break
                d3=x/sin(θ2) # 帰りの移動幅
                print('radians of turn3 is '+str(degrees(β1)))
                print('distance of turn2 is '+str(d3))
                print('radians of turn4 is '+str(degrees(θ2)))
                d4=((s31*sin(γ2)-distance2)/tan(γ2))+((s31*sin(γ2)-distance2)/tan(θ2)) # 帰りの移動距離
                distance2+=x
                print('distance of main2 is '+str(d4))
                for i in range(100):
                    distance99+=y
                    if d4<=distance99:
                        d00=d4-(distance99-y)
                        print('distance d00 is '+str(d00))
                        break
                distance9=y
                distance99=y
                
                # 最後の頂点を越えそうになった場合の処理
                if distance1>s31*sin(γ2):
                    print('distance1 is '+str(distance1))
                    print('distance s31*sin(γ2) is '+str(s31*sin(γ2)))
                    print('radians of turnlast is '+str(β1))
                    d000=s31*sin(γ2)-(distance1-x) # 残りの幅
                    d5=d000/sin(θ2) # 行きの移動幅
                    print('distance d000 is '+str(d000))
                    print('distance d5 is '+str(d5))
                    break
    
    print('----------------------------------------')
    print('roop2 finish')
    print('----------------------------------------')   


def main():
    moveby0()
    moveby_roop1()
    moveby_roop2()

if __name__ == "__main__":
    main()
    
