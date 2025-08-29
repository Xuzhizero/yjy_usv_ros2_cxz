#!/usr/bin/env python3
# coding: utf-8
import math
import numpy as np
from control_planner.lonlat2coor import geodetic_to_polar


def transformLat(x, y):
    # 根据输入的 x, y 计算纬度的变换值
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320.0 * math.sin(y / 30.0 * math.pi)) * 2.0 / 3.0
    return ret


def transformLon(x, y):
    # 根据输入的 x, y 计算经度的变换值
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret


def delta(lon, lat):
    # 计算经纬度偏差
    a = 6378245
    ee = 0.006693421622965823
    dLon = transformLon(lon - 105.0, lat - 35.0)
    dLat = transformLat(lon - 105.0, lat - 35.0)
    radLat = lat / 180.0 * math.pi
    magic = math.sin(radLat)
    magic = 1 - ee * magic * magic
    sqrtMagic = math.sqrt(magic)
    d = [0, 0]
    d[0] = dLon * 180.0 / (a / sqrtMagic * math.cos(radLat) * math.pi)
    d[1] = dLat * 180.0 / (a * (1 - ee) / (magic * sqrtMagic) * math.pi)
    return np.array(d)


def WGS84ToGCJ02(WGS):
    # 将WGS84坐标转换为GCJ02坐标
    d = delta(WGS[0], WGS[1])
    GCJ = WGS + d
    return GCJ


def GCJ02ToWGS84(GCJ):
    # 输入为纬度、经度的形式，首先调整为经度、纬度的形式
    GCJ = (GCJ[1], GCJ[0])
    # 初始化 WGS 坐标
    WGS = GCJ
    # 计算 GCJ02 坐标转换为 WGS84 后的值
    temp = WGS84ToGCJ02(WGS)
    # 计算差值
    dx = temp - GCJ
    # 当差值足够小时，停止循环
    while np.min(np.abs(dx)) > 1e-6:
        WGS = WGS - dx
        temp = WGS84ToGCJ02(WGS)
        dx = temp - GCJ
    # 将结果调整为纬度、经度的形式后返回
    return (WGS[1], WGS[0])


# Example usage:
# Convert GCJ02 coordinates to WGS84 coordinates
GCJ = (30.298487,120.088317)  # input as (latitude, longitude)
WGS = GCJ02ToWGS84(GCJ)
print(WGS)  # output as (latitude, longitude)
p_fromGPS = (30.300795382999997,120.08347503000002)#lat, lon
az, distance = geodetic_to_polar(p_fromGPS[0],p_fromGPS[1],WGS[0],WGS[1])
print("distance",distance)
