#! /usr/bin/env python3
from geographiclib.geodesic import Geodesic
import math
import rclpy
import time
from sensor_msgs.msg import NavSatFix


def geodetic_to_polar(lat1,lon1,lat2,lon2):
    geodict = Geodesic.WGS84.Inverse(lat1,lon1,lat2,lon2)
    # 距离 float格式, meter 
    distance = geodict['s12']
    
    # 点1基准方位角
    # 方位角是从某点的指北方向线起，依顺时针方向到目标方向线之间的水平夹角
    az = geodict['azi1'] #degree            
    return (az,distance)


def polar_to_cartesian(theta, r): #在此场景下，角度是从指北方向线起，顺时针旋转到目标方向的，而我们通常的极坐标系是从指东方向线起，逆时针旋转
    # 将角度转换为弧度
    theta = math.radians(theta) 
    # 旋转坐标系以适应从北方开始，顺时针的角度
    theta = math.pi/2 - theta
    # 计算直角坐标
    e = r * math.cos(theta)
    n = r * math.sin(theta)

    return e, n

def latlon2en(lat1,lon1,lat2,lon2):
    # integration of geodetic_to_polar and polar_to_cartesian
    az,distance = geodetic_to_polar(lat1,lon1,lat2,lon2)
    e, n = polar_to_cartesian(az,distance)
    return e,n

lat, lon = 0,0
lat0, lon0 = 0,0


def gps_callback(data):
    global lat,lon
    if data.longitude>120 and data.latitude>30:
        lon = data.longitude
        lat = data.latitude

if __name__ == "__main__":
    # geodetic_to_polar测试用例
    # 注意其参数顺序有些奇怪，分别是 点1纬度，点1经度，点2纬度，点2经度
    # 北纬和东经为正数，南纬和西经为负数
    #30.269544,120.124521 beimen
    #30.296921,120.092121 dongsan
    geodict = Geodesic.WGS84.Inverse(30.301635056, 120.08237971, 30.302254712, 120.081761628)
    
    # 距离 float格式
    distance = geodict['s12']

    # 点1基准方位角
    # 方位角是从某点的指北方向线起，依顺时针方向到目标方向线之间的水平夹角
    az = geodict['azi1']             
    print(az,distance)
    # 实测比较准确

    #######################
    # polar_to_cartesian测试用例
    r = distance
    theta = az  # 45度，顺时针旋转

    x, y = polar_to_cartesian(theta, r)

    print(f"Cartesian coordinates: ({x}, {y})")
    rospy.init_node("lonlat2coor")

    while (lat0<30 or lon0<120) and not rospy.is_shutdown():
        rospy.Subscriber("fix",NavSatFix,gps_callback)
        lat0, lon0 = lat, lon
    print("\n lat0:%f, lon0:%f"%(lat0,lon0)) 
    loop_counter = 0
    while not rospy.is_shutdown():
        rospy.Subscriber("fix",NavSatFix,gps_callback)
        az, distance = geodetic_to_polar(30.301638477, 120.082161758, lat,lon)          
        

        r = distance
        theta = az  # 45度，顺时针旋转

        x, y = polar_to_cartesian(theta, r)

        if loop_counter > 100:
            print("lat:%f, lon:%f"%(lat,lon))
            print("az:%f,distanc:%f"%(az,distance))
            print(f"Cartesian coordinates: ({x}, {y})")
            loop_counter = 0
        time.sleep(0.01)
        loop_counter += 1
    