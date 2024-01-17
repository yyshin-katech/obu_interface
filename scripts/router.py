#!/usr/bin/env python

import os
import sys
import time
import json
import struct
import threading
import datetime
import math
from pyproj import Proj, transform

#from socket import *
import socket
import rospy
import roslibpy
from std_msgs.msg import String
from sensor_msgs.msg import TimeReference
from sensor_msgs.msg import NavSatFix
from obu_interface.msg import chassis_msg
from obu_interface.msg import localization2D_msg_bus

# file_path = "/home/katech/sejong/src/obu_interface/scripts/json/pvd.json"
file_path = "/mnt/hgfs/vm_shared/sejong/src/obu_interface/scripts/json/pvd.json"

with open(file_path, 'rw') as file:
    global data
    data = json.load(file)
    # print(data)

global pvd_string
pvd_string = json.dumps(data).encode('UTF-8')

HOST = '192.168.137.200'
# HOST = socket.gethostname()
PORT = 9100

# TEST UDP Local host
# HOST = '192.168.1.55'
# PORT = 9100

utm_zone = 52

Header = []
packet_seq = {0, 0, 0, 0}
timestamp = {0, 0, 0, 0, 0, 0, 0, 0}
packet_type = 0
body_length = {0, 0, 0, 0}

Body = []
data_length = {0, 0, 0, 0}

PVD_packet = []
# timeref = TimeReference()
# fix = NavSatFix()

chassis = chassis_msg()
local = localization2D_msg_bus()

# def timeref_callback(msg):
#   timeref.time_ref = msg.time_ref

# def fix_callback(msg):
#    fix.latitude = msg.latitude
#    fix.longitude = msg.longitude
#    fix.altitude = msg.altitude

def chassis_callback(msg):
   chassis.speed = msg.speed
   chassis.ax = msg.ax
   chassis.ay = msg.ay
   chassis.right_turn_signal = msg.right_turn_signal
   chassis.left_turn_signal = msg.left_turn_signal
   chassis.hazard_signal = msg.hazard_signal

def localization_callback(msg):
   local.east = msg.east
   local.north = msg.north
   local.yaw = msg.yaw
   local.vel = msg.vel

class PVD_encoding_Thread(threading.Thread):
    def __init__(self, client_socket):
        super(PVD_encoding_Thread, self).__init__()
        rospy.init_node('sejong_obu_node', anonymous=True)

        # rospy.Subscriber("time_reference", TimeReference, timeref_callback)
        # rospy.Subscriber("fix", NavSatFix, fix_callback)

        rospy.Subscriber("/sensors/chassis", chassis_msg, chassis_callback)
        rospy.Subscriber("/udp/localization", localization2D_msg_bus, localization_callback)
        
        self.client = client_socket
        self.pvd_packet_seq = 0
        self.timestamepq = 0
        self.body_length = 0
        self.month = 0
        self.day = 0
        self.hour = 0
        self.minute = 0
        self.second = 0
#        self.utm_to_wgs84 = Proj(init='epsg:32651')
  
    def utm_to_wgs84(self, utm_easting, utm_northing, utm_zone):
      utm_proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84')
      wgs84_proj = Proj(proj='latlong', datum='WGS84')
      wgs84_longitude, wgs84_latitude = transform(utm_proj, wgs84_proj, utm_easting, utm_northing)
      return wgs84_longitude, wgs84_latitude

    def Encoding_Header(self, Header):
        self.Header = Header

        self.packet_seq = struct.pack('>i', self.pvd_packet_seq)
        self.pvd_packet_seq += 1

        self.timestamp = struct.pack('>q', self.timestamepq)
        self.timestamepq = rospy.Time.now().to_sec() - 32400
        self.time_ref_datetime = datetime.datetime.fromtimestamp(self.timestamepq)
        self.month = self.time_ref_datetime.month
        self.day = self.time_ref_datetime.day
        self.hour = self.time_ref_datetime.hour
        self.minute = self.time_ref_datetime.minute
        self.second = self.time_ref_datetime.second
        
        data['thePosition']['utcTime']['year'] = 2024
        data['thePosition']['utcTime']['month'] = self.month
        data['thePosition']['utcTime']['day'] = self.day
        data['thePosition']['utcTime']['hour'] = self.hour
        data['thePosition']['utcTime']['minute'] = self.minute
        data['thePosition']['utcTime']['second'] = self.second

        # print(self.month)
        # print(self.day)
        # print(self.hour)
        # print(self.minute)
        # print(self.second)

        # self.latitude = int(fix.latitude * 10000000)
        # self.longitude = int(fix.longitude * 10000000)
        # data['thePosition']['lat'] = self.latitude
        # data['thePosition']['long'] = self.longitude
        # data['thePosition']['elevation'] = int(fix.altitude * 10)
              
        # self.wgs84_longitude, self.wgs84_latitude = self.utm_to_wgs84(local.east, local.north)
        self.wgs84_longitude, self.wgs84_latitude = self.utm_to_wgs84(local.east, local.north, utm_zone)

        # print(local.east)
        # print(local.north)

        # print(self.wgs84_longitude * 10000000)
        # print(self.wgs84_latitude * 10000000)
        data['thePosition']['lat'] = int(self.wgs84_latitude * 10000000)
        data['thePosition']['long'] = int(self.wgs84_longitude * 10000000)
        self.rad_temp = (-local.yaw + math.pi / 2) % (2 * math.pi) 

        self.heading_temp = (math.degrees(self.rad_temp))
        # if self.heading_temp > 360.0:
        #    self.heading_temp = self.heading_temp - 180.0

        data['thePosition']['heading'] = int(self.heading_temp * 80)
        data['thePosition']['elevation'] = 23
        data['thePosition']['speed']['speed'] = int(chassis.speed * 50)

        # print(data['thePosition']['lat'])
        # print(data['thePosition']['long'])
        # print(data['thePosition']['heading'])
        # print(self.rad_temp)
        print(self.heading_temp)
        print("  ")

        data['dataSet']['lights']['rightTurnSignalOn'] = int(chassis.right_turn_signal)
        data['dataSet']['lights']['leftTurnSignalOn'] = int(chassis.left_turn_signal)
        data['dataSet']['lights']['hazardSignalOn'] = int(chassis.hazard_signal)
        
        pvd_string = json.dumps(data).encode('UTF-8')
        self.packet_type = struct.pack('B',21)  # PVD

        self.body_length = struct.pack('>i',len(pvd_string)+4) #self.Cal_Body_Length(self.pvd_string)

        # self.Header.append(self.packet_seq)
        # self.Header.append(self.timestamp)
        # self.Header.append(self.packet_type)
        # self.Header.append(self.body_length)
        self.Header = self.packet_seq + self.timestamp + self.packet_type + self.body_length

        data_length = struct.pack('>i', len(pvd_string))
        
        self.Body = data_length + pvd_string

        # print(self.Header)
        # print(self.Body)

        self.PVD_packet = self.Header + self.Body
        # print(self.PVD_packet)

    def run(self):
        #      self.open_pvd_json()
        while not rospy.is_shutdown():
         
          self.Encoding_Header(Header)

        # print(self.PVD_packet)
        # self.pvd_string = json.dumps(data).encode('UTF-8')#, indent=2)
        # body = bytes(pvd_string)
        # print(pvd_string)
          self.client.sendall(self.PVD_packet)
        # self.client.sendto(self.PVD_packet, (HOST, PORT))
          time.sleep(1)

def main():
    
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.bind(('', PORT))
    client_socket.connect((HOST, PORT))
    # client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # client_socket.bind((HOST, PORT))

    thread_pvd = PVD_encoding_Thread(client_socket)

    thread_pvd.start()

    thread_pvd.join()

    client_socket.close()



if __name__ == '__main__':
    try:  
      main()

    except rospy.ROSInterruptException:
      pass

#  pvd_string = json.dumps(data).encode('UTF-8')#, indent=2)
#  body = bytes(pvd_string)


#  thread_pvd = PVD_encoding_Thread(RelaySW_Client)


#  except Exception as e:
#    print(e)
