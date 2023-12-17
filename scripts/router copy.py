#!/usr/bin/env python

import os
import sys
import time
import json
import struct
import threading

#from socket import *
import socket
import rospy
import roslibpy
from std_msgs.msg import String

file_path = "/home/katech/sejong/src/obu_interface/scripts/json/pvd.json"

with open(file_path, 'rw') as file:
  global data
  data = json.load(file)

global pvd_string
pvd_string = json.dumps(data).encode('UTF-8')

HOST = "192.168.137.200"
PORT = 9100

Header = []
packet_seq = {0,0,0,0}
timestamp = {0,0,0,0,0,0,0,0}
packet_type = 0
body_length = {0,0,0,0}

Body = []
data_length = {0,0,0,0}
      
class PVD_encoding_Thread(threading.Thread):
  def __init__(self, client_socket):
      super(PVD_encoding_Thread, self).__init__()
      self.client = client_socket
      self.pvd_packet_seq = 0
      self.timestamep = 0
      self.body_length = 0
      
#  def open_pvd_json():
#    with open(file_path, 'rw') as file:
#      global data
#      data = json.load(file)

  def Cal_Body_Length(self, pvd_body):
      body_length = len(pvd_body)
      return body_length
  
  #def Encoding_body(self):
     #self.data_length = len(pvd_body)

  def Encoding_Header(self, Header, pvd_header):
     self.Header = Header
     self.pvd_header = pvd_header

     self.packet_seq = struct.pack('>i', self.pvd_packet_seq)
     self.pvd_packet_seq += 1

     self.timestamp = struct.pack('>q', self.timestamp)
     self.packet_type = 21  #PVD
     self.body_length = self.Cal_Body_Length(self.pvd_body)

  def run(self):
#      self.open_pvd_json()

      #self.pvd_string = json.dumps(data).encode('UTF-8')#, indent=2)
      #body = bytes(pvd_string)
      print(pvd_string)
      #self.client.sendall(pvd_string)

def main():
  client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  client_socket.connect((HOST, PORT))

  thread_pvd = PVD_encoding_Thread(client_socket)
  thread_pvd.start()

  thread_pvd.join()

  client_socket.close()


if __name__ == '__main__':
  main()

#  pvd_string = json.dumps(data).encode('UTF-8')#, indent=2)
#  body = bytes(pvd_string)
  

#  thread_pvd = PVD_encoding_Thread(RelaySW_Client)
  

#  except Exception as e:
#    print(e)

