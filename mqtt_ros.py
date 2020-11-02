#!/usr/bin/env python2.7
import rospy
import threading
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt

client = mqtt.Client()
heartbeatCNT = 0

def callback(msg):
  apiID = "5e19e3b29d0ce61f6f234129"
  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y
  z = msg.pose.pose.position.z
  client.publish("topic/odomRawPOS", "{}||{}|{}|{}".format(apiID,x,y,z), 1)

def hbCheck():
  heartbeatCNT += 1
  if heartbeatCNT > 5:
    print("Missing ping from Supervisor")
  threading.Timer(5.0,hbCheck).start()

def hbUpdate(client, userdata, msg):
  if msg.payload.decode() == "ping":
    heartbeatCNT = 0

#mqtt conection status
def mqtt_on_connect(client, userdata, flags, rc):
  client.message_callback_add("topic/supervisorHB", hbUpdate)
  client.subscribe("topic/supervisorHB")

def main():
  rospy.init_node('mqtt_listener', anonymous=True)
  rospy.Subscriber('odom', Odometry, callback)
  client.connect("localhost",1883,60)
  client.on_connect = mqtt_on_connect
  client.loop_start()
  rospy.spin()

if __name__ == '__main__':
  hbCheck()
  main()
