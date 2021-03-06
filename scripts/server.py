#! /usr/bin/env python
# -*- coding=utf-8 -*-
import socket
import time,os,fcntl
import struct
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Odometry

#message proto
# id | length | data
def recv_msg(sock):
    try:
        # Read message length and unpack it into an integer
        raw_id = recvall(sock, 4)
        if not raw_id:
            return None
        id = struct.unpack('>I', raw_id)[0]
        print "receive id: ",id
        raw_msglen = recvall(sock, 4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return recvall(sock, msglen)
    except Exception ,e:
        return None



def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = ''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

##把接受的数据重新打包成ros topic发出去
def msg_construct(data):

    list = data.split(',')

    follower_odom = Odometry()
    follower_odom.header.stamp = rospy.Time.now()
    follower_odom.header.frame_id = "/odom"
    #pose.child_frame_id = list[0]
    follower_odom.pose.pose.position.x = float(list[0])
    follower_odom.pose.pose.position.y = float(list[1])
    follower_odom.pose.pose.position.z = 0
    follower_odom.pose.pose.orientation.x = 0
    follower_odom.pose.pose.orientation.y = 0
    follower_odom.pose.pose.orientation.z = float(list[2])
    follower_odom.pose.pose.orientation.w = float(list[3])
    follower_odom.twist.twist.linear.x = float(list[4])
    follower_odom.twist.twist.linear.y = 0
    follower_odom.twist.twist.linear.z = 0
    follower_odom.twist.twist.angular.x = 0
    follower_odom.twist.twist.angular.y = 0
    follower_odom.twist.twist.angular.z = float(list[5])
    #pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

    return follower_odom


#初始化socket，监听8000端口
odom_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
odom_socket.bind(('192.168.1.113',8000))
odom_socket.listen(10)

(client,address) = odom_socket.accept()

rospy.init_node("client_node")
odom_pub = rospy.Publisher("/follower_odom",Odometry,queue_size=30)
r = rospy.Rate(1000)

#设置noblock，否则会阻塞在接听，下面while不会一直循环，只有在有数据才进行下一次循环
fcntl.fcntl(client, fcntl.F_SETFL, os.O_NONBLOCK)


while not rospy.is_shutdown():
    data = recv_msg(client)
    if data:
        odom_pub.publish(msg_construct(data))
    r.sleep()
