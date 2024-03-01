#!/usr/bin/python3
import numpy as np
import serial
import multiprocessing as mp
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Uart():
    def __init__(self,mp):
        self.LIN_VEL =  mp.Value('i',0)
        self.ANG_VEL =  mp.Value('i',0)
        self.port = "/dev/ttyUSB0"
        self.baudrate = 115200
        self.data_preview = ""
        self.ser = self.check_port_open()
        self.pub_timer = 0.05

        self.MAX_LIN_VEL = mp.Value('d',0.5)
        self.MAX_ANG_VEL = mp.Value('d',0.40)
        

    def check_port_open(self):
        try:
            ser = serial.Serial(self.port,self.baudrate, timeout=1000 ,stopbits=1)
            print(self.port + ': port is Open.')
            return ser
        except:
            print(self.port + ': open port Fail.')
        return 0

def Receive_uart(Obj_uart): 
    while(1):
        try:
            s = Obj_uart.ser.readline()
            dataMessage = s.split()
            if ((str(dataMessage[0]))[2] == 'x') and ((str(dataMessage[3]))[-2]) == 'x':
                Obj_uart.LIN_VEL.value = int(dataMessage[1])
                Obj_uart.ANG_VEL.value = int(dataMessage[2])    
  
        except:
            Obj_uart.ser = Obj_uart.check_port_open()

class PubTwist(Node):
    def __init__(self,Obj_uart:Uart):
        super().__init__('rf_teleop_pub')
        self.Obj_uart = Obj_uart
        self.pub_Twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(self.Obj_uart.pub_timer, self.timer_callback)


    def timer_callback(self):
        LIN_VEL_data = float(self.Obj_uart.LIN_VEL.value)
        ANG_VEL_data = float(self.Obj_uart.ANG_VEL.value)
        LIN_VEL = (LIN_VEL_data*self.Obj_uart.MAX_LIN_VEL.value) /500
        ANG_VEL = (ANG_VEL_data*self.Obj_uart.MAX_ANG_VEL.value) /500

        self.get_logger().info("LIN_VEL = %.3f ANG_VEL = %.3f " % (LIN_VEL, ANG_VEL))
        twist = Twist()
        twist.linear.x = round(LIN_VEL,3)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = round(ANG_VEL,3)
        self.pub_Twist.publish(twist)

def pub(Obj_uart):
    rclpy.init()
    pub_Twist = PubTwist(Obj_uart)
    rclpy.spin(pub_Twist)
    pub_Twist.destroy_node()
    rclpy.shutdown()

def main():
    try:
        Obj_uart = Uart(mp)   

        if(Obj_uart.ser!=0):
            p_read  = mp.Process(target=Receive_uart,args=(Obj_uart,))
            p_pub = mp.Process(target=pub,args=(Obj_uart,))

            p_read.start()
            p_pub.start()
            while(1):
                1
        
    except KeyboardInterrupt:
        if(Obj_uart.ser!=0):
            p_read.kill()
            p_pub.kill()

    finally:
        print("\n\nAll process is shutdown.")

if __name__ == '__main__':
    main()