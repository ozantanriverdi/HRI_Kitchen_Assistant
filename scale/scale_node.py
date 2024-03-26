#!/usr/bin/env python3
import rospy
import serial
import os
from std_msgs.msg import String
import sys
import threading

#print(sys.path)
#sys.path.append('/home/ozan/catkin_ws/src/assistant_scale')
#from assistant_scale.config import config

#CONFIGPATH = os.path.join(sys.path[0], '..', 'config.json')
value = '0'
scale_value = 0
def talker():
    rospy.init_node("scale_node", anonymous=True)
    pub = rospy.Publisher('scale', String, queue_size=10)
    
    
    rate = rospy.Rate(20)
    ser = serial.Serial('/dev/ttyACM0')
    
    
    while not rospy.is_shutdown():
        try:
            global value, scale_value
            try:
                scale_value = ser.readline().strip().decode('utf-8')
            except UnicodeDecodeError:
                pass
            #Displaying the scale value
            '''display_value = float(scale_value) - 2.3
            rounded_value = round(display_value, 2)
            print(rounded_value)
            
            pub.publish(str(rounded_value))'''
            
            threading.Thread(target=handleInput, args=(scale_value,)).start()
            print(value)
            pub.publish(value)
            rate.sleep()
        except KeyboardInterrupt:
            ser.close()
            
def handleInput(input):
    global value
    try:
        display_value = float(input) - 9.0
        rounded_value = round(display_value, 2) 
        value = str(rounded_value) 
    except ValueError:
        pass
       
        
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass