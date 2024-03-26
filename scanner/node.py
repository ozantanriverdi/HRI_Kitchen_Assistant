#!/usr/bin/env python3

import rospy
import json
import scan
from std_msgs.msg import String
import time
import threading 

scanner = None
lastScan = 0

def initScanner():
    global scanner
    scanner = scan.Scanner(0)
    while(scanner is None):
        time.sleep(0.2)
    
def talker():
    global scanner
    rospy.init_node("scanner_node", anonymous=True)
    pub = rospy.Publisher('spices', String, queue_size=10)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        temp = json.dumps(scanner.output)
        rospy.loginfo('spices SENT: ' + str(temp))
        pub.publish(temp)
        rate.sleep()
            



if __name__ == '__main__':
    try:
        initScanner()
        talker()
        
    except rospy.ROSInterruptException:
        pass
