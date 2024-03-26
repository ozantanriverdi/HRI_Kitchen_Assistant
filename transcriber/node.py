#!/usr/bin/env python3

import rospy
import json
import micRecorder as mic
from std_msgs.msg import String
import time


micRecorder = None


def initMicrophoneRecorder():
    global micRecorder
    micRecorder = mic.MicRecorder()
    while(micRecorder is None):
        time.sleep(0.2)
    while(micRecorder.transcriber is None):
        time.sleep(0.2)
    





def talker():
    global micRecorder
    rospy.init_node("transcriber_node", anonymous=True)
    pub = rospy.Publisher('transcribed_text', String, queue_size=10)
    rospy.Subscriber('feedback', String, callback)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        temp = json.dumps(micRecorder.transcriber.output)
        rospy.loginfo('transcribed_text SENT: ' + str(temp))
        pub.publish(temp)
        rate.sleep()
    
def callback(data):
    data = json.loads(data.data)
    if len(data.keys())> 0:
        rospy.loginfo("feedBack.shouldListen RECEIVED = " + str(data))
    if 'shouldListen' in data.keys():
        micRecorder.shouldListen = (data['shouldListen'] == "True")


    





if __name__ == '__main__':
    try:
        initMicrophoneRecorder()
        talker()
        
    except rospy.ROSInterruptException:
        pass
