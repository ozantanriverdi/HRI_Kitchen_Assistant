#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
import threading
import time
import inputClassifier as ic


lastTextIndex = -1
inputClassifier = None

def initClassifier():
    global inputClassifier
    inputClassifier = ic.Classifier()
    while(inputClassifier is None):
        time.sleep(0.2)
    while(inputClassifier.textHandler is None):
        time.sleep(0.2)
    while(inputClassifier.ttsModule is None):
        time.sleep(0.2)


def talker():
    global inputClassifier

    rospy.init_node("input_handler_node", anonymous=True)

    pubcom = rospy.Publisher('command', String, queue_size=10)

    rospy.Subscriber('transcribed_text', String, callback)
    rospy.Subscriber('spices', String, callback2)
    rospy.Subscriber('stuck', String, callback3)

    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        
        temp = json.dumps(inputClassifier.output)
        pubcom.publish(temp)
        rospy.loginfo('\ncommand SENT: ' + str(temp))

        rate.sleep()


def callback(data):
    global lastTextIndex, inputClassifier
    data = json.loads(data.data)
    rospy.loginfo("\nfeedBack.transcribed_text RECEIVED = " + str(data))
    if 'text' in data.keys():
        if lastTextIndex != data['index']:
            lastTextIndex = data['index']
            inputClassifier.classify(data['text'])

def callback2(data):
    global inputClassifier
    data = json.loads(data.data)
    rospy.loginfo("\nfeedBack.spices RECEIVED = " + str(data))
    if 'spices' in data.keys():
        inputClassifier.updateSpices(data)

def callback3(data):
    global inputClassifier
    data = json.loads(data.data)
    rospy.loginfo("\nfeedBack.stuck RECEIVED = " + str(data))
    if 'stuck' in data.keys() and data['stuck'] == 'True':
        inputClassifier.handleStuck()


if __name__ == '__main__':
    try:
        initClassifier()
        talker()

    except rospy.ROSInterruptException:
        pass