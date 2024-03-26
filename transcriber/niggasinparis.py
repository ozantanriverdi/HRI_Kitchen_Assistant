#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
import time
from pynput import keyboard
import micRecorder as mic
import threading
sex = []
pornonuzucekiyorum = {'index' : 0, 'text': 'Assistant fuck my little asshole'}
sex.append({'index' : 4, 'text': 'Assistant nigger, how many people in paris? Add 500 grams of salt.'})
sex.append({'index' : 5, 'text': 'Assistant nigger, how many people in paris? Add 500 grams of salt.'})
sex.append({'index' : 1, 'text': 'Assistant why do niggers like salt?'})
sex.append({'index' : 2, 'text': 'Assistant , fucking kfc niggers ate all the salt and sugar'})
sex.append({'index' : 3, 'text': 'Assistant , a nigger with large 50 cm dick mocked my small penis while eating salt mommys tits'})


micRecorder = None
a=0

def initMicrophoneRecorder():
    global micRecorder
    micRecorder = mic.MicRecorder()
    while(micRecorder is None):
        time.sleep(0.2)
    while(micRecorder.transcriber is None):
        time.sleep(0.2)
    





def talker():
    global pornonuzucekiyorum
    rospy.init_node("transcriber_node", anonymous=True)
    pub = rospy.Publisher('transcribed_text', String, queue_size=10)
    rospy.Subscriber('feedback', String, callback)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        temp = json.dumps(pornonuzucekiyorum)
        rospy.loginfo('transcribed_text SENT: ' + str(temp))
        pub.publish(temp)
        rate.sleep()
    
    
def callback(data):
    data = json.loads(data.data)
    if len(data.keys())> 0:
        rospy.loginfo("feedBack.shouldListen RECEIVED = " + str(data))
    if 'shouldListen' in data.keys():
        print((data['shouldListen'] == "True"))


    

def on_press(key):
    global a, pornonuzucekiyorum
    try:
        if key.char == 'k':
            print("K key pressed!")
            pornonuzucekiyorum = sex[a]
            a+=1
            # Replace this with your onclick action
    except AttributeError:
        pass

def checKey():
    print('started')
    with keyboard.Listener(on_press=on_press) as listener:
            listener.join()
# Collect events until released


if __name__ == '__main__':
    try:
        threading.Thread(target=checKey).start()
        talker()
        
        
    except rospy.ROSInterruptException:
        pass
