import cv2
import math
import numpy as np
import pyapriltags
import json
import os 
import sys
import time
import threading
from pathlib import Path
TAGSPATH = os.path.join(sys.path[0], '..', '..', 'Data', 'JSONData', 'tags.json')
REFERENCE_ROBOT_DIFF = [0.1, 0, 0,1]
SPICE_TAG_DIFF = [0, 0, 0]
CAMADJUSTER = 1
TAGFAMILY = 'tag16h5'

class Scanner():
    def __init__(self, camIndex = -1):
        self.output = {"spices": [], "coords": []}
        self.detector = self.initDetector()
        self.cam = self.initCam(camIndex)
        self.validTagsList, self.referenceId, self.spiceNameFromTagId = self.initTagList()
        threading.Thread(target= self.startScanning).start()








    def initDetector(self):
            return pyapriltags.Detector(families=TAGFAMILY,
                                        nthreads=1,
                                        quad_decimate=1.0,
                                        quad_sigma=0.0,
                                        refine_edges=1,
                                        decode_sharpening=0.25,
                                        debug=0)
    def initCam(self, camIndex):
        return cv2.VideoCapture(camIndex) 
        
    def get_current_frame(self):
        if not self.cam.isOpened():
            #print("Camera not on")
            return False, None

        ret, frame = self.cam.read()
        return ret, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    
    def initTagList(self):
        taglist = []
        referenceId = None
        nameFromTag = {}
        with open(TAGSPATH) as file:
            data =json.load(file)
        for a in data.keys():
            taglist.append(int(a))
            if data[a].lower() == 'reference':
                referenceId = int(a)
            else:
                nameFromTag[int(a)] = data[a]

        return taglist, referenceId, nameFromTag
    
    def handleFrame(self, frame):
        scannedTags = []
        tags = self.detector.detect(frame, True, [640, 640, 320, 240], 0.03)
        # print('valid: ', self.validTagsList)
        # print('tags:')
        for tag in tags:
            if tag.tag_id in self.validTagsList and tag.decision_margin > 20: 
                self.handleTag(tag, frame, scannedTags)
        cv2.imshow('Frame', frame)
        self.handlePositions(scannedTags)
        if(len(scannedTags)==0):
            self.output = {"spices": [], "coords": []}

    
    def handleTag(self, tag, frame, scannedTags):
        cv2.polylines(frame, [tag.corners.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)
        distance = math.sqrt(sum(x**2 for x in tag.pose_t))
        cv2.putText(frame, f"ID: {tag.tag_id}", org=(int(tag.center[0]), int(tag.center[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 0, 0), thickness=2)
        scannedTags.append([tag.tag_id, tag.pose_t])
        # print(f"Tag {tag.tag_id} found at/n x: {tag.pose_t[0]}/n y: {tag.pose_t[1]}/n z: {tag.pose_t[2]}")


    def handlePositions(self, scannedTags):
        spices = []
        coords = []
        referenceTag = None
        for a in scannedTags:
            if a[0] == self.referenceId:
                referenceTag = a
        if referenceTag is None:
            return
        for a in scannedTags:
            if a[0] in self.validTagsList and a[0] != self.referenceId:
                spices.append(self.spiceNameFromTagId[a[0]])
                b=calculatePosition(referenceTag[1], a[1])
                c=[]
                c.append(b[0])
                c.append(b[2])
                c.append(b[1])
                coords.append(c)
                
        self.output ['spices']= spices 
        self.output ['coords']= coords 
        # print(self.output) 
    

    def startScanning(self):
        while True:
            time.sleep(0.2)
            (a, b) = self.get_current_frame()
            if(a):
                self.handleFrame(b) 
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cam.release()
        cv2.destroyAllWindows() 
        exit(0)

    
        

    
    
    
    
    # def test(self,frame):
    #     self.scannedTags = []
    #     sifirpos = None
    #     tags = self.detector.detect(frame, True, [640, 640, 320, 240], 0.03)

    #     done = False
    #     for tag in tags:
    #         if tag.tag_id == 8 and tag.decision_margin > 50: 
    #             print('0 BULDUM AMK')
    #             sifirpos = tag.pose_t
    #             done = True
    #     for tag in tags:
    #         if done and tag.tag_id == 12 and tag.decision_margin > 50: 
    #             print('4 BULDUM AMK')
    #             self.dif(tag, frame, sifirpos)
    #     cv2.imshow('Frame', frame)
    #     #self.handlePositions()
    # def dif(self, tag, frame, sifirpos):
    #     cv2.polylines(frame, [tag.corners.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)
    #     fark = [tag.pose_t[0] - sifirpos[0], tag.pose_t[1] - sifirpos[1], tag.pose_t[2] - sifirpos[2]]
    #     squared_sum = sum(x**2 for x in fark)
    #     length = math.sqrt(squared_sum)
    #     cv2.putText(frame, f"Distance: {length}", org=(int(tag.center[0]), int(tag.center[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 0, 0), thickness=2)
        
    # def testScan(self):
    #     self.test(self.get_current_frame())


def offset():
    result = []
    for a in range(3):
        result.append(float((SPICE_TAG_DIFF[a] + REFERENCE_ROBOT_DIFF[a])/CAMADJUSTER))
    return result

def calculatePosition(reference, spice):
    result = []
    for a in range(3):
        result.append(float(CAMADJUSTER*(spice[a] - reference[a] + offset()[a])))
    
    return result

    
