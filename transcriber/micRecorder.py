import sounddevice as sd
import soundfile as sf
from datetime import datetime
import os
import json
import threading
import math
import numpy as np
import wave
import threading
import time
import transcriber as tr
import sys
SOUNDFILEPATH = os.path.join(sys.path[0], '..', '..', 'Data', 'Recordings')

class MicRecorder:
    def __init__(self):
        self.cleanUp()
        self.isRecording = False    
        self.counter = 2
        self.threshold = -20
        self.audioArray = []
        self.shouldListen = False 
        self.index = 0
        self.transcriber = tr.Transcriber(self)
        self.start()

    def cleanUp(self):
        for f in os.listdir(SOUNDFILEPATH):
            if f.endswith('.wav'):
                os.remove(os.path.join(SOUNDFILEPATH, f))

    def activateMicrophone(self): 
        with sd.InputStream(callback=self.micLoop, channels=1, samplerate=48000):
            while(True):
                time.sleep(1)  
                
    def createPath(self):
        return os.path.join(SOUNDFILEPATH,('Recording' +str(self.index) + '.wav'))

    def saveAudio(self):
        with sf.SoundFile(self.createPath(), mode='w', samplerate=48000, 
                      subtype='PCM_16', channels=1) as file:
            for array in self.audioArray:
                file.write(array)
        self.transcriber.filePath = self.createPath()
        threading.Thread(target=self.transcriber.transcribeAudio).start()
        self.index += 1

    def micLoop(self, indata, frames, time, status):
        rms = np.sqrt(np.mean(indata**2))
        decibel_level = calculate_db(rms)
        if(self.shouldListen and not self.isRecording and decibel_level>self.threshold):
            self.isRecording = True
            self.lastSoundTime = datetime.now()
            self.audioArray.clear()
            self.audioArray.append(indata.copy())
        if(self.isRecording):
            self.audioArray.append(indata.copy())
            if (decibel_level>self.threshold):
                self.lastSoundTime = datetime.now()
            else:
                if((datetime.now()-self.lastSoundTime).total_seconds()>self.counter):
                    self.isRecording = False
                    self.saveAudio()


    def start(self):
        threading.Thread(target=self.activateMicrophone).start()


def calculate_db(rms):
    return 20 * math.log10(rms)
