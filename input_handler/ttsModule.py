import pyttsx3
import json
import random
import sys
import os
import time
import threading
PHRASESPATH = os.path.join(sys.path[0], '..', '..', 'Data', 'JSONData', 'phrases.json')
class TTSModule:
    def __init__(self):
        self.data = self.loadPhrases()
        threading.Thread(target=self.setup).start()

    def setup(self):
        self.ttsEngine = pyttsx3.init() 
        voices = self.ttsEngine.getProperty('voices')
        self.ttsEngine.setProperty('voice', voices[11].id)
        self.ttsEngine.setProperty('rate', 125)
        self.ttsEngine.startLoop()


    def loadPhrases(self):
        with open(PHRASESPATH) as file:
            return json.load(file)

    def react(self, type, amount=None, spice=None):  
        if amount is not None and spice is not None:
           self.talk(random.choice(self.data[type]).format(amount=amount, spice=spice))
        elif amount is None and spice is not None:
           self.talk(random.choice(self.data[type]).format(spice=spice))
        else:
            self.talk(random.choice(self.data[type]))

        
    def talk(self, text):
        print(f'\n\n\nTTS:\n{text}\n\n\n\n')
        self.ttsEngine.say(text)