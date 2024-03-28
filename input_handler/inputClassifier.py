import commandFormer
import textHandler
import ttsModule
class Classifier:
    def __init__(self):
        self.spicesAndCoords = {"spices": [], "coords": []}
        self.textHandler = textHandler.TextHandler(self)
        self.ttsModule = ttsModule.TTSModule()
        self.index =0
        self.output = {}
        pass

    def updateSpices(self, data):
        self.spicesAndCoords = {"spices": [], "coords": []}
        print("updateSpices")
        for spiceIndex in range(len(data['spices'])):
            for coordIndex in range(len(data['coords'])):
                if spiceIndex == coordIndex:
                    self.spicesAndCoords['spices'].append(data['spices'][spiceIndex])
                    self.spicesAndCoords['coords'].append(data['coords'][coordIndex])
                    print(f"spice: {data['spices'][spiceIndex]} coord: {data['coords'][coordIndex]}")
                    print(self.spicesAndCoords)
    
    def handleStuck(self):
        self.ttsModule.react('stuck')
    

    
        
    def classify(self, input):
        self.textHandler.processText(input)

    