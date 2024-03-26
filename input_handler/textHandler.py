import json
import os
import sys
import commandFormer
import time
from word2number import w2n
SIZESPATH = os.path.join(sys.path[0], '..', '..', 'Data', 'JSONData', 'sizes.json')
class TextHandler:
    def __init__(self, classifier):
        self.classifier = classifier
        self.sizesData = self.loadSizes()
        self.index = 0

    def loadSizes(self):
        with open(SIZESPATH) as file:
            return json.load(file)

    def processText(self, input):
        if 'assistant' in input.lower():
            print('command has keyword')
            if len(self.classifier.spicesAndCoords['spices']) == 0:
                print('1')
                self.react('no_scanned_spices')
            else:
                self.spices = self.classifier.spicesAndCoords['spices']
                self.checkSpices(input)
    
    def checkSpices(self, input):
        self.input = input
        matching_spices = []
        for spice in self.spices:
            if spice.lower() in self.input.lower():
                matching_spices.append(spice.lower())
        self.handleSpices(matching_spices)

    def handleSpices(self, matching_spices):
        if len(matching_spices) == 0:
            self.handleResult(type='no_spices')
           

        elif len(matching_spices) == 1:
            self.checkAmount(matching_spices[0])
        else:
            self.handleResult(type='multiple_spices')

    def checkAmount(self, spice):
        amounts = {}
        types = ['small', 'medium', 'large', 'custom']
        for a in types:
            amounts[a] = []
            for word in self.sizesData[a]:
                if word.lower() in self.input.lower():
                    amounts[a].append(word.lower())

        totalNotEmpty = 0
        for a in amounts:
            if len(amounts[a]) > 0:
                totalNotEmpty += 1
        if totalNotEmpty == 0:
            self.handleResult(type='no_amount', spice=spice)
        elif totalNotEmpty > 1:
            self.handleResult(type='multiple_amounts', spice=spice)
        elif totalNotEmpty == 1:
            self.clarifyAmount(amounts, spice)

    def clarifyAmount(self,amounts,  spice):
        if len(amounts['small']) > 0:
            self.handleResult('spice_and_amount', amount='small amount', amountType='small_amount', spice=spice)
        elif len(amounts['medium']) > 0:
            self.handleResult('spice_and_amount', amount='medium amount', amountType='medium_amount', spice=spice)
        elif len(amounts['large']) > 0:
            self.handleResult('spice_and_amount', amount='large amount', amountType='large_amount', spice=spice)
        elif len(amounts['custom']) > 0:
            custom_word = amounts['custom'][0]
            words = self.input.lower().split() 
            custom_amount = None
            for word in words:
                if custom_word in word:
                    temp =  words[words.index(word) - 1]
                    temp.replace(',', '')
                    temp.replace('.', '')
                    if  temp.isnumeric():
                        custom_amount = temp
                    else:
                        a = 0
                        number = ''
                        while a < words.index(word):
                            try:
                                w2n.word_to_num(words[words.index(word) - 1 - a])
                                number = words[words.index(word) - 1 - a] + ' ' + number
                                a+=1
                            except ValueError:
                                break
                        if(number == ''):
                            print('no number')
                        else:
                            custom_amount = w2n.word_to_num(number)

            self.handleResult('spice_and_amount', amount=custom_amount, amountType=custom_word , spice=spice)
    
    def react(self, type, amount=None, spice=None):
        self.classifier.ttsModule.react(type, amount, spice)
        
    def handleResult(self, type, amount='', amountType ='', spice=''):
        if type == 'no_spices':
            self.react('no_spices')
        elif type == 'multiple_spices':
            self.react('multiple_spices')
        elif type == 'no_amount':
            self.react('no_amount', spice=spice)
        elif type == 'multiple_amounts':
            self.react('multiple_amounts', spice=spice)
        elif type == 'spice_and_amount':
            if amountType in (['small_amount' , 'medium_amount' ,'large_amount']):
                self.classifier.output={'index' : self.index, 'coords' : self.getCoordFromSpiceName(spice), 'amount' : commandFormer.handlePreset(amountType)}
                self.index += 1
                self.react('spice_and_amount', amount=amount, spice=spice)
                
            else:
                a= amount +' '+amountType
                self.classifier.output={'index' : self.index, 'coords' : self.getCoordFromSpiceName(spice), 'amount' : commandFormer.handleCustom(amountType, amount)}
                self.index += 1
                self.react('spice_and_amount', amount=a, spice=spice)

    def getCoordFromSpiceName(self, spice):
        for a in range(len(self.classifier.spicesAndCoords['spices'])):
            if spice.lower() in self.classifier.spicesAndCoords['spices'][a].lower():
                return self.classifier.spicesAndCoords['coords'][a]