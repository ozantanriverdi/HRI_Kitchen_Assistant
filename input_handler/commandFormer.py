import os
import json
import sys

SMALLAMOUNT=10
MEDIUMAMOUNT=20
LARGEAMOUNT=30
GRAMS = 1
PINCHES = 2
TEASPOON = 10
TABLESPOON = 20

def handlePreset(type):
    if type == 'small_amount':
        return (SMALLAMOUNT)
    elif type == 'medium_amount':
        return (MEDIUMAMOUNT)
    elif type == 'large_amount':           
        return (LARGEAMOUNT)
def handleCustom(type, amount):
    if(type in 'grams'):
        return (GRAMS*int(amount))
    elif(type in 'pinches'):    
        return (PINCHES*int(amount))
    elif(type in 'teaspoons'):
        return (TEASPOON*int(amount))
    elif(type in'tablespoons'):
        return (TABLESPOON*int(amount))