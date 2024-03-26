from word2number import w2n

input = 'add grams sugar'
words = input.split()  
custom_amount = None
for word in words:
    if 'gram' in word:
        if  words[words.index(word) - 1].isnumeric():
            custom_amount = words[words.index(word) - 1]
        else:
            a=0
            number = ''
            while a<words.index(word):
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
