#!/usr/bin/env python

from select import select
import numpy as np

actions = np.array(['Tool-use','Handover', 'StoreInShelf', 'StoreInBin', 'Pour'])
products = np.array(['Cola', 'Pasta', 'wineglass', 'milkbox', 'hansaplast'])

def select_action():
    choice = -1
    while not (choice in [0, 1, 2, 3, 4, 5]):
        print('\n')
        print('PLEASE SELECT AN ACTION:')
        print('0. STOP PROGRAM')
        print('1. TOOL-USE')
        print('2. HANDOVER')
        print('3. STORE IN SHELF')
        print('4. STORE IN BIN')
        print('5. POUR')

        try:
            choice = int(input('\n'))
        except:
            print("INVALID INPUT")
        return choice

def select_product():
    choice = -1
    while not (choice in [0, 1, 2, 3, 4, 5]):
        print('\n')
        print('PLEASE SELECT A PRODUCT:')
        print('0. START AGAIN')
        print('1. COLA')
        print('2. PASTA')

        try: 
            choice = int(input('\n'))
        except:
            print("INVALID INPUT")
        return choice

def start():
    action, product = -1, -1
    action = select_action()

    if action == 0:
        print('GOOD BYE! ')
        exit()
    else:
        print(actions[action-1])
        product = select_product()
    
    if product == 0:
        return 1, ''
    else: 
        print(products[product-1])
        print('\n')
        print('YOUR CHOICE IS:')
        print("Perform the task ", actions[action-1], 'on the product ', products[product-1], '.')
        # return 0, str(actions[action-1] + '(' + products[products-1] + ', PLAN)')

        return actions[action-1], products[product-1]

if __name__ == '__main__':
    x,y = start()
    print(x)
    print(y)
 