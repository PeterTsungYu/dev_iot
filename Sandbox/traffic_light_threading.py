import Modbus
import threading

import threading
import time,random
event = threading.Event()
#set為綠燈，clear為紅燈
def light():
    count = 0
    if not event.isSet():
        event.set()#設置初始狀態為綠燈
    while True:
        if count <10:
            #綠燈
            print('green', count)
            event.set()
        elif count <13:
            #黃燈
            print('yellow', count)

        elif count <25:
            #紅燈
            print('red', count)
            event.clear()
        else:
            count = 0
            event.set()
        count +=1
        time.sleep(1)

def car(n):
    while True:
        time.sleep(1)#random.randrange(3)
        if event.isSet():#綠燈狀態
            print(f'car{n} is running...')
            #event.wait()
        else:
            print(f'car{n} is waitting...')
            event.wait()#阻塞等待標誌位被設定

def main():
    t_light = threading.Thread(target=light)
    t_light.start()
    for i in range(3):
        t_car = threading.Thread(target=car,args=(i,))
        t_car.start()

main()