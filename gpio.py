# %% 
import numpy as np
import RPi.GPIO as GPIO
import time
import threading

print('Import Succeed')

# %%
def read(channel):
    # while a >=1.6V(3.3/2) input to the pin, it will detect as 1
    #print('read in')
    global pulse_times
    pulse_times.append(time.time())    


def flag_min():
    global pulse_times
    global pulse_times_min
    pulse_times_min.append(pulse_times)
    pulse_times = []
    

# %%
try:
    pulse_times_min = []
    pulse_times = []
    WAIT_TIME_SECONDS = 60 # per min
    ticker = threading.Event()
    count = 0

    # read High as 3.3V
    channel_DGM = 18
    GPIO.setmode(GPIO.BCM)
    # make sure it is not a float port
    GPIO.setup(channel_DGM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    # Edge detection
    GPIO.add_event_detect(channel_DGM, GPIO.RISING, callback=read)
    print('Port setup')

    start = time.time()
    flag_time = start
    #if ticker_2.wait(WAIT_TIME_SECONDS * 5):    
    while not ticker.wait(WAIT_TIME_SECONDS):    
        flag_min()
        print(f"Flag time is {time.time() - flag_time}")
        flag_time = time.time()
        
        count += 1
        if count == 3: # record for mins
            break
        

except KeyboardInterrupt:  
    print(f"Program duration: {time.time() - start}")
    print(f"Keyboard Interrupt...")
    print("=="*30)
except Exception as ex:
    print ("communicating error " + str(ex))    
    print("=="*30)
finally:
    #print(pulse_times_min)
    #print([len(i) for i in pulse_times_min])
    #print(pulse_times)
    print("Clean up ports...")  
    GPIO.cleanup()
    print("=="*30)  

# %%
print("Data Analysis...")

average_flow_rates_min = []

for pulse_times in pulse_times_min:
    average_flow_rates = []

    for interval in range(5, 55, 5):
        #print(interval)
        flow_rate_lst = [] 

        for i in range(interval, len(pulse_times), interval):
            try:
                #print(f"this is the {i}")
                # flow rate in [liter/s]
                # 0.1 liter / pulse
                flow_rate = 60 * 0.1 * (interval-1) / (pulse_times[i-1] - pulse_times[i-interval])
                flow_rate_lst.append(round(flow_rate, 2)) 
            except Exception as ex:
                print("Error: " + str(ex))

        #print(f"In each set-up interval ({interval}[s]), the flow rates are calculated as:\n{flow_rate_lst} in liter/s")
        average_flow_rate = round(sum(flow_rate_lst) / len(flow_rate_lst), 2)
        #print(f"During the period of {pulse_times[interval*int(len(pulse_times)/interval) - 1] - pulse_times[0]}[s], the average flow rate is:\n{average_flow_rate} [liter/s]")
        #print("-"*30)

        average_flow_rates.append(average_flow_rate)

    average_flow_rates_min.append(average_flow_rates)

print(average_flow_rates)
print(average_flow_rates_min)

# %%
#f __name__ == "__main__":
    #main()
