# %% 
import RPi.GPIO as GPIO
import time

print('Import Succeed')

# %%
def read(channel):
    # while a >=1.6V(3.3/2) input to the pin, it will detect as 1
    #processpulse(channel_DGM, GPIO.input(channel))
    print('read in')
    pulse_times.append(time.time())
    #print(f"Channel_{channel} is Rising to {GPIO.input(channel)}")
# %%
try:
    pulse_times = []

    # read High as 3.3V
    channel_DGM = 18
    GPIO.setmode(GPIO.BCM)
    # make sure it is not a float port
    GPIO.setup(channel_DGM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    # Edge detection
    GPIO.add_event_detect(channel_DGM, GPIO.RISING, callback=read)
    print('Port setup')

    start = time.time()
    while True:    
        pass

except KeyboardInterrupt:  
    print("Keyboard Interrupt")
except Exception as ex:
    print ("communicating error " + str(ex))    
finally:
    print(time.time()-start)
    print("Clean up ports...")  
    GPIO.cleanup()
    print("=="*30)  

# %%
print("Data Analysis...")

for interval in range(2, 20, 2):
    print(interval)
    flow_rate_lst = [] 

    for i in range(interval, len(pulse_times), interval):
        try:
            print(f"this is the {i}")
            # flow rate in [liter/s]
            # 0.1 liter / pulse
            flow_rate = 60 * 0.1 * (interval-1) / (pulse_times[i-1] - pulse_times[i-interval])
            flow_rate_lst.append(round(flow_rate, 2)) 
        except Exception as ex:
            print("Error: " + str(ex))

    print(f"In each set-up interval ({interval}[s]), the flow rates are calculated as:\n{flow_rate_lst} in liter/s")
    average_flow_rate = round(sum(flow_rate_lst) / len(flow_rate_lst), 2)
    print(f"During the period of {pulse_times[interval*int(len(pulse_times)/interval) - 1] - pulse_times[0]}[s], the average flow rate is:\n{average_flow_rate} [liter/s]")

# %%
#f __name__ == "__main__":
    #main()
