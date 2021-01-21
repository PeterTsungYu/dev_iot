# %% 
import RPi.GPIO as GPIO
import time

print('Succeed')

# %%
interval = 5
lastsend = 0
pulse_time = {
    'rising' : [],
    'falling' : []
}

# read High as 3.3V
channel_DGM = 18

def func_callback(channel):
    # while a >=1.6V(3.3/2) input to the pin, it will detect as 1
    #processpulse(channel_DGM, GPIO.input(channel))
    print('sth')
    
# %%
try:
    GPIO.setmode(GPIO.BCM)
    # make sure it is not a float port
    GPIO.setup(channel_DGM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    # Edge detection
    GPIO.add_event_detect(channel_DGM, GPIO.BOTH)
    print('Port setup')

    while True:
        if GPIO.event_detected(channel_DGM):
            edge_time = time.time()
            print(GPIO.input(channel_DGM))
            if GPIO.input(channel_DGM):
                pulse_time['rising'].append(edge_time)
            else:
                pulse_time['falling'].append(edge_time)

except KeyboardInterrupt:  
    print(pulse_time)
except Exception as ex:
    print ("communicating error " + str(ex))    
finally:  
    GPIO.cleanup()  

# %%
GPIO.cleanup()
# %%
if __name__ == "__main__":
    main()
