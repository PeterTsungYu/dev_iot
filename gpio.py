# %% 
import RPi.GPIO as GPIO
import time

print('Succeed')

# %%
# GPIO nymbering 
GPIO.setmode(GPIO.BCM)

print('Succeed')

# %%
# read High as 3.3V
channel_DGM = 18

try:
    # make sure it is not a float port
    GPIO.setup(channel_DGM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    #GPIO.setup(channel_DGM, GPIO.IN) 
    print(GPIO.input(channel_DGM)) # while a >=1.6V(3.3/2) input to the pin, it will detect as 1

except Exception as ex:
    print ("Input port is not High" + str(ex))
    exit()

# %%
try:
    GPIO.wait_for_edge(channel_DGM, GPIO.RISING)  
    while True:            # this will carry on until you hit CTRL+C  
        if GPIO.input(channel_DGM): 
            print("Channel_DGM is 1")  
        else:  
            print("Channel_DGM is 0")
        time.sleep(0.1) 
  
except KeyboardInterrupt:  
    GPIO.cleanup()          
# %%
GPIO.cleanup() 
# %%
