from gpiozero import CPUTemperature
import RPi.GPIO as GPIO
import time

max_threashold = float(55.0)
min_threashold = float(35.0)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(8, GPIO.OUT)
state = not GPIO.input(8)

cpu = CPUTemperature()
print("CPU temperature now:", cpu.temperature)

def change_state(switch, state):
    print("+ Switching", switch, "fan")
    print("+ CPU temperature now:", cpu.temperature)
    GPIO.output(8, state)
    return not state

try:
    while 1:
        if cpu.temperature >= max_threashold and not state:
            state = change_state("on", state)          

        elif cpu.temperature < min_threashold and state:
            state = change_state("off", state)
        
        else:
            time.sleep(5)
        
except KeyboardInterrupt:
    GPIO.cleanup()
    
except Exception as e:
    GPIO.cleanup()
    print(e)