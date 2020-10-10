import time
import datetime
import math
import RPi.GPIO as GPIO
import tm1637
from bmx280 import *
from Monitoring import dht22_monitor

Display = tm1637.TM1637(20,21,tm1637.BRIGHT_TYPICAL)

Display.Clear()
Display.SetBrightnes(3)

bmx280Begin()     

while True:
    for i in range(0,11):
        now = datetime.datetime.now()
        hour = now.hour
        minute = now.minute
        second = now.second
        Display.Show([ int(hour / 10), hour % 10, int(minute / 10), minute % 10 ])
        Display.ShowDoublepoint(i % 2)
        time.sleep(1)
    
    bmx280ReadData()
    temperature, pressure, humidity = bmx280GetTemperature(), bmx280GetPressure(), bmx280GetHumidity()

    if humidity is not None and temperature is not None and pressure is not None:
        Display.Clear()
        Display.Show([int(temperature/10), int(temperature % 10), 38, 36])
        time.sleep(5)
        Display.Show([int(humidity/10), int(humidity % 10), 25, 36])
        time.sleep(5)
        length = int(math.log10(pressure))
        Display.Show([int(pressure/math.pow(10, length)), int((pressure % math.pow(10, length))/math.pow(10, length-1)), 25, 10])
        time.sleep(5)

        dht22_monitor.sendDataToGrafana(humidity, temperature, pressure)
    
    else:
        time.sleep(1)