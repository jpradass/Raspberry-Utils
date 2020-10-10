import board
import busio
import Adafruit_DHT
import time
import datetime
import RPi.GPIO as GPIO
import tm1637

DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4

Display = tm1637.TM1637(20,21,tm1637.BRIGHT_TYPICAL)

Display.Clear()
Display.SetBrightnes(3)

while True:
    try:
        humidity, temp = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
        if humidity is not None and temp is not None:
            Display.Clear()
            Display.Show([int(temp/10), int(temp % 10), 38, 36])
            time.sleep(5)
            Display.Show([int(humidity/10), int(humidity % 10), 25, 36])
            time.sleep(5)
    except:
        pass
    for i in range(0,11):
        now = datetime.datetime.now()
        hour = now.hour
        minute = now.minute
        second = now.second
        Display.Show([ int(hour / 10), hour % 10, int(minute / 10), minute % 10 ])
        Display.ShowDoublepoint(i % 2)
        time.sleep(1)
    
    else:
        time.sleep(1)

