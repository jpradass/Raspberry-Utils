import requests
import Adafruit_DHT
import re
import time
import datetime

DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4
SONOFF_IP = "192.168.1.50"
TEMP_THRESHOLD = float(27)

def check_status():
    response = requests.get("http://"+SONOFF_IP+"?m=1", timeout = 3).content.decode("utf-8")
    return re.search("(O[N|F]{1,2})", response).group()

def toggle(status):
    print("["+ str(datetime.datetime.now()) +"] switching", status)
    requests.get("http://"+SONOFF_IP+"?m=1&o=1", timeout = 3)

while True:
    _, temp = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)
    if temp is not None:
        if temp > TEMP_THRESHOLD and check_status() == "OFF":
            toggle("ON")
        if temp < TEMP_THRESHOLD and check_status() == "ON":
            toggle("OFF")
    time.sleep(60)

