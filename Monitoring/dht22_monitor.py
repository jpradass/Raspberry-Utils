import time
import requests

INFLUX_URL = 'http://localhost:8086/write?db=DHT22'

def sendDataToGrafana(humidity, temp, pressure):

    requests.post(INFLUX_URL, data='temperature value=' + str(temp))
    requests.post(INFLUX_URL, data='humidity value=' + str(humidity))
    requests.post(INFLUX_URL, data='pressure value=' + str(pressure))

