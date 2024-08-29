from FT_SENSOR import FTSensor
import time

sensor = FTSensor(port='/dev/ttyUSB0')

if sensor.ft_sensor_init():
    sensor.ft_sensor_continues()

    while True:
        if sensor.decoded_data:
            print(sensor.force)
        time.sleep(0.01)
