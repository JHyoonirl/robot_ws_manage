from FT_SENSOR import FTSensor
import time

sensor = FTSensor(port='/dev/ttyUSB0')

if sensor.ft_sensor_init():
    sensor.ft_sensor_continues()

    while True:
        if hasattr(sensor, 'decoded') and sensor.decoded:
            print(sensor.decoded)
        time.sleep(0.01)  # Adjust the sleep time as necessary
