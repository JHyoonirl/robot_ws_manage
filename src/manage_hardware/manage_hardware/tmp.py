from FT_SENSOR import FTSensor
import time

sensor = FTSensor(port='/dev/ttyUSB0')

if sensor.ft_sensor_init():
    sensor.ft_sensor_continues()
    old_time = time.time()
    while True:
        
        if sensor.decoded and (time.time() - old_time) > 0.05:
            # print(sensor.decoded)
            old_time = time.time()
            print(sensor.decoded)
        # time.sleep(0.01)  # Adjust the sleep time as necessary
