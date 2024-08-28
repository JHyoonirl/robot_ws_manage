import numpy as np
import time
import serial
import threading

class FTSensor:
    def __init__(self, port='/dev/ttyUSB0'):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        self.lock = threading.Lock()
        self.data_queue = bytearray()

    def ft_sensor_init(self):
        ''' FT sensor의 초기화 함수 '''
        self.data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_filter = [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_read = [0x0B, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.send_data_without_read(self.data_bias)
            time.sleep(0.1)
            self.send_data_without_read(self.data_baudrate)
            time.sleep(0.1)
            self.send_data_without_read(self.data_filter)
            time.sleep(0.1)
            self.send_data_without_read(self.data_read)
            print('FT sensor 초기화 성공')
            return True
        except Exception as e:
            print('FT sensor 초기화 실패:', e)
            return False
    
    def ft_sensor_continues(self):
        read_thread = threading.Thread(target=self.read_serial_data)
        process_thread = threading.Thread(target=self.process_data)
        read_thread.start()
        process_thread.start()


    def read_serial_data(self):
        ''' Thread to continuously read data from the serial port. '''
        while True:
            data = self.ser.read(100)
            if data:
                with self.lock:
                    self.data_queue.extend(data)
            time.sleep(0.001)  # short sleep to avoid hogging CPU

    def process_data(self):
        ''' Thread to decode the data checking SOP and EOP. '''
        self.time1 = time.time()
        while True:
            with self.lock:
                if len(self.data_queue) >= 19:
                    # Check for SOP and EOP
                    if self.data_queue[0] == 0x55 and self.data_queue[18] == 0xAA:

                        self.decoded = self.decode_received_data(self.data_queue[:19])
                        if self.decoded:
                            print(self.decoded)
                            self.data_queue = self.data_queue[19:]  # Remove processed data
                    else:
                        self.data_queue.pop(0)  # Remove the first byte and recheck in the next cycle

    def decode_received_data(self, packet):
        ''' Decoding logic for received data packet. '''
        self.force = [int.from_bytes(packet[2+i*2:4+i*2], byteorder='big', signed=True) / 50 for i in range(3)]
        self.torque = [int.from_bytes(packet[8+i*2:10+i*2], byteorder='big', signed=True) / 2000 for i in range(3)]
        return self.force + self.torque

    def send_data_without_read(self, data):
        ''' Send commands to the FT sensor. '''
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.ser.write(packet)

    def calculate_checksum(self, data):
        ''' Calculate checksum for a data packet. '''
        return sum(data) % 256

if __name__ == "__main__":
    ft_sensor = FTSensor()
    if ft_sensor.ft_sensor_init():
        read_thread = threading.Thread(target=ft_sensor.read_serial_data)
        process_thread = threading.Thread(target=ft_sensor.process_data)
        read_thread.start()
        process_thread.start()
        read_thread.join()
        process_thread.join()
