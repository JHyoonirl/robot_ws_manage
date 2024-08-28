import numpy as np
import time
import serial

class FTSensor:
    def __init__(self, port='COM4'):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        self.data_queue = bytearray()
        self.decoded = None

    def ft_sensor_init(self):
        self.data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_filter = [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.send_data_without_read(self.data_bias)
            time.sleep(0.1)
            self.send_data_without_read(self.data_baudrate)
            time.sleep(0.1)
            self.send_data_without_read(self.data_filter)
            time.sleep(0.1)
            print('FT sensor 초기화 성공')
            return True
        except Exception as e:
            print('FT sensor 초기화 실패:', e)
            return False

    def main_loop(self):
        while True:
            self.read_and_process_data()

    def read_and_process_data(self):
        data = self.send_data_with_read([0x0A, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00])
        if data:
            self.data_queue.extend(data)

        if len(self.data_queue) >= 19:
            if self.data_queue[0] == 0x55 and self.data_queue[18] == 0xAA:
                packet = self.data_queue[:19]
                self.data_queue = self.data_queue[19:]  # Remove the processed packet from the queue
                self.decoded = self.decode_received_data(packet)
                if self.decoded:
                    print(self.decoded)
            else:
                # Remove incorrect data until a new SOP is found
                sop_index = self.data_queue.find(0x55, 1)  # Start looking after the first byte
                if sop_index == -1:
                    self.data_queue.clear()
                else:
                    self.data_queue = self.data_queue[sop_index:]

    def decode_received_data(self, packet):
        self.force = [int.from_bytes(packet[2+i*2:4+i*2], byteorder='big', signed=True) / 50 for i in range(3)]
        self.torque = [int.from_bytes(packet[8+i*2:10+i*2], byteorder='big', signed=True) / 2000 for i in range(3)]
        return self.force + self.torque

    def send_data_without_read(self, data):
        self.send_data_with_read(data)

    def send_data_with_read(self, data):
        sop = bytes([0x55])
        eop = bytes([0xAA])
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.ser.write(packet)
        return self.ser.read(19)  # Adjusted to ensure response is captured correctly

    def calculate_checksum(self, data):
        return sum(data) % 256

if __name__ == "__main__":
    ft_sensor = FTSensor()
    if ft_sensor.ft_sensor_init():
        ft_sensor.main_loop()
