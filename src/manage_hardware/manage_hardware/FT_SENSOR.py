import numpy as np
import time
import serial
import queue

class FTSensor:
    def __init__(self, port='COM10'):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0  # 블로킹 모드로 변경
        )
        self.decoded = None
        self.running = True
        self.new_data_available = False

    def ft_sensor_init(self):
        # 초기화 데이터 전송 로직
        initial_commands = [
            [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],  # Bias
            [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],  # Baud rate
            [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00],  # Filter
        ]
        try:
            for command in initial_commands:
                self.send_data_without_read(command)
                time.sleep(0.2)
            print('FT sensor 초기화 성공')
            return True
        except Exception as e:
            print('FT sensor 초기화 실패:', e)
            return False

    def data_read_and_process(self):
        while self.running:
            self.data_read = [0x0B, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]   # Continuous read
            data = self.send_data_with_read(self.data_read)
            try:
                if data[0] == 0x55 and data[18] == 0xAA:
                    decoded_data = self.decode_received_data(data)
                    if decoded_data:
                        self.decoded = decoded_data
                        self.new_data_available = True
                        # print(self.decoded)  # 데이터 처리 결과 출력
                return self.decoded
            except:
                pass
                return False

    def decode_received_data(self, packet):
        force = [int.from_bytes(packet[2+i*2:4+i*2], byteorder='big', signed=True) / 50 for i in range(3)]
        torque = [int.from_bytes(packet[8+i*2:10+i*2], byteorder='big', signed=True) / 2000 for i in range(3)]
        return force + torque

    def send_data_with_read(self, data):
        sop = bytes([0x55])
        eop = bytes([0xAA])
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.ser.write(packet)
        return self.ser.read(19)  # Adjusted to ensure response is 
    def send_data_without_read(self, data):
        sop = bytes([0x55])
        eop = bytes([0xAA])
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.ser.write(packet)
        return self.ser.read(19)  # Adjusted to ensure response is captured correctly

    def calculate_checksum(self, data):
        return sum(data) % 256

    def stop(self):
        self.running = False

if __name__ == "__main__":
    ft_sensor = FTSensor()
    if ft_sensor.ft_sensor_init():
        ft_sensor.data_read_and_process()