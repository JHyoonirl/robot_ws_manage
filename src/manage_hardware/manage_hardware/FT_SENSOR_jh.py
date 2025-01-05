import serial
import time

class FTSensor:
    def __init__(self, port='/dev/ttyUSB0'):
        self.sensor = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=None  # 블로킹 없이 설정
        )

        self.data_buffer = bytearray()
        self.last_time = time.time()
        self.packet_count = 0
        self.decoded_force = None
        self.decoded_torque = None

    def ft_sensor_init(self):
        # 초기화 데이터 전송 로직
        initial_commands = [
            [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],  # Bias
            [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],  # Baud rate
            [0x08, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00],  # Filter
            [0x0F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]   # Output rate
        ]
        for command in initial_commands:
            self.send_data_without_read(command)
            time.sleep(0.2)
        print('FT sensor 초기화 성공')
        return True

    def ft_sensor_once_data(self):
        cmd = [0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]   # Continuous read
        try:
            self.send_data_without_read(cmd)
            time.sleep(0.2)
            print('FT sensor start 성공')
            return True
        except Exception as e:
            print('FT sensor start 실패:', e)
            return False
    
    def ft_sensor_continuous_data(self):
        cmd = [0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]   # Continuous read
        try:
            self.send_data_without_read(cmd)
            time.sleep(0.2)
            print('FT sensor start 성공')
            return True
        except Exception as e:
            print('FT sensor start 실패:', e)
            return False
        
    def ft_sensor_continuous_data_read(self):
        data = self.sensor.read(1)
        if data[0] == 0x55:
            data = self.sensor.read(1)
            if data[0] == 0x0B:
                data = self.sensor.read(17)
                if data[16] == 0xAA:
                    packet = data[0:16]
                    self.decoded_force, self.decoded_torque = self.decode_received_data(packet)
                    # print(self.decoded_force, self.decoded_torque)
                    return self.decoded_force, self.decoded_torque
        else:
            print('No')
            return [False, False, False], [False, False, False]    
        
    def ft_sensor_stop_data(self):
        cmd = [0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]   # Continuous read
        try:
            self.send_data_without_read(cmd)
            time.sleep(0.2)
            print('FT sensor Stop 성공')
            return True
        except Exception as e:
            print('FT sensor Stop 실패:', e)
            return False
        
    def ft_sensor_bias_set(self):
        cmd = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.send_data_without_read(cmd)
            time.sleep(0.2)
            print('FT sensor bias 성공')
            return True
        except Exception as e:
            print('FT sensor bias 실패:', e)
            return False

    def decode_received_data(self, packet):
        force = [int.from_bytes(packet[i*2:2+i*2], byteorder='big', signed=True) / 50 for i in range(3)]
        torque = [int.from_bytes(packet[6+i*2:8+i*2], byteorder='big', signed=True) / 1000 for i in range(3)]
        return force, torque

    def send_data_without_read(self, data):
        sop = bytes([0x55])
        eop = bytes([0xAA])
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.sensor.write(packet)

    def calculate_checksum(self, data):
        return sum(data) % 256

    def stop(self):
        self.running = False
        self.sensor.close()

if __name__ == "__main__":
    print('Do not access this code directly')
