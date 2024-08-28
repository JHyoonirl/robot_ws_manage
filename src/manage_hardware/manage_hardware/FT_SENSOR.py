import numpy as np
import time
import serial

class FTSensor:
    def __init__(self):
        self.ser = serial.Serial(
            port='COM4',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )

    def ft_sensor_init(self):
        ''' FT sensor의 초기화 함수 '''
        self.data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_filter = [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_read = [0x0B, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.send_data_without_read(self.data_bias)
            time.sleep(0.01)
            self.send_data_without_read(self.data_baudrate)
            time.sleep(0.01)
            self.send_data_without_read(self.data_filter)
            time.sleep(0.1)
            self.send_data_without_read(self.data_read)
            print('FT sensor 초기화 성공')
            return True
        except Exception as e:
            print('FT sensor 초기화 실패:', e)
            return False

    def calculate_checksum(self, data):
        ''' 데이터의 체크섬을 계산합니다. '''
        return sum(data) % 256

    def decode_received_data(self, packet):
        ''' 수신된 패킷을 처리합니다. '''
        force_raw = []
        torque_raw = []
        force = []
        torque = []
        if len(packet) == 19 and packet[0] == 0x55 and packet[-1] == 0xAA:  # Check if packet starts with SOP and ends with EOP
            received_packet = packet
            force_raw.append( received_packet[2] << 8 | received_packet[3])
            force_raw.append( received_packet[4] << 8 | received_packet[5])
            force_raw.append( received_packet[6] << 8 | received_packet[7])
            torque_raw.append( received_packet[8] << 8 | received_packet[9])
            torque_raw.append( received_packet[10] << 8 | received_packet[11])
            torque_raw.append( received_packet[12] << 8 | received_packet[13])
            for i in range(0, 3):
                force_temp  = force_raw[i].to_bytes(2, 'big')
                torque_temp  = torque_raw[i].to_bytes(2, 'big')
                # [Divider] Force: 50, Torque: 2000
                # Note: Resolution of RFT76-HA01 is same with RFT40-SA01
                force.append(int.from_bytes(force_temp, "big", signed=True) / 50)
                torque.append(int.from_bytes(torque_temp, "big", signed=True) / 2000)
            # 여기서 데이터 처리를 수행하거나 필요한 작업을 수행할 수 있습니다.
            data = force + torque
        return data

    def read_serial_data(self):
        ''' 시리얼 데이터를 안정적으로 읽고 처리합니다. '''
        buffer = b''
        past_time = time.time()  # Start timing before the loop
        while True:
            data = self.ser.read(19)
            if data:
                buffer += data
                if len(buffer) > 19:
                    buffer = buffer[-19:]  # Keep last 25 bytes

                if len(buffer) == 19:
                    if buffer[0] == 0x55 and buffer[18] == 0xAA:
                        decoded = self.decode_received_data(buffer)
                        buffer = buffer[19:-1]
                        print(decoded)
                        cur_time = time.time()
                        elapsed_time = cur_time - past_time
                        print(f"Elapsed time: {elapsed_time:.3f} seconds")
                        past_time = time.time()  # Reset timing after processing
                        if decoded:
                            buffer = b''  # Clear buffer after processing
                    else:
                        buffer = buffer[1:]  # Shift buffer to find new possible packet start
            time.sleep(0.001)

    def send_data_without_read(self, data):
        ''' 데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다. '''
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop
        self.ser.write(packet)

if __name__ == "__main__":
    ft_sensor = FTSensor()
    if ft_sensor.ft_sensor_init():
        ft_sensor.read_serial_data()

