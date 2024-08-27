import numpy as np
import time
import serial

class FTSensor:
    def __init__(self):
        
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',\
            baudrate=115200,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
                timeout=0)
        if self.ser != False:
            self.ft_sensor_init()
    
    def ft_sensor_init(self):
        '''
        ft_sensor의 초기화 함수
        '''

        self.data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_filter = [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_name = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_read = [0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        try:
            self.send_data_without_read(self.data_bias)
            self.send_data_without_read(self.data_baudrate)
            # print(data_bias)
            time.sleep(0.01)
            self.send_data_without_read(self.data_filter)
            time.sleep(0.1)
            print('FT sensor 초기화 성공')
            return True
        except:
            print('FT sensor 초기화 실패')
            return False
    
    def calculate_checksum(self, data):
            """
            데이터의 체크섬을 계산합니다.
            """
            checksum = sum(data) % 256
            return checksum

    def decode_received_data(self, data):
        """
        수신된 데이터에서 Start of Packet (SOP)와 End of Packet (EOP)을 찾아 데이터를 출력합니다.
        """
        sop_index = data.find(bytes([0x55]))  # SOP (Start of Packet)의 인덱스 찾기
        eop_index = data.find(bytes([0xAA]))  # EOP (End of Packet)의 인덱스 찾기
        force_raw = []
        torque_raw = []
        force = []
        torque = []
        try:
            if sop_index != -1 and eop_index != -1:  # SOP와 EOP 모두 존재하는 경우
                received_packet = data[sop_index:eop_index+1]  # SOP부터 EOP까지의 패킷 추출
                # print(received_packet)
                # print(received_packet[0])
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
                # ///print(force)
            # else:
                # print("Incomplete packet received")
                data = force + torque
            return data
        except:
            pass
            return False

    def send_data_with_read(self, data):
        """
        데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
        """
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)

        # 데이터와 체크섬 조합
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop

        # 시리얼 포트로 전송
        self.ser.write(packet)
        # print("send packet: {0}".format(packet.hex()))
        sent_data = self.ser.read(19)
        return sent_data

    def send_data_without_read(self, data):
        """
        데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
        """
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)

        # 데이터와 체크섬 조합
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop

        # 시리얼 포트로 전송
        self.ser.write(packet)

if __name__ == "__main__":
    ft_sensor = FTSensor()
    init_status = ft_sensor.ft_sensor_init()
    if init_status == True:
        while True:
            ft_sensor.send_data_without_read(ft_sensor.data_read)
            read_msg = ft_sensor.ser.read(19)
            decoded_data = ft_sensor.decode_received_data(read_msg)
            print(decoded_data)
    else:
        pass
