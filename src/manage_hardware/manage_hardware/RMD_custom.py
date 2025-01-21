
import can
import os
import time
import traceback

class RMD:
    def __init__(self, identifier):
        """
        Constructs all the necessary attributes for the RMDX8 object.
        """
        self.bus = None
        self.identifier = identifier

    def setup(self, bustype, channel):
        """
        Setup the can bus connection.

        Returns
        -------
        self.bus : type
            The bus used to communicate with the motor.
        """

        if bustype == 'socketcan':
            try:
                os.system(f"sudo /sbin/ip link set {channel} up type can bitrate 1000000")
                time.sleep(0.1)
            except Exception as e:
                print(e)

            try:
                bus = can.interface.Bus(bustype='socketcan_native', channel=channel)
            except OSError:
                print('err: PiCAN board was not found.')
                exit()
            except Exception as e:
                print(e)

        elif bustype == 'slcan':
            try:
                bus = can.interface.Bus(bustype='slcan', channel=channel, bitrate=1000000)
            except OSError:
                print('err: SLCAN board was not found.')
                exit()
            except Exception as e:
                print(e)
        else:
            print('err: Bus type is not provided or unsupported type')
            exit()

        self.bus = bus
        return self.bus
    
    def close(self):
        self.bus.shutdown()

    def send_cmd(self, data, delay):
        """
        Send frame data to the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.
        delay : int/float
            The time to wait after sending data to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        try:
            message = can.Message(arbitration_id=self.identifier,
                                    data=data, is_extended_id=False)
            self.bus.send(message)
            time.sleep(delay)
            received_message = self.bus.recv()
        except:
            traceback_message = traceback.format_exc()
            print('error' + traceback_message)
        return received_message
    
    def read_pid(self):
        """
        Read the motor's current PID parameters from the ROM.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    
    def write_pid_rom(self, data):
        """
        Write PID parameters to the ROM.
        It will save after turning off the motor
        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x32, 0x00, data[0], data[1],
                    data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.001)
    
    def write_pid_ram(self, data):
        """
        Write PID parameters to the RAM.
        It will not save after turning off the motor

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x31, 0x00, data[0], data[1],
                    data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.001)
    
    def read_acceleration(self, data):
        """
        Read the motor's acceleration data.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x42, data[0], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    
    def write_acceleration(self, index, data):
        """
        Write the acceleration to the RAM of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x43, index[0], 0x00, 0x00,
                    data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.001)

    def status_motor(self):
        '''
        voltage: unit[V]
        temperature: unit[deg]
        torque_current: unit[A]
        speed: unit[deg/s]
        angle: unit[deg]
        
        '''
        response_1 = self.raw_read_motor_status_1()
        response_2 = self.raw_read_motor_status_2()
        data_1 = response_1.data
        data_2 = response_2.data

        voltage = int.from_bytes(data_1[4:6], byteorder='little', signed=True) * 0.1
        temperature = data_2[1]
        torque_current = int.from_bytes(data_2[2:4], byteorder='little', signed=True) * 0.01
        speed = int.from_bytes(data_2[4:6], byteorder='little', signed=True)
        angle = self.read_multi_turns_angle()
        # print(f"Temperature: {temperature}°C, Torque voltage: {voltage*0.1:.1f}V, Torque current: {torque_current*0.01:.2f}A, Speed: {speed}rpm, Angle: {angle:.2f}")
        return voltage, temperature, torque_current, speed, angle
    def raw_read_motor_status_1(self):
        """
        Reads the motor's error status, voltage, temperature and other information.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9A, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def raw_read_motor_status_2(self):
        """
        Reads the motor temperature, voltage, speed and encoder position.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9C, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    
    def read_multi_turns_angle(self):
        data = self.raw_read_multi_turns_angle().data
        self.angle_current = int.from_bytes(data[4:8], byteorder='little', signed=True) * 0.01
        return self.angle_current
    
    def raw_read_multi_turns_angle(self):
        """
        Read the multi-turn angle of the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x92, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    
    def raw_motor_brake_release(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x77, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    
    def raw_motor_brake_lock(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x78, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def raw_motor_off(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x80, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)

    def raw_motor_stop(self):
        """
        Stop the motor, but do not clear the operating state and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x81, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    
    def raw_motor_run(self):
        """
        Resume motor operation from the motor stop command.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x88, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    

    def torque_closed_loop(self, torque_input):
        torque_array = [0x00, 0x00]
        torque_array[0:2] = self.byteArray(torque_input,2)
        data_array =  self.raw_torque_closed_loop(torque_array).data
        # print(data_array)
        temperature = int(data_array[1])
        torque = int.from_bytes(data_array[2:4], byteorder='little', signed=True)
        speed = int.from_bytes(data_array[4:6], byteorder='little', signed=True)
        angle = int.from_bytes(data_array[6:8], byteorder='little', signed=True)
        return temperature, torque, speed, angle

    def raw_torque_closed_loop(self, data):
        """
        Control torque current output of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA1, 0x00, 0x00, 0x00,
                    data[0], data[1], 0x00, 0x00]
        return self.send_cmd(message, 0.001)
    
    def speed_closed_loop(self, speed_input):
        torque_array = [0x00, 0x00, 0x00, 0x00]
        torque_array[0:4] = self.byteArray(speed_input, 4)
        data_array =  self.raw_speed_closed_loop(torque_array).data
        temperature = int(data_array[1])
        torque = int.from_bytes(data_array[2:4], byteorder='little', signed=True)
        speed = int.from_bytes(data_array[4:6], byteorder='little', signed=True)
        angle = int.from_bytes(data_array[6:8], byteorder='little', signed=True)
        return temperature, torque, speed, angle
    
    def raw_speed_closed_loop(self, data):
        """
        Control the speed of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA2, 0x00, 0x00, 0x00,
                    data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.001)
    

    def position_closed_loop(self, angle, VELOCITY_LIMIT):
        
        angle_array = [0xF4, 0x01, 0, 0x10, 0, 0]
        angle_array[0:2] = self.byteArray(VELOCITY_LIMIT, 2)
        angle_array[2:6] = self.byteArray(int(angle * 100), 4)
        # print(angle_array)
        self.raw_position_closed_loop(angle_array)


    def raw_position_closed_loop(self, data):
        """
        Control the position of the motor (multi-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA4, 0x00, data[0], data[1],
                    data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.001)

    
    def byteArray(self, data, size):
        ''' data must be a list that is consisted of hex components '''
        return data.to_bytes(size, 'little', signed=True)