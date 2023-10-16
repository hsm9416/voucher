
import serial
import rospy
import numpy
import math
import time

# gait cycle 을 외부 ROS topic 으로 받아오는 코드
class GaitCycle:
    def __init__(self,gait_cycle) -> None:
        self.gait_cycle = gait_cycle

    def gait_cycle_callback(data):
        gait_value = data.data
        if gait_value :
            rospy.loginfo("Received gait cycle value: %d", gait_value)
        else:
            rospy.logwarn("Received out-of-range gait cycle value: %d", gait_value)

    def listener():
        rospy.init_node('gait_cycle_listener', anonymous=True)
        rospy.Subscriber("/gait_cycle", Int32, gait_cycle_callback)
        rospy.spin()

    if __name__ == '__main__':
        listener()

class ImpedanceControl:
    def __init__(self, M, B, K):
        self.M = M  # Mass coefficient
        self.B = B  # Damping coefficient
        self.K = K  # Spring coefficient
        
        # Initial values
        self.a = 0.0  # Current acceleration
        self.v = 0.0  # Current velocity
        self.x = 0.0  # Current position

    def compute_force(self, a_d, v_d, x_d):
        """Compute the external force based on desired and current states."""
        tau = self.M * (a_d - self.a) + self.B * (v_d - self.v) + self.K * (x_d - self.x)
        return tau

    def update_state(self, a_new, v_new, x_new):
        """Update the current state of the system."""
        self.a = a_new
        self.v = v_new
        self.x = x_new

# # USART 설정 및 초기화 함수
def initialize_serial():
    try:
        ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        return ser
    except serial.SerialException as e:
        print("Warning: Serial communication failed. Please check the connection and settings.")
        print("Detailed error:", e)
        return None

# 데이터 수신 및 필드 추출 함수
def extract_fields(ser):
        
    line = ser.readline().decode('utf-8').strip()
    if line:
        fields = line.split('|')

        fields = [0,1,2,3,4,5,6,7,8,9,10]
        return {
            'robot_time': fields[0],
            'l_hip_angle': fields[1],
            'r_hip_angle': fields[2],
            'l_hip_velocity': fields[3],
            'r_hip_velocity': fields[4],
            'l_hip_current': fields[5],
            'r_hip_current': fields[6],
            'l_hip_target_speed': fields[7],
            'r_hip_target_speed': fields[8],
            'control_mode': fields[9],
            'control_interval': fields[10]
        }
    # return None
def set_switch():
    while True:
        switch = input("Switch: ")
        if switch in ['1', '2', '3', '4']:
            return switch
        else:
            print("Invalid input.")
    
# 메인 코드


ser = initialize_serial()
if ser:
    switch = set_switch()  # Ask the user to set the switch.
    print(f"switch set to: {switch}")

    data = extract_fields(ser)
    if data:
        if switch == '1':
            switch_1(data)
        elif switch == '2':
            switch_2(data)
        elif switch == '3':
            switch_3(data)
        elif switch == '4':
            switch_4(data)
    else:
        print("No data received.")

# 메인 코드
ser = initialize_serial()
if ser:
    switch = set_switch() 
    print(f"switch set to: {switch}")

    data = extract_fields(ser)

    robot_time = data['robot_time']
    l_hip_angle = data['l_hip_angle']
    r_hip_angle = data['r_hip_angle']
    l_hip_velocity = data['l_hip_velocity']
    r_hip_velocity = data['r_hip_velocity']
    l_hip_current = data['l_hip_current']
    r_hip_current = data['r_hip_current']
    l_hip_target_speed = data['l_hip_target_speed']
    r_hip_target_speed = data['r_hip_target_speed']
    control_mode = data['control_mode']
    control_interval = data['control_interval']

    print(robot_time)

    # parameters
    k = 1
    b = 1

    l_deired_angle = 0
    r_deired_angle = 0
    l_deired_velocity = 0
    r_deired_velocity = 0

    l_tau = k(l_deired_angle - l_hip_angle) + b(l_deired_velocity - l_hip_velocity)
    r_tau = k(r_deired_angle - r_hip_angle) + b(r_deired_velocity - r_hip_velocity)

    # send torque
    ser.write(l_tau)
    ser.write(r_tau)


