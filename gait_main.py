# import serial

# # USART 설정 및 초기화 함수
def initialize_serial():
#     try:
#         ser = serial.Serial(
#             port='/dev/ttyUSB0',
#             baudrate=9600,
#             bytesize=serial.EIGHTBITS,
#             parity=serial.PARITY_NONE,
#             stopbits=serial.STOPBITS_ONE,
#             timeout=1
#         )
#         return ser
#     except serial.SerialException as e:
#         print("Warning: Serial communication failed. Please check the connection and settings.")
#         print("Detailed error:", e)
        return None

# 데이터 수신 및 필드 추출 함수
def extract_fields(ser):
        
    # line = ser.readline().decode('utf-8').strip()
    # if line:
        # fields = line.split('|')

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

def switch_1(data):

    # min,max data recording start

def switch_2(data):

    # min,max data recording stop

def switch_3(data):

    # algorithm start
def switch_4(data):

    # algorithm stop

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


