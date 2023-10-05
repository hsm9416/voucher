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

def set_mode():
    while True:
        mode = input("Mode: ")
        if mode in ['1', '2', '3', '4']:
            return mode
        else:
            print("Invalid input.")

def mode_1(data):
    # Example algorithm for Mode 1
    print(data['l_hip_angle'])
    # ... (your algorithm code here)

def mode_2(data):
    # Example algorithm for Mode 2
    print(data['l_hip_velocity'])
    # ... (your algorithm code here)

def mode_3(data):
    # Example algorithm for Mode 3
    print(data['r_hip_angle'])
    # ... (your algorithm code here)

def mode_4(data):
    # Example algorithm for Mode 4
    print(data['r_hip_velocity'])
    # ... (your algorithm code here)

# 메인 코드
ser = initialize_serial()
if ser:
    mode = set_mode()  # Ask the user to set the mode.
    print(f"Mode set to: {mode}")

    data = extract_fields(ser)
    if data:
        if mode == '1':
            mode_1(data)
        elif mode == '2':
            mode_2(data)
        elif mode == '3':
            mode_3(data)
        elif mode == '4':
            mode_4(data)
    else:
        print("No data received.")

# 메인 코드
ser = initialize_serial()
if ser:
    mode = set_mode() 
    print(f"Mode set to: {mode}")

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