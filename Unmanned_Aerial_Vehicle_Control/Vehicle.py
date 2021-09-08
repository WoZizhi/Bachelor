import os
import math
import time
import socket
import picamera
import threading
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)
#vehicle = connect('127.0.0.1:14550',wait_ready=True,baud=921600)

IP_vehicle = '10.10.10.10'  # 3070 无线网卡所设置的 ip
#IP_vehicle = '192.168.1.127' 
Port_vehicle = 6000
IP_ground = '10.10.10.6'
#IP_ground = '192.168.1.128'
Port_ground1 = 7000
Port_ground2 = 8000
Port_video = 8080
Buffer = 4096

vehicle.mode = VehicleMode("GUIDED_NOGPS")  # 用于进行室内无 GPS 信号下的控制
vehicle.armed = True
DEFAULT_TAKEOFF_THRUST = 0.7
SMOOTH_TAKEOFF_THRUST = 0.6

r, w = os.pipe()

udp_vehicle_client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

udp_vehicle_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_vehicle_server.bind((IP_vehicle,Port_vehicle))

# 发送对应 MAVLink 消息至 Pixhawk
def send_attitude_target(roll_angle=None,pitch_angle=None,yaw_angle=None,thrust=0.5):
    if roll_angle is None:
        roll_angle = vehicle.attitude.roll
    if pitch_angle is None:
        pitch_angle = vehicle.attitude.pitch
    if yaw_angle is None:
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        0, # Target system
        0, # Target component
        0b00000111, # ignore rate
        to_quaternion(roll_angle,pitch_angle,yaw_angle), # attitude Quaternion
        0, # Body roll rate in radian(if use rate rather than angle)
        0, # Body pitch rate in radian(if use rate rather than angle)
        0, # Body yaw rate in radian(if use rate rather than angle)
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# 姿态保持(默认实时改变, duration = 0)
def set_attitude(roll_angle=None,pitch_angle=None,yaw_angle=None,thrust=0.5,duration=0):
    send_attitude_target(roll_angle,pitch_angle,yaw_angle,thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle,pitch_angle,yaw_angle,thrust)
        time.sleep(0.1)
    
def to_quaternion(roll=0.0,pitch=0.0,yaw=0.0):
    # 角度转弧度
    t0 = math.cos(math.radians(yaw*0.5))
    t1 = math.sin(math.radians(yaw*0.5))
    t2 = math.cos(math.radians(roll*0.5))
    t3 = math.sin(math.radians(roll*0.5))
    t4 = math.cos(math.radians(pitch*0.5))
    t5 = math.sin(math.radians(pitch*0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# 起飞前完成对应的检查
def arm_check():
    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)
 
# 一键起飞模式
# 起飞至预设高度(气压计检测)--千万不要在地下室尝试此功能，因为会一直往上飞。。。
def takeoff_nogps(aTargetAltitude):
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    arm_check()
    print("Taking off!")
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" % (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)
        time.sleep(0.2)
        
# 横滚控制模式
def roll_control(data):
    arm_check()
    if data == "a":
        print("Move left")
        set_attitude(roll_angle=5)
    if data == "d":
        print("Move right")
        set_attitude(roll_angle=-5)
        
# 俯仰控制模式
def pitch_control(data):
    arm_check()
    if data == "w":
        print("Move forward")
        set_attitude(pitch_angle=5)
    if data == "s":
        print("Move backward")
        set_attitude(pitch_angle=-5)
    
# 偏航控制模式(暂未开发，目前所用为无头模式，方便初学者操控)
def yaw_control(data):
    arm_check()
    pass

# 上升
def up_control():
    print("up!")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    arm_check()
    set_attitude(thrust=0.7)

# 下降
def down_control():
    print("down!")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    arm_check()
    set_attitude(thrust=0.3)
    
# 着陆
def land_control():
    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")

# 传输视频流(通过树莓派摄像头)
def vehicle_video_send(w):
    w = os.fdopen(w, "wb")
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 280)
        camera.framerate = 25
        camera.vflip = True
        #camera.hflip = Ture # 是否进行水平翻转 
        # 开启摄像头，并预热两秒
        camera.start_preview()
        time.sleep(2)
        # 开始录制并传输
        camera.start_recording(w, format='h264')
        camera.wait_recording(3600)
        print("end")
    
# 这里之所以还要设置一个管道读取函数，是因为树莓派摄像头传输需要一个管道介质
def pipe_read(r):
    r = os.fdopen(r, "rb")
    print("start")
    while True:
        data = r.read(Buffer)
        udp_vehicle_client.sendto(data, (IP_ground, Port_video))
        
# 消息发送函数，设定对应的头标识
def vehicle_message_send():
    while True:
        Battery = str(vehicle.battery).partition('level=')[2] # return '100' 
        Altitude = str(vehicle.location.global_relative_frame).partition('alt=')[2] # return '0.0' 
        Attitude_roll = str(vehicle.attitude).partition('roll=')[2] # return '0.123' 
        Attitude_pitch = str(vehicle.attitude).partition('yaw=')[0].partition('pitch=')[2] # return '0.456' 
        Attitude_yaw = str(vehicle.attitude).partition('roll=')[0].partition('yaw=')[2] # return '0.789' 
        Ground_speed = str(vehicle.groundspeed) # return '1'
        Air_speed = str(vehicle.airspeed) # return '1'
        Velocity = vehicle.velocity # return [1, 1, 1] (vx,vy,vz)
        Velocity.insert(0,'j')
        Velocity.insert(2,'q')
        Velocity.insert(4,'k')
        Velocity_temp = [str(i) for i in Velocity]
        Velocity_str = ''.join(Velocity_temp) # return 'j1q2k3' (jvx,qvy,kvz)
        data_send1 = 'b'+Battery+'a'+Altitude+'r'+Attitude_roll+'p'+Attitude_pitch+'y'+Attitude_yaw
        data_send2 = 'c'+Ground_speed+'d'+Air_speed+Velocity_str
        data_send = data_send1+data_send2
        udp_vehicle_client.sendto(data_send.encode(),(IP_ground,Port_ground1))
        
# 消息接收函数，根据地面端发送的不同指令执行对应的飞行姿态
def vehicle_recv():
    while True:      
        data_origin = udp_vehicle_server.recvfrom(Buffer)[0]
        data = str(data_origin).partition("b'")[2].partition("'")[0]
        print('get '+data)
        if(data == "w" or data == "s"):
            pitch_control(data)
        elif(data == "a" or data == "d"):
            roll_control(data)
        elif(data == "Up"):
            up_control()
        elif(data == "Down"):
            down_control()
        elif(data == "q"):
            takeoff_nogps(1.5)
        elif(data == "l"):
            land_control()

# 设定四个线程，分别执行消息发送、消息接收、视频流拍摄(管道写入)与视频流传输(管道读取)
threading.Thread(target=vehicle_message_send).start()
threading.Thread(target=vehicle_recv).start()
threading.Thread(target=vehicle_video_send, args=(w,)).start()
threading.Thread(target=pipe_read, args=(r,)).start()
