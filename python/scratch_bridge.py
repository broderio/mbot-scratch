import asyncio
import websockets
import struct
import threading
import json
import math
import queue
import time
import subprocess

from serializer import *
from serial import Serial
from serial.tools.list_ports import comports

class Pose:
    def __init__(self):
        self.utime: int = 0
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.wz: float = 0.0
        self.stopYield: bool = False

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.previous_error: float = 0
        self.integral: float = 0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
    
    def reset(self) -> None:
        self.previous_error = 0
        self.integral = 0

class RobotState:
    def __init__(self):
        self.pose = Pose()
        self.alpha = 0.98
        self.prev_theta_imu: float = 0.0
        self.prev_utime: int = 0
        self.dt: float = 0.0

        self.at_goal: bool = True
        self.goal_type: str = "none"
        self.goal_val: float = 0.0
        self.lin_cmd: SerialTwist2D = SerialTwist2D()
        self.ang_cmd: SerialTwist2D = SerialTwist2D()
        self.lin_to_pid = PIDController(0.0, 0.0, 0.0)
        self.lin_at_pid = PIDController(0.2, 0.01, 0.0)
        self.ang_to_pid = PIDController(0.0, 0.0, 0.0)
        self.ang_at_pid = PIDController(0.2, 0.01, 0.0)
    
    def update(self, vel: SerialTwist2D, imu: SerialMbotImu) -> SerialTwist2D or None:
        # Check if this is the first update
        if (self.prev_utime == 0):
            self.prev_utime = vel.utime
            return None

        # Calculate pose in spacial frame
        theta_imu = imu.angles_rpy[2]
        self.dt = (vel.utime - self.prev_utime) / 1e6
        dtheta_imu = theta_imu - self.prev_theta_imu
        cos_theta = math.cos(self.pose.theta)
        sin_theta = math.sin(self.pose.theta)
        self.pose.vx = vel.vx * cos_theta - vel.vy * sin_theta
        self.pose.vy = vel.vx * sin_theta + vel.vy * cos_theta
        self.pose.wz = dtheta_imu / self.dt * self.alpha + vel.wz * (1 - self.alpha)

        self.pose.x += self.pose.vx * self.dt
        self.pose.y += self.pose.vy * self.dt
        self.pose.theta += self.pose.wz * self.dt

        # Wrap theta to [-pi, pi)
        self.pose.theta = (self.pose.theta + math.pi) % (2 * math.pi) - math.pi

        # Check if the robot is at the goal
        drive_cmd = None
        if (not self.at_goal):
            if (self.goal_type == "FT"):
                drive_cmd = self.forward_to()
            elif (self.goal_type == "BT"):
                drive_cmd = self.backward_to()
            elif (self.goal_type == "LT"):
                drive_cmd = self.left_to()
            elif (self.goal_type == "RT"):
                drive_cmd = self.right_to()
            elif (self.goal_type == "LIN"):
                drive_cmd = self.lin_at()
            elif (self.goal_type == "ANG"):
                drive_cmd = self.ang_at()
            elif (self.goal_type == "S"):
                drive_cmd = SerialTwist2D()
                self.reset()
                self.at_goal = True

        # Update previous values
        self.prev_theta_imu = theta_imu
        self.prev_utime = vel.utime

        return drive_cmd

    def reset(self) -> None:
        self.lin_cmd = SerialTwist2D()
        self.ang_cmd = SerialTwist2D()
        self.lin_at_pid.reset()
        self.ang_at_pid.reset()
        self.at_goal = True

    def set_goal(self, goal_type: str, goal_val: float) -> None:
        self.at_goal = False
        self.goal_type = goal_type
        self.goal_val = goal_val

    def forward_to(self) -> float or None:
        pass

    def backward_to(self) -> float or None:
        pass

    def left_to(self) -> float or None:
        pass

    def right_to(self) -> float or None:
        pass

    def lin_at(self) -> SerialTwist2D or None:
        error = self.goal_val - self.pose.vx
        ang_error = 0 - self.pose.wz
        print(self.lin_cmd.vx)
        if (math.fabs(error) < 0.005):
            self.reset()
            return None
        self.lin_cmd.vx += self.lin_at_pid.update(error, self.dt)
        self.lin_cmd.wz += self.ang_at_pid.update(ang_error, self.dt)
        return self.lin_cmd

    def ang_at(self) -> SerialTwist2D or None:
        error = self.goal_val - self.pose.wz
        lin_error = 0 - self.pose.vx
        print(error)
        if (math.fabs(error) < 0.02):
            self.reset()
            return None
        self.ang_cmd.wz += self.ang_at_pid.update(error, self.dt)
        self.ang_cmd.vx += self.lin_at_pid.update(lin_error, self.dt)
        return self.ang_cmd
    
class Robot:
    count = 0
    ser = None
    def __init__(self):
        self.assigned = False
        self.id = Robot.count
        self.websocket = None
        self.state = RobotState()
        Robot.count += 1
    
    def __del__(self):
        Robot.count -= 1

robots = {}
serial_mtx = threading.Lock()
socket_queues = {}
serial_queue = queue.Queue(-1)

def get_robots(filename: str) -> dict[str, Robot]:
    robots = {}
    with open(filename) as f:
        for line in f:
            mac_address = line.strip()
            if (mac_address[0] == '#'):
                continue
            robots[mac_address] = Robot()
            socket_queues[mac_address] = queue.Queue()
    return robots

def pair_robots(ser: Serial) -> None:
    mac_file = open('macs.txt', 'r')
    macs = mac_file.readlines()
    mac_file.close()
    macs = [mac.strip() for mac in macs]
    for mac in macs:
        data_ser = SerialTwist2D()
        out_bytes = encode_cmd_packet(encode_vel_cmd(data_ser), mac)
        ser.write(out_bytes)


def get_packet_serial(ser: Serial) -> (str, PacketsWrapper) or None:
    # Wait for the start of a packet
    chunk_size = 100
    trigger = ser.read(chunk_size)
    start_index = trigger.find(b'\xff')
    if start_index == -1:
        return None

    # Read the rest of the data
    packet_length = 2 + 6 + 204 + 1
    bytes_left = packet_length - (chunk_size - start_index - 1)
    data = trigger[start_index + 1:] + ser.read(bytes_left)

    length = data[:2]
    mac = data[2:8]
    packet = data[8:212]
    cs_bytes = data[212:213]

    # Verify length
    length_int = struct.unpack('<H', length)[0]
    if length_int != 204:
        return None
    
    # Verify checksum
    cs_int = struct.unpack('B', cs_bytes)[0]
    cs_calc = checksum(packet)
    if cs_int != cs_calc:
        return None 
    
    # Deserialize packet
    packets = PacketsWrapper()
    packets.decode(packet)

    # Return reassembled packet
    return mac, packets

def serial_thread(ser: Serial) -> None:
    while True:
        data = get_packet_serial(ser)
        if (data):
            mac, packets = data
            mac_str = mac_bytes_to_str(mac)
            robot = robots[mac_str]
            data_ser = robot.state.update(packets.mbot_vel, packets.imu)
            
            if (robot.assigned):
                socket_queues[mac_str].put(robot.state.pose.__dict__)
            
            if (data_ser):
                out_bytes = encode_cmd_packet(encode_vel_cmd(data_ser), mac_str)
                ser.write(out_bytes)
            
def socket_thread(ser: Serial) -> None:
    while True:
        if (not serial_queue.empty()):
            mac, msg = serial_queue.get()
            robot = robots[mac]
            if robot.assigned:
                robot.state.pose.stopYield = False
                scratch_msg = json.loads(msg)
                print(scratch_msg)
                robot.state.reset()
                robot.state.set_goal(scratch_msg['type'], scratch_msg['val'])

async def handler(websocket: websockets.WebSocketServerProtocol, path: str) -> None:
    print(f"New client connected")

    global robots
    
    mac, robot = next(((mac, robot) for mac, robot in robots.items() if not robot.assigned), (None, None))
    if robot:
        print(f"Robot {robot.id} assigned")
        robot.assigned = True
        robot.websocket = websocket
        await websocket.send(f"Robot {robot.id} assigned to you.")
    else:
        await websocket.send("No robots available.")
        print(f"No robots available, disconnecting client.")
        return
    
    try:
        while True:
            if (not socket_queues[mac].empty()):
                await robot.websocket.send(json.dumps(socket_queues[mac].get()))
            try:
                msg = await asyncio.wait_for(robot.websocket.recv(), timeout=0.001)
                # print(msg)
                serial_queue.put((mac, msg))
            except asyncio.TimeoutError:
                # print(f"Robot {robot.id} socket queue size: {socket_queues[mac].qsize()}")
                continue
    except websockets.exceptions.ConnectionClosed:
        print(f"Robot {robot.id} freed")
        robot.assigned = False
        robot.websocket = None

async def run_in_executor(func, *args):
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, asyncio.run, func(*args))

def main() -> None:
    # Print the server address
    result = subprocess.run(["ipconfig", "getifaddr", "en0"], capture_output=True, text=True)
    print(f"http://{result.stdout.strip()}:8601")

    # Find the COM port for the ESP32-S3
    ports = comports()
    for port, _, hwid in ports:
        if '303A:1001' in hwid or '10C4:EA60' in hwid or '303A:4001' in hwid:
            port_name = port
            break
    else:
        print('ESP32-S3 COM port not found')
        return

    # OPen the serial port
    ser = Serial(port_name, 921600)

    # Get and pair the robots from the file
    global robots
    robots = get_robots("macs.txt")
    print("Robots:")
    for mac in robots.keys():
        print(f"\t{mac}")
    pair_robots(ser)

    # Start the server
    start_server = websockets.serve(handler, "0.0.0.0", 8765)
    loop = asyncio.get_event_loop()
    server = loop.run_until_complete(start_server)

    # Create and start threads for serial_thread and socket_thread
    serial_handle = threading.Thread(target=serial_thread, args=(ser,), daemon=True)
    socket_handle = threading.Thread(target=socket_thread, args=(ser,), daemon=True)
    serial_handle.start()
    socket_handle.start()

    try:
        loop.run_forever()
    finally:
        server.close()
        loop.run_until_complete(server.wait_closed())
        loop.close()

    # Join the threads to the main thread
    serial_handle.join()
    socket_handle.join()

if __name__ == "__main__":
    main()