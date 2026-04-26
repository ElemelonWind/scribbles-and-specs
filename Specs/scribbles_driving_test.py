import socket
import time

host = '192.168.4.1'
port = 3333

heading = 0
x_start = 0
y_start = 200

def send_location(sock, heading, x, y):
    """Send a location (heading and position) to the device."""
    payload = bytearray(3)
    payload[0] = heading.to_bytes(1, 'big')[0]
    payload[1] = x.to_bytes(1, 'big')[0]
    payload[2] = y.to_bytes(1, 'big')[0]
    sock.sendall(payload)
    print(f"Sent position {x} {y}")

def send_waypoint(sock, x_goal, y_goal):
    """Send a waypoint goal to the device."""
    payload = bytearray()
    payload.append(1 << 7)
    payload.append(y_goal)
    payload.append(x_goal)
    print(payload)
    sock.sendall(payload)
    print(f"Sent goal {x_goal} {y_goal}")

with socket.create_connection((host, port), timeout=5) as sock:
    print("Connected to Scribbles")
    
    # Send initial location
    send_location(sock, heading, x_start, y_start)
    
    # First waypoint
    x_goal_1 = 200
    y_goal_1 = 200
    send_waypoint(sock, x_goal_1, y_goal_1)
    time.sleep(2)
    
    # Send positions towards first waypoint
    for x in range(x_start, x_goal_1 + 1, 50):
        send_location(sock, heading, x, y_start)
        time.sleep(1)
    
    # Second waypoint
    x_goal_2 = 100
    y_goal_2 = 100
    send_waypoint(sock, x_goal_2, y_goal_2)
    time.sleep(2)
    
    # Send positions towards second waypoint
    for x in range(x_goal_1, x_goal_2 - 1, -50):
        send_location(sock, heading, x, y_start)
        time.sleep(1)