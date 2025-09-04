import socket
import numpy as np
import joblib
import re
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# --- Visualization Functions ---
def get_drawing(x, y, d):
    min_y, max_y = 1.0, 13.0
    min_x, max_x = 1.0, 6.0
    if not (min_x <= x <= max_x and min_y <= y <= max_y):
        return None
    drawings = [
        (1, 3, 5),
        (2, 8, 9),
        (3, 10, 11)
    ]
    d = d % 360
    heading_rad = np.deg2rad((-d) % 360)
    for num, y0, y1 in drawings:
        vec_top = np.array([0 - x, y1 - y])
        vec_bot = np.array([0 - x, y0 - y])
        angle_top = np.arctan2(vec_top[1], vec_top[0])
        angle_bot = np.arctan2(vec_bot[1], vec_bot[0])
        heading = ((heading_rad + np.pi) % (2 * np.pi)) - np.pi
        angle_min, angle_max = min(angle_top, angle_bot), max(angle_top, angle_bot)
        margin = np.deg2rad(2)
        if angle_min - margin <= heading <= angle_max + margin:
            return num
    return None

def show_map(x, y, d, drawing):
    if not hasattr(show_map, 'fig'):
        show_map.fig = plt.figure(figsize=(12, 6))
        show_map.ax = show_map.fig.add_subplot(121)
        show_map.ax2 = show_map.fig.add_subplot(122)
        show_map.ax.set_xlim(-1, 7)
        show_map.ax.set_ylim(0, 14)
        show_map.ax.set_aspect('equal')
        show_map.ax.set_title('Visitor Map')
        show_map.ax.set_xlabel('x (0 wall - 6)')
        show_map.ax.set_ylabel('y (0 bottom - 14 top)')
        for i in range(1, 7):
            show_map.ax.axvline(i, color='gray', linestyle='--', linewidth=0.5)
        for i in range(1, 14):
            show_map.ax.axhline(i, color='gray', linestyle='--', linewidth=0.5)
        show_map.ax.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5)
        show_map.ax.axvline(0, color='black', linewidth=2)
        show_map.wall_patches = [
            Rectangle((-0.2, 3), 0.2, 2, color='red', label='Drawing 1 (y=3-5)'),
            Rectangle((-0.2, 8), 0.2, 1, color='green', label='Drawing 2 (y=8-9)'),
            Rectangle((-0.2, 10), 0.2, 1, color='blue', label='Drawing 3 (y=10-11)')
        ]
        for patch in show_map.wall_patches:
            show_map.ax.add_patch(patch)
        show_map.ax.legend(loc='upper right')
        show_map.ax2.set_xlim(0, 1)
        show_map.ax2.set_ylim(0, 1)
        show_map.ax2.axis('off')
    # Remove previous visitor and arrows
    for artist in show_map.ax.artists[:]:
        artist.remove()
    for line in show_map.ax.lines[:]:
        line.remove()
    for arrow in show_map.ax.patches[len(show_map.wall_patches):]:
        arrow.remove()
    show_map.ax.plot(x, y, 'ko', markersize=10, label='Visitor')
    arrow_len = 0.5
    phi_rad = np.deg2rad((90 - d) % 360)
    dx = arrow_len * -np.sin(phi_rad)
    dy = arrow_len * np.cos(phi_rad)
    show_map.ax.arrow(x, y, dx, dy, head_width=0.2, head_length=0.3, fc='k', ec='k')
    show_map.ax2.clear()
    show_map.ax2.set_xlim(0, 1)
    show_map.ax2.set_ylim(0, 1)
    show_map.ax2.axis('off')
    if drawing is not None:
        color = ['red', 'green', 'blue'][drawing - 1]
        show_map.ax2.add_patch(Rectangle((0.2, 0.4), 0.6, 0.4, color=color))
        show_map.ax2.text(0.5, 0.5, f'Drawing {drawing}', ha='center', va='center', fontsize=20, color='white')
        show_map.ax2.text(0.5, 0.3, f'Name: Drawing {drawing}', ha='center', va='center', fontsize=12)
    else:
        show_map.ax2.text(0.5, 0.5, 'Not looking at any drawing', ha='center', va='center', fontsize=12)
    plt.draw()
    plt.pause(0.01)

plt.ion()

# --- Model and Server Setup ---
scaler = joblib.load('scaler.joblib')
knn = joblib.load('knn_model.joblib')
HOST = '0.0.0.0'
PORT = 8080
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print(f"Server listening on {HOST}:{PORT}")

try:
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")
        data = client_socket.recv(1024)
        if data:
            try:
                decoded_data = data.decode('utf-8')
                print(f"Received data: {decoded_data}")
                heading_match = re.search(r'Heading:\s*(-?\d+\.\d+|-?\d+)', decoded_data)
                heading = float(heading_match.group(1)) if heading_match else None
                rssi_values = {}
                for minor in [12, 14, 16, 18, 20]:
                    match = re.search(rf'{minor}:\s*(-?\d+)', decoded_data)
                    if match:
                        rssi_values[minor] = float(match.group(1))
                if len(rssi_values) == 5 and heading is not None:
                    b12 = rssi_values[12]
                    b14 = rssi_values[14]
                    b16 = rssi_values[16]
                    b18 = rssi_values[18]
                    b20 = rssi_values[20]
                    if -200 not in [b12, b14, b16, b18, b20]:
                        rssi = np.array([b12, b14, b16, b18, b20]).reshape(1, -1)
                        print(f"RSSI values: b12={b12}, b14={b14}, b16={b16}, b18={b18}, b20={b20}")
                        rssi_std = scaler.transform(rssi)
                        coord = knn.predict(rssi_std)
                        x, y = float(coord[0][0]), float(coord[0][1])
                        print(f"Predicted coordinate: x={x:.2f}, y={y:.2f}, heading={heading}")
                        drawing = get_drawing(x, y, heading)
                        if drawing:
                            print(f"The visitor is looking at drawing {drawing}.")
                        else:
                            print("The visitor is not looking at any drawing.")
                        show_map(x, y, heading, drawing)
                    else:
                        print("Invalid RSSI values (-200 detected). Waiting for next data...")
                else:
                    print("Received data does not have all required RSSI values or heading. Waiting for next data...")
            except Exception as e:
                print(f"Error processing data: {e}")
        client_socket.close()
except KeyboardInterrupt:
    print("\nServer stopped by user.")
finally:
    plt.ioff()
    plt.close()
    server_socket.close()
