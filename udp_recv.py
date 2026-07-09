import socket
import numpy as np

UDP_PORT = 5005

ROWS = 6
COLS = 20

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))

print(f"Listening on UDP port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(65535)

    if len(data) != ROWS * COLS * 4:
        print(f"Bad packet size: {len(data)} bytes")
        continue

    values = np.frombuffer(data, dtype=">f4")

    matrix_6x20 = values.reshape((ROWS, COLS))

    # Convert to 20 rows x 6 columns
    matrix_20x6 = matrix_6x20.T

    np.set_printoptions(precision=5, suppress=True, linewidth=200)

    print("shape:", matrix_20x6.shape)
    print(matrix_20x6)
    print("-" * 80)