from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import signal
import sys

PORT = '/dev/ttyUSB0'

lidar = RPLidar(PORT)

def signal_handler(sig, frame):
    print('\nParando escaneo...')
    lidar.stop()
    lidar.disconnect()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

print("Iniciando escaneo con RPLIDAR...")

fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')
scan_data = np.zeros(360)

# Actualizar los datos del escáner
def update():
    global scan_data
    for scan in lidar.iter_measurments():
        angle = int(scan[2])
        distance = scan[3]
        if 0 <= angle < 360:
            scan_data[angle] = distance

        if angle == 359:
            ax.clear()
            ax.set_theta_zero_location('top')
            ax.set_theta_direction(-1)
            angles = np.radians(np.arange(360))
            distances = scan_data
            ax.plot(angles, distances, '.')
            ax.set_rmax(3000)  # hasta 3 metros
            plt.pause(0.001)

try:
    update()
except KeyboardInterrupt:
    signal_handler(None, None)
