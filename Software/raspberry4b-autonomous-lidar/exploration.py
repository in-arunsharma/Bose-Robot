#!/usr/bin/env python3
import smbus2, time, random
from rplidar import RPLidar
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread

# Parámetros MD25
MD25_ADDR = 0x58
REG_L = 0
REG_R = 1
NEUTRAL = 128
MAX_SPEED = 200

bus = smbus2.SMBus(1)

def map_speed(pct):
    return int(NEUTRAL + (pct * (127/100)))

def set_speed(l_pct, r_pct):
    bus.write_byte_data(MD25_ADDR, REG_L, map_speed(l_pct))
    bus.write_byte_data(MD25_ADDR, REG_R, map_speed(r_pct))

# Exploración con LIDAR
PORT = '/dev/ttyUSB0'
lidar = RPLidar(PORT)
lidar.connect()
lidar.start_motor()
lidar.start()

GRID_SIZE = 200    # celdas
RES = 0.05         # 5 cm por celda
grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
pos = np.array([GRID_SIZE//2, GRID_SIZE//2])
angle_robot = 0

plt.ion()
fig, ax = plt.subplots(figsize=(6,6))

def draw_map():
    ax.clear()
    ax.imshow(grid, cmap='Greys', origin='lower')
    ax.plot(pos[0], pos[1], 'ro')
    plt.pause(0.01)

def exploration():
    global pos, angle_robot

    for scan in lidar.iter_scans():
        # 1) mapeo
        for _, ang, dist in scan:
            if dist == 0: continue
            rad = np.deg2rad(ang + angle_robot)
            d = dist / 1000.0
            dx = int(d * np.cos(rad)/RES)
            dy = int(d * np.sin(rad)/RES)
            x = pos[0] + dx
            y = pos[1] + dy
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                grid[y, x] = 1

        # 2) frente libre?
        front = [d for _, ang, d in scan if abs((ang-0)%360) < 20]
        minf = min(front) if front else 1000
        if minf > 500:
            set_speed(50, 50)
            # avanzar
            pos += np.array([np.cos(np.deg2rad(angle_robot)),
                             np.sin(np.deg2rad(angle_robot))])
        else:
            set_speed(0, 0)
            # girar 90° aleatorio
            turn = random.choice([90, -90])
            angle_robot = (angle_robot + turn) % 360
            set_speed(0,0)
            time.sleep(0.5)

        draw_map()

def main():
    try:
        exploration()
    except KeyboardInterrupt:
        pass
    finally:
        set_speed(0, 0)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        bus.close()
        print("Exploración finalizada.")

if __name__=='__main__':
    main()