#!/usr/bin/env python3
import time
import smbus2
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
from inputs import get_key

# === MD25 Motor Controller ===
MD25_ADDR    = 0x58
REG_SPEED1   = 0
REG_SPEED2   = 1
MD25_NEUTRAL = 128
SPEED_PCT    = 50

bus = smbus2.SMBus(1)

def map_speed(pct):
    """Convierte porcentaje [-100..100] a valor MD25 [1..255]"""
    return int(MD25_NEUTRAL + (pct * 127 / 100))

def set_md25(left_pct, right_pct):
    """Envía velocidad a MD25 en % para cada rueda"""
    l = map_speed(left_pct)
    r = map_speed(right_pct)
    bus.write_byte_data(MD25_ADDR, REG_SPEED1, l)
    bus.write_byte_data(MD25_ADDR, REG_SPEED2, r)

# === Retractable Wheels (L298N) ===
GPIO.setmode(GPIO.BCM)
MOTOR_PINS = {
    'EN_A': 13, 'IN1': 6,  'IN2': 5,
    'EN_B': 12, 'IN3': 16, 'IN4': 26
}
for pin in MOTOR_PINS.values():
    GPIO.setup(pin, GPIO.OUT)

pwmA = GPIO.PWM(MOTOR_PINS['EN_A'], 100)
pwmB = GPIO.PWM(MOTOR_PINS['EN_B'], 100)
pwmA.start(0)
pwmB.start(0)
RETRACT_SPEED = 30  # % duty cycle

def set_retract(direction):
    if direction == 'forward':
        GPIO.output(MOTOR_PINS['IN1'], GPIO.HIGH)
        GPIO.output(MOTOR_PINS['IN2'], GPIO.LOW)
        GPIO.output(MOTOR_PINS['IN3'], GPIO.HIGH)
        GPIO.output(MOTOR_PINS['IN4'], GPIO.LOW)
        pwmA.ChangeDutyCycle(RETRACT_SPEED)
        pwmB.ChangeDutyCycle(RETRACT_SPEED)
    elif direction == 'backward':
        GPIO.output(MOTOR_PINS['IN1'], GPIO.LOW)
        GPIO.output(MOTOR_PINS['IN2'], GPIO.HIGH)
        GPIO.output(MOTOR_PINS['IN3'], GPIO.LOW)
        GPIO.output(MOTOR_PINS['IN4'], GPIO.HIGH)
        pwmA.ChangeDutyCycle(RETRACT_SPEED)
        pwmB.ChangeDutyCycle(RETRACT_SPEED)

def stop_retract():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

# === Servos (PCA9685) ===
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
SERVO_UP   = 500  # 0°
SERVO_DOWN = 150  # 180°

def move_servos(pulse):
    pca.channels[0].duty_cycle = pulse * 16
    pca.channels[8].duty_cycle = pulse * 16

# === MD25 Initialization ===
# modo 0 = control por registro, comando 50 = reset encoder
bus.write_byte_data(MD25_ADDR, 15, 0)
bus.write_byte_data(MD25_ADDR, 16, 50)
print("MD25 Initialized")

print("  Control manual:")
print("  WASD: mover MD25")
print("  Flechas: ruedas retráctiles")
print("  O/L: subir/bajar servos")
print("  E: salir y parar todo")

REFRESH_RATE = 0.03
pressed = set()

try:
    while True:
        events = get_key()
        for e in events:
            if e.ev_type == "Key":
                if e.state == 1:
                    pressed.add(e.code)
                else:
                    pressed.discard(e.code)

        # — MD25 principal (W/A/S/D) —
        if 'KEY_W' in pressed:
            set_md25(SPEED_PCT, SPEED_PCT)
        elif 'KEY_S' in pressed:
            set_md25(-SPEED_PCT, -SPEED_PCT)
        elif 'KEY_A' in pressed:
            set_md25(-SPEED_PCT, SPEED_PCT)
        elif 'KEY_D' in pressed:
            set_md25(SPEED_PCT, -SPEED_PCT)
        else:
            set_md25(0, 0)

        # — Ruedas retráctiles (flechas) —
        if 'KEY_UP' in pressed:
            set_retract('forward')
        elif 'KEY_DOWN' in pressed:
            set_retract('backward')
        else:
            stop_retract()

        # — Servos (O/L) —
        if 'KEY_O' in pressed:
            move_servos(SERVO_UP)
        elif 'KEY_L' in pressed:
            move_servos(SERVO_DOWN)

        # — Exit (E) —
        if 'KEY_E' in pressed:
            break

        time.sleep(REFRESH_RATE)

except KeyboardInterrupt:
    pass

finally:
    print("Parando todo y limpiando recursos...")
    set_md25(0, 0)
    stop_retract()
    GPIO.cleanup()
    pca.deinit()
    bus.close()
    print("Listo.")
