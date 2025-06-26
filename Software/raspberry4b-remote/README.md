# Manual Control Script

Este script (`manual_control.py`) permite manejar a BOSE en Raspberry Pi 4 con:

1. **Tracción principal** (MD25) con teclas W/A/S/D  
2. **Ruedas retráctiles** (L298N) con flechas ↑/↓  
3. **Servos** (PCA9685) con O (subir) / L (bajar)  
4. **Salir** con E o Ctrl+C (detiene todo y limpia GPIO/I²C)

---

## Requisitos

- Python 3  
- Módulos:
  ```bash
  pip3 install smbus2 RPi.GPIO adafruit-circuitpython-pca9685 inputs