# Exploración Autónoma 🧭

Este script busca que BOSE, equipado con un **RPLiDAR C1 – 360°**, explorar de forma autónoma y segura entornos potencialmente peligrosos (zona de rescate, desastres, entornos de riesgo, etc.).

- Realiza un **mapeado 2D en tiempo real** del entorno.  
- Se mueve tipo "Roomba": avanza, detecta obstáculos y cambia de rumbo.  
- Genera una visualización gráfica del mapa y del recorrido del robot.  
- Se basa en el controlador del motor MD25 y el sistema de LIDAR.

---

## Requisitos

- Python 3  
- Paquetes necesarios:
  ```bash
  pip3 install smbus2 numpy matplotlib rplidar