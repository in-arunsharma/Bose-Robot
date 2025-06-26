## Visual SLAM Python Implementation

Este proyecto se centra en la implementación de un sistema de Visual SLAM monocular. Es una primera aproximacion del visual slam para el proyectio de RLP para el robot BOSE, inspirado en ORB-SLAM2, utilizando una cámara para extraer características, rastrear la trayectoria de la cámara y generar un mapa 3D del entorno.

### Estructura del proyecto

```
visual-slam/
├── configs/               # Parámetros de cámara y SLAM (ORB, matcher, tracking, mapping, I/O)
│   └── monocular.yaml     # Configuración principal (intrínsecos, distorsión, thresholds, rutas)
│
├── data/                  # Datos de entrada y salida
│   └── hospital_video.mp4 # Vídeo monocular de prueba para la ejecución del SLAM
│
├── src/
│   ├── orbslam2/          # Implementación del núcleo SLAM
│   │   ├── __init__.py    # Inicialización del paquete y versión
│   │   ├── extractor.py   # Extracción de características ORB
│   │   ├── initializer.py # Inicialización del mapa (primeros dos fotogramas)
│   │   ├── matcher.py     # Emparejamiento de descriptores ORB
│   │   ├── tracker.py     # Lógica de tracking frame-a-frame y pose estimation
│   │   ├── local_mapper.py# Creación y refinamiento de keyframes y puntos de mapa
│   │   └── utils.py       # Funciones auxiliares (config, calibración, triangulación)
│   └── tests/             # Tests unitarios con pytest
│       ├── test_matcher.py      # Validación de emparejamiento de descriptores
│       └── test_orb_extractor.py# Pruebas de extracción de características ORB
│
└── README.md              # Guía de uso, instalación y ejemplos de ejecución
```

### Descripción general

* **Visual SLAM**: Procesa un vídeo monocular para extraer puntos clave ORB, seguirlos en cada fotograma y estimar la posición de la cámara.
* **Mapeo 3D**: A partir de los fotogramas extraídos, triangula puntos y construye una nube de puntos 3D que se exporta en formato PLY.
* **Modularidad**: Cada componente (extracción, inicialización, matching, tracking, mapeo) está organizado en módulos claros para facilitar mantenimiento y extensión.

### Instalación

1. Clonar el repositorio:

   ```bash
   git clone https://github.com/tu_usuario/visual-slam.git
   cd visual-slam
   ```
2. Crear un entorno virtual y activar
3. Instalar dependencias

### Uso

1. Ajustar parámetros en `configs/monocular.yaml` (intrínsecos, thresholds, rutas de I/O).
2. Ejecutar el script principal (por ejemplo, `run_slam.py`):

   ```bash
   python run_slam.py --config configs/monocular.yaml
   ```
3. Visualizar el mapa:

   * El mapa 3D se guardará en `data/map.ply`.
   * Opcionalmente, utilizar un visor de nubes de puntos para inspeccionar el resultado.

---