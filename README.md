# Sistema-para-mapeo-de-interiores-y-asistencia-en-la-navegacion-mediante-TurtleBot-4-TFG

# Preparación del entorno y ejecución de la simulación

## Requisitos previos

Antes de ejecutar el sistema es necesario disponer de:

- Ubuntu 24.04
- ROS 2 Jazzy instalado correctamente
- Gazebo Harmonic instalado
- Dependencias de TurtleBot 4 y Nav2 instaladas
- Dependencias para la detección visual

El sistema de detección basado en YOLO requiere la instalación de las siguientes dependencias de Python:
```bash
sudo pip3 install "numpy<2.0.0" "opencv-python<4.10" ultralytics
```
Estas restricciones de versión son necesarias para evitar problemas de compatibilidad entre las librerías `ultralytics`, `opencv-python` y `numpy`.

---

## Preparación del entorno ROS 2

Es necesario añadir las siguientes líneas al archivo `~/.bashrc`:
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1
```

Para utilizar la GPU durante el renderizado de Gazebo Harmonic y mejorar el rendimiento de la simulación, se puede añadir la siguiente variable al archivo `~/.bashrc` habiendo configurado antes su uso:
```bash
export GZ_RENDER_ENGINE=ogre2
```
En cada nueva terminal es necesario cargar el entorno modificando el nombre del espacio de trabajo:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```
---

## Ejecución completa

### Terminal 1

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 2

```bash
ros2 launch turtlebot4_guidance_system sistema_guiado.launch.py
```
Este lanzamiento inicializa automáticamente:

- Gazebo Harmonic.
- El modelo del TurtleBot 4 Lite.
- El sistema de localización AMCL.
- La pila de navegación Nav2.
- La detección visual basada en YOLO.
- RViz2.
- Visualización de las imágenes de la cámara y la detección con `rqt_image_view`
- La interfaz gráfica del sistema de guiado.

---
