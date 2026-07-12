# Sistema-para-mapeo-de-interiores-y-asistencia-en-la-navegacion-mediante-TurtleBot-4-TFG
Repositorio del proyecto de desarrollo e integración de un sistema autónomo de navegación y percepción visual para el TurtleBot 4 Lite. Incluye el entorno virtual, el sistema de guiado e interfaz de usuario, el nodo de detección mediante YOLO y los modelos entrenados para la detección de señales para la integración total en simulación y configuraciones adicionales para un despliegue parcial en el robot real.
## Estructura del repositorio
### `turtlebot4_guidance_system`

Paquete principal del proyecto. Implementa el sistema de guiado autónomo y contiene:

- Lanzadores (`launch`) del sistema completo.
- Configuración de navegación (`Nav2`, `AMCL`, mapas, RViz, etc.).
- Interfaz gráfica desarrollada para la selección de destinos.
- Lógica de navegación y guiado del robot.
---

### `yolobot`

Paquete encargado del sistema de percepción visual basado en YOLO. Contiene:

- Nodo `yolo_detector`.
- Modelo entrenado (`.pt`).
---

### `YOLO_training`

Directorio que contiene todo el código y recursos utilizados durante el entrenamiento de los modelos de detección:
El modelo finalmente utilizado en el proyecto fue sacado del entrenamiento mediante este archivo:
```text
YOLO_training/entrenamiento_final.py
```
---

### `real_robot`

Directorio con algunas configuraciones sobre el TurtleBot 4 Lite real y códigos utilizados. Incluye:

- `robot_guia_beep.py`, encargado de integrar señales acústicas con el sistema de navegación y guiado para asistir al usuario.
- Configuraciones específicas necesarias para el funcionamiento sobre el robot físico.
- Mapas realizados mediante SLAM de localizaciones reales.

---
## Preparación del entorno y ejecución de la simulación

### Requisitos previos

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

### Compilación del proyecto

Tras clonar el repositorio, es necesario compilar los paquetes ROS2:

```bash
cd ~/ros2_ws

colcon build --packages-select \
turtlebot4_guidance_system \
yolobot \
--symlink-install
```
### Preparación del entorno ROS 2

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

### Ejecución completa

# Terminal 1

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

#### Terminal 2

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

  
<img width="1848" height="1046" alt="cerradaportada" src="https://github.com/user-attachments/assets/369e1d99-9f74-402a-ab4d-4535b9b05183" />

---
