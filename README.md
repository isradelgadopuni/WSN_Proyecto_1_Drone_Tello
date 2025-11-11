# 🛰️ WSN Proyecto 1 – Drone Tello (ROS2 Jazzy)

Proyecto modular en **ROS2 Jazzy** para el **control, planeación de misión y monitoreo** de un **drone DJI Tello**.  
El sistema se organiza en múltiples nodos independientes que se comunican entre sí mediante tópicos ROS2, permitiendo la gestión integral del dron: conexión, visión, telemetría, seguridad y autonomía.  
Además, incluye integración con **Prometheus + Grafana** para la **exportación y visualización de métricas** en tiempo real (batería, altura, velocidad, detección de objetos, entre otras).


---

## 📦 Estructura general (`src/`)

Cada paquete es un nodo ROS2 autónomo, con sus propios `setup.py`, `package.xml` y pruebas básicas.

```
src/
├── battery_failsafe/      # Nodo de seguridad ante fallos de batería
├── driver_tello/          # Nodo principal de control del dron (interfaz Tello SDK)
├── drone_connector/       # Nodo de conexión y comunicación entre módulos
├── mission_planner/       # Nodo de planificación de misión y manejo de waypoints
├── telemetry_monitor/     # Nodo de monitoreo de telemetría y estado del dron
├── video_viewer/          # Nodo para visualización del stream de video
└── vision_detector/       # Nodo de detección de objetos (visión por computadora)
└── exporter_node/         # Nodo de exportación hacia Prometheus
```

> 💡 **Nota:**  
> Los nombres de carpetas, paths locales y contenedor (`wsn_tello_proyecto1`) pueden modificarse según tu entorno.  
> Este README usa las rutas y nombres de ejemplo del proyecto del autor.

---

## 🧩 Descripción de nodos principales

| Nodo | Paquete | Función |
|------|---------|---------|
| **0. driver_node** | `driver_tello` | Interfaz principal con el dron DJI Tello mediante SDK. Inicia la comunicación base y publica/escucha en tópicos comunes. |
| **1. drone_connector** | `drone_connector` | Coordina la comunicación entre módulos y maneja mensajes ROS entre el driver y los demás nodos. |
| **2. video_viewer** | `video_viewer` | Procesa y muestra el stream de video proveniente del dron. |
| **3. telemetry_monitor** | `telemetry_monitor` | Supervisa datos de vuelo, batería, posición y estado general. |
| **4. battery_failsafe** | `battery_failsafe` | Gestiona condiciones críticas de energía y activa protocolos de emergencia. |
| **5. mission_planner** | `mission_planner` | Ejecuta misiones automáticas, rutas o comportamientos predefinidos. |
| **6. object_detector** | `vision_detector` | Aplica detección de objetos en tiempo real sobre el stream de video. |
| **7. prom_exporter** | `tello_prom_exporter` | **Exporta métricas a Prometheus** leyendo tópicos ROS2 y expone `:8000/metrics`. Métricas: `objects_red_count`, `objects_black_count`, `tello_battery_percent`, `tello_height_raw`, `tello_velocity_{x,y,z}_mps`, `tello_speed_norm_mps`. Parámetros: `drone_id`, `qos_mode` (`best_effort`/`reliable`), y nombres de tópicos (`/objects/red_count`, `/objects/black_count`, `/tello/{battery,height,velocity}`). |

> 🔗 Todos los nodos se comunican bajo ROS2.  
> El nodo `driver_node` (NODO 0) **debe iniciarse primero**, ya que los demás dependen de los tópicos que este publica.

---

## 🐳 Uso con Docker

### 1️⃣ Construir la imagen
```bash
docker build -t wsn_proyecto1_img:proyecto .
```

### 2️⃣ Crear el contenedor
```bash
docker run -it \
  --name wsn_tello_proyecto1 \
  --hostname host_tello_proyecto1 \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=/tmp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/Documentos/WSN/Proyecto_1/proyecto1_ws:/root/ros2_ws:rw" \
  wsn_proyecto1_img:proyecto
```

### 3️⃣ Dentro del contenedor – preparar entorno
```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Ejecución de nodos

Cada nodo puede iniciarse en una terminal diferente mediante `docker exec`.  
Todos los comandos asumen que el contenedor activo se llama **`wsn_tello_proyecto1`**.

---

#### 🟢 NODO 0 – `driver_tello`
*(debe iniciarse primero)*
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run driver_tello driver_node"
```

#### 🟠 NODO 1 – `drone_connector`
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run drone_connector drone_connector"
```

#### 🟣 NODO 2 – `video_viewer`
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run video_viewer video_viewer"
```

#### 🔵 NODO 3 – `telemetry_monitor`
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run telemetry_monitor monitor_node"
```

#### 🟡 NODO 4 – `battery_failsafe`
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run battery_failsafe battery_failsafe_node"
```

#### 🟤 NODO 5 – `mission_planner`
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run mission_planner mission_planner_node"
```

#### ⚫ NODO 6 – `object_detector`
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run object_detector object_detector"
```

#### ⚪ NODO 7 – `tello_prom_exporter`
```bash
docker exec -it wsn_tello_proyecto1 bash -lc \
"source /opt/ros/jazzy/setup.bash && \
 source /root/ros2_ws/install/setup.bash && \
 ros2 run exporter_node tello_prom_exporter"
```

---

## 🎥 Control del stream de video
Para activar o desactivar el stream del Tello:

```bash
ros2 service call /tello/stream_on std_srvs/srv/Trigger {}
ros2 service call /tello/stream_off std_srvs/srv/Trigger {}
```

---

## ⚙️ Dependencias principales
- **ROS2 Jazzy**
- **colcon** (para build del workspace)
- **djitellopy / OpenCV / numpy** *(según implementación de visión)*
- **Docker 24+** y **X11** habilitado (para visualización de video)

---

## 🧠 Notas de operación
- Ejecutar siempre primero el **NODO 0 (`driver_tello`)**.  
- Si un nodo dependiente no encuentra un tópico activo, mostrará **alertas o warnings** indicando qué componente falta.  
- Todos los nodos fueron diseñados para funcionar **de forma independiente** y modular.

---

## 👨‍💻 Autor
**Israel Delgado - Anthony Domínguez - Sebastián Guazhima**  
Universidad de Cuenca – 2025  
Facultad de Ingeniería – Proyecto de Redes de Sensores Inalámbricos (WSN)

---

© 2025 Israel Delgado - Anthony Domínguez - Sebastián Guazhima. Todos los derechos reservados.
