# Robot diferencial en Gazebo — ROS 2 Jazzy

Proyecto de la asignatura Robótica, Universidad EIA. Se desarrolla un robot móvil diferencial simulado en Gazebo y conectado con ROS 2. Incluye la descripción física del robot, dos ruedas motrices y una rueda de apoyo (caster), un sensor LiDAR, una cámara, un entorno de pruebas propio y un nodo de estimación de odometría para comparar resultados con Gazebo.

## 1. Objetivo

Modelar y simular un robot diferencial que pueda desplazarse mediante teleoperación, recibir datos de sus sensores y calcular su pose a partir de las velocidades angulares de las ruedas. La estimación propia se compara con la odometría publicada por el plugin `DiffDrive`.

## 2. Requisitos

Entorno de referencia:

- Ubuntu 24.04.
- ROS 2 Jazzy.
- Gazebo Sim compatible con ROS 2 Jazzy.
- `colcon`, `xacro`, `robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge` y `rviz2`.
- `teleop_twist_keyboard` para conducción manual.
- Python 3 y NumPy para el cálculo de odometría.

Con ROS 2 Jazzy ya instalado y configurado.

## 3. Organización del paquete

```text
taller_RM/
└── src/
    └── mi_robot_diferencial/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── mi_robot.launch.py
        ├── urdf/
        │   └── mi_robot.urdf.xacro
        ├── worlds/
        │   └── pista.sdf
        ├── config/
        │   └── bridge.yaml
        ├── rviz2/
        │   └── control.rviz
        ├── scripts/
        │   └── comparacion.py 
        └── msg/                    # Si se definieron mensajes propios
```

- `urdf/`: define geometrías, inercias, articulaciones y sensores del robot mediante Xacro.
- `worlds/`: contiene el escenario de Gazebo y las colisiones de la pista.
- `config/bridge.yaml`: establece qué tópicos pasan entre Gazebo y ROS 2 y en qué dirección.
- `scripts/comparacion.py`: calcula la odometría a partir de las ruedas y la compara con la simulación.
- `rviz2/control.rviz`: configuración de visualización del modelo y los sensores.
- `launch/mi_robot.launch.py`: inicia la simulación y sus componentes.

## 4. Modelo y funcionamiento

### Tracción diferencial

El robot se desplaza mediante dos ruedas motrices y un apoyo caster. El plugin `DiffDrive` recibe comandos de velocidad desde `/cmd_vel` y proporciona odometría. Los parámetros de radio y separación de ruedas deben coincidir entre el Xacro, el plugin y el nodo de comparación.

### Estimación y comparación de odometría

El nodo `comparacion.py` utiliza las velocidades angulares de las ruedas recibidas en `/joint_states`. Con el modelo cinemático diferencial obtiene las velocidades del robot, integra numéricamente su pose y compara las coordenadas y la orientación con `/odom`.

La comparación de errores se consulta en el tópico `/Poses_y_errores` si está habilitada la versión del nodo que publica ese mensaje.

### LiDAR y cámara

El LiDAR simulado publica medidas de distancia en `/scan`. La cámara publica la imagen y los datos de calibración en los tópicos configurados en el bridge. Para visualizarlos correctamente, sus `frame_id` deben estar conectados al árbol TF.

### Pista

El mundo `pista.sdf` define el suelo y los elementos del escenario. En este caso se usaron figuras rectangulares, cilindricas y trapezoidales con diferentes formas y tamaños para crear la pista.

## 5. Compilación y ejecución

Abra una terminal y ejecute:

```bash
cd ~/ros2_ws_dif
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mi_robot_diferencial
source install/setup.bash
ros2 launch mi_robot_diferencial mi_robot.launch.py
```

El archivo launch está previsto para iniciar Gazebo, insertar el robot en el mundo (spawn), publicar su descripción mediante `robot_state_publisher`, iniciar el puente ROS 2–Gazebo, abrir RViz2 y ejecutar el nodo de comparación, según la configuración del paquete.

Cuando no haya cambios en el código o en los recursos instalados, no es necesario recompilar; basta con cargar el entorno y ejecutar el launch.

## 6. Teleoperación

Con la simulación ejecutándose, abra otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_dif/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

La teleoperación publica comandos de velocidad que el bridge reenvía al plugin de tracción de Gazebo. El objetivo es recorrer la pista y comprobar el desplazamiento del robot.

## 7. Verificación de tópicos

En otra terminal, después de cargar el entorno:

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 topic echo /scan --once
ros2 topic echo /Poses_y_errores
```

El último comando corresponde al nodo de comparación si esa publicación está implementada. Para verificar transformaciones TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

En RViz2 debe seleccionarse un `Fixed Frame` existente y conectado, habitualmente `odom`. Es posible visualizar el modelo del robot, las transformaciones TF y el LiDAR. La cámara también puede comprobarse con `rqt_image_view` si está instalado y su tópico está correctamente puenteado.

## 8. Aspectos a documentar y validar

- Coherencia entre dimensiones físicas, inercias, colisiones y parámetros del plugin.
- Correspondencia entre los `frame_id` de sensores y el árbol TF.
- Comparación de las poses estimadas y de Gazebo, junto con sus errores.
- Justificación del rango, campo de visión y muestras del LiDAR, así como resolución y frecuencia de la cámara.
- Verificación de colisiones de los muros, obstáculos y mallas STL.
- Recorrido completo de la pista mediante teleoperación.

Los cálculos, las decisiones de diseño y sus referencias bibliográficas se desarrollan en el documento de Notion de la entrega.

## 9. Documentación y entrega

- [Notion del proyecto](https://app.notion.com/p/TALLER-ROBOTICA-MOVIL-3f8c6262bdb780d5ab37d21fdc82c915?source=copy_link).
- [Repositorio Git](https://github.com/Noseiden/Taller_Parcial3).
- Integrantes: [Mazuq Yucef Gallego Muñoz & Santiago Andrés García Ortiz].

## 10. Referencias de código y recursos de apoyo

El desarrollo del proyecto tomó como referencia los siguientes recursos proporcionados para la asignatura:

1. **Guía paso a paso: Robot diferencial en Gazebo con LiDAR y cámara.** Documento de apoyo para la creación del robot en URDF/Xacro, la integración de sensores, el puente de comunicación entre Gazebo y ROS 2 y la puesta en marcha de la simulación. Disponible en: [Guía en Notion](https://healthy-relish-579.notion.site/Gu-a-paso-a-paso-Robot-diferencial-en-Gazebo-con-LiDAR-y-c-mara-3eac8766b33781f0ae9cf17045e8993a).

2. **`ros2_ws_dif_C2.zip`.** Espacio de trabajo de referencia suministrado en el [material de la asignatura](https://healthy-relish-579.notion.site/Rob-tica-2026-2-395c8766b337807ba3d6eabe43adf35b). Ubicación: **Códigos de clase → Semana 12**. Se utilizó como recurso de apoyo para el desarrollo y la organización del proyecto en ROS 2.

Estos recursos se citan como material de consulta y base de referencia; las adaptaciones y modificaciones del proyecto se encuentran en los archivos de este repositorio.