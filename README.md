# Grupo 04 - TM5-900

Proyecto del Primer Parcial Práctico de la materia IMT-342 Robótica.

## Integrantes

- Nicolás Ulloa
- Nicolás Taboada

## Robot

TM5-900 de Techman Robot.

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- RViz2
- colcon
- vcs

## Clonar el repositorio

```bash
git clone https://github.com/Nulloa7/grupo04_tm5_900.git
cd grupo04_tm5_900
```

## Descargar dependencias externas

```bash
mkdir -p src
vcs import src < dependencias.repos
```

## Instalar dependencias de ROS 2

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Compilar

```bash
colcon build --symlink-install
source install/setup.bash
```

## Lanzar el robot en RViz2

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch grupo04_tm5_900_bringup display.launch.py
```

## Ejecutar cinemática directa

En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo04_tm5_900_kinematics fk_node
```

## Ejecutar cinemática inversa

En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo04_tm5_900_kinematics ik_node
```

## Enviar un objetivo cartesiano

En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 0.3, y: 0.2, z: 0.5}"
```

## Tópicos principales

- `/joint_states`
- `/target`

## Paquetes propios

- `grupo04_tm5_900_bringup`
- `grupo04_tm5_900_kinematics`

## Dependencia externa

Se utiliza el repositorio oficial de Techman Robot:

```text
https://github.com/TechmanRobotInc/tmr_ros2.git
```

Commit utilizado:

```text
c40bfc0d296337311f311c837861fd6318b00407
```

Esta dependencia se descarga mediante el archivo:

```text
dependencias.repos
```

## Estructura del proyecto

```text
grupo04_tm5_900/
├── src/
│   ├── grupo04_tm5_900_bringup/
│   └── grupo04_tm5_900_kinematics/
├── dependencias.repos
├── README.md
└── .gitignore
```

## Nota importante

Las carpetas:

```text
build/
install/
log/
```

no se incluyen en el repositorio porque se generan automáticamente al compilar el workspace con:

```bash
colcon build --symlink-install
```

## Funcionalidad principal

El paquete `grupo04_tm5_900_kinematics` contiene:

- `fk_node.py`: calcula la cinemática directa del TM5-900 a partir de `/joint_states`.
- `ik_node.py`: recibe un objetivo cartesiano mediante `/target`, resuelve la cinemática inversa numérica y publica la solución articular.
