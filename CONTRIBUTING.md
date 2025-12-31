# Contributing to ros2-sim-vnc

¡Gracias por tu interés en contribuir! Este documento describe cómo colaborar de forma efectiva con el proyecto.

## Principios Generales

Este proyecto sigue los **lineamientos en [AGENTS.md](AGENTS.md)**:
- **Cambios pequeños y autoexplicativos**: agrupa ediciones relacionadas en un único commit bien titulado.
- **Español neutral**: documentación y comentarios en español claro.
- **Convenciones ROS 2**: `rclcpp`/`rclpy`, `snake_case` para nodos/tópicos, YAML para parámetros.
- **Buenas prácticas de seguridad**: watchdogs, parada de emergencia, sin hardcoding de valores críticos.

## Flujo de Trabajo

### 1. Preparación Local

```bash
# Clonar el repositorio
git clone https://github.com/CarryMankeke/ros2-dev-docker
cd ros2-sim-vnc

# Crear rama de feature/fix
git checkout -b feature/descripcion-corta
# o
git checkout -b fix/descripcion-del-issue

# Compilar localmente en Docker
docker compose build
docker compose up -d
docker compose exec -T ros2-vnc bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /home/ros/ros2_ws
  colcon build --symlink-install
  colcon test
'
```

### 2. Hacer Cambios

#### Código Python
- Usa **type hints** (`def foo(x: int) -> str:`).
- Sigue **PEP 8** (line length ≤ 120 caracteres).
- Incluye **docstrings** en clases y funciones públicas.
- Verifica con `flake8 --max-line-length=120`:
  ```bash
  find ros2_ws/src -name "*.py" -type f | xargs flake8 --max-line-length=120
  ```

#### URDF/XACRO
- Mantén dimensiones **escalables** con `<xacro:arg name="scale">`.
- Documenta **parámetros** in-line (especialmente en macros).
- Usa **prefixes dinámicos**: `<xacro:arg name="prefix">`.
- Valida con `xacro --inorder` (no dejes errores de variable no definida).

#### YAML (Configuración)
- Incluye **comentarios explicativos** (ej: `# Max vel base [m/s]`).
- Mantén **consistencia con URDF/XACRO** (joints, sensores, escalas).
- Usa **explícito QoS** en tópicos críticos.
- Valida sintaxis: `python3 -c "import yaml; yaml.safe_load(open('file.yaml'))"`

#### Launch Files
- Usa **nombres descriptivos** para `DeclareLaunchArgument`.
- Documenta **valores por defecto** y **rango válido** en comentarios.
- Evita **expresiones Python complejas** en `PythonExpression` (usa variables intermedias).
- Prueba localmente:
  ```bash
  ros2 launch <package> <file>.launch.py --show-args
  ```

### 3. Commits

Formato recomendado de mensaje de commit (en español):

```
[TIPO] Descripción breve (≤50 caracteres)

Explicación detallada si es necesario (≤72 caracteres por línea).
Incluye **por qué** el cambio es necesario, no solo **qué**.

- Punto 1
- Punto 2

Fixes #<issue_number> (si aplica)
```

**Tipos comunes:**
- `[fix]`: Corrección de bug
- `[feat]`: Nueva característica
- `[docs]`: Cambios de documentación
- `[refactor]`: Refactorización sin cambio funcional
- `[ci]`: Cambios en CI/CD
- `[chore]`: Actualizaciones de dependencias, tareas mantenimiento

**Ejemplo:**
```
[fix] Permisos ejecutables en joint_state_aggregator.py

El script no era ejecutable, lo que causaba fallo al lanzar
con 'ros2 run mm_bringup joint_state_aggregator.py'.
Se aplica chmod +x y se verifica shebang.

Fixes #42
```

### 4. Pull Request

1. **Push** tu rama: `git push origin feature/descripcion`
2. **Abre un PR** en GitHub con:
   - Título claro y descriptivo
   - Descripción en español explicando:
     - **Qué** cambió
     - **Por qué** fue necesario
     - **Cómo** se validó (tests locales, comandos de prueba)
   - Referencia a issues relacionados (`Fixes #<number>`)
3. **Espera review** y responde comentarios
4. **CI/CD debe pasar**: GitHub Actions ejecuta `colcon build` y tests básicos

### 5. Validación Pre-Merge

El PR debe cumplir:
- ✅ GitHub Actions CI/CD verde
- ✅ Cambios documentados (CHANGELOG.md actualizado)
- ✅ Code review aprobado
- ✅ Sin conflictos con `main`

---

## Tareas Frecuentes

### Agregar un Nuevo Paquete ROS 2

1. Crear estructura:
   ```bash
   cd ros2_ws/src
   ros2 pkg create --build-type ament_cmake my_new_package
   ```
2. Agregar dependencias en `package.xml`:
   ```xml
   <depend>rclpy</depend>  <!-- o rclcpp -->
   <depend>rclcpp</depend>
   ```
3. Incluir en `modes.launch.py` si es relevante
4. Documentar en README.md
5. Commit con `[feat] Agregar paquete my_new_package`

### Modificar URDF/XACRO de Base o Brazo

1. **Editar** `ros2_ws/src/mm_base_description/urdf/mm_base.urdf.xacro` (o `mm_arm_description`)
2. **Validar** con:
   ```bash
   xacro --inorder ros2_ws/src/mm_base_description/urdf/mm_base.urdf.xacro prefix:=mm_base_ scale:=1.0
   ```
3. **Actualizar configs** relacionadas (ej: `base_controllers.yaml` si cambia geometría)
4. **Lanzar prueba**:
   ```bash
   docker compose exec -T ros2-vnc bash -lc '
   source /opt/ros/jazzy/setup.bash
   source /home/ros/ros2_ws/install/setup.bash
   ros2 launch mm_bringup display.launch.py
   '
   ```
5. Commit con `[refactor] Ajustar geometría base` o similar

### Agregar Nuevo Parámetro Dinámico

1. **Declarar** en `modes.launch.py`:
   ```python
   DeclareLaunchArgument('my_param', default_value='default_val')
   ```
2. **Usar** en launch files y nodos:
   ```python
   my_param = LaunchConfiguration('my_param')
   my_node = Node(..., parameters=[{'my_param': my_param}])
   ```
3. **Documentar** en `info.txt` y README.md
4. Commit con `[feat] Agregar parámetro my_param`

### Actualizar Dependencias ROS 2

1. **Listar** versiones disponibles:
   ```bash
   apt-cache policy ros-jazzy-<package>
   ```
2. **Actualizar** `package.xml` con versión exacta:
   ```xml
   <exec_depend>ros2_control (>=2.25,<3.0)</exec_depend>
   ```
3. **Probar** compilación:
   ```bash
   docker compose exec -T ros2-vnc bash -lc 'cd /home/ros/ros2_ws && colcon build'
   ```
4. **Actualizar** CHANGELOG.md
5. Commit con `[chore] Actualizar ros2_control a >=2.25`

---

## Reporte de Bugs

Si encuentras un bug:

1. **Verifica** si ya existe un issue similar
2. **Crea un issue** con:
   - Título claro y específico
   - Descripción del comportamiento observado vs. esperado
   - Pasos para reproducir
   - Logs de error (usa ` ```markdown \n ...\n ``` `)
   - Información del sistema (SO, versión ROS, commit actual)

**Ejemplo:**
```
Título: joint_state_aggregator no publica cuando falta /mm_arm/joint_states

**Descripción del bug:**
El nodo joint_state_aggregator no publica nada si la base está activa pero el brazo no.

**Pasos para reproducir:**
1. Lanzar sim sin brazo: `ros2 launch mm_bringup modes.launch.py launch_moveit:=false`
2. Ver topic `/joint_states` vacío

**Logs:**
[Pega aquí contenido de ros2 topic echo]

**Sistema:**
- OS: Ubuntu 22.04
- ROS: Jazzy (commit abc123)
```

---

## Preguntas y Soporte

- **Dudas de diseño**: Abre una **Discussion** en GitHub
- **Bugs**: Abre un **Issue** con título claro
- **Features**: Abre un **Issue** con label `enhancement` para discutir primero

---

## Estilo y Convenciones

### Nombres de Variables/Funciones

```python
# ✅ Bueno
joint_states_topic = '/mm_base/joint_states'
def compute_mecanum_kinematics(wheel_vels):
    pass

# ❌ Malo
JST = '/mm_base/joint_states'
def compute_kin():
    pass
```

### Comentarios

```python
# ✅ Bueno: explica **por qué**, no qué
# Usa mecánica holonómica para movimiento lateral sin girar
wheel_cmd_x = joy_x * scale_linear

# ❌ Malo: obvia
# Multiplica joy_x por escala
wheel_cmd_x = joy_x * scale_linear
```

---

## Merging y Release

El mantenedor del proyecto:
1. Revisa y aprueba el PR
2. **Squash merge** a `main` (mantiene historia limpia)
3. **Taguea** versión: `git tag -a v0.2.0 -m "Release v0.2.0"`
4. **Publica** release en GitHub con notas de cambio
5. Actualiza versión en `package.xml`

---

**¡Gracias nuevamente por contribuir!** 🎉
