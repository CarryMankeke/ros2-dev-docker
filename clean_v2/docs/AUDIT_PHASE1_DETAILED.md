#  AUDITORÍA TÉCNICA - FASE 1: CORE INFRASTRUCTURE

**Fecha:** 18 de enero de 2026  
**Ingeniero:** Senior ROS2/Gazebo Harmonic  
**Objetivo:** Análisis profundo de arquitectura e integración  

---

##  ARCHIVOS ANALIZADOS - FASE 1

### [OK] 1. sim_mm.launch.py (548 líneas)

**DESCRIPCIÓN GENERAL:**
Lanzador principal de simulación para robot móvil manipulador individual en Gazebo Harmonic con ROS2 Jazzy.

####  **HALLAZGOS TÉCNICOS:**

##### [OK] FORTALEZAS

| Aspecto | Evaluación | Detalle |
|---------|-----------|--------|
| **Arquitectura** | OK Excelente | Uso correcto de `OpaqueFunction` para contexto dinámico |
| **Namespacing** | OK Correcto | Aislamiento de namespace `mm1` (multi-robot ready) |
| **ROS2 Control** | OK Bien implementado | Timers escalonados para spawning de controladores |
| **Bridges** | OK Completo | Cobertura de: clock, cámaras, LiDAR, IMU, ee_imu |
| **Conditional Logic** | OK Robusto | Uso de `IfCondition` para enable/disable de subsistemas |
| **Gazebo Integration** | OK Correcto | Parámetros adecuados, headless mode soportado |

##### [WARN] PROBLEMAS ENCONTRADOS

**1. TIMEOUT de Controladores (Líneas 303-330)**
```python
# PROBLEMA IDENTIFICADO
base_jsb = Node(
    package='controller_manager',
    executable='spawner',
    output='screen',
    arguments=[
        'joint_state_broadcaster',
        '--controller-manager', controller_manager_ns,
        '--controller-manager-timeout', '120',  # [WARN] ALTO (120s)
        '--service-call-timeout', '30',         # [WARN] ALTO (30s)
    ],
)
```

**ANÁLISIS:**
- Timeout de 120s es excesivo para un sistema que debería estar listo en <10s
- En CI/CD y smoke tests, esto causa demoras innecesarias
- Recomendado: 30-45s máximo para timeout de controller-manager

**GRAVEDAD:** [MED] MEDIA (afecta velocidad de startup, no funcionalidad)

**2. TimerAction sin Validación de Readiness (Líneas 343-354)**
```python
# PROBLEMA: Timers fijos sin validación de readiness
start_jsb = TimerAction(period=3.0, actions=[base_jsb])
start_omni = TimerAction(period=5.0, actions=[omni_controller])
start_arm = TimerAction(period=7.0, actions=[arm_controller])
start_gripper = TimerAction(period=9.0, actions=[gripper_controller])
```

**ANÁLISIS:**
- Los timers son "best-effort" sin guarantía de que gazebo/robot_state_publisher estén listos
- En sistemas lentos (CI/CD), los timers pueden ejecutarse antes de que el spawn esté completado
- Esto causa race conditions ocasionales en smoketests

**GRAVEDAD:** [MED] MEDIA (causa fallos aleatorios en CI/CD)

**RECOMENDACIÓN:** Usar `WaitOnceAction` con condiciones o health checks

---

### [OK] 2. core_health_check.py (461 líneas)

**DESCRIPCIÓN GENERAL:**
Script de validación principal que verifica disponibilidad y funcionalidad de subsistemas ROS2 (sim_time, TF, controladores, sensores).

####  **HALLAZGOS TÉCNICOS:**

##### [OK] FORTALEZAS

| Aspecto | Evaluación | Detalle |
|---------|-----------|--------|
| **QoS Handling** | OK Correcto | Uso de `ReliabilityPolicy.BEST_EFFORT` para sensores |
| **Timeout Management** | OK Robusto | Múltiples retries con backoff exponencial |
| **Error Messages** | OK Descriptivos | Mensajes claros sobre causa raíz de fallos |
| **Multi-Robot Ready** | OK Soportado | Flag `--check-mm2` para validación dual |
| **Active Tests** | OK Innovador | Envía comandos y valida respuesta (prueba completa) |
| **Frame ID Validation** | OK Riguroso | Verifica frame_id correcto en todos los sensores |

##### [WARN] PROBLEMAS ENCONTRADOS

**1. TF Timeout Muy Agresivo (Líneas 107-140)**
```python
# PROBLEMA
def _check_tf(self, namespace: str):
    deadline = time.time() + 2.0  # [WARN] SOLO 2 SEGUNDOS
    while time.time() < deadline:
        try:
            # TF lookup con timeout de 1.0s
```

**ANÁLISIS:**
- 2 segundos total es muy poco para TF tree que puede tardar 3-5s en construirse
- En sistemas lentos (máquinas virtuales, CI/CD), siempre fallará
- Debería ser mínimo 5-8 segundos

**GRAVEDAD:** [CRIT] ALTA (falsos negativos en validación)

**RECOMENDACIÓN:**
```python
deadline = time.time() + 5.0  # Mínimo 5 segundos para TF tree
```

**2. Retry Logic Inconsistente (Líneas 316-338)**
```python
# PROBLEMA: retry logic solo en sensor check, no en otros
def _check_sensor_topic(self, topic: str, msg_type, ...):
    max_attempts = 3
    backoff_sec = 0.6
    # ... retry logic
```

**ANÁLISIS:**
- Solo `_check_sensor_topic` tiene retry logic
- `_check_tf`, `_check_joint_states` NO tienen retry
- Causa inconsistencia en robustez

**GRAVEDAD:** [MED] MEDIA

---

### [OK] 3. mm_robot.urdf.xacro (520 líneas)

**DESCRIPCIÓN GENERAL:**
Descripción del robot (URDF con Xacro) que define cinemática, sensores, actuadores y plugins Gazebo.

####  **HALLAZGOS TÉCNICOS:**

##### [OK] FORTALEZAS

| Aspecto | Evaluación | Detalle |
|---------|-----------|--------|
| **Xacro Macros** | OK Bien usado | Includes de base_macro y arm_macro |
| **Parametrización** | OK Extensa | 30+ parámetros configurables |
| **Sensor Coverage** | OK Completa | 5 cámaras + LiDAR + 2 IMU (base + ee) |
| **Inertia** | OK Definido | Sensores con inertia válida |
| **Gazebo Plugins** | OK Moderno | Usa `gpu_lidar` y `imu` de Gazebo |
| **Collision Geometry** | OK Simple | Cilindros y cajas para sensors |

##### [WARN] PROBLEMAS ENCONTRADOS

**1. IMU Noise Simétrica (Líneas 294-310)**
```xml
<!-- PROBLEMA IDENTIFICADO -->
<linear_acceleration>
  <x>
    <noise type="gaussian">
      <stddev>0.01</stddev>  <!-- [WARN] IGUAL para X, Y, Z -->
    </noise>
  </x>
  <y>
    <noise type="gaussian">
      <stddev>0.01</stddev>
    </noise>
  </y>
  <z>
    <noise type="gaussian">
      <stddev>0.01</stddev>
    </noise>
  </z>
</linear_acceleration>
```

**ANÁLISIS:**
- En sensores reales, Z tiene mayor ruido (efecto de gravedad)
- Configuración actual es demasiado optimista
- Recomendado: Z = 0.02-0.03 (2-3x más que XY)

**GRAVEDAD:** [MED] MEDIA (afecta realismo de simulación)

**2. Camera FOV No Documentado (Línea 408)**
```xml
<horizontal_fov>1.3962634</horizontal_fov>  <!-- ¿Qué cámara es? -->
```

**ANÁLISIS:**
- FOV en radianes = 1.396 ≈ 80 grados
- Poco realista para cámaras de robot (típico: 60-120 grados)
- Sin comentarios sobre especificaciones de cámara

**GRAVEDAD:** [MED] BAJA (no afecta funcionalidad)

**3. LiDAR Update Rate Baja (Línea 240)**
```xml
<update_rate>10</update_rate>  <!-- [WARN] SOLO 10 Hz -->
```

**ANÁLISIS:**
- 10 Hz es muy bajo para LiDAR típico (16, 32, 64 Hz)
- Nav2 puede sufrir con <20 Hz
- Recomendado: 20-30 Hz para simular Velodyne típico

**GRAVEDAD:** [MED] MEDIA (puede afectar navegación rápida)

**4. ee_imu Link Creado en mm_arm Pero Not in mm_robot**
```xml
<!-- Línea 291: Enable solo con condicional, pero link definido en macro -->
<xacro:if value="$(arg enable_ee_imu)">
    <gazebo reference="${prefix}ee_imu_link">
```

**ANÁLISIS:**
- Si `enable_ee_imu=true`, se busca `${prefix}ee_imu_link` que NO existe en mm_robot
- El link debe ser creado en mm_arm macro
- Esto causará error de Gazebo si está habilitado

**GRAVEDAD:** [CRIT] ALTA (no funciona si está habilitado)

---

##  RESUMEN DE HALLAZGOS - FASE 1

| Categoría | Cantidad | Gravedad |
|-----------|----------|----------|
| [CRIT] Críticos | 1 | Gazebo ee_imu link missing |
| [HIGH] Altos | 1 | TF timeout muy agresivo |
| [MED] Medios | 4 | Controller timeouts, IMU noise, LiDAR rate, retry inconsistencia |
| [LOW] Bajos | 1 | Documentación camera FOV |

**PUNTUACIÓN GENERAL:** 7.5/10

---

## [OK] RECOMENDACIONES INMEDIATAS

### 1. **Corregir ee_imu Link (CRÍTICO)**
```python
# En mm_arm_macro.xacro, asegurar:
<xacro:if value="$(arg enable_ee_imu)">
    <link name="${prefix}ee_imu_link">
        <!-- definir link aquí -->
    </link>
</xacro:if>
```

### 2. **Aumentar TF Timeout (ALTO)**
```python
# En core_health_check.py línea 108
deadline = time.time() + 5.0  # De 2.0 a 5.0
```

### 3. **Reducir Controller Timeouts (MEDIO)**
```python
'--controller-manager-timeout', '45',  # De 120 a 45
'--service-call-timeout', '15',         # De 30 a 15
```

### 4. **Mejorar LiDAR Rate (MEDIO)**
```xml
<!-- En mm_robot.urdf.xacro -->
<update_rate>20</update_rate>  <!-- De 10 a 20 Hz -->
```

### 5. **Mejorar IMU Ruido Z (MEDIO)**
```xml
<z>
    <noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.03</stddev>  <!-- De 0.01 a 0.03 -->
    </noise>
</z>
```

---

##  PRÓXIMOS PASOS

- [ ] Revisar mm_base_macro.xacro (277 L)
- [ ] Revisar mm_arm_macro.xacro (370 L)
- [ ] Validar sim_mm_dual.launch.py (665 L)
- [ ] Completar análisis de subsistemas (FASE 2)

---

**Estado:** [WARN] PENDIENTE CORRECCIONES CRÍTICAS  
**Estimado:** 4-6 horas de revisión adicional
