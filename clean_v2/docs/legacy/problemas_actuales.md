# PROBLEMAS ACTUALES - ros2-sim-vnc
**Fecha:** 30 de diciembre de 2025  
**Estado:** Análisis completo realizado  

---

## 📋 RESUMEN EJECUTIVO

Tu proyecto está **bien estructurado pero tiene problemas críticos sin resolver** y varias debilidades arquitectónicas. La auditoría anterior identificó problemas pero algunos **aún están pendientes**.

**Puntuación actual:** 7/10  
**Bloqueadores immediatos:** 3  
**Deuda técnica:** Media-Alta

---

## 🔴 PROBLEMAS CRÍTICOS PENDIENTES

### **P1: SRDF no parametrizado (ROS 2 launch files)**
- **Ubicación:** `ros2_ws/src/mm_bringup/config/mm_arm.srdf`
- **Problema:** SRDF está hardcodeado con nombres de joints. Si cambias `base_scale` o `arm_scale`, los nombres siguen siendo `mm_arm_joint_*`, pero MoveIt 2 y los controladores esperan alineación exacta.
- **Impacto:** 🔴 CRÍTICO - MoveIt 2 no funcionará si usas `arm_scale != 1.0`
- **Síntoma:** Error en `move_group` cuando intenta planificar
- **Solución pendiente:** Generar SRDF dinámicamente desde Xacro o template

### **P2: Controllers desincronizados con escala**
- **Ubicación:** `config/base_controllers.yaml`, línea ~20-25
- **Problema:** 
  ```yaml
  # base_controllers.yaml (HARDCODEADO)
  wheel_separation_x: 0.33  # DEBE SER 0.33 * scale
  wheel_separation_y: 0.33  # DEBE SER 0.33 * scale
  wheel_radius: 0.06        # DEBE SER 0.06 * scale
  ```
  vs.
  ```xacro
  <!-- mm_base.urdf.xacro (DINÁMICO) -->
  <xacro:property name="wheel_separation_x" value="${0.33 * scale}"/>
  ```
- **Impacto:** 🔴 CRÍTICO - Si usas `base_scale:=2.0`, la cinemática del robot será incorrecta (el controller no conoce la nueva escala)
- **Síntoma:** Base se mueve con velocidades erráticas o no responde bien a teleop cuando cambias escala
- **Solución pendiente:** Generar `base_controllers.yaml` desde template o usar parámetro dinámic override en spawner

### **P3: Falta validación de rutas en launch files**
- **Ubicación:** Múltiples `*.launch.py`
- **Problema:** Si `nav2_params.yaml` o `bridge_params.yaml` no existen, el launch falla silenciosamente o con error poco claro
- **Impacto:** 🔴 CRÍTICO - Errores confusos para nuevos usuarios
- **Síntoma:** `FileNotFoundError` o `rclpy exception` sin contexto
- **Solución pendiente:** Validación explícita de archivos al inicio del launch

---

## 🟠 PROBLEMAS MAYORES

### **M1: Sin versionado exacto de dependencias**
- **Ubicación:** `package.xml` (todos los paquetes)
- **Problema:** 
  ```xml
  <depend>ros2_control</depend>  <!-- Version? -->
  <depend>gazebo_ros</depend>    <!-- Qué version? -->
  ```
- **Impacto:** 🟠 MAYOR - Reproducibilidad. En CI/CD puede usar v0.15 o v1.0
- **Solución:** Fijar versiones exactas (ej: `ros2_control (>= 0.26, < 0.27)`)

### **M2: Joint names hardcodeados en YAML**
- **Ubicación:** `config/arm_controllers.yaml`, `config/joy_teleop.yaml`
- **Problema:** 
  ```yaml
  joints:
    - mm_arm_joint_1
    - mm_arm_joint_2
    # ... hardcodeado
  ```
  Si cambias `arm_prefix`, esto no se actualiza automáticamente
- **Impacto:** 🟠 MAYOR - Difícil mantener múltiples instancias del brazo
- **Solución:** Generar desde Xacro o usar búsqueda dinámica de joints

### **M3: Sin launch_testing**
- **Ubicación:** No hay tests de integración
- **Problema:** Nadie garantiza que `modes.launch.py` lance sin errores
- **Impacto:** 🟠 MAYOR - Regressions no detectadas
- **Solución:** Crear `test_launch.py` con pytest

### **M4: Rutas temporales sin limpiar**
- **Ubicación:** `sim.launch.py`, línea ~360
- **Problema:** 
  ```python
  model_cache_dir = os.path.expanduser(f"~/.cache/mm_bringup")
  # Archivos nunca se limpian, acumulan basura
  ```
- **Impacto:** 🟠 MAYOR - Espacio en disco crece indefinidamente
- **Solución:** Implementar cleanup de archivos > 7 días

### **M5: Falta documentación de QoS**
- **Ubicación:** `config/bridge_params.yaml`, scripts
- **Problema:** Se usan `SensorDataQoS` y `ReliableQoS` pero sin justificación en comentarios
- **Impacto:** 🟠 MAYOR - Difícil entender por qué un tópico falla
- **Solución:** Documentar inline: `# SensorDataQoS: best_effort para baja latencia en /joy`

---

## 🟡 PROBLEMAS MENORES

### **m1: Expresiones Python largas en launch files**
- **Ubicación:** `modes.launch.py`, varias condiciones
- **Problema:** Expresiones complejas como `PythonExpression(['...&& ...'])` se vuelven ilegibles
- **Solución:** Extraer a funciones o usar `DeclareLaunchArgument` con defaults más claros

### **m2: Sin log rotation en supervisord**
- **Ubicación:** `supervisord.conf`
- **Problema:** Logs crecen indefinidamente en contenedor
- **Solución:** Agregar `stdout_logfile_maxbytes` y `stdout_logfile_backups`

### **m3: Documentación de argumentos incompleta**
- **Ubicación:** `modes.launch.py`, línea ~40
- **Problema:** Se declaran 40+ argumentos pero sin describir cuáles son "obligatorios" vs "opcionales"
- **Solución:** Agregar comentario DOC o usar `DeclareLaunchArgument(..., description="...")`

### **m4: Sin validación de entrada en joy_teleop.py**
- **Ubicación:** `scripts/joy_teleop.py`, línea ~150
- **Problema:** Si `/joy` publica valores fuera de rango, no hay saturación explícita
- **Solución:** Agregar `np.clip()` o validación

### **m5: Falta checklist pre-commit**
- **Ubicación:** No existe `.pre-commit-config.yaml`
- **Problema:** Cualquiera puede hacer push con errores (flake8 no se ejecuta localmente)
- **Solución:** Crear `.pre-commit-config.yaml` con flake8, black, yaml linter

---

## ⚠️ PROBLEMAS DE CONFIGURACIÓN

### **Arq1: SRDF vs Xacro desalojados**
```
mm_arm.urdf.xacro (parametrizado)  ✅
     ↓
mm_arm.urdf (generado)             ✅
     ↓
mm_arm.srdf (HARDCODEADO)          ❌ NO SINCRONIZADO
```
**Riesgo:** Si cambias joints en Xacro, SRDF no se actualiza.

### **Arq2: No hay "single source of truth" para parámetros**
- `wheel_separation_x` definida en 3 lugares:
  1. `mm_base.urdf.xacro` (parametrizado) ✅
  2. `base_controllers.yaml` (hardcodeado) ❌
  3. `joy_teleop.py` (hardcodeado) ❌

---

## 📊 TABLA RESUMEN

| ID | Tipo | Problema | Status | Prioridad | Esfuerzo |
|----|------|----------|--------|-----------|----------|
| P1 | 🔴 | SRDF no parametrizado | ⏳ PENDIENTE | INMEDIATO | Alto |
| P2 | 🔴 | Controllers desincronizados | ⏳ PARCIAL | INMEDIATO | Medio |
| P3 | 🔴 | Sin validación rutas | ⏳ PENDIENTE | INMEDIATO | Bajo |
| M1 | 🟠 | Sin versiones exactas | ⏳ PENDIENTE | CORTO | Bajo |
| M2 | 🟠 | Joint names hardcodeados | ⏳ PENDIENTE | CORTO | Medio |
| M3 | 🟠 | Sin launch_testing | ⏳ PENDIENTE | CORTO | Alto |
| M4 | 🟠 | Cleanup de temp files | ⏳ PENDIENTE | CORTO | Bajo |
| M5 | 🟠 | Documentación QoS | ⏳ PENDIENTE | CORTO | Bajo |
| m1 | 🟡 | Expresiones largas | ⏳ PENDIENTE | LARGO | Bajo |
| m2 | 🟡 | Log rotation | ⏳ PENDIENTE | LARGO | Bajo |

---

## 🎯 PLAN DE ACCIÓN RECOMENDADO

### **INMEDIATO (Antes de usar en "producción")**
1. ✅ Validar rutas en launch files (P3) - 30 min
2. ✅ Generar SRDF desde Xacro (P1) - 2-3 hrs
3. ✅ Sincronizar controllers con Xacro (P2) - 1-2 hrs

### **CORTO PLAZO (Antes de siguiente release)**
1. Fijar versiones en `package.xml` (M1) - 30 min
2. Crear template para joint names (M2) - 1 hr
3. Agregar launch_testing (M3) - 2-3 hrs

### **LARGO PLAZO (Roadmap v0.3+)**
1. Refactorizar expresiones Python (m1)
2. Implementar cleanup temporal (M4)
3. Agregar log rotation (m2)

---

## 🔗 REFERENCIAS

- Audit Report: [AUDIT_REPORT.md](AUDIT_REPORT.md)
- Changelog: [CHANGELOG.md](CHANGELOG.md)
- Contributing: [CONTRIBUTING.md](CONTRIBUTING.md)

