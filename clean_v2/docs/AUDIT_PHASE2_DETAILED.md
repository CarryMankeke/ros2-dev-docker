#  AUDITORÍA TÉCNICA - FASE 2: ROBOT DESCRIPTION

**Fecha:** 18 de enero de 2026  
**Ingeniero:** Senior ROS2/Gazebo Harmonic  
**Objetivo:** Análisis de cinemática, física y geometría del robot  

---

##  ARCHIVOS ANALIZADOS - FASE 2

### [OK] 1. mm_arm_macro.xacro (370 líneas)

**DESCRIPCIÓN GENERAL:**
Macro Xacro que define el brazo manipulador (7 DOF + gripper) con cinemática UR-like, sensores (cámara EE, IMU opcional) y propiedades físicas.

####  **HALLAZGOS TÉCNICOS:**

##### [OK] FORTALEZAS

| Aspecto | Evaluación | Detalle |
|---------|-----------|--------|
| **Inertia Calculations** | OK Correcto | Box/Cylinder macros usan fórmulas correctas |
| **Parametrización** | OK Extensa | 22 parámetros configurables (arm_mount_height, sensores) |
| **Joint Limits** | OK Realista | Limits coherentes con UR (±π radianes) |
| **Link Masses** | OK Progresivo | Decrece de base→ee (4.0→0.3 kg) |
| **Conditional Links** | OK Bien usado | ee_imu solo si `enable_ee_imu=true` |
| **Scale Support** | OK Implementado | Todos los parámetros usan `scale` factor |

##### [WARN] PROBLEMAS ENCONTRADOS

**1. Velocidad de Muñeca Muy Alta (Línea 244)**
```xml
<!-- PROBLEMA IDENTIFICADO -->
<joint name="${prefix}arm_wrist_3_joint" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
    <!-- [WARN] VELOCIDAD = 2.0 rad/s -->
</joint>
```

**ANÁLISIS:**
- 2.0 rad/s para muñeca es poco realista (114.6 °/s)
- UR Cobot: típicamente 1.0-1.2 rad/s
- Wrist 3 tiene baja inercia, pero velocidad debería ser ~1.5 rad/s
- Puede causar oscillaciones en MoveIt planning

**GRAVEDAD:** [MED] MEDIA (afecta estabilidad planificación)

**2. Esfuerzo (Effort) No Documentado (Líneas 152-244)**
```xml
<!-- PROBLEMA: No hay consistencia de effort -->
<limit effort="50" velocity="1.0"/>   <!-- Shoulder: 50 Nm -->
<limit effort="50" velocity="1.0"/>   <!-- Shoulder: 50 Nm -->
<limit effort="40" velocity="1.2"/>   <!-- Elbow: 40 Nm -->
<limit effort="20" velocity="1.5"/>   <!-- Wrist 1: 20 Nm -->
<limit effort="15" velocity="1.8"/>   <!-- Wrist 2: 15 Nm -->
<limit effort="10" velocity="2.0"/>   <!-- Wrist 3: 10 Nm -->
```

**ANÁLISIS:**
- Valores de effort son arbitrarios sin documentación
- UR3: típicamente 150 Nm (shoulder), 150 Nm, 28 Nm (elbow), 28 Nm, 28 Nm, 14 Nm
- Valores actuales son muy bajos (~10-20% de robot real)
- Esto puede causar fallos de trajectory execution en MoveIt

**GRAVEDAD:** [CRIT] ALTA (fallos en MoveIt trajectory execution)

**3. Falta Amortiguamiento (Damping) en Joints**
```xml
<!-- PROBLEMA: No hay damping specification -->
<joint name="${prefix}arm_shoulder_pan_joint" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="50" velocity="1.0"/>
    <!-- NO Sin <damping> o <friction> -->
</joint>
```

**ANÁLISIS:**
- Sin damping, la simulación Gazebo puede ser inestable
- UR típicamente tiene damping ~0.5 Nm·s/rad
- Falta specification causa movimiento oscilatorio en sim

**GRAVEDAD:** [MED] MEDIA (afecta realismo simulación)

**4. Gripper Joint Type Incorrecto (Línea 352)**
```xml
<!-- PROBLEMA IDENTIFICADO -->
<joint name="${prefix}gripper_joint" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="0.0" upper="0.04" effort="20" velocity="0.2"/>
</joint>
```

**ANÁLISIS:**
- Prismatic es correcto para gripper lineal
- PERO: los dedos típicamente son paralelos, no lineales a lo largo del eje X
- Debería ser verificado si se considera par de dedos (2x prismatic)
- Setup actual sugiere gripper simple de 1 dedo

**GRAVEDAD:** [MED] BAJA (depende de diseño real del gripper)

---

### [OK] 2. mm_base_macro.xacro (277 líneas)

**DESCRIPCIÓN GENERAL:**
Macro Xacro que define la base móvil con 4 ruedas omnidireccionales, cámaras (4 direcciones) y propiedades físicas.

####  **HALLAZGOS TÉCNICOS:**

##### [OK] FORTALEZAS

| Aspecto | Evaluación | Detalle |
|---------|-----------|--------|
| **Wheel Configuration** | OK Correcto | 4 ruedas omnidireccionales (continuous joints) |
| **Inertia Matrix** | OK Correcto | Box inertia para base, cylinder para ruedas |
| **Camera Coverage** | OK Completa | 4 cámaras (front, left, right, rear) |
| **Fixed Joints** | OK Correcto | Cámaras con fixed joints (sin libertad) |
| **Mass Distribution** | OK Realista | Base 15kg, ruedas 1kg cada una |
| **Coordinate System** | OK Standard | base_footprint → base_link → wheels |

##### [WARN] PROBLEMAS ENCONTRADOS

**1. Posición de Ruedas Poco Realista (Líneas 44-48)**
```xml
<!-- PROBLEMA IDENTIFICADO -->
<xacro:property name="wheel_front_x" value="${base_length / 1.9}"/>
<xacro:property name="wheel_rear_x" value="${-base_length / 1.9}"/>
<xacro:property name="wheel_left_y" value="${(base_width / 3.5) + wheel_radius}"/>
<xacro:property name="wheel_right_y" value="${-(base_width / 3.5) - wheel_radius}"/>
```

**ANÁLISIS:**
- Con base_length=0.50m, wheel_front_x = 0.263m (muy adelante)
- Distancia entre ejes (base_length/1.9 * 2) = 0.526m (casi igual al largo total)
- Típicamente: ruedas están a ±base_length/2.5 a ±base_length/2.0

**CÁLCULO ACTUAL:**
```
Distancia entre ejes (X): 0.263m - (-0.263m) = 0.526m
Distancia entre ruedas (Y): 0.37m izq - (-0.37m der) = 0.74m
Razón: 0.526 / 0.74 = 0.71 (base muy larga, estrecha)
```

**GRAVEDAD:** [MED] MEDIA (afecta cinemática omnidireccional)

**2. Base Footprint sin Masa (Línea 50)**
```xml
<!-- PROBLEMA -->
<link name="${prefix}base_footprint"/>
<!-- No tiene masa ni inercia -->

<joint name="${prefix}base_footprint_joint" type="fixed">
    <parent link="${prefix}base_footprint"/>
    <child link="${prefix}base_link"/>
    <origin xyz="0 0 ${base_mount_z}" rpy="0 0 0"/>
</joint>
```

**ANÁLISIS:**
- base_footprint sin masa es correcto por estándar ROS
- PERO: base_footprint a wheel_radius arriba del suelo
- En TF tree, esto causa confusión (footprint no está realmente en contacto)
- Standard ROS: base_footprint en Z=0 (suelo), base_link en Z=height/2

**GRAVEDAD:** [HIGH] ALTO (violates ROS convention)

**3. Wheel Collision Sin Fricción Especificada**
```xml
<!-- PROBLEMA: Sin material/friction -->
<link name="${prefix}front_left_wheel_link">
    <collision>
        <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
    </collision>
</link>
```

**ANÁLISIS:**
- Sin `<contact>` properties o material friction en Gazebo
- Wheels pueden slipping excesivamente
- Debería especificar: mu, mu2, kp, kd para wheels

**GRAVEDAD:** [MED] MEDIA (afecta física wheels)

**4. Wheel Radio Muy Pequeño para Omnidireccional (Línea 40)**
```xml
<xacro:property name="wheel_radius" value="${0.07 * scale}"/>
<!-- [WARN] 0.07m = 7cm de radio -->
```

**ANÁLISIS:**
- Con base_length=0.50m, wheel_radius=0.07m
- Razón: 0.50 / (2×0.07) = 3.57
- Omnidirectional típicamente: 4.0-6.0
- Wheel muy pequeño puede causar:
  - Slipping en rampas
  - Baja velocidad máxima
  - Inestabilidad en giros

**GRAVEDAD:** [MED] MEDIA

---

##  RESUMEN DE HALLAZGOS - FASE 2

| Categoría | Cantidad | Gravedad |
|-----------|----------|----------|
| [CRIT] Críticos | 1 | Effort values demasiado bajos en arm |
| [HIGH] Altos | 1 | base_footprint violates ROS convention |
| [MED] Medios | 5 | Wrist speed, damping missing, wheel geometry, friction, wheel radius |
| [LOW] Bajos | 1 | Gripper joint type verification |

**PUNTUACIÓN GENERAL:** 6.8/10

---

## [OK] RECOMENDACIONES INMEDIATAS

### 1. **Aumentar Effort en Arm (CRÍTICO)**
```xml
<!-- En mm_arm_macro.xacro -->
<!-- Cambiar de: effort="50" a effort="150" para shoulder -->
<joint name="${prefix}arm_shoulder_pan_joint" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="150" velocity="1.0"/>
</joint>

<!-- Cambiar de: effort="50" a effort="150" para shoulder lift -->
<joint name="${prefix}arm_shoulder_lift_joint" type="revolute">
    <limit lower="-1.57" upper="1.57" effort="150" velocity="1.0"/>
</joint>

<!-- Cambiar de: effort="40" a effort="28" para elbow -->
<joint name="${prefix}arm_elbow_joint" type="revolute">
    <limit lower="-2.0" upper="2.0" effort="28" velocity="1.2"/>
</joint>

<!-- Cambiar de: effort="20,15,10" a effort="28,28,14" para wrists -->
<limit effort="28" velocity="1.5"/>
<limit effort="28" velocity="1.5"/>  <!-- Wrist 2, aumentar velocidad -->
<limit effort="14" velocity="1.5"/>  <!-- Wrist 3, reducir velocidad -->
```

### 2. **Agregar Damping en Joints (MEDIO)**
```xml
<!-- En cada joint, después de <limit> -->
<dynamics damping="0.5" friction="0.0"/>
```

### 3. **Corregir base_footprint en Z (ALTO)**
```xml
<!-- En mm_base_macro.xacro línea 65 -->
<!-- ANTES: -->
<origin xyz="0 0 ${base_mount_z}" rpy="0 0 0"/>

<!-- DESPUÉS: -->
<origin xyz="0 0 0" rpy="0 0 0"/>
<!-- Agregar joint separado para mount height -->
```

### 4. **Agregar Wheel Friction (MEDIO)**
```xml
<!-- En Gazebo, agregar en mm_robot.urdf.xacro -->
<gazebo reference="${prefix}front_left_wheel_link">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
</gazebo>
```

### 5. **Documentar Effort y Velocity (MEDIO)**
```xml
<!-- Agregar comentarios con valores de referencia -->
<!-- UR3: shoulder 150Nm, elbow 28Nm, wrist 28Nm, wrist2 28Nm, wrist3 14Nm -->
<!-- Velocidades: shoulder 1.0 rad/s, arm 1.2, elbow 1.2, wrist 1.5, wrist2 1.8, wrist3 1.5 -->
```

---

##  PRÓXIMOS PASOS

- [ ] Revisar sim_mm_dual.launch.py (665 L) - Multi-robot setup
- [ ] Validar SRDF configuration (MoveIt planning groups)
- [ ] Revisar Nav2 parameters
- [ ] Completar análisis de subsistemas (FASE 3+)

---

**Estado:** [WARN] PENDIENTE CORRECCIONES (1 CRÍTICO, 1 ALTO)  
**Estimado:** 2-3 horas de correcciones

---

##  COMPARATIVA CON ROBOTS INDUSTRIALES

| Parámetro | Actual | UR3 | Diferencia |
|-----------|--------|-----|-----------|
| Base Length | 0.50m | N/A | - |
| Base Width | 0.60m | N/A | - |
| Wheel Radius | 0.07m | N/A | - |
| Arm Length | 1.2m | 0.5m | OK Bigger |
| Shoulder Effort | 50 Nm | 150 Nm | NO 3x Lower |
| Elbow Effort | 40 Nm | 28 Nm | NO 1.4x Higher |
| Wrist Effort | 10-20 Nm | 14-28 Nm | NO Lower |
| Joint Damping | None | 0.5 Nm·s/rad | NO Missing |

