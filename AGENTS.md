# AGENTS: Lineamientos para este repositorio

Este archivo sirve como guía experta para quienes colaboren con el proyecto de teleoperación ROS 2 Jazzy + Gazebo Harmonic. Se aplica a **todo el árbol del repositorio**.

## Estilo de contribución
- Mantén los cambios pequeños y autoexplicativos; agrupa ediciones relacionadas en un único commit bien titulado.
- Añade resúmenes claros en español en las descripciones de PR y commits; incluye por qué el cambio es necesario.
- Sigue las convenciones ROS 2: `rclcpp`/`rclpy` sin atajos, nombres de nodos y tópicos en `snake_case`, parámetros en YAML con comentarios breves.
- Evita envolturas `try/catch` alrededor de imports o includes.

## Documentación y pseudocódigo
- Escribe en español neutral; incluye glosario o notas si se usan siglas menos comunes.
- Prefiere listas numeradas o secciones con encabezados para detallar flujos, mapeos de controles y dependencias.
- Incluye advertencias de seguridad (watchdogs, parada de emergencia) cuando describas nodos de control.
- Cita fuentes oficiales (ROS 2 Jazzy, Gazebo Harmonic, Nav2, MoveIt 2) cuando justifiques una práctica recomendada.

## Código y pruebas
- Si agregas scripts o configuraciones, incluye pasos de ejecución o comandos de prueba mínimos en el archivo modificado.
- Usa QoS explícito para tópicos críticos; documenta valores y razones.
- Mantén la sincronización de `use_sim_time` y `clock` en ejemplos de lanzamiento.

## Estructura y trazabilidad
- Coloca nuevos documentos bajo `docs/` con nombres descriptivos; referencia desde el README si es contenido principal.
- No muevas archivos grandes sin justificarlo en el commit message.
- Cuando definas mapeos de joystick, indica modo base/brazo y los presets del gripper de forma consistente en todo el repositorio.
