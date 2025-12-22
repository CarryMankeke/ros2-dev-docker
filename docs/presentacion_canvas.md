# Plantilla de presentacion (Canvas) - Proyecto simulacion de robot

Guia breve para armar las slides en Canvas cumpliendo los requisitos del curso. Ajusta el contenido con tu robot, capturas y datos personales.

## Formato base
- Tipografia uniforme para titulos y texto (p. ej. Sans clean).
- Colores: fondo claro con acentos en 1-2 colores (ej. azul/gris). Evita mezclar demasiados tonos.
- Numerar paginas y numerar tablas/figuras (Fig. 1, Tabla 1).
- Resolucion de imagenes: usa capturas nativas de RViz/Gazebo en buena calidad.
- Referencias en formato ISO690 o IEEE; unifica el estilo.

## Estructura sugerida de slides
1) **Portada**
   - Membrete + logo universidad/departamento.
   - Titulo: "Simulacion de robot en ROS2, Gazebo y RViz".
   - Nombre del estudiante, profesor, fecha, semestre.

2) **Indice**
   - Lista de secciones principales.

3) **Introduccion y objetivo**
   - Objetivo del proyecto.
   - Herramientas usadas (ROS2 Jazzy, Gazebo, RViz, micro-ROS, ros2_control).

4) **Diseno del robot (URDF/XACRO)**
   - Tipo de robot (diferencial/omni/ackermann/articulado).
   - Componentes clave y frames (base, ruedas/articulaciones, sensores).
   - (Fig. 1) Diagrama o captura del modelo.

5) **Sensores y plugins**
   - Sensores incluidos (ej. LIDAR, camara) y plugins de Gazebo.
   - Topicos publicados y frecuencia si aplica.
   - (Fig. 2) Captura de vista de sensor o esquema.

6) **Control con ros2_control**
   - Controlador usado (diff/ackermann/arm), joints controlados.
   - Teleop por teclado y topicos de comando.
   - Diagrama breve del flujo de control.

7) **Integracion micro-ROS**
   - Nodo o firmware usado, topicos de entrada/salida.
   - Esquema de conectividad (ROS2 host <-> micro-ROS).

8) **Visualizacion en RViz**
   - RobotModel y TF visibles.
   - Uso de joint_state_publisher para articulaciones.
   - (Fig. 3) Captura de RViz mostrando modelo y frames.

9) **Simulacion en Gazebo**
   - Mundo cargado y procedimiento de spawn.
   - Sensores funcionando en simulacion.
   - (Fig. 4) Captura de Gazebo con robot y sensores.

10) **Launchfiles y flujo de ejecucion**
    - Principales launch (display, gazebo, control).
    - Comando clave: `ros2 launch ...`.
    - Diagrama sencillo de arranque.

11) **Resultados y pruebas**
    - Movimientos logrados, lectura de sensores.
    - Breve tabla o lista de pruebas realizadas.
    - (Fig. 5) Captura/clip de robot moviendose.

12) **Conclusiones y trabajo futuro**
    - Logros, limitaciones, ideas de mejora.

13) **Referencias**
    - Lista en ISO690 o IEEE.

## Notas rapidas para Canvas
- Crea una pagina por slide con el orden anterior; activa numeracion de pagina.
- Usa cajas de texto separadas para titulos, cuerpo y pies de figura.
- Para figuras, incluye caption y numerala (ej. "Fig. 2 - Vista de LIDAR en Gazebo").
- Verifica contraste (texto oscuro sobre fondo claro) y alineacion consistente.
- Exporta o publica manteniendo la resolucion de las capturas.
