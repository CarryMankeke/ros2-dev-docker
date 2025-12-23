// pseudocódigo firmware ESP32 + shield Funduino para publicar sensor_msgs/Joy vía micro-ROS
// Compatibilidad: Windows 10 x64 (WSL2) o macOS arm64 corriendo agente micro-ROS por UDP 8888.
// Seguridad: botón F como deadman, watchdog de conexión WiFi.

procedimiento setup():
  crear:
    - configurar pines del shield: botones A-F, joystick con ADC en VRx/VRy, botón SW
    - inicializar WiFi con SSID/PASS y agente micro-ROS (set_microros_wifi_transports)
    - inicializar nodo micro-ROS "esp32_joy" y publisher /joy
  leer:
    - calibrar ejes analógicos (centro ~2048) y debounce de botones
  actualizar:
    - publicar sensor_msgs/Joy cada 20 ms con axes[0..5], buttons[0..7]
    - asignar modo operativo (BASE/BRAZO/COMBO) según combinación de botones
    - reenviar heartbeats al agente; si falla, reintentar conexión
  borrar:
    - apagar WiFi y detener publicaciones al recibir señal de parada
