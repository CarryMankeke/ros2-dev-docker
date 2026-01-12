// ESP32 con shield Funduino y micro-ROS por WiFi/UDP que publica /joy.
//
// Requisitos (Arduino IDE / PlatformIO):
// - Placa: ESP32 (nucleo Arduino)
// - Libreria: micro_ros_arduino
// - Lado ROS: agente micro-ROS en ejecucion (udp4 puerto 8888 por defecto)
//
// Layout de botones (shield Funduino):
// - A/B/C/D (cruceta)
// - E (cambio de banco de articulaciones)
// - F (deadman)
// - SW (click del joystick, cambio de modo)
//
// Prueba minima: abrir el monitor serial (115200), mover el stick y pulsar botones.
//
// Nota: el shield trae pines con nomenclatura de Arduino; aqui se asignan a GPIO reales del ESP32.
// Ajusta los GPIO mas abajo segun tu cableado.

#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joy.h>

// =======================
// WiFi y agente micro-ROS
// =======================
static char *WIFI_SSID     = "Galaxy A52s 5G CB0F";
static char *WIFI_PASSWORD = "hzxd2620";

// IP del equipo o router donde corre el agente micro-ROS (en la misma WiFi)
static char AGENT_IP[] = "10.96.130.73";
static const uint AGENT_PORT = 8888;

// =======================
// Frecuencia de publicación
// =======================
static const uint32_t PUB_HZ = 100;
static const uint32_t PUB_PERIOD_MS = 1000 / PUB_HZ;

// =======================
// Mapeo de pines (ajusta según tu hardware)
// =======================
// Ejes analógicos (el shield muestra A0/A1); en ESP32 se usan GPIO con ADC.
// Evita ADC2 cuando usas WiFi; mejor ADC1 (GPIO32-39).
static const int PIN_X = 34; // ADC1 en ESP32
static const int PIN_Y = 35; // ADC1 en ESP32

// Botones digitales (GPIO libres con resistencia pull-up interna)
static const int PIN_SW = 25; // clic del joystick

static const int PIN_A  = 26;
static const int PIN_B  = 27;
static const int PIN_C  = 14;
static const int PIN_D  = 12;

static const int PIN_E = 13;
static const int PIN_F = 33;

// =======================
// Estructura del mensaje Joy
// =======================
// ejes:
//  0: left_stick_x  (-1..1)  eje X del joystick
//  1: left_stick_y  (-1..1)  eje Y del joystick
//  2: dpad_x        (-1,0,1) A/D
//  3: dpad_y        (-1,0,1) C/B
//
// botones:
//  0 A
//  1 B
//  2 C
//  3 D
//  4 E
//  5 F
//  6 SW

static const size_t AXES_LEN = 4;
static const size_t BUTTONS_LEN = 7;

// =======================
// Objetos de micro-ROS
// =======================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t joy_pub;

rcl_timer_t timer;
rclc_executor_t executor;

sensor_msgs__msg__Joy joy_msg;

// Estado de la aplicación
uint32_t publish_count = 0;

// =============== utilidades de depuración ===============
const char* wifi_status_to_str(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:    return "IDLE";
    case WL_NO_SSID_AVAIL:  return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_DONE";
    case WL_CONNECTED:      return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:return "CONNECTION_LOST";
    case WL_DISCONNECTED:   return "DISCONNECTED";
    default:                return "UNKNOWN";
  }
}

void log_rcl_ret(const char *label, rcl_ret_t ret) {
  if (ret != RCL_RET_OK) {
    Serial.printf("[micro-ROS] %s failed: %d\r\n", label, (int)ret);
  } else {
    Serial.printf("[micro-ROS] %s OK\r\n", label);
  }
}

void wait_for_wifi() {
  Serial.printf("[WiFi] Conectando a SSID '%s'...\r\n", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(250);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    Serial.printf("\r\n[WiFi] Estado: %s. Reintentando en 1s...\r\n", wifi_status_to_str(WiFi.status()));
    delay(1000);
  }
  Serial.printf("\r\n[WiFi] Conectado. IP: %s\r\n", WiFi.localIP().toString().c_str());
}

// =============== funciones auxiliares ===============
static inline int read_button(int pin) {
  // Con pull-up activa, PRESIONADO equivale a nivel bajo
  return digitalRead(pin) == LOW ? 1 : 0;
}

static inline void setup_button_pin(int pin) {
  pinMode(pin, INPUT_PULLUP);
}

static inline float map_axis_to_unit(int raw, int center, int deadband) {
  int v = raw - center;
  if (abs(v) < deadband) return 0.0f;

  // Normaliza a [-1,1] asumiendo ADC 0..4095 (12 bits en ESP32)
  float f = (float)v / 2048.0f;
  if (f > 1.0f) f = 1.0f;
  if (f < -1.0f) f = -1.0f;
  return f;
}

// Temporizador: publica el mensaje /joy
void timer_cb(rcl_timer_t *timer, int64_t /*last_call_time*/) {
  if (timer == NULL) return;

  // Lectura de botones
  int bA = read_button(PIN_A);
  int bB = read_button(PIN_B);
  int bC = read_button(PIN_C);
  int bD = read_button(PIN_D);
  int bE = read_button(PIN_E);
  int bF = read_button(PIN_F);
  int bSW = read_button(PIN_SW);

  // Ejes analógicos
  int rawX = analogRead(PIN_X);
  int rawY = analogRead(PIN_Y);

  // Supone un centro aproximado en 2048; se puede calibrar al inicio si hace falta.
  const int center = 2048;
  const int deadband = 80;

  float x = map_axis_to_unit(rawX, center, deadband);
  float y = map_axis_to_unit(rawY, center, deadband);

  // Cruz direccional virtual con A/D y C/B
  int dpad_x = (bD ? 1 : 0) + (bA ? -1 : 0);
  int dpad_y = (bC ? 1 : 0) + (bB ? -1 : 0);

  // Deadman: si no está pulsado, se envían ceros por seguridad
  if (!bF) {
    x = 0.0f;
    y = 0.0f;
    dpad_x = 0;
    dpad_y = 0;
  }

  // Carga los ejes del mensaje Joy
  joy_msg.axes.data[0] = x;
  joy_msg.axes.data[1] = y;
  joy_msg.axes.data[2] = (float)dpad_x;
  joy_msg.axes.data[3] = (float)dpad_y;

  joy_msg.buttons.data[0] = bA;
  joy_msg.buttons.data[1] = bB;
  joy_msg.buttons.data[2] = bC;
  joy_msg.buttons.data[3] = bD;
  joy_msg.buttons.data[4] = bE;
  joy_msg.buttons.data[5] = bF;
  joy_msg.buttons.data[6] = bSW;

  // Publica el mensaje
  rcl_publish(&joy_pub, &joy_msg, NULL);

  publish_count++;
  if (publish_count % (PUB_HZ * 2) == 0) { // cada ~2 segundos a 50 Hz
    Serial.printf("[PUB] axes: %.2f %.2f dpad=%d,%d deadman=%d\r\n",
                  x, y, dpad_x, dpad_y, bF);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Configuración del ADC
  analogSetAttenuation(ADC_11db);
  // Si está disponible: analogReadResolution(12);

  // Botones con resistencia pull-up
  setup_button_pin(PIN_SW);
  setup_button_pin(PIN_A);
  setup_button_pin(PIN_B);
  setup_button_pin(PIN_C);
  setup_button_pin(PIN_D);
  setup_button_pin(PIN_E);
  setup_button_pin(PIN_F);

  // Transporte WiFi de micro-ROS sobre UDP
  wait_for_wifi();
  Serial.printf("[micro-ROS] Configurando transporte WiFi -> agente %s:%u\r\n", AGENT_IP, AGENT_PORT);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

  allocator = rcl_get_default_allocator();

  // Inicializa soporte y nodo
  log_rcl_ret("support_init", rclc_support_init(&support, 0, NULL, &allocator));
  log_rcl_ret("node_init", rclc_node_init_default(&node, "esp32_joy_node", "", &support));

  // Publicador del tópico /joy
  log_rcl_ret(
    "publisher_init",
    rclc_publisher_init_default(
    &joy_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"
    )
  );

  // Reserva la memoria para ejes y botones
  joy_msg.axes.data = (float*)malloc(sizeof(float) * AXES_LEN);
  joy_msg.axes.size = AXES_LEN;
  joy_msg.axes.capacity = AXES_LEN;

  joy_msg.buttons.data = (int32_t*)malloc(sizeof(int32_t) * BUTTONS_LEN);
  joy_msg.buttons.size = BUTTONS_LEN;
  joy_msg.buttons.capacity = BUTTONS_LEN;

  // Configura temporizador y ejecutor
  log_rcl_ret(
    "timer_init",
    rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(PUB_PERIOD_MS),
    timer_cb
    )
  );

  log_rcl_ret("executor_init", rclc_executor_init(&executor, &support.context, 1, &allocator));
  log_rcl_ret("executor_add_timer", rclc_executor_add_timer(&executor, &timer));

  Serial.println("micro-ROS /joy publisher ready");
}

void loop() {
  // Si el WiFi se cae, intenta reconectar sin bloquear
  static uint32_t last_reconnect = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - last_reconnect > 5000) {
    Serial.printf("[WiFi] Estado %s, reintentando...\r\n", wifi_status_to_str(WiFi.status()));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    last_reconnect = millis();
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  delay(2);
}
