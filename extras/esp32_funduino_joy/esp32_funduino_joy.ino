// ESP32 + Funduino Joystick Shield + micro-ROS (WiFi/UDP) -> publishes /joy
//
// Requirements (Arduino IDE / PlatformIO):
// - Board: ESP32 (Arduino core)
// - Library: micro_ros_arduino
// - ROS side: micro-ROS Agent running (udp4 port 8888 typical)
//
// Buttons (shield):
//  A,B,C,D,E(Select),F(Start=deadman),SW(click)
//
// Modes:
//  0 = BASE
//  1 = ARM
//  2 = COMBO
//
// NOTE: The shield has Arduino pin labels. On ESP32, map them to real GPIO.
// Edit the GPIO pins below to match your wiring.

#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joy.h>

// =======================
// WiFi + Agent
// =======================
static char *WIFI_SSID     = "Galaxy A52S 5G CB0F";
static char *WIFI_PASSWORD = "hzxd2620";

// IP of the PC/router where micro-ROS agent runs (same WiFi)
static char AGENT_IP[] = "192.168.1.100";
static const uint AGENT_PORT = 8888;

// =======================
// Publish rate
// =======================
static const uint32_t PUB_HZ = 50;
static const uint32_t PUB_PERIOD_MS = 1000 / PUB_HZ;

// =======================
// Pin mapping (ADJUST)
// =======================
// Analog axes (shield says A0/A1). On ESP32 use ADC GPIOs.
// Avoid ADC2 with WiFi; prefer ADC1 (GPIO32-39).
static const int PIN_X = 34; // ADC1
static const int PIN_Y = 35; // ADC1

// Digital buttons (use free GPIOs; with internal pullup)
static const int PIN_SW = 25; // joystick click

static const int PIN_A  = 26;
static const int PIN_B  = 27;
static const int PIN_C  = 14;
static const int PIN_D  = 12;

static const int PIN_E_SELECT = 13; // Select = cycle mode
static const int PIN_F_START  = 33; // Start = deadman

// =======================
// Joy message layout
// =======================
// axes:
//  0: left_stick_x  (-1..1)  joystick X
//  1: left_stick_y  (-1..1)  joystick Y
//  2: dpad_x        (-1,0,1) buttons A/D (left/right)
//  3: dpad_y        (-1,0,1) buttons B/C (down/up)
//  4: mode          (0,1,2)  BASE/ARM/COMBO
//
// buttons:
//  0 A
//  1 B
//  2 C
//  3 D
//  4 E (Select)
//  5 F (Start / deadman)
//  6 SW (home request)

static const size_t AXES_LEN = 5;
static const size_t BUTTONS_LEN = 7;

// =======================
// micro-ROS objects
// =======================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t joy_pub;

rcl_timer_t timer;
rclc_executor_t executor;

sensor_msgs__msg__Joy joy_msg;

// State
volatile int mode = 0; // 0=BASE,1=ARM,2=COMBO
bool prev_select = false;

// =============== helpers ===============
static inline int read_button(int pin) {
  // With pullup: PRESSED = LOW
  return digitalRead(pin) == LOW ? 1 : 0;
}

static inline float map_axis_to_unit(int raw, int center, int deadband) {
  int v = raw - center;
  if (abs(v) < deadband) return 0.0f;

  // Normalize to [-1,1], assuming ADC 0..4095 (ESP32 12-bit)
  float f = (float)v / 2048.0f;
  if (f > 1.0f) f = 1.0f;
  if (f < -1.0f) f = -1.0f;
  return f;
}

// Timer callback: publish /joy
void timer_cb(rcl_timer_t *timer, int64_t /*last_call_time*/) {
  if (timer == NULL) return;

  // Buttons
  int bA  = read_button(PIN_A);
  int bB  = read_button(PIN_B);
  int bC  = read_button(PIN_C);
  int bD  = read_button(PIN_D);
  int bE  = read_button(PIN_E_SELECT);
  int bF  = read_button(PIN_F_START);  // deadman
  int bSW = read_button(PIN_SW);       // home request

  // Rising edge to change mode with Select (E)
  bool sel = (bE == 1);
  if (sel && !prev_select) {
    mode = (mode + 1) % 3; // BASE->ARM->COMBO->...
  }
  prev_select = sel;

  // Axes
  int rawX = analogRead(PIN_X);
  int rawY = analogRead(PIN_Y);

  // Quick center assumption ~2048. You can calibrate at boot if needed.
  const int center = 2048;
  const int deadband = 80;

  float x = map_axis_to_unit(rawX, center, deadband);
  float y = map_axis_to_unit(rawY, center, deadband);

  // Virtual D-pad with A/D and B/C
  // A=left, D=right, C=up, B=down
  int dpad_x = (bD ? 1 : 0) + (bA ? -1 : 0);
  int dpad_y = (bC ? 1 : 0) + (bB ? -1 : 0);

  // Deadman: if not pressed, send zeros (safety)
  if (!bF) {
    x = 0.0f;
    y = 0.0f;
    dpad_x = 0;
    dpad_y = 0;
  }

  // Fill Joy
  joy_msg.axes.data[0] = x;
  joy_msg.axes.data[1] = y;
  joy_msg.axes.data[2] = (float)dpad_x;
  joy_msg.axes.data[3] = (float)dpad_y;
  joy_msg.axes.data[4] = (float)mode;

  joy_msg.buttons.data[0] = bA;
  joy_msg.buttons.data[1] = bB;
  joy_msg.buttons.data[2] = bC;
  joy_msg.buttons.data[3] = bD;
  joy_msg.buttons.data[4] = bE;
  joy_msg.buttons.data[5] = bF;
  joy_msg.buttons.data[6] = bSW;

  // Publish
  rcl_publish(&joy_pub, &joy_msg, NULL);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC setup
  analogSetAttenuation(ADC_11db);
  // If supported: analogReadResolution(12);

  // Buttons with pullup
  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(PIN_A,  INPUT_PULLUP);
  pinMode(PIN_B,  INPUT_PULLUP);
  pinMode(PIN_C,  INPUT_PULLUP);
  pinMode(PIN_D,  INPUT_PULLUP);
  pinMode(PIN_E_SELECT, INPUT_PULLUP);
  pinMode(PIN_F_START,  INPUT_PULLUP);

  // micro-ROS WiFi transport (UDP)
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

  allocator = rcl_get_default_allocator();

  // Init support + node
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_joy_node", "", &support);

  // Publisher /joy
  rclc_publisher_init_default(
    &joy_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"
  );

  // Prepare sequences
  joy_msg.axes.data = (float*)malloc(sizeof(float) * AXES_LEN);
  joy_msg.axes.size = AXES_LEN;
  joy_msg.axes.capacity = AXES_LEN;

  joy_msg.buttons.data = (int32_t*)malloc(sizeof(int32_t) * BUTTONS_LEN);
  joy_msg.buttons.size = BUTTONS_LEN;
  joy_msg.buttons.capacity = BUTTONS_LEN;

  // Timer + executor
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(PUB_PERIOD_MS),
    timer_cb
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  Serial.println("micro-ROS /joy publisher ready");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  delay(2);
}
