/*
 * Smart Tank Robot - ESP32-CAM firmware
 *
 * Responsibilities:
 *   - camera MJPEG stream (built-in ESP32 camera webserver on port 81)
 *   - HTTP command endpoints on port 80:
 *       /forward /backward /left /right /stop
 *       /servo?angle=N        (0..180)
 *       /scan                  (sweep US sensor, return JSON distances)
 *       /status                (current state JSON)
 *   - low-level motor, servo, ultrasonic control
 *
 * Pin map:
 *   GPIO12 -> L298N IN1   (left side direction A)
 *   GPIO13 -> L298N IN2   (left side direction B)
 *   GPIO14 -> L298N IN3   (right side direction A)
 *   GPIO15 -> L298N IN4   (right side direction B)
 *   GPIO2  -> Servo signal
 *   GPIO16 -> Ultrasonic TRIG
 *   GPIO4  -> Ultrasonic ECHO (through voltage divider to 3.3V)
 *
 * Notes:
 *   - GPIO12 is boot-sensitive - keep it LOW at boot (done in setup()).
 *   - GPIO4 may light the onboard flash LED - confirm on your board.
 *   - HC-SR04 ECHO must be dropped from 5V to ~3.3V with a divider.
 *   - All grounds must be common.
 */

#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include <ESP32Servo.h>

// --------------------------------------------------------------------
// USER CONFIG
// --------------------------------------------------------------------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// --------------------------------------------------------------------
// PINS
// --------------------------------------------------------------------
#define IN1_PIN   12
#define IN2_PIN   13
#define IN3_PIN   14
#define IN4_PIN   15
#define SERVO_PIN  2
#define TRIG_PIN  16
#define ECHO_PIN   4

// --------------------------------------------------------------------
// AI-Thinker camera pins
// --------------------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --------------------------------------------------------------------
// Globals
// --------------------------------------------------------------------
WebServer server(80);
Servo usServo;

String currentState      = "STOPPED";
bool   autonomousMode    = false;        // mirrored from laptop for /status
float  lastDistLeft      = -1.0f;
float  lastDistCenter    = -1.0f;
float  lastDistRight     = -1.0f;

void startCameraServer();   // defined in esp32-camera's app_httpd

// --------------------------------------------------------------------
// Motor helpers
// --------------------------------------------------------------------
void motorStop() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  currentState = "STOPPED";
}

void motorForward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  currentState = "FORWARD";
}

void motorBackward() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  currentState = "BACKWARD";
}

void motorLeft() {
  // spin in place: left backward, right forward
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  currentState = "TURN_LEFT";
}

void motorRight() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  currentState = "TURN_RIGHT";
}

// --------------------------------------------------------------------
// Ultrasonic
// --------------------------------------------------------------------
float measureDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);   // 30 ms timeout
  if (duration == 0) return -1.0f;
  float distance = (duration * 0.0343f) / 2.0f;   // cm
  if (distance < 2.0f || distance > 400.0f) return -1.0f;
  return distance;
}

float measureAverage(int samples = 3) {
  float sum  = 0; int ok = 0;
  for (int i = 0; i < samples; i++) {
    float d = measureDistanceCm();
    if (d > 0) { sum += d; ok++; }
    delay(15);
  }
  return ok ? sum / ok : -1.0f;
}

// --------------------------------------------------------------------
// Scan sweep
// --------------------------------------------------------------------
void performScan() {
  usServo.write(150);  delay(350);
  lastDistLeft   = measureAverage();

  usServo.write(90);   delay(300);
  lastDistCenter = measureAverage();

  usServo.write(30);   delay(350);
  lastDistRight  = measureAverage();

  usServo.write(90);   delay(200);   // re-center
}

// --------------------------------------------------------------------
// JSON helpers
// --------------------------------------------------------------------
String statusJson() {
  String j = "{";
  j += "\"state\":\""          + currentState + "\",";
  j += "\"distance_left\":"    + String(lastDistLeft,   1) + ",";
  j += "\"distance_center\":"  + String(lastDistCenter, 1) + ",";
  j += "\"distance_right\":"   + String(lastDistRight,  1) + ",";
  j += "\"autonomous\":"       + String(autonomousMode ? "true" : "false");
  j += "}";
  return j;
}

// --------------------------------------------------------------------
// HTTP handlers
// --------------------------------------------------------------------
void sendJson(const String& body) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", body);
}

void handleForward()  { motorForward();  sendJson("{\"ok\":true,\"state\":\"FORWARD\"}"); }
void handleBackward() { motorBackward(); sendJson("{\"ok\":true,\"state\":\"BACKWARD\"}"); }
void handleLeft()     { motorLeft();     sendJson("{\"ok\":true,\"state\":\"TURN_LEFT\"}"); }
void handleRight()    { motorRight();    sendJson("{\"ok\":true,\"state\":\"TURN_RIGHT\"}"); }
void handleStop()     { motorStop();     sendJson("{\"ok\":true,\"state\":\"STOPPED\"}"); }

void handleServo() {
  int angle = 90;
  if (server.hasArg("angle")) angle = server.arg("angle").toInt();
  angle = constrain(angle, 0, 180);
  usServo.write(angle);
  sendJson("{\"ok\":true,\"angle\":" + String(angle) + "}");
}

void handleScan() {
  performScan();
  sendJson(statusJson());
}

void handleStatus() {
  // update center measurement quickly for freshness
  float d = measureAverage(1);
  if (d > 0) lastDistCenter = d;
  sendJson(statusJson());
}

void handleNotFound() { server.send(404, "text/plain", "not found"); }

// --------------------------------------------------------------------
// Setup / loop
// --------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // motor pins LOW *before* anything else so GPIO12 isn't HIGH at boot
  pinMode(IN1_PIN, OUTPUT); digitalWrite(IN1_PIN, LOW);
  pinMode(IN2_PIN, OUTPUT); digitalWrite(IN2_PIN, LOW);
  pinMode(IN3_PIN, OUTPUT); digitalWrite(IN3_PIN, LOW);
  pinMode(IN4_PIN, OUTPUT); digitalWrite(IN4_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  // servo
  usServo.setPeriodHertz(50);
  usServo.attach(SERVO_PIN, 500, 2400);
  usServo.write(90);

  // camera init
  camera_config_t cam_config;
  cam_config.ledc_channel = LEDC_CHANNEL_0;
  cam_config.ledc_timer   = LEDC_TIMER_0;
  cam_config.pin_d0       = Y2_GPIO_NUM;
  cam_config.pin_d1       = Y3_GPIO_NUM;
  cam_config.pin_d2       = Y4_GPIO_NUM;
  cam_config.pin_d3       = Y5_GPIO_NUM;
  cam_config.pin_d4       = Y6_GPIO_NUM;
  cam_config.pin_d5       = Y7_GPIO_NUM;
  cam_config.pin_d6       = Y8_GPIO_NUM;
  cam_config.pin_d7       = Y9_GPIO_NUM;
  cam_config.pin_xclk     = XCLK_GPIO_NUM;
  cam_config.pin_pclk     = PCLK_GPIO_NUM;
  cam_config.pin_vsync    = VSYNC_GPIO_NUM;
  cam_config.pin_href     = HREF_GPIO_NUM;
  cam_config.pin_sscb_sda = SIOD_GPIO_NUM;
  cam_config.pin_sscb_scl = SIOC_GPIO_NUM;
  cam_config.pin_pwdn     = PWDN_GPIO_NUM;
  cam_config.pin_reset    = RESET_GPIO_NUM;
  cam_config.xclk_freq_hz = 20000000;
  cam_config.pixel_format = PIXFORMAT_JPEG;
  cam_config.frame_size   = FRAMESIZE_QVGA;  // 320x240 -> cheap & fast
  cam_config.jpeg_quality = 12;
  cam_config.fb_count     = psramFound() ? 2 : 1;

  if (esp_camera_init(&cam_config) != ESP_OK) {
    Serial.println("Camera init failed - restarting");
    delay(2000);
    ESP.restart();
  }

  // wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // camera MJPEG server on :81/stream (provided by esp32-camera library)
  startCameraServer();

  // HTTP command server on :80
  server.on("/forward",  HTTP_GET, handleForward);
  server.on("/backward", HTTP_GET, handleBackward);
  server.on("/left",     HTTP_GET, handleLeft);
  server.on("/right",    HTTP_GET, handleRight);
  server.on("/stop",     HTTP_GET, handleStop);
  server.on("/servo",    HTTP_GET, handleServo);
  server.on("/scan",     HTTP_GET, handleScan);
  server.on("/status",   HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Command server ready on :80");
}

unsigned long lastCenterProbe = 0;

void loop() {
  server.handleClient();

  // opportunistic center distance refresh so /status is always recent
  if (millis() - lastCenterProbe > 200) {
    float d = measureDistanceCm();
    if (d > 0) lastDistCenter = d;
    lastCenterProbe = millis();
  }
}
