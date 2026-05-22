#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WebSocketsClient.h>
#include <ESP32Servo.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigenDense.h>

#define PWM_LEFT_PIN 19
#define PWM_RIGHT_PIN 18
#define GPS_RX_PIN 16 // Pino RX do ESP32 para o módulo GPS
#define GPS_TX_PIN 17  // Pino TX do ESP32 para o módulo GPS
#define GPS_BAUD 9600
#define TYPE_COMMUNICATION 2 // 1 WiFI 2 Ethernet

#define INTERVAL 500

const char* ssid = "tp-link_ROS";
const char* password = "sirolab2024";

// Configuração do WebSocket
WebSocketsClient webSocket;
#if TYPE_COMMUNICATION == 1
    const char *rosbridge_address =  "192.168.0.105"; // Endereço do servidor ROS (rosbridge), IP do meu computador
#else 
    const char *rosbridge_address =  "192.168.0.106"; // Endereço do servidor ROS (rosbridge), IP do meu computador
#endif

//192.168.0.100 ethernet
//192.168.0.101 wifi
const int rosbridge_port = 9090;

int pwm_left_hold = 0.0;
int pwm_right_hold = 0.0;

float gps_lat;
float gps_lng;

int angle;

//Publishers
const char *pub_topic_compass = "/boat/compass";
const char *pub_topic_gps = "/boat/gps";
const char *pub_topic_pwm_hold = "/boat/pwm_hold";

//Subscribers
const char *sub_topic_pwm = "/boat/pwm";

void ros_pub_int32(String topic, int data);
void ros_pub_int32_multiarray(String topic, int data1, int data2);
void ros_pub_float32_multiarray(String topic, double data1, double data2);
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void pwm_write(int pwm_left, int pwm_right);
void taskSendData(void *pvParameters);

Servo esc_left; // motor direito
Servo esc_right; // motor direito

// Objeto TinyGPS++ para trabalhar com os dados do GPS
TinyGPSPlus gps;

// Comunicação Serial para comunicar-se com GPS
HardwareSerial SerialGPS(2); // Use a porta serial 2 do ESP32 (Serial2)
QMC5883LCompass compass;


SemaphoreHandle_t pwmSemaphore;