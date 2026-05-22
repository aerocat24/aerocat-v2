#pragma once
#include "header.h"

void wifi_init() {

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());

  // Conecta ao servidor ROS (rosbridge)
  webSocket.begin(rosbridge_address, rosbridge_port, "/");
  webSocket.onEvent(webSocketEvent);
}

void ros_pub_int32(String topic, int data) {

  String publish_msg;

  publish_msg = "{\"op\": \"publish\", \"topic\": \"" + topic +
                "\", \"msg\": {\"data\": " + String(data) + "}}";
  // Serial.println(publish_msg);
  Serial.print("publish_int32: ");
  Serial.println(data);
  webSocket.sendTXT(publish_msg);
}

void ros_pub_float32_multiarray(String topic, double data1, double data2) {

  String publish_msg;

  // Constrói a mensagem JSON para Float32MultiArray com duas posições
  publish_msg = "{\"op\": \"publish\", \"topic\": \"" + topic +
                "\", \"msg\": {\"layout\": {}, \"data\": [" + String(data1, 8) +
                ", " + String(data2, 8) + "]}}";

  // Exibe a mensagem no monitor serial para debug
  Serial.print("publish_float32_multiarray: [");
  Serial.print(data1, 8);
  Serial.print(", ");
  Serial.print(data2, 8);
  Serial.println("]");

  // Envia a mensagem pelo WebSocket
  webSocket.sendTXT(publish_msg);
}

void ros_pub_int32_multiarray(String topic, int data1, int data2) {

  String publish_msg;

  // Constrói a mensagem JSON para Int32MultiArray com duas posições
  publish_msg = "{\"op\": \"publish\", \"topic\": \"" + topic +
                "\", \"msg\": {\"layout\": {}, \"data\": [" + String(data1) +
                ", " + String(data2) + "]}}";

  // Exibe a mensagem no monitor serial para debug
  Serial.print("publish_int32_multiarray: [");
  Serial.print(data1);
  Serial.print(", ");
  Serial.print(data2);
  Serial.println("]");

  // Envia a mensagem pelo WebSocket
  webSocket.sendTXT(publish_msg);
}


void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {

  switch (type) {
  case WStype_TEXT: {
    
    String message = String((char *)payload);

    // Usando ArduinoJson para parsear a mensagem JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);

    // Verifica se o parsing foi bem-sucedido
    if (error) {
      Serial.print("Falha ao parsear JSON: ");
      Serial.println(error.f_str());
      return;
    }

    // verifica se é do topico /pwm
    String topic_received = doc["topic"];

    if (strcmp(topic_received.c_str(), sub_topic_pwm) == 0) {
      // Extrai os valores do JSON (tópico /esp32/data)
      JsonArray data = doc["msg"]["data"].as<JsonArray>();

      if (data.size() >= 2) {

        //verifica se o semaforo está disponivel
        if (xSemaphoreTake(pwmSemaphore, portMAX_DELAY) == pdTRUE) {
          pwm_left_hold = data[0].as<int>();
          pwm_right_hold = data[1].as<int>();
          xSemaphoreGive(pwmSemaphore);
        }

        pwm_write(pwm_left_hold, pwm_right_hold);

      } else {
        Serial.println("Erro: Dados incompletos no array");
      }
    }

    break;
  }

  case WStype_DISCONNECTED:
    Serial.println("Desconectado do WebSocket!");
    // desliga os motores por segurança
    pwm_write(0, 0);
    break;

  case WStype_ERROR:
    Serial.println("Erro no WebSocket!");
    // desliga os motores por segurança
    pwm_write(0, 0);
    break;

  case WStype_CONNECTED:
    Serial.println("Conectado ao WebSocket!");

    // Cadastro de Publishers
    webSocket.sendTXT("{\"op\": \"advertise\", \"topic\": \"" +
                      String(pub_topic_compass) +
                      "\", \"type\": \"std_msgs/Int32\"}");

    webSocket.sendTXT("{\"op\": \"advertise\", \"topic\": \"" +
                      String(pub_topic_gps) +
                      "\", \"type\": \"std_msgs/Float32MultiArray\"}");

    webSocket.sendTXT("{\"op\": \"advertise\", \"topic\": \"" +
                        String(pub_topic_pwm_hold) +
                        "\", \"type\": \"std_msgs/Int32MultiArray\"}");
                      

    // Cadastro de Subscribers
    webSocket.sendTXT("{\"op\": \"subscribe\", \"topic\": \"" +
                      String(sub_topic_pwm) +
                      "\", \"type\": \"std_msgs/Int32MultiArray\"}");

    break;

  default:
    break;
  }
}
