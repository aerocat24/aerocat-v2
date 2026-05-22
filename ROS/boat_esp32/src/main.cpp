#include "comunnication.h"
#include "general_function.h"
#include "header.h"
#include <Arduino.h>

void setup() {

  Serial.begin(115200);

  // configuração bussula
  compass.init();
  compass.setCalibrationOffsets(-184.00, -30.00, 163.00);
  compass.setCalibrationScales(0.79, 0.78, 2.17);

  // configuração gps
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  wifi_init();

  esc_left.attach(PWM_LEFT_PIN, 1000, 2000);
  esc_right.attach(PWM_RIGHT_PIN, 1000, 2000);

  //tempo para a configuração dos esc's
  delay(7000);  

  esc_left.write(0);  // pwm inicial
  esc_right.write(0); // pwm inicial

  pwmSemaphore = xSemaphoreCreateBinary(); //cria semáforo binário
  xSemaphoreGive(pwmSemaphore); // coloca como disponível

  xTaskCreatePinnedToCore(
    taskSendData,       // função da tarefa
    "taskSendData",     // nome
    4096,                 // stack
    NULL,                 // param
    1,                    // prioridade
    NULL,                 // handle
    1                    // núcleo (ESP32 core 1)
  );
}

void loop() {
  
  //verifica se tem eventos acontecendo no callback
  webSocket.loop();

}

void taskSendData(void *pvParameters){
  
  while(1){

    if (WiFi.status() == WL_CONNECTED && webSocket.isConnected())  {

      while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
      }
  
      if (gps.location.isUpdated()) {
  
        gps_lat = gps.location.lat();
        gps_lng = gps.location.lng();
  
        // ler valor da bussula
        compass.read();
        angle = compass.getAzimuth() -30;
  
        // normaliza o angulo para a faixa [-180, 180)
  
        if (angle > 180) {
          angle -= 360;
        } else if (angle < -180) {
          angle += 360;
        }
  
        Serial.print("LAT: ");
        Serial.println(gps_lat, 8);
        Serial.print("LONG: ");
        Serial.println(gps_lng, 8);
  
        Serial.print("angle: ");
        Serial.println(angle);
        Serial.println("");
        
        //verifica se o semaforo está disponivel
        if (xSemaphoreTake(pwmSemaphore, portMAX_DELAY) == pdTRUE) {
          // publica o pwm hold
          ros_pub_int32_multiarray(pub_topic_pwm_hold, pwm_left_hold, pwm_right_hold);
          xSemaphoreGive(pwmSemaphore);
        }
        // publica o gps
        ros_pub_float32_multiarray(pub_topic_gps, gps_lat, gps_lng);
        // publica a bussula
        ros_pub_int32(pub_topic_compass, angle);
      }
  
    } 
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
