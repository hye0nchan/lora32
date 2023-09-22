#include <LoRa.h>
#include <ArduinoJson.h>
#include "boards.h"
#include <WiFi.h>
#include <QueueArray.h>

#define MAX_MESSAGE_SIZE 256  // 이 부분을 추가


void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 0, 4);
  initBoard();
  // When the power is turned on, a delay is required.
  delay(1500);
  Serial.println("LoRa Receiver");

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
  if (!LoRa.begin(LoRa_frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop()
{
  //  delay(10000);
  //   ESP.restart();
    String isSend = "no";
    while (Serial2.available()) {
    String fromRaspberry = Serial2.readStringUntil('\n');

    // Remove any residual carriage return character
    fromRaspberry.trim();

    // Print the received string (for debugging)
    Serial.println("Received from Raspberry: " + fromRaspberry);

    if (fromRaspberry == "sensor1") {
      LoRa.beginPacket();
      LoRa.print("sensor1");
      LoRa.endPacket();
    }

    else if (fromRaspberry == "sensor0") {
      LoRa.beginPacket();
      LoRa.print("sensor0");
      LoRa.endPacket();
    }
  }

    // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    char recv[MAX_MESSAGE_SIZE];
    int len = LoRa.readBytesUntil('\n', recv, MAX_MESSAGE_SIZE);
    recv[len] = '\0';
    char* packet = strdup(recv);

    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, packet);
    if (error) {
            Serial.println("JSON parsing failed");
        } else {
            Serial.print("packet: ");
            String output;
            serializeJson(doc, output);  // JSON 객체를 시리얼로 출력
            output += "\n";
            Serial.println(output);
            Serial2.println(output);
            doc.clear();
            delay(1000);
            isSend = "yes";

    #ifdef HAS_DISPLAY
    if (u8g2) {
        char buf[256];
        u8g2->setFlipMode(1);
        u8g2->clearBuffer();
        snprintf(buf, sizeof(buf), "send?: %s", isSend);
        u8g2->drawStr(0, 45, buf);
        u8g2->sendBuffer();
    }
    #endif
    }
      free(packet); 
  }

}
