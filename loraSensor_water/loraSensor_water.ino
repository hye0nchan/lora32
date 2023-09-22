#include <HardwareSerial.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <EEPROM.h> 
#include <Preferences.h>
#include "boards.h"

const int MAX_BUFFER = 64; //리시브 버퍼 사이즈 설정
const int DE = 2; //DE로 사용할 핀 설정
const int RE = 15; //RE로 사용할 핀 설정
const int RX_PIN = 4;
const int TX_PIN = 0;
const int SERIAL_BAUDRATE = 115200;
const int SERIAL2_BAUDRATE = 9600;
const int MIN_BYTE_RECEIVED = 11;
const int MAX_BYTE_JSON = 512;
const int DELAY_TIME  =10;
const int LORA_PACKET_SIZE_THRESHOLD = 0;
const int maxRetries = 3;

int loRaCount = 0;
bool loRaOnOff = false;

float tempData = 0.0;
float humData = 0.0;
float ecData = 0.0;
float phData = 0.0;

unsigned long lastTime = 0;
unsigned long interval = 5000;

DynamicJsonDocument doc(1024);
Preferences prefs;
const byte sensorRequestFrame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09}; // 센서 리퀘스트 값

void handleLoRaInitError();
void clearSerialBuffer(HardwareSerial &serial);
void sendData(const byte *data, size_t len);
void handleNotEnoughData();
void sendLoRaPacket();

void readAndPrintSensorData(int retryCount = 0);

void setup() {
  initBoard(); // LoRa 보드 초기화
  delay(1500);

  Serial.println("LoRa Sender");
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN); // Lora 핀 초기화

  if (!LoRa.begin(LoRa_frequency)) { // 설정된 주파수에 맞게 LoRa 시작
    handleLoRaInitError();
  }

  Serial2.begin(SERIAL2_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // 0번과 4번핀을 RX, TX로 지정 후, Serial 통신 시작
  Serial.begin(SERIAL_BAUDRATE); 
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);

  prefs.begin("serialNumber", false);
  //prefs.putString("serialNumber", "2305193Fg7");
  prefs.putInt("nodeSequence", 0);
  prefs.putInt("nodeType", 0);
  
  delay(1000);

  String readSerialNumber = prefs.getString("serialNumber"," ");
  Serial.print(readSerialNumber);

  sendData(sensorRequestFrame, sizeof(sensorRequestFrame));
  delay(10);
  readAndPrintSensorData();
}

void loop() {
  doc.clear();
  doc["id"] = 1;
  int packetSize = LoRa.parsePacket();
  if (packetSize > LORA_PACKET_SIZE_THRESHOLD) {
        Serial.print("Received packet '");
        String recv = "";
        while (LoRa.available()) {
            recv += (char)LoRa.read();
        }
        if(recv == "sensor1"){
          loRaOnOff = true;
          Serial.println(recv);
          clearSerialBuffer(Serial2);
          sendData(sensorRequestFrame, sizeof(sensorRequestFrame));
          delay(DELAY_TIME);
          readAndPrintSensorData();
          sendLoRaPacket();
      }
  }
// delay(5000);

#ifdef HAS_DISPLAY
    if (u8g2) {
        char buf[256];
        u8g2->setFlipMode(1);
        u8g2->clearBuffer();
        u8g2->drawStr(0, 12, "Transmitting: OK!");
        snprintf(buf, sizeof(buf), "ec: %f", ecData);
        u8g2->drawStr(0, 45, buf);
        snprintf(buf, sizeof(buf), "pH: %f", phData);
        u8g2->drawStr(0, 60, buf);
        snprintf(buf, sizeof(buf), "temp: %f", tempData);
        u8g2->drawStr(0, 30, buf);
        u8g2->sendBuffer();
    }
#endif


unsigned long currentTime = millis();

if(currentTime - lastTime >= interval){
  loRaOnOff = false;
  lastTime = currentTime;
  sendData(sensorRequestFrame, sizeof(sensorRequestFrame));
  delay(10);
  readAndPrintSensorData();
  delay(1);
  clearSerialBuffer(Serial2);
}
}

void handleLoRaInitError(){
  Serial.println("Starting LoRa failed!");
  while(1);
}

void clearSerialBuffer(HardwareSerial &serial) { // Serial 버퍼 초기화
  while(serial.available()) serial.read();
}

void sendData(const byte *data, size_t len) { // 센서에 데이터 요청하는 함수
  digitalWrite(DE, HIGH); // HiGH = 송신, LOW = 수신
  digitalWrite(RE, HIGH);
  delay(1);
  Serial2.write(data, len); // 데이터와 길이를 받고 Serial을 통해 송신
  Serial2.flush(); // 버퍼 초기화
  delay(1);
  digitalWrite(DE, LOW); // 수신 모드로 변경
  digitalWrite(RE, LOW);
}

void handleNotEnoughData(){
  doc["error"] = "not enough data";
  String output;
  serializeJson(doc, output);
  if(loRaOnOff){
    LoRa.beginPacket();
    LoRa.print(output);
    LoRa.endPacket();
  }
  doc.clear();
  Serial.println("Not enough data received.");
}

void readAndPrintSensorData(int retryCount) { // 수신 데이터 해석
if (retryCount >= maxRetries) {
    Serial2.println("error");
    return;
  }
  byte receiveBuffer[MAX_BUFFER]; // 수신 데이터를 처리할 배열 선언
  int byteReceived = 0; // 인덱스 0부터 시작
  delay(100);
  while (Serial2.available()) { // Serial2가 활성화 될 때, 즉 읽을 데이터가 남아있을 때
    if (byteReceived < MAX_BUFFER) { // 배열의 크기만큼 반복
      receiveBuffer[byteReceived] = Serial2.read(); // 들어온 데이터를 배열에 순서대로 저장
      Serial.print(receiveBuffer[byteReceived], HEX); // 저장된 배열을 HEX로 출력
      Serial.print(" ");
      byteReceived++;
    }
  }
  Serial.print("\n");

  if(byteReceived > MIN_BYTE_RECEIVED) {  // 11번 인덱스까지 값이 있는 경우만 파싱을 수행
;
    float ec = word(receiveBuffer[4], receiveBuffer[5]) / 100.0;
    float ph = word(receiveBuffer[6], receiveBuffer[7]) / 100.0;
    //int hum = word(receiveBuffer[8], receiveBuffer[9]);
    float temp = word(receiveBuffer[10], receiveBuffer[11]) / 10.0;

    int intEc = (int)ec;
    int intPh = (int)ph;
    int intTemp = (int)temp;
    //int intHum = (int)hum;

    //bool isHumNormal = (hum >= 10 && hum <= 90);
    bool isTempNormal = (temp < 50);
    bool isEcNormal = (ec < 10 );
    bool ispHNormal = (ph > 2 && ph <10 );

    //  if (!isHumNormal || !isTempNormal || !isEcNormal || !ispHNormal) {
      if (!isTempNormal || !isEcNormal || !ispHNormal) {
      Serial.println("Abnormal sensor reading. Retrying...");
      sendData(sensorRequestFrame, sizeof(sensorRequestFrame));
      delay(10);
      readAndPrintSensorData(retryCount + 1);
      return;
    }
    
    doc["id"] = 1;

    JsonArray sensors = doc.createNestedArray("sensors");

    // JsonObject humidity_sensor = sensors.createNestedObject();
    // humidity_sensor["sensor_id"] = 2; // For example, 1 is for humidity
    // char humbuffer[6];
    // dtostrf(hum, 7, 3, humbuffer);
    // String humStr = String(humbuffer);
    // humidity_sensor["value"] = humStr;

    JsonObject ph_sensor = sensors.createNestedObject();
    ph_sensor["sensor_id"] = 16; // For example, 3 is for co2
    char phbuffer[7];
    dtostrf(ph, 7, 3, phbuffer);
    String phStr = String(phbuffer);
    ph_sensor["value"] = phStr;

    JsonObject temperature_sensor = sensors.createNestedObject();
    temperature_sensor["sensor_id"] = 29; // For example, 2 is for temperature
    char tempbuffer[7];
    dtostrf(temp, 7, 3, tempbuffer);
    String tempStr = String(tempbuffer);
    temperature_sensor["value"] = tempStr;

    JsonObject ec_sensor = sensors.createNestedObject();
    ec_sensor["sensor_id"] = 30; // For example, 2 is for temperature
    char ecbuffer[7];
    dtostrf(ec, 7, 3, ecbuffer);
    String ecStr = String(ecbuffer);
    ec_sensor["value"] = ecStr;

    
    
    String output;
    serializeJson(doc, output);
    output += "\n";

    Serial.println(output);

    //String sensorData = "ec: " + String(ec) + ", ph: " + String(ph) + ", hum: " + String(hum) + ", temp: " + String(temp);
    //Serial.print(sensorData);
    if(loRaOnOff){
      LoRa.beginPacket();
      LoRa.print(output);
      LoRa.endPacket();
      
      //Serial.print(doc);
    }
    doc.clear();

  } else {
    handleNotEnoughData();
  }
}

void sendLoRaPacket(){
}




