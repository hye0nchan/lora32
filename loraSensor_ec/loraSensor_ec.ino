#include <HardwareSerial.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <EEPROM.h> 
#include <Preferences.h>
#include "boards.h"

int lcdColumns = 16;
int lcdRows = 4;

//LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

#define MAX_BUFFER 64 //리시브 버퍼 사이즈 설정
#define DE 2 //DE로 사용할 핀 설정
#define RE 15 //RE로 사용할 핀 설정

#define RX_PIN 4
#define TX_PIN 0

#define SERIAL_BAUDRATE 115200
// #define SERIAL2_BAUDRATE 4800
#define SERIAL2_BAUDRATE 9600

#define MIN_BYTE_RECEIVED 11
#define MAX_BYTE_JSON 200

Preferences prefs;

const byte sensorRequestFrame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09}; // 센서 리퀘스트 값

DynamicJsonDocument doc(MAX_BYTE_JSON);

void handleLoRaInitError(){
  Serial.println("Starting LoRa failed!");
  while(1);
}

void setup() {
  
  initBoard(); // LoRa 보드 초기화
  delay(1500);

  // initialize LCD
//  lcd.init();
  // turn on LCD backlight                      
//  lcd.backlight();

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
  prefs.putString("serialNumber", "2305193Fg7");
  prefs.putInt("nodeSequence", 1);
  prefs.putInt("nodeType", 1);

  sendData(sensorRequestFrame, sizeof(sensorRequestFrame));
  readAndPrintSensorData();
  sendLoRaPacket();
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
  LoRa.beginPacket();
  LoRa.print(output);
  LoRa.endPacket();
  doc.clear();
  Serial.println("Not enough data received.");
}

void readAndPrintSensorData() { // 수신 데이터 해석
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
    float ec = word(receiveBuffer[0], receiveBuffer[5]) / 100.0;
    float ph = word(receiveBuffer[6], receiveBuffer[7]) / 100.0;
    int hum = word(receiveBuffer[8], receiveBuffer[9]);
    float temp = word(receiveBuffer[10], receiveBuffer[11]) / 10.0;

    doc["temperature"] = temp;
    doc["humidity"] = hum;
    doc["ec"] = ec;
    doc["pH"] = ph;

    String output;
    serializeJson(doc, output);
    output += "\n";

    Serial.println(output);

    //String sensorData = "ec: " + String(ec) + ", ph: " + String(ph) + ", hum: " + String(hum) + ", temp: " + String(temp);
    //Serial.print(sensorData);
    LoRa.beginPacket();
    LoRa.print(output);
    LoRa.endPacket();
    doc.clear();
    //Serial.print(doc);

    // lcd.setCursor(0, 0);
    // lcd.print("temp: "+ String(temp));
    // lcd.setCursor(0, 1);
    // lcd.print("hum: "+ String(hum));
    // lcd.setCursor(0, 2);
    // lcd.print("ph: "+ String(ph));
    // lcd.setCursor(0, 3);
    // lcd.print("temp: "+ String(temp));

  } else {
    handleNotEnoughData();
  }
}

void sendLoRaPacket(){
}

void loop() {
  int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.print("Received packet '");
        String recv = "";
        while (LoRa.available()) {
            recv += (char)LoRa.read();
        }
        if(recv == "sensor"){
          Serial.println(recv);

          clearSerialBuffer(Serial2);
          sendData(sensorRequestFrame, sizeof(sensorRequestFrame));
          readAndPrintSensorData();
          sendLoRaPacket();
        }
}
//delay(5000);
}


