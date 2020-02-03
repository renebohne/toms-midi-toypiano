#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <VEML7700.h>

#include <WiFi.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#define SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

#define LEDPIN 13
#define BUTTONPIN 12

#define PIANO_PIN1 34
#define PIANO_PIN2 35
#define PIANO_PIN3 32
#define PIANO_PIN4 33

uint8_t old1 = 0;
uint8_t old2 = 0;
uint8_t old3 = 0;
uint8_t old4 = 0;

#define SEALEVELPRESSURE_HPA (1031.80f)

#define I2C_SDA_PIN          23
#define I2C_SCL_PIN          22

#define BMP_SDA_PIN          22
#define BMP_SCL_PIN          23

//  VEML6070 with Rset=270k on breakout => UVA sensitivity: 5.625 uW/cm²/step
#define VEML6070_I2C_ADDR 0x38 //0x38 and 0x39
// Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

VEML7700 veml7700;

#define BMP280_I2C_ADDR 0x76 // 0x76 and 0x77
Adafruit_BMP280 bmp; // I2C

// Vars
int val;
char buf[12];
long lastMsg = 0;
char msg[50];
int value = 0;
int loop_counter = 0;



BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

uint8_t midiPacket[] = {
   0x80,  // header
   0x80,  // timestamp, not implemented
   0x00,  // status
   0x3c,  // 0x3c == 60 == middle c
   0x00   // velocity
};

void setupVEML6070()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.beginTransmission(VEML6070_I2C_ADDR);
  Wire.write((IT_1<<2) | 0x02);
  Wire.endTransmission();
  delay(500);
}

void setupVEML7700()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  veml7700.begin();
}

void setupBMP280()
{
  Wire.begin(BMP_SDA_PIN, BMP_SCL_PIN);
  if (!bmp.begin(BMP280_I2C_ADDR)) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
}

uint16_t readVEML6070()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  byte msb=0, lsb=0;
  uint16_t uv;

  Wire.requestFrom(VEML6070_I2C_ADDR+1, 1); //MSB
  delay(1);
  if(Wire.available())
  {
    msb = Wire.read();
  }
  else
  {
    return 0;
  }

  Wire.requestFrom(VEML6070_I2C_ADDR+0, 1); //LSB
  delay(1);
  if(Wire.available())
  {
    lsb = Wire.read();
  }
  else
  {
    return 0;
  }

  uv = (msb<<8) | lsb;

  return uv;
}

float readVEML7700()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    float lux;
    veml7700.getALSLux(lux);
    return lux;
}

void printValues() {
      setupVEML6070();
      uint16_t uv = readVEML6070();
      Serial.print("UV A = ");
      Serial.print(uv);
      Serial.println(" uW/cm^2/steps");

      float uv_translated = 5.625f * uv;
      Serial.print("Publish uv message: ");
      Serial.print(uv_translated);
      Serial.println(" uW/cm^2");
      String str_uv = String(uv_translated);
      str_uv.toCharArray(msg, 50);

      float lux = readVEML7700();
      //publish to mqtt
      String str_lux = String(lux);
      str_lux.toCharArray(msg, 50);
      Serial.print("Publish lux message: ");
      Serial.println(msg);

      setupBMP280();
      float temperature = bmp.readTemperature();
      String str_temperature = String(temperature);
      str_temperature.toCharArray(msg, 50);
      Serial.print("Publish temperature message: ");
      Serial.println(msg);

      float pressure = bmp.readPressure() / 100.0f; //hPa
      String str_pressure = String(pressure);
      str_pressure.toCharArray(msg, 50);
      Serial.print("Publish pressure message: ");
      Serial.println(msg);

      int btnstate = digitalRead(BUTTONPIN);
      String str_button = String(btnstate);
      str_button.toCharArray(msg, 50);
      Serial.print("Publish button message: ");
      Serial.println(msg);

      Serial.println("---------------------------");
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println(F("midi toy piano"));

  pinMode(BUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  setupVEML6070();
  setupVEML7700();
  setupBMP280();

  pinMode(PIANO_PIN1, INPUT);
  pinMode(PIANO_PIN2, INPUT);
  pinMode(PIANO_PIN3, INPUT);
  pinMode(PIANO_PIN4, INPUT);

  BLEDevice::init("MIDI ToyPiano");
  Serial.println(); // gap

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    BLEUUID(CHARACTERISTIC_UUID),
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();
}

void sendNoteOn(uint8_t note)
{
  if (deviceConnected) {
   // note down
   midiPacket[2] = 0x90; // note down, channel 0
   midiPacket[3] = note;
   midiPacket[4] = 127;  // velocity
   pCharacteristic->setValue(midiPacket, 5); // packet, length in bytes
   pCharacteristic->notify();
  }
}


void sendNoteOff(uint8_t note)
{
  if (deviceConnected) {
   midiPacket[2] = 0x80; // note up, channel 0
   midiPacket[3] = note;
   midiPacket[4] = 0;    // velocity
   pCharacteristic->setValue(midiPacket, 5); // packet, length in bytes)
   pCharacteristic->notify();
  }
}

void loop() {
  // slow down the publishing of new data repecting short delay
  /*
  if (loop_counter > 100)
  {
    printValues();
    loop_counter = 0;
  }
  else
  {
    loop_counter++;
  }
  */

  int new1 = digitalRead(PIANO_PIN1);
  int new2 = digitalRead(PIANO_PIN2);
  int new3 = digitalRead(PIANO_PIN3);
  int new4 = digitalRead(PIANO_PIN4);

  if(new1 != old1)
  {
    old1 = new1;
    if( new1 == HIGH)
    {

      sendNoteOn(60);//C4
    }
    else
    {
      sendNoteOff(60);//C4
    }
  }
  if(new2 != old2)
  {
    old2 = new2;
    if( new2 == HIGH)
    {

      sendNoteOn(62);//D4
    }
    else
    {
      sendNoteOff(62);//D4
    }
  }
  if(new3 != old3)
  {
    old3 = new3;
    if( new3 == HIGH)
    {

      sendNoteOn(64);//E4
    }
    else
    {
      sendNoteOff(64);//E4
    }
  }
  if(new4 != old4)
  {
    old4 = new4;
    if( new4 == HIGH)
    {

      sendNoteOn(65);//F4
    }
    else
    {
      sendNoteOff(65);//F4
    }
  }

}
