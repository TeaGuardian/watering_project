#include <Wire.h>
#include <AHTxx.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Adafruit_INA219.h>
#include "SparkFun_ENS160.h"

#define SERIAL_1_RX 4
#define SERIAL_1_TX 6
#define SERIAL_2_RX 5
#define SERIAL_2_TX 7

#define MOLSTURE_1 A7
#define MOLSTURE_2 A6
#define BUTTON_1 A3
#define BUTTON_2 A2
#define VALVE_CO2 8
#define VALVE_WATER 9
#define LED_LOAD 11
#define LED_DATA 10
#define SPEAKER 12

#define MODEL_ID 0
#define BAUD_RATE 9600
#define SETUP_DELAY 1000 //delay between setups
#define LED_CLEAR_DELAY 110 //delay before clearing

unsigned short int local_id = 0;
unsigned long last_time = 0;
bool led_load_fl;
bool led_data_fl;
bool serial_switch;

/*  A5 - SCL    A4 - SDA   for i2c sensors
    - ENS160 (gas sensor)
    - AHTX (temperature and humidity sensor)
    - INA219 (dc current sensor)
    
    command code list:
    0 - reindex all
    1 - fetch all
    2 - is_exist?
    3 - get model id

    local code list:
    7  - set index
    8  - molsture_1
    9  - molsture_2
    10 - button_1
    11 - button_2
    12 - set valve_co2
    13 - set valve_water
    14 - set led_load
    15 - set speaker
    16 - AQI (ens160)
    17 - eCO2 (ens160)
    18 - TVOC (ens160)
    19 - temperature (ath2x)
    20 - humidity (ath2x)
    21 - shunt_voltage (ina219)
    22 - bus_voltage (ina219)
    23 - current (ina219)
*/

struct Package {
  unsigned short int sender;
  unsigned short int target;
  bool is_com;
  char command_code;
  float data;
  unsigned short int int_data;
};

SoftwareSerial serial_1(SERIAL_1_RX, SERIAL_1_TX);
SoftwareSerial serial_2(SERIAL_2_RX, SERIAL_2_TX);

SparkFun_ENS160 ens160;
AHTxx aht20(AHTXX_ADDRESS_X38, AHT2x_SENSOR); // I2C address and type of the AHT21 sensor
Adafruit_INA219 ina219;

void writeToEE(int address, unsigned short int value) {
  EEPROM.put(address, value);
}

unsigned short int readFromEE(int address) {
  unsigned short int value;
  EEPROM.get(address, value);
  return value;
}

void setup() {
  Wire.begin();
  serial_1.begin(BAUD_RATE);
  serial_2.begin(BAUD_RATE);
  pinMode(MOLSTURE_1, INPUT);
  pinMode(MOLSTURE_2, INPUT);
  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);
  pinMode(VALVE_CO2, OUTPUT);
  pinMode(VALVE_WATER, OUTPUT);
  pinMode(LED_DATA, OUTPUT);
  pinMode(LED_LOAD, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  digitalWrite(LED_DATA, HIGH);
  if (readFromEE(1) != 0) {
    local_id = readFromEE(1);
    tone(SPEAKER, 1600, 100);
  } else {
    tone(SPEAKER, 800, 300);
  }
  bool ens = false;
  bool ath = false;
  bool ina = false;
  while (!ens || !ath || !ina) {  //ожидаем инициализацию
    led_data_fl = !led_data_fl;
    digitalWrite(LED_DATA, led_data_fl ? HIGH : LOW);
    if (!ens) {ens = ens160.begin();}
    if (!ath) {ath = aht20.begin();}
    if (!ina) {ina = ina219.begin();}
    delay(400);
  }
  
  ens160.setOperatingMode(SFE_ENS160_RESET);
  ens160.setOperatingMode(SFE_ENS160_STANDARD);
  tone(SPEAKER, 400, 300);
  digitalWrite(LED_DATA, LOW);
}

Package buf;
Package resp;


void loop() {
  if (millis() > 4000) {
  if (abs(millis() - last_time) > SETUP_DELAY) {  //обновляем данные датчика для точных показателей
    ens160.setTempCompensationCelsius(aht20.readTemperature());
    ens160.setRHCompensationFloat(aht20.readHumidity()); 
    analogWrite(LED_DATA, 20);
    last_time = millis();
  }
  if ((millis() % LED_CLEAR_DELAY) == 0) { //сбрасываем светодиод данных
    digitalWrite(LED_DATA, LOW);
  }
  }
  bool catched_data = false;
  if (serial_switch) {
    serial_1.listen();
    if (serial_1.readBytes((byte*)&buf, sizeof(buf))) {
      catched_data = true;
      analogWrite(LED_DATA, 120);
      }
  } else {
    serial_2.listen();
    if (serial_2.readBytes((byte*)&buf, sizeof(buf))) {
      catched_data = true;
      analogWrite(LED_DATA, 120);
      }
  }

  if (catched_data) {
    if (buf.is_com && (buf.target == local_id || buf.command_code <= 1)){
      resp.sender = local_id;
      resp.target = buf.sender;
      resp.is_com = false;
      resp.command_code = buf.command_code;
      bool do_resp = true;
      switch(buf.command_code) {
        case 0:
          local_id = buf.target;
          writeToEE(1, local_id);
          buf.sender = local_id;
          buf.target = local_id + 1;
          if (!serial_switch) {
            serial_1.write((byte*)&buf, sizeof(buf));
          } else {
            serial_2.write((byte*)&buf, sizeof(buf));
          }
          resp.sender = local_id;
          resp.target = 0;
          break;
        case 1:
          if (!serial_switch) {
            serial_1.write((byte*)&buf, sizeof(buf));
          } else {
            serial_2.write((byte*)&buf, sizeof(buf));
          }
          resp.target = 0;
          break;
        case 2:
          break;
        case 3:
          resp.int_data = MODEL_ID;
          break;
        case 7:
          local_id = buf.int_data;
          writeToEE(1, local_id);
          resp.sender = local_id;
          break;
        case 8:
          resp.int_data = analogRead(MOLSTURE_1);
          break;
        case 9:
          resp.int_data = analogRead(MOLSTURE_2);
          break;
        case 10:
          resp.int_data = digitalRead(BUTTON_1) ? 1 : 0;
          break;
        case 11:
          resp.int_data = digitalRead(BUTTON_2) ? 1 : 0;
          break;
        case 12:
          digitalWrite(VALVE_CO2, buf.int_data == 1 ? HIGH : LOW);
          break;
        case 13:
          digitalWrite(VALVE_WATER, buf.int_data == 1 ? HIGH : LOW);
          break;
        case 14:
          analogWrite(LED_LOAD, buf.int_data);
          break;
        case 15:
          tone(SPEAKER, buf.int_data, buf.data);
          break;
        case 16:
          resp.int_data = ens160.getAQI();
          break;
        case 17:
          resp.int_data = ens160.getECO2();
          break;
        case 18:
          resp.int_data = ens160.getTVOC();
          break;
        case 19:
          resp.data = aht20.readTemperature();
          break;
        case 20:
          resp.data = aht20.readHumidity();
          break;
        case 21:
          resp.data = ina219.getShuntVoltage_mV();
          break;
        case 22:
          resp.data = ina219.getBusVoltage_V();
          break;
        case 23:
          resp.data = ina219.getCurrent_mA();
          break;
        default:
          do_resp = false;
          break;
      }
      if (do_resp) {
        if (serial_switch) {
          serial_1.write((byte*)&resp, sizeof(resp));
        } else {
          serial_2.write((byte*)&resp, sizeof(resp));
        }
        digitalWrite(LED_DATA, HIGH);
        delay(1);
      }
    } else {
      if (!serial_switch) {
        serial_1.write((byte*)&buf, sizeof(buf));
        digitalWrite(LED_DATA, HIGH);
      } else {
        serial_2.write((byte*)&buf, sizeof(buf));
        digitalWrite(LED_DATA, HIGH);
      }
    }
  }
  serial_switch = !serial_switch;
  delay(1);
}
