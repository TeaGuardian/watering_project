#include <SoftwareSerial.h>

#define SERIAL_1_RX 5
#define SERIAL_1_TX 4
#define DELIMITER '#'
#define BAUD_RATE 9600

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

String packageToString(const Package& pkg) {
  String str = String(pkg.sender) + DELIMITER
              + String(pkg.target) + DELIMITER
              + String(pkg.is_com) + DELIMITER
              + String((int)pkg.command_code) + DELIMITER
              + String(pkg.data) + DELIMITER
              + String(pkg.int_data);
  return str;
}

Package stringToPackage(const String& str) {
  Package pkg;
  int index = 0;
  
  pkg.sender = str.substring(index, str.indexOf(DELIMITER)).toInt();
  index = str.indexOf(DELIMITER, index) + 1;
  
  pkg.target = str.substring(index, str.indexOf(DELIMITER, index)).toInt();
  index = str.indexOf(DELIMITER, index) + 1;
  
  pkg.is_com = str.substring(index, str.indexOf(DELIMITER, index)).toInt();
  index = str.indexOf(DELIMITER, index) + 1;
  
  pkg.command_code = (char)str.substring(index, str.indexOf(DELIMITER, index)).toInt();
  index = str.indexOf(DELIMITER, index) + 1;
  
  pkg.data = str.substring(index, str.indexOf(DELIMITER, index)).toFloat();
  index = str.indexOf(DELIMITER, index) + 1;
  
  pkg.int_data = str.substring(index).toInt();
  
  return pkg;
}

Package buf;
Package resp;
void setup() {
  serial_1.begin(BAUD_RATE);
  Serial.begin(BAUD_RATE);
}

void loop() {
  if (serial_1.readBytes((byte*)&buf, sizeof(buf))) {
    //Serial.write((byte*)&buf, sizeof(buf));
    Serial.println(packageToString(buf));
  }
  if (Serial.available()) {
    buf = stringToPackage(Serial.readString());
    //Serial.println(packageToString(buf));
    serial_1.write((byte*)&buf, sizeof(buf));
  }
  /*
  if (Serial.readBytes((byte*)&buf, sizeof(buf))) {
    serial_1.write((byte*)&buf, sizeof(buf));
  }
  */
}
