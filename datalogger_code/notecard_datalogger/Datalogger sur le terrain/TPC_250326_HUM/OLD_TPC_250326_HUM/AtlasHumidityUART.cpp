#include "AtlasHumidityUART.h"

AtlasHumidityUART::AtlasHumidityUART(HardwareSerial &serial) : _serial(serial) {}

void AtlasHumidityUART::begin() {
  _serial.begin(9600); // RX/TX sont définis dans setup() côté ESP32
  delay(1000);
  _serial.print("O,T,1\r");   // Activer température
  delay(500);
  _serial.print("O,DEW,1\r"); // Activer point de rosée
  delay(500);
}

bool AtlasHumidityUART::read(float &humidity, float &temperature, float &dewPoint) {
  _serial.print("R\r");
  String response = readLine(1500);

  if (response.length() == 0) return false;

  // Format attendu : "45.8,22.1,DEW,12.4"
  int idx1 = response.indexOf(',');
  int idx2 = response.indexOf(',', idx1 + 1);
  int idx3 = response.indexOf(',', idx2 + 1);

  if (idx1 == -1 || idx2 == -1 || idx3 == -1) return false;

  humidity = response.substring(0, idx1).toFloat();
  temperature = response.substring(idx1 + 1, idx2).toFloat();
  dewPoint = response.substring(idx3 + 1).toFloat();

  return true;
}

String AtlasHumidityUART::readLine(unsigned long timeout) {
  String result = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (_serial.available()) {
      char c = _serial.read();
      if (c == '\r') return result;
      result += c;
    }
  }
  return "";
}