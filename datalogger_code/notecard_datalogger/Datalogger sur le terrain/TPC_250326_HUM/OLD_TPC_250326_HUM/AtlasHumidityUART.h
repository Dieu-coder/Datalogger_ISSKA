#ifndef ATLAS_HUMIDITY_UART_H
#define ATLAS_HUMIDITY_UART_H

#include <Arduino.h>

class AtlasHumidityUART {
  public:
    AtlasHumidityUART(HardwareSerial &serial);
    void begin();
    bool read(float &humidity, float &temperature, float &dewPoint);

  private:
    HardwareSerial &_serial;
    String readLine(unsigned long timeout = 1000);
};

#endif