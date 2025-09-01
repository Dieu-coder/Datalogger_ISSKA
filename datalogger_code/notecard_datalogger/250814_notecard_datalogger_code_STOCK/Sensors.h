//declaration of the sensors' class. All the methods are defined in the Sensors.cpp file 
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>   // Print, uint8_t

class Sensors {
public:
  Sensors() {}

  // --- Noms & comptage ---
  const char* const* get_names();  // pointeur sur tableau de C-strings
  int get_nb_values();

  // --- Sorties sans heap ---
  void printFileHeader(Print& out); // "name;name;..."
  void printFileData(Print& out); // "v1;v2;..."
  void printSerial(Print& out); // "name: value\n..."

  // Pour ton OLED: met dans 'buf' le même contenu que printSerial()
  void formatSerialToBuffer(char* buf, size_t bufsize) const;

  // --- Mesure & MUX ---
  void measure();
  void tcaselect(uint8_t channel);

  // --- Décimales par capteur (CSV/Serial) ---
  void setCsvDecimals(uint8_t idx, uint8_t decimals);
  void setSerialDecimals(uint8_t idx, uint8_t decimals);
  void setCsvDecimalsAll(const uint8_t* decimals, int len);
  void setSerialDecimalsAll(const uint8_t* decimals, int len);

  // --- Calibration simple (offset/scale) ---
  void setCalibration(uint8_t idx, float offset, float scale);
  void setCalibrationAll(const float* offsets, const float* scales, int len);

  // --- Introspection ---
  const float* get_values() const;
  uint8_t      getHealthMask() const;

private:
  // Données internes définies/initialisées dans Sensors.cpp
};

#endif