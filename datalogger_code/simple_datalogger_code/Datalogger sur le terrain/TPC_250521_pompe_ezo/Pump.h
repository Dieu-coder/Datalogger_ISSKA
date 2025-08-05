#ifndef PUMP_H
#define PUMP_H

#ifndef SD_CS_PIN
#define SD_CS_PIN 5
#endif

#include <Arduino.h>
#include <SD.h>
#include <U8x8lib.h>
struct ClassDose {
  int lower;   // lower bound of height class
  int upper;  // upper bound
  int dose;  // associated dose
  int numberOfInjections;
};

struct InjectionData {
  float heights[5];  // Array of 24 slopes
  int boot_step;
};

extern float measuredWaterheight;
extern const int nb_values;    // Declare the size
extern float values[];
extern int doseTableSize;
extern ClassDose doseTable[10];

class Pump {
  public:
    // Constructeur
    Pump();
    void configure(int &time_step);

    void handleInjections2(int &bootCount, int &time_step);
    void displayConfiguration(U8X8 &u8x8);
    void setMeasuredWaterHeight(float height);

  private:                 // Lit la réponse du port série
    void pumpSleep();                             // Met la pompe en veille
    bool handleError(String& response);    
    String latestErrorMessage = "*OFF";
    static InjectionData data;
    String readPumpResponse(); 
    int pumpActivationCount;
    void sendCommand(const String& pumpcommand); // Lance un cycle de la pompe
    void save_in_SD(int &bootCount);
    int countLines(File &file);
  };

  extern Pump pump;

#endif // PUMP_H