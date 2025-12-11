// Sensors.h — Déclaration de la classe Sensors
// (Interface identique à ta version d’origine)

#ifndef sensors_h
#define sensors_h

class Sensors {
  public:
    Sensors(){};
    String* get_names();
    int get_nb_values();
    String getFileHeader();
    String getFileData();
    String serialPrint();
    void measure();
    void tcaselect(uint8_t i);   // Signature d’origine conservée (void)
};

#endif