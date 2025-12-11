//declaration of the sensors' class. All the methods are defined in the Sensors.cpp file 
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
    void tcaselect(uint8_t i);
};
#endif