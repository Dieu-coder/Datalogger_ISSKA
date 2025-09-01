/* *******************************************************************************
This sensor code is for the two sensors that are installed by default on most PCB.
We have the SHT35 (I2c adress 0x44 and TCA1) and the BMP581 (I2c adress 0x46 and TCA2)

To add new sensors here are the things you must modify in this file:
  - Update the String names[] array with all the measurements names that you need
  - Import the library of your sensor
  - create an instance of the sensors'library class
  - In the measure() method, measure your sensor's value and store the value in the float values[] array.
    You must give the same index to a value than to its name in the names[] arrays.
    E.g. if names={"var0","var1","var2"}, then to store the value of var2 you must write: values[2] = mysensor.getvalue2() 

If you have any questions contact me per email at nicolas.schmid.6035@gmail.com
**********************************************************************************
*/  


//include libraries for the sensors
#include <Wire.h>
#include "Sensors.h"
#include "SHT31.h"
#include "SparkFun_BMP581_Arduino_Library.h"
//#include <TinyPICO.h>
#include "SparkFun_SCD4x_Arduino_Library.h" 
#include <Adafruit_ADS1X15.h>
#include <UMS3.h>
#define ADC_MAX 32768 //ADC 16 bits mais 1 bit de signe -> 2^15 = 32768
#define V_REF 2.048 // 2x gain   +/- 2.048V -> V_REF = 2.048

//parameters which depend on the PCB version
#define I2C_MUX_ADDRESS 0x73 //I2C adress of the multiplexer set on the PCB
#define BMP581_sensor_ADRESS 0x46 //I2C adress of the BMP581 set on the PCB
#define SHT35_sensor_ADRESS 0x44 //I2C adress of the SHT35 set on the PCB

//float O2_voltage=1.168; Old value
float O2_voltage=1.053;

//create an instance of the sensors' classes

UMS3 tiS3;
SHT31 sht;
BMP581 bmp;
SCD4x co2_sensor(SCD4x_SENSOR_SCD41); //initialise SCD41 CO2 sensor in soil
Adafruit_ADS1115 ads1115;  // Construct an ads1115 

//declare arrays where the names and values of the sensors are stored
String names[]={"Vbatt","tempSHT","humSHT","tempBMP","pressBMP","O2","V_O2","CO2","T_CO2","H_CO2"}; //update this list if you add sensors!!!
const int nb_values = sizeof(names)/sizeof(names[0]);
float values[nb_values];

//return the names array pointer
String* Sensors::get_names(){ 
  return names;
}

//return the number of values
int Sensors::get_nb_values(){ 
  return nb_values;
}

// Return the file header string in the format "<sensor 1 name>;<sensor 2 name>;"
String Sensors::getFileHeader () { 
  String header_string = "";
  for(int i = 0; i< nb_values; i++){
    header_string = header_string + names[i] + ";";
  }
  return header_string; 
}

// Return sensors data string formated to be write to the CSV file. The format is the following: "<sensor 1 value>;<sensor 2 value>;"
String Sensors::getFileData () { 
  String datastring = "";
  for(int i = 0; i< nb_values; i++){
    datastring = datastring + String(values[i],3) + ";";
  }
  return datastring;
}

// Return a string with the name of the sensors and their value to be shown on the display. Also prints the string into the serial"
String Sensors::serialPrint() { //Display sensor mesures to Serial for debug purposes
  String sensor_display_str = "";
  for(int i = 0; i< nb_values; i++){
    sensor_display_str = sensor_display_str + names[i]+": "+String(values[i],4) + "\n";
  }
  Serial.print(sensor_display_str);
  return sensor_display_str;
}

//measure all sensors'values and store them in the values arrays
void Sensors::measure() {
  delay(5); //delay of 5 ms to ensure that sensors are properly powered

  //measure the battery volatge of the battery which powers the datalogger
  values[0]=tiS3.getBatteryVoltage(); //Vbatt

  //connect and start the SHT35 PCB sensor 
  tcaselect(1);
  delay(3); //wait 3ms for the multiplexer to switch
  sht.begin(); 
  sht.read();
  values[1]=sht.getTemperature(); //tempSHT
  values[2]=sht.getHumidity(); //humSHT

  //connect and start the BMP581 PCB sensor 
  tcaselect(2);
  delay(3);
  bmp.beginI2C(BMP581_sensor_ADRESS);
  delay(100);
  bmp5_sensor_data data = {0,0};
  int8_t err = bmp.getSensorData(&data);
  values[3]=data.temperature; //tempBMP
  values[4]=data.pressure/100; //pressBMP (in millibar)

  tcaselect(7);
  delay(100);
  co2_sensor.begin(false, false, false);
  co2_sensor.measureSingleShot();
  delay(12000); //this sensor needs at least 4-10 seconds to measure something good (heating time)
  co2_sensor.readMeasurement();
  values[7]=co2_sensor.getCO2();
  values[8]=co2_sensor.getTemperature();
  values[9]=co2_sensor.getHumidity();
  delay(100);
  //O2 measurement
  //SGX-40X sensor: https://www.sgxsensortech.com/content/uploads/2022/04/DS-0143-SGX-4OX-datasheet-v5.pdf
  tcaselect(6);
  delay(10000);
  Serial.println("Start of O2 measurement");
  float adcValue, voltage;
  ads1115.setGain(GAIN_TWO);// 2x gain   +/- 2.048V -> V_REF = 2.048
  ads1115.begin();
  delay(100);
  adcValue=ads1115.readADC_SingleEnded(0);
  Serial.print("ADC count value: ");Serial.println(adcValue);
  voltage = adcValue/ADC_MAX*V_REF;
  //float O2_voltage_tcomp=0.002864*values[3]+1.124689;
  float O2_voltage_tcomp=0.002864*values[3]+1.02889;
  Serial.print("Voltage: ");Serial.println(voltage,4);
  //O2 sensor is the SGX-40X
  //0% -> 0V    18.4% -> 1.2V
  //Mettre un meilleur calcul ici + compensation de la température
  values[5] = (voltage*0.21/O2_voltage_tcomp)*100;
  values[6]=voltage;


}

// multiplex bus selection for the first multiplexer 
void Sensors::tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(I2C_MUX_ADDRESS);
  Wire.write(1 << i);
  Wire.endTransmission();
}







