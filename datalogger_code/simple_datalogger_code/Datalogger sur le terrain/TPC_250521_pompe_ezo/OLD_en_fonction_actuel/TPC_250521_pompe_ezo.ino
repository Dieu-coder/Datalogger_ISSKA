/* *******************************************************************************
Base code for datalogger without wireless transmission. This code was rewritten in august 2023.

The conf.txt file on the SD card must have the following structure:
  300; //time step in seconds (must be either a multiple of 60, or it must be a divider of 60)
  1; //boolean value to choose if you want to set the RTC clock with time from SD card
  2023/03/31 16:05:00; //time to set the RTC if setRTC = 1

If you have any questions contact me per email at nicolas.schmid.6035@gmail.com
**********************************************************************************
*/  

#include <Wire.h>
#include <TinyPICO.h>
#include <RTClib.h>
#include <SD.h>
#include <U8x8lib.h>
#include "Sensors.h" //file sensor.cpp and sensor.h must be in the same folder

TinyPICO tp = TinyPICO();
RTC_DS3231 rtc; 
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); //initialise something for the OLED display
Sensors sensor = Sensors();


int SD_date_time[6]; //[year,month,day,hour,minute,second]
float start_time_programm; //time when the programm starts. Used to adjust more precisely the rtc clock

//PINS to power different devices
#define Enable5VPIN 25 //turns the 3V to 5V converter on if switched to high
#define MOSFET_SENSORS_PIN 14 //pin to control the power of the sensors, the screen and sd card reader

//SD card
#define SD_CS_PIN 5//Define the pin of the tinypico connected to the CS pin of the SD card reader
#define DATA_FILENAME "/data.csv"
#define CONFIG_FILENAME "/conf.txt"

//Variables which must be stored on the RTC memory, to not be erease in deep sleep (only about 4kB of storage available)
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool first_time = true;
RTC_DATA_ATTR int start_time; 
RTC_DATA_ATTR int start_date_time[6]; //[year,month,day,hour,minute,second] of the starting time
RTC_DATA_ATTR int time_step;
RTC_DATA_ATTR bool SetRTC; //read this value from the conf.txt file on the SD card. If true we set the clock time with time on the conf.txr file


//********************** MAIN LOOP OF THE PROGRAMM *******************************
//The setup is recalled each time the micrcontroller wakes up from a deepsleep
void setup(){ 

  start_time_programm = micros();

  //Start serial communication protocol with external devices (e.g. your computer)
  Serial.begin(115200);

  power_external_device();

  Wire.begin(); // initialize I2C bus 
  Wire.setClock(50000); // set I2C clock frequency to 50kHz

  if(first_time){ //this snipet is only executed when the microcontroller is turned on for the first time
    tp.DotStar_SetPower(true); //This method turns on the Dotstar power switch, which enables the tinypico LED
    tp.DotStar_SetPixelColor(25, 25, 25 ); //Turn the LED white while SD card is not initialised

    //write Init on the display
    u8x8.begin();
    u8x8.setBusClock(50000); //Give the I2C frequency information to the display
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r); //set the font
    u8x8.setCursor(0, 0);
    u8x8.print("Init...");
    Serial.println("Init...");

    delay(2000); //wait 2 seconds to see the led and the screen 

    initialise_SD_card();  //read conf.txt file values and store them on RTC memory to access them rapidely, write header on data.csv

    //initialise the RTC clock
    rtc.begin();
    rtc.disable32K(); //The 32kHz output is enabled by default. We don't need it so we disable it
    if (SetRTC) adjust_rtc_time_with_time_from_SD();

    put_usefull_values_on_display(); // show RTC time, battery voltage, time step and sensors values on the display

    get_next_rounded_time(); //this finds the next rounded time and puts it as starting time
    first_time = false; //to make the initialization only once
    deep_sleep_mode(0); //deep sleep until starting time 
  }

  else{ //normal operation
    bootCount++; //to know how many times we have booted the  microcontroller
    Serial.print("***********Starting the card, Boot count: ");
    Serial.print(bootCount);
    Serial.println(" ***********************");
    
    //setup for the clock
    rtc.begin();
    rtc.disable32K(); //The 32kHz output is enabled by default. We don't need it so we disable it
    readRTC();//print current time

    if (bootCount<=3){ //Turn the LED blue during first 3 measurements ( only to check that the logger works normally)
      tp.DotStar_SetPower(true);
      tp.DotStar_SetPixelColor(0, 0, 25 ); 
    }

    mesure_all_sensors();

    if(tp.GetBatteryVoltage()>3.3){
      deep_sleep_mode(time_step);
    }
    else{
      deep_sleep_mode(3600); //sleep during ten years (don't wake up) to avoid damaging the battery
    }
  }
}

void loop(){ //No loop is used 
}

void power_external_device(){
  //Turn on Pin to control the gate of MOSFET for all the sensors, display and microSD card
  pinMode(MOSFET_SENSORS_PIN, OUTPUT);
  digitalWrite(MOSFET_SENSORS_PIN, HIGH);
}

void adjust_rtc_time_with_time_from_SD(){
  DateTime SD_time = DateTime(SD_date_time[0],SD_date_time[1],SD_date_time[2],SD_date_time[3],SD_date_time[4],SD_date_time[5]); // convert time written on conf.txt into DateTime type
  float adjustement_time = micros();
  rtc.adjust(SD_time+2+(adjustement_time-start_time_programm)/1000000); //compensate the booting time to be as accurate as possible with the RTC time.
  Serial.print("rtc set to: ");
  Serial.println(rtc.now().timestamp(DateTime::TIMESTAMP_FULL)); //print time set on RTC
}

void put_usefull_values_on_display(){
 
  //Display Vattery voltage
  float batvolt = tp.GetBatteryVoltage();
  u8x8.display(); // Power on the OLED display
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print("Batt: ");
  u8x8.print(batvolt);
  u8x8.print("V");

  //Display current time on RTC
  u8x8.setCursor(0, 2);
  u8x8.print(readRTC());

  //Display value of timestep read on conf.txt
  u8x8.setCursor(0, 4);
  u8x8.print("timestep: ");
  u8x8.print(time_step);
  u8x8.print("s");
  delay(4000);

  //Display sensor values
  sensor.measure();   //measure all sensors
  String measured_values_str = sensor.serialPrint();
  u8x8.clear();
  u8x8.setCursor(0, 0);
  int line_count = 0;
  for (char const &c: measured_values_str) {
    if(c=='\n'){
      line_count++;
      if (line_count%8 ==0){ //if we have more than 8 sensors to display and there is no more space on the screen
        delay(8000);
        u8x8.clear();
        u8x8.setCursor(0, 0);
      }
      else u8x8.println();
    }
    else u8x8.print(c);
  }
  delay(8000); //wait 8 seconds to see the values on the display
}

void get_next_rounded_time(){ 
  //this function finds the next rounded time and stores it in the start_date_time array and the start_time variable
  //the time step must be either less than 60 or a multiple of 60 otherwise we will have an unexpected behaviour
  DateTime start_datetime;
  DateTime current_time=rtc.now(); //get current time
  //compute the number of seconds left until the next rounded minute (where seconds is :00)
  int seconds_remaining = 60-current_time.second()%60; 
  DateTime time_seconds_rounded_up = current_time+seconds_remaining;
  if(time_step<60){ 
    start_datetime = time_seconds_rounded_up;
  }
  else{ //time_step>=60
    int rounded_min = time_step/60; //gives the time step in minute
    if(time_seconds_rounded_up.minute()%rounded_min==0){
      start_datetime = time_seconds_rounded_up; //minute already multiple of time_step
    }
    else{ //compute the number of minutes until the next rounded time (in minutes)
      int minutes_remaining = rounded_min - time_seconds_rounded_up.minute()%rounded_min; 
      start_datetime = (time_seconds_rounded_up + 60*minutes_remaining);
    }
  }
  start_time = start_datetime.unixtime();
  start_date_time[0]=start_datetime.year();
  start_date_time[1]=start_datetime.month();
  start_date_time[2]=start_datetime.day();
  start_date_time[3]=start_datetime.hour();
  start_date_time[4]=start_datetime.minute();
  start_date_time[5]=start_datetime.second();
  Serial.print("start time: ");
  Serial.println(start_datetime.timestamp(DateTime::TIMESTAMP_FULL));
}

void initialise_SD_card(){ //open SD card. Write header in the data.csv file. read values from the conf.txt file.

  //initialisation
  bool SD_error = false;
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    SD_error = true;
  }
  else{
    //read value on the conf.txt file and store the values on RTC memory
    File configFile = SD.open(CONFIG_FILENAME);
    if (configFile.available()) {
      //read time step
      time_step = configFile.readStringUntil(';').toInt();
      while (configFile.peek() != '\n'){ //if there are any black space in the config file
        configFile.seek(configFile.position() + 1);
      }
      configFile.seek(configFile.position() + 1); //to remove the \n character itself

      //read setRTC bool value
      String str_SetRTC = configFile.readStringUntil(';');
      if (str_SetRTC=="0") SetRTC = false;
      else if (str_SetRTC=="1") SetRTC = true;
      else {
        Serial.println("problem by reading the SetRTC value from SD card");
        SD_error = true;
      }
      while (configFile.peek() != '\n'){ //if there are any black space if the config file
        configFile.seek(configFile.position() + 1);
      }
      configFile.seek(configFile.position() + 1); //to remove the \n character itself

      //read time stored on the conf.txt file which is in the following format: year/month/day hour:minute:second
      SD_date_time[0] = configFile.readStringUntil('/').toInt(); //year
      SD_date_time[1] = configFile.readStringUntil('/').toInt(); //month
      SD_date_time[2] = configFile.readStringUntil(' ').toInt(); //day
      SD_date_time[3] = configFile.readStringUntil(':').toInt(); //hour
      SD_date_time[4] = configFile.readStringUntil(':').toInt(); //minute
      SD_date_time[5] = configFile.readStringUntil(';').toInt(); //second
    }
    configFile.close();

    //print values read on the conf.txt file
    Serial.println("Values read from SD Card:");
    Serial.print("- time step: ");
    Serial.println(time_step);
    Serial.print("- Set the RTC clock: ");
    Serial.println(SetRTC);
    if(SetRTC){
      Serial.print("- Time on SD when you should push the reboot button : ");
      for(int i = 0; i<6;i++){
        Serial.print(SD_date_time[i]);
        Serial.print("/");
      }
      Serial.println();
    }
  }

  //display error or success message, wether ther was an error or not
  u8x8.setCursor(0, 2);
  if (SD_error == true){
    u8x8.print("SD Failure !");
    tp.DotStar_SetPixelColor( 50, 0, 0 ); //set led to red
  }
  //write header in SD
  else {
    File dataFile = SD.open(DATA_FILENAME, FILE_APPEND);
    String header = sensor.getFileHeader();
    dataFile.println();
    dataFile.println();
    dataFile.println();
    dataFile.print("ID;DateTime;");
    dataFile.println(header);
    dataFile.close();
    // show on display and LED if values on SD card are OK
    u8x8.setCursor(0, 2);
    u8x8.print("Success !");
    tp.DotStar_SetPixelColor( 0, 50, 0 );
  }
  delay(1000); //in [ms] in order to see the LED and the success message
}

void deep_sleep_mode(int sleeping_time){ 
  //turn off all things that might consume some power
  digitalWrite(MOSFET_SENSORS_PIN, LOW);
  tp.DotStar_SetPower(false); //LED of the TinyPico
  rtc.writeSqwPinMode(DS3231_OFF);
  //set alarm on RTC
  rtc.clearAlarm(1); //to clear a potentially previously set alarm set on alarm1
  rtc.disableAlarm(2); //disable alarm2 to ensure that no previously set alarm causes any problem
  rtc.setAlarm1(start_time+bootCount*sleeping_time,DS3231_A1_Hour);  
  pinMode(GPIO_NUM_15, INPUT_PULLUP);
  //specify wakeup trigger and enter deep sleep
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 0); //2nd param. is logical level of trigger signal (tinypico is wake-up when signal is low in this case)
  esp_deep_sleep_start();
}

String readRTC() { //print current time
  DateTime now = rtc.now();
  String time_str = String(now.day()) + "/" + String(now.month()) + " " + String(now.hour())+ ":" + String(now.minute()) + ":" + String(now.second());
  Serial.print("Current time: ");
  Serial.println(time_str);
  return time_str;
}

void save_data_in_SD(){ 
  //put the measured values in a string with the following format: "ID;DateTime;VBatt;tempEXT;pressEXT;HumExt;..." 
  String data_message = (String) bootCount;
  char buffer [35] = "";
  DateTime starting_time_dt = DateTime(start_date_time[0],start_date_time[1],start_date_time[2],start_date_time[3],start_date_time[4],start_date_time[5]);
  DateTime curr_time = starting_time_dt +(bootCount-1)*time_step;
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", curr_time.day(), curr_time.month(), curr_time.year(), curr_time.hour(), curr_time.minute(), curr_time.second());
  data_message = data_message + ";"+ buffer + ";";
  String sensor_data = sensor.getFileData(); 
  data_message = data_message + sensor_data;
  Serial.println(data_message);

  //initialise SD card
  if (!SD.begin(SD_CS_PIN)) {  //if there is a problem with the SD card turn on display to show to the user that there is a problem with the SD card
    Serial.println("SD card initialization failed!");
    u8x8.begin();
    u8x8.setBusClock(50000); 
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    u8x8.setCursor(0, 0);
    u8x8.print("SD Failure !");
    tp.DotStar_SetPower(true);
    tp.DotStar_SetPixelColor( 50, 0, 0 ); //set led to red
    delay(5000); //in [ms] in order to see the LED and display because there is an important and recurent error 
  }
  else{ //if there is no error with the SD card store the data message
    File dataFile = SD.open(DATA_FILENAME, FILE_APPEND);
    dataFile.print(data_message);
    dataFile.println();
    dataFile.close();
  }
}

void mesure_all_sensors(){
  Serial.println("Start of the measurements");
  sensor.measure(); // measures all sensors and stores the values in an array
  sensor.serialPrint(); //print all the sensors' values in Serial monitor
  save_data_in_SD(); //save measuremetns on the SD card
}
