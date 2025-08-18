/* *******************************************************************************
Base code for datalogger with a Notecard connected to the TCA3 of the multiplexer. This code was rewritten in august 2023.

The conf.txt file must have the following structure:
  300; //time step in seconds (put at least 240 seconds)
  1; //boolean value to choose if you want to set the RTC clock with GSM time (recommended to put 1)
  12; // number of measurements to be sent at once

To modify the time step or the number of measurements to send at once, add these command on the data.qi file of the device on notehub:
{"time_setp":300} ->this sets the time set to 300 seconds
{"nb_meas":6} ->this sets the number of measurements sent at once to 6

If you have any questions contact me per email at nicolas.schmid.6035@gmail.com
**********************************************************************************
*/  

#include <Wire.h>
#include <TinyPICO.h>
#include <RTClib.h>
#include <SD.h>
#include <U8x8lib.h>
#include "Sensors.h" //file sensor.cpp and sensor.h must be in the same folder¨
#include <Notecard.h>

TinyPICO tp = TinyPICO();
RTC_DS3231 rtc; 
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); //initialise something for the OLED display
Sensors sensor = Sensors();
Notecard notecard;

//PINS to power different devices
#define Enable5VPIN 27 //turns the 3V to 5V converter on if switched to high
#define MOSFET_SENSORS_PIN 14 //pin which controls the power of the sensors, the screen and sd card reader
#define MOSFET_NOTECARD_PIN 4 //pin which controls the power of the notecard
#define NOTECARD_I2C_MULTIPLEXER_CHANNEL 3 //channel of the multiplexer to which the notecard is connected

#define myProductID "com.gmail.ouaibeer:datalogger_isska" //for the notecard to know with which hub to connect

//SD card
#define SD_CS_PIN 5//Define the pin of the tinypico connected to the CS pin of the SD card reader
#define DATA_FILENAME "/data.csv"
#define CONFIG_FILENAME "/conf.txt"

//Variables which must be stored on the RTC memory, to not be erease in deep sleep (only about 4kB of storage available)
RTC_DATA_ATTR int bootCount_since_change = 0; //boot count since external change from notehub
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool first_time = true;
RTC_DATA_ATTR int start_time; 
RTC_DATA_ATTR int start_date_time[6]; //[year,month,day,hour,minute,second] of the starting time
RTC_DATA_ATTR int time_step;
RTC_DATA_ATTR int nb_meas_sent_at_once; //number of measurements sent at once with the notecard
RTC_DATA_ATTR bool SetRTC; //read this value from the conf.txt file on the SD card. True means set the clock time with GSM signal


//********************** MAIN LOOP OF THE PROGRAMM *******************************
//The setup is recalled each time the micrcontroller wakes up from a deepsleep
void setup(){ 

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

    initialise_SD_card(); //read conf.txt file values and store them on RTC memory to access them rapidely, write header on data.csv

    //initialise the RTC clock
    rtc.begin();
    rtc.disable32K(); //The 32kHz output is enabled by default. We don't need it so we disable it
    if (SetRTC) set_RTC_with_GSM_time(); //also gives the number of bars of cellular network (0 to 4) 

    put_usefull_values_on_display(); // show RTC time, battery voltage, time step and sensors values on the display

    get_next_rounded_time(); //this finds the next rounded time and puts it as starting time
    first_time = false; //to make the initialization only once
    deep_sleep_mode(0); //deep sleep until starting time 
  }

  else{ //normal operation (called everytime but not the first time)
    bootCount++; //update how many times we have booted the microcontroller
    bootCount_since_change++; //update how many times we have booted the microcontroller since there was a change of parameters sent from notehub
    Serial.print("***********Starting the card, Boot count: ");
    Serial.print(bootCount);
    Serial.println(" ***********************");

    Serial.print("Boot count since change: ");
    Serial.println(bootCount_since_change);
    Serial.print("Start time: ");
    Serial.println(start_time);
    
    //setup for the clock
    rtc.begin();
    rtc.disable32K(); //The 32kHz output is enabled by default. We don't need it so we disable it
    readRTC();//print current time

    if (bootCount<=3){ //Turn the LED blue during first 3 measurements ( only to check that the logger works normally)
      tp.DotStar_SetPower(true);
      tp.DotStar_SetPixelColor(0, 0, 25 ); 
    }

    mesure_all_sensors();

    if (bootCount_since_change%nb_meas_sent_at_once == 0){ //after a number nb_meas_sent_at_once of measurements, send the last few stored measurements over GSM
      send_data_overGSM();
    }

    if(tp.GetBatteryVoltage()>3.5){
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
  //Turn on Pin to control the gate of MOSFET for all the sensors, display and microSD card(turned on)
  pinMode(MOSFET_SENSORS_PIN, OUTPUT);
  digitalWrite(MOSFET_SENSORS_PIN, HIGH);

  //Pin to control the gate of MOSFET for notecard (turned off)
  pinMode(MOSFET_NOTECARD_PIN, OUTPUT);
  digitalWrite(MOSFET_NOTECARD_PIN, LOW);
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

  //Display value of nuber of measurements to be sent at once read on conf.txt
  u8x8.setCursor(0, 6);
  u8x8.print("Nb meas: ");
  u8x8.print(nb_meas_sent_at_once);

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
  //The computation of the starting time only works if the time step is either less than 60 or a multiple of 60 otherwise we will have an unexpected behaviour.

  DateTime current_time=rtc.now(); //get current time
  int seconds_remaining = 60-current_time.second()%60;  //compute the number of seconds left until the next rounded minute (where seconds is :00)
  DateTime time_seconds_rounded_up = current_time+seconds_remaining;

  DateTime start_datetime;
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
  //we store the start_time in RTC memory, but since a DateTime type cannot be stored in RTC we store it as unixtime, and store the year, month, day, hour, minute and seconds in an array
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
  bool SD_error = false;
  if (!SD.begin(SD_CS_PIN)) { //initialisation, do this snipet of code only if initialization fails
    Serial.println("SD card initialization failed!");
    SD_error = true;
  }
  else{ //Do this snipet of code only if initialization is a success
    File configFile = SD.open(CONFIG_FILENAME);
    if (configFile.available()) {

      //read time step
      time_step = configFile.readStringUntil(';').toInt();
      while (configFile.peek() != '\n'){ //if there are any black space if the config file
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

      //read number of measurement to be sent at once
      String str_nb_meas_sent_at_once = configFile.readStringUntil(';');
      nb_meas_sent_at_once = str_nb_meas_sent_at_once.toInt();
    }
    configFile.close();

    //print values read on the conf.txt file
    Serial.println("Values read from SD Card:");
    Serial.print("- Deep sleep time: ");
    Serial.println(time_step);
    Serial.print("- Set the RTC clock with GSM: ");
    Serial.println(SetRTC);
    Serial.print("- Number of measurements to be sent at once: ");
    Serial.println(nb_meas_sent_at_once);
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
  digitalWrite(MOSFET_NOTECARD_PIN, LOW);
  tp.DotStar_SetPower(false); //LED of the TinyPico
  rtc.writeSqwPinMode(DS3231_OFF);
  //set alarm on RTC
  rtc.clearAlarm(1); //to clear a potentially previously set alarm set on alarm1
  rtc.disableAlarm(2); //disable alarm2 to ensure that no previously set alarm causes any problem
  rtc.setAlarm1(start_time+bootCount_since_change*sleeping_time,DS3231_A1_Hour);  
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
  DateTime curr_time = starting_time_dt +(bootCount_since_change-1)*time_step;
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", curr_time.day(), curr_time.month(), curr_time.year(), curr_time.hour(), curr_time.minute(), curr_time.second());
  data_message = data_message + ";"+ buffer + ";";
  String sensor_data = sensor.getFileData(); 
  data_message = data_message + sensor_data;
  Serial.println(data_message);

  //initialise SD card
  if (!SD.begin(SD_CS_PIN)) { //if there is a problem with the SD card turn on display to show to the user that there is a problem with the SD card
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

void initialize_notecard(){
 //power the notecard and switch the multiplexer to its I2c line
  digitalWrite(MOSFET_NOTECARD_PIN, HIGH); 
  sensor.tcaselect(NOTECARD_I2C_MULTIPLEXER_CHANNEL);
  delay(100);

  //start notecard
  notecard.begin();
  delay(400);

  //set the notecard mode and tell where to send data
  J *req_set = notecard.newRequest("hub.set");    
  JAddStringToObject(req_set, "product", myProductID);
  JAddStringToObject(req_set, "mode", "minimum");
  notecard.sendRequest(req_set);
}

bool synchronize_notecard(){ //returns true if it was able to connect the notecard to GSM nework
  //send command to sync the notecard
  notecard.sendRequest(notecard.newRequest("hub.sync"));
  //wait for syncronisation
  bool GSM_time_out = false;
  unsigned long start_time_connection = micros();
  int completed=0;
  do{ //this loop wait until then connection is completed or until we reach 4 minutes (timeout)
    J *SyncStatus = notecard.requestAndResponse(notecard.newRequest("hub.sync.status"));
    completed = (int)JGetNumber(SyncStatus,"completed");
    notecard.deleteResponse(SyncStatus); //delete the response
    if((micros()-start_time_connection)/1000000>240){ //after 240s stop waiting for the connexion
      GSM_time_out = true;
      return false;
    }
    delay(500); //check connection status every 500ms
  } while (completed==0 && !GSM_time_out); 

  if (completed!=0){
    Serial.println("The notecard is syncronized!");
    return true;
  }
  else{
    Serial.println("The notecard failed to synchronize!");
    return false;
  }
}

void get_external_parameter(){   //get parameters from notehub like timestep or nb_meas
  //get the number of changes made on notehub
  J *notecard_changes = notecard.requestAndResponse(notecard.newRequest("file.changes"));
  J *changes_infos = JGetObjectItem(notecard_changes,"info");
  J *data_changes = JGetObjectItem(changes_infos,"data.qi");
  int nb_changes = (int) JGetInt(data_changes,"total");
  notecard.deleteResponse(notecard_changes);
  Serial.print("Nb Changes: ");
  Serial.println(nb_changes);
  int new_time_step = 0;
  int new_nb_meas_sent_at_once = 0;

  if (nb_changes >0){ // only do something if there were changes on notehub
    while (nb_changes>0){
      //the parameters changed come in a FIFO queue, so oldest changes comes first in this loop and can later be overwritten by new changes if there were sevral changes of the same parameter
      J *req = notecard.newRequest("note.get"); 
      JAddStringToObject(req, "file", "data.qi"); 
      JAddBoolToObject(req, "delete", true);
      J *parameter_changed = notecard.requestAndResponse(req);
      J *parameter_changed_body = JGetObjectItem(parameter_changed,"body");

      //adjust the correct parameter depending on its name
      int temp_time_step = (int) JGetInt(parameter_changed_body,"time_step"); 
      if(temp_time_step != 0){
        new_time_step = temp_time_step;
      }
      int temp_nb_meas_sent_at_once = (int) JGetInt(parameter_changed_body,"nb_meas");
      if(temp_nb_meas_sent_at_once != 0){
        new_nb_meas_sent_at_once = temp_nb_meas_sent_at_once;
      }
      notecard.deleteResponse(parameter_changed);
      nb_changes-=1;
    }
    if(new_time_step!=0 || new_nb_meas_sent_at_once!=0){ //only do changes if one of the parameters changed on notehub had the correct name
      if(new_time_step==0) new_time_step = time_step; //no change in time step
      if(new_nb_meas_sent_at_once==0) new_nb_meas_sent_at_once = nb_meas_sent_at_once; //no change in nb_meas_sent_at_once
      Serial.print("new time step: ");
      Serial.println(new_time_step);
      Serial.print("new number of measurements to send at once: ");
      Serial.println(new_nb_meas_sent_at_once);
      synchronize_notecard(); //sync the fact that we read and deleted parameters to remove them from the notehub queue

      //write changes to sd card conf.txt file
      Serial.println("change conf file");
      String new_conf_file_str = String(new_time_step)+"; //time step in seconds \n";
      new_conf_file_str = new_conf_file_str + String(SetRTC) +"; //set RTC with GSM time (1 or 0) \n";
      new_conf_file_str = new_conf_file_str + String(new_nb_meas_sent_at_once) +"; //number of measurements to be sent at once via gsm \n";
      SD.remove(CONFIG_FILENAME); //delete conf file
      File configFile = SD.open(CONFIG_FILENAME, FILE_APPEND); //create a new conf file with new parameters
      configFile.print(new_conf_file_str);
      configFile.close();

      if(bootCount!=0){
        //since we might have changed the time step, the computation of the wake up time: wkae_up_time = start_time + bootcount_since_change * time_step no longer holds
        //the solution is to redefine the start time with the next wakeup time and to set the bootcount since change to 0
        DateTime starting_time_dt = DateTime(start_date_time[0],start_date_time[1],start_date_time[2],start_date_time[3],start_date_time[4],start_date_time[5]);
        DateTime new_start_time = starting_time_dt +bootCount_since_change*time_step;
        start_time = new_start_time.unixtime();
        start_date_time[0]=new_start_time.year();
        start_date_time[1]=new_start_time.month();
        start_date_time[2]=new_start_time.day();
        start_date_time[3]=new_start_time.hour();
        start_date_time[4]=new_start_time.minute();
        start_date_time[5]=new_start_time.second();
        bootCount_since_change = 0;
      }
      time_step=new_time_step;
      nb_meas_sent_at_once=new_nb_meas_sent_at_once;
    }
  }
}

void set_RTC_with_GSM_time(){ //gets the time from GSM and adjust the rtc clock with the swiss winter time (UTC+1)
  Serial.println("Try to use the notehub to request the GSM time ....");
  unsigned int GSM_time = getGSMtime()+2; // I added an offset of 2 seconds since the time was somehow always off by 2 seconds
  rtc.adjust(DateTime(GSM_time));  
  readRTC();
  sleep(3);
}

unsigned int getGSMtime(){
  //power the notecard and switch the multiplexer to its I2c line
  initialize_notecard();

  //inform the user with the display and the RGB led that the notecard is synchronizing
  u8x8.setCursor(0, 4);
  u8x8.print("Get GSM time...");
  tp.DotStar_SetPixelColor(25, 0, 25 ); //LED Purple

  if(synchronize_notecard()){
    get_external_parameter(); //get changes from notehub
    J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time"));
    unsigned int recieved_time = (unsigned int)JGetNumber(rsp, "time");
    notecard.deleteResponse(rsp);

    //show the numbers of bars of GSM network (0 to 4) like on a phone
    J *gsm_info = notecard.requestAndResponse(notecard.newRequest("card.wireless"));
    J *net_infos = JGetObjectItem(gsm_info,"net");
    int bars = JGetNumber(net_infos, "bars");
    notecard.deleteResponse(gsm_info);
    Serial.print("Bars: ");
    Serial.println(bars);
    u8x8.setCursor(0, 6);
    u8x8.print("GSM Bars: ");
    u8x8.print(bars);
    tp.DotStar_SetPixelColor(0, 50, 0 ); //LED Green
    Serial.println("The RTC time is adjusted with the GSM time (GMT + 1)");
    digitalWrite(MOSFET_NOTECARD_PIN, LOW); //turn off the notecard
    sensor.tcaselect(0); //connect the multiplexer to another I2c line to avoid having the notecard interfer with other devices
    return recieved_time+3600; //+3600 for GMT+1 (winter time in switzerland)
  }
  else{
    Serial.println("could not connect the Notecard, we use RTC time instead");
    u8x8.setCursor(0, 6);
    u8x8.print("Failed to connect");
    tp.DotStar_SetPixelColor(50, 0, 0 ); //LED Red
    digitalWrite(MOSFET_NOTECARD_PIN, LOW); //turn off the notecard
    sensor.tcaselect(0); //connect the multiplexer to another I2c line to avoid having the notecard interfer with other devices
    return (unsigned int)rtc.now().unixtime(); //if notecard couldn't connect just return the current time of the rtc.
  }
}

void send_data_overGSM(){

  //read the lines to be sent in the SD memory
  File myFile = SD.open(DATA_FILENAME, FILE_READ);
  int position_in_csv = myFile.size(); //get the position of the last character in the csv file

  //find the position where the last few lines of the CSV begin but starting from the end of the CSV since the file might be very long
  for (int i = 0; i < nb_meas_sent_at_once;i++){
    position_in_csv = position_in_csv -20; // avoid the last twenty characters since a line is certainly longer than 20 characters
    myFile.seek(position_in_csv); 
    while (myFile.peek() != '\n'){
      position_in_csv = position_in_csv - 1;
      myFile.seek(position_in_csv); // Go backwards until we detect the previous line separator
    }
  }
  myFile.seek(position_in_csv+1); //at this point we are at the beginning of the nb_meas_sent_at_once last lines of the csv file

  //creat a matrix with the values to be sent. The matrix will look like this:
    /*
  data_matrix
          Vbat sensor1 sensor2 ...
  meas1     x       x       x     
  meas2     x       x       x     
  meas3     x       x       x     
  ...
  
  The time array contains the measurement times in unixtime (UTC+0)

  time array
          time
  meas1     x         
  meas2     x         
  meas3     x      
  ...
  
  */
  String data_matrix[nb_meas_sent_at_once][sensor.get_nb_values()]; //the "2 + ..." is there because we also put Bootcount and DateTime in our matrix
  int time_array[nb_meas_sent_at_once];
  int counter=0;
  while (myFile.available()) {
    for (int i =0; i<2+sensor.get_nb_values();i++){ //the +2 is for datetime and bootcount which are not sensors data but still stored on sd card
      String element = myFile.readStringUntil(';');
      if (element.length()>0 &&i==1){ //convert string time to unixtime ->first parse the time string to transform it in datetime format
        String day_stored = element.substring(0,2);
        String month_stored = element.substring(3,5);
        String year_stored = element.substring(6,10);
        String hour_stored = element.substring(11,13);
        String minute_stored = element.substring(14,16);
        String second_stored = element.substring(17,19);
        DateTime datetime_stored = DateTime(year_stored.toInt(),month_stored.toInt(),day_stored.toInt(),hour_stored.toInt(),minute_stored.toInt(),second_stored.toInt());
        time_array[counter] = datetime_stored.unixtime()-3600; //since time stored on SD is UTC+1 but we need UTC+0 for the notecard
      }
      if (element.length()>0 &&i>1) data_matrix[counter][i-2] = element;
    }
    myFile.readStringUntil('\n');
    counter++;
  }
  myFile.close();

  //print the values which will be sent
  Serial.println("We now try to send several measurements over GSM all at once");
  for (int i =0; i<nb_meas_sent_at_once;i++) {
    Serial.print("line to be sent-> ");
    Serial.print("time: ");
    Serial.print(time_array[i]); 
    Serial.print("   data: ");
    for(int j=0;j<sensor.get_nb_values();j++){    
      Serial.print(data_matrix[i][j]);
      Serial.print("  ");
    }
    Serial.println();
  }

  initialize_notecard();

  //turn LED yellow while sending data
  tp.DotStar_SetPixelColor(25, 25, 0 ); 

  //create the Jason file to be sent
  String* sensor_names=sensor.get_names();
  J *body = JCreateObject();
  J *data = JCreateObject();
  JAddItemToObject(body, "data", data);
  for(int i=0; i<sensor.get_nb_values();i++){
    J *sensor = JAddArrayToObject(data,sensor_names[i].c_str());
    for(int j=0; j<nb_meas_sent_at_once;j++){
      J *sample = JCreateObject();
      JAddItemToArray(sensor, sample);
      JAddStringToObject(sample, "value", data_matrix[j][i].c_str());
      JAddStringToObject(sample, "epoch", String(time_array[j]).c_str()); 
    }
  }

  //print the Jason data
  char* tempString = JPrint(body);
  Serial.println("Print the body object ...");
  Serial.println(tempString);

  //create a request to queue the data to be sent
  J *req_queue_data = notecard.newRequest("note.add");
  JAddItemToObject(req_queue_data, "body", body);
  notecard.sendRequest(req_queue_data);

  //syncronizing notecard with notehub to send the data
  if(synchronize_notecard()){
    Serial.println("data successfully sent!");
    tp.DotStar_SetPixelColor(0, 50, 0 ); //LED green
    get_external_parameter(); //if parameters changed on notehub, adjust the parameters
  }
  else{
    Serial.println("failed to send the data"); 
    tp.DotStar_SetPixelColor(50, 0, 0 ); //LED red
  }

  digitalWrite(MOSFET_NOTECARD_PIN, LOW); 
  sensor.tcaselect(0); //connect the multiplexer to another I2c line to avoid having the notecard interfer with other devices
  delay(100); 
}
 

