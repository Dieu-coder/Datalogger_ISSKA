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
#define Enable5VPIN 25 //turns the 3V to 5V converter on if switched to high
#define MOSFET_SENSORS_PIN 14 //pin which controls the power of the sensors, the screen and sd card reader
#define MOSFET_NOTECARD_PIN 4 //pin which controls the power of the notecard
#define NOTECARD_I2C_MULTIPLEXER_CHANNEL 3 //channel of the multiplexer to which the notecard is connected

#define myProductID "com.gmail.ouaibeer:datalogger_isska" //for the notecard to know with which hub to connect
// "m" = LTE-M, "nb" = NB-IoT, "auto" = scan
#define DESIRED_RAT "m"

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
RTC_DATA_ATTR int failed_sync_count = 0;
RTC_DATA_ATTR int inbound_period = 4;  // 1 sync IN toutes les 4 séances d’envoi par défaut
RTC_DATA_ATTR uint32_t send_success_count = 0;  // # de sync OUT réussies depuis le boot


// =================== HELPERS ===================
static void i2c_bus_hard_reset(int sda=21, int scl=22) {
  // Libère le bus si SDA reste LOW (9 pulses SCL)
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(2);

  if (digitalRead(sda) == LOW) {
    pinMode(scl, OUTPUT);
    for (int i=0; i<9 && digitalRead(sda)==LOW; ++i) {
      digitalWrite(scl, LOW);  delayMicroseconds(5);
      digitalWrite(scl, HIGH); delayMicroseconds(5);
    }
  }

  // Recréer le driver proprement
  Wire.end();
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(1);
  Wire.begin();
  Wire.setTimeout(300);
  Wire.setClock(100000);
}

static void releaseNotecardBus() {
  // Si le TCA (rail capteurs) est coupé, ne surtout pas parler I2C
  if (digitalRead(MOSFET_SENSORS_PIN) == LOW) return;
  sensor.tcaselect(0);
}
// -- DEBUG: impression du body JSON en streaming, sans JPrint ni copies --
static void print_json_body_chunk(J *body, const char* const* sensor_names, int ns) {
  if (!body) { Serial.println("BODY: (null)"); return; }
  J *data = JGetObjectItem(body, "data");
  if (!data) { Serial.println("BODY: pas de clé 'data'"); return; }

  Serial.println("BODY (JSON envoyé) :");
  Serial.println("{\"data\":{");

  for (int i = 0; i < ns; i++) {
    const char* name = sensor_names[i];
    Serial.print("  \""); Serial.print(name); Serial.print("\":[");

    J *arr = JGetObjectItem(data, name);      // tableau JSON pour ce capteur
    bool first = true;
    for (J *it = (arr ? arr->child : nullptr); it; it = it->next) {
      if (!first) Serial.print(",");
      Serial.print("{\"epoch\":");

      // epoch (numérique)
      double epoch = JGetNumber(it, "epoch");
      Serial.print((unsigned long)epoch);

      // value (string si tu envoies des strings, sinon nombre)
      Serial.print(",\"value\":");
      const char *vstr = JGetString(it, "value");
      if (vstr) {
        Serial.print("\""); Serial.print(vstr); Serial.print("\"");
      } else {
        // au cas où 'value' serait numérique (chemin non utilisé dans ta config actuelle)
        double vnum = JGetNumber(it, "value");
        char nb[24];
        // 6 décimales par défaut; ajuste si besoin
        dtostrf(vnum, 0, 6, nb);
        Serial.print(nb);
      }
      Serial.print("}");

      first = false;
    }
    Serial.println("],");
  }

  Serial.println("}}");
}

// Attend jusqu'à 'max_wait_s' que le modem soit attaché.
// On ne s'appuie pas sur 'bars' (souvent 0). On logge l'état toutes les ~5 s.
static bool wait_for_network(unsigned long max_wait_s = 120,
                             bool require_bars = false, int min_bars = 1) {
  unsigned long start_ms = millis();
  unsigned long last_log_s = 9999;

  while ((millis() - start_ms) < max_wait_s * 1000UL) {
    J *rsp = notecard.requestAndResponse(notecard.newRequest("card.wireless"));
    J *net = JGetObjectItem(rsp, "net");
    const char *status = net ? JGetString(net, "status") : nullptr;
    int bars = net ? (int)JGetNumber(net, "bars") : 0;
    notecard.deleteResponse(rsp);

    bool attached = status && (
      strcmp(status, "registered") == 0 ||
      strcmp(status, "attached")   == 0 ||
      strcmp(status, "connected")  == 0
    );
    if (attached && (!require_bars || bars >= min_bars)) {
      Serial.printf("📶 modem prêt: status=%s, bars=%d\n", status, bars);
      return true;
    }

    unsigned long now_s = millis() / 1000UL;
    if (now_s - last_log_s >= 5) {
      Serial.printf("⏳ modem: status=%s, bars=%d\n", status ? status : "?", bars);
      last_log_s = now_s;
    }
    delay(1500); // backoff léger, pas de session ouverte
  }
  Serial.println("⏱️ modem pas prêt (timeout)");
  return false;
}
static bool sync_once(bool want_in, unsigned long timeout_s = 190) {
  J *req = notecard.newRequest("hub.sync");
  JAddBoolToObject(req, "out", true);
  if (want_in) JAddBoolToObject(req, "in", true);
  notecard.sendRequest(req);
  return wait_sync_completed(timeout_s);
}

static void powerOffNotecard() {
  digitalWrite(MOSFET_NOTECARD_PIN, LOW);
  sensor.tcaselect(0);
}

static bool wait_sync_completed(unsigned long timeout_s) {
  const unsigned long TO_US = timeout_s * 1000000UL;
  unsigned long start = micros();
  while (true) {
    J *st = notecard.requestAndResponse(notecard.newRequest("hub.sync.status"));
    if (!st) {
      if ((micros() - start) > TO_US) return false;
      delay(500);
      continue;
    }
    int completed = (int)JGetNumber(st, "completed");
    notecard.deleteResponse(st);
    if (completed) return true;
    if ((micros() - start) > TO_US) return false;
    delay(500);
  }
}

// OUTBOUND uniquement (push → court)
static bool sync_out(unsigned long timeout_s = 300) {
  J* req = notecard.newRequest("hub.sync");
  JAddBoolToObject(req, "out", true);
  notecard.sendRequest(req);
  return wait_sync_completed(timeout_s);
}

// INBOUND uniquement (pull → court)
static bool sync_in(unsigned long timeout_s = 90) {
  J* req = notecard.newRequest("hub.sync");
  JAddBoolToObject(req, "in", true);
  notecard.sendRequest(req);
  return wait_sync_completed(timeout_s);
}

static bool initialize_notecard_safe() {
  const uint32_t T_INIT_BUDGET_MS = 8000;  // budget total init
  uint32_t t0 = millis();

  // Power ON
  digitalWrite(MOSFET_NOTECARD_PIN, HIGH);
  delay(30);
  sensor.tcaselect(NOTECARD_I2C_MULTIPLEXER_CHANNEL);

  // Petit délai de boot
  delay(300);

  // 2 tentatives max : reset bus + power-cycle si besoin
  for (int attempt = 0; attempt < 2; ++attempt) {

    notecard.begin();
    delay(400);

    J *ver = notecard.requestAndResponse(notecard.newRequest("card.version"));
    if (ver) {
      notecard.deleteResponse(ver);

      // hub + RAT (comme avant)
      J *req_set = notecard.newRequest("hub.set");
      JAddStringToObject(req_set, "product", myProductID);
      JAddStringToObject(req_set, "mode", "minimum");
      notecard.sendRequest(req_set);

      J *w = notecard.newRequest("card.wireless");
      JAddStringToObject(w, "mode", DESIRED_RAT);
      notecard.sendRequest(w);

      // (log bars optionnel)
      J *rsp = notecard.requestAndResponse(notecard.newRequest("card.wireless"));
      if (rsp) {
        J *net = JGetObjectItem(rsp,"net");
        int bars = JGetNumber(net,"bars");
        Serial.print("Wireless set, bars="); Serial.println(bars);
        notecard.deleteResponse(rsp);
      }
      return true;
    }

    // Échec : on reset bus et power-cycle rapidement
    Serial.println("❌ Notecard: pas de réponse, reset I2C + power-cycle");
    i2c_bus_hard_reset();
    digitalWrite(MOSFET_NOTECARD_PIN, LOW);
    delay(100);
    digitalWrite(MOSFET_NOTECARD_PIN, HIGH);
    delay(400);

    if (millis() - t0 > T_INIT_BUDGET_MS) break;
  }

  // Abandon court et propre
  sensor.tcaselect(0);
  digitalWrite(MOSFET_NOTECARD_PIN, LOW);
  return false;
}


//********************** MAIN LOOP OF THE PROGRAMM *******************************
//The setup is recalled each time the micrcontroller wakes up from a deepsleep
void setup(){ 

  //Start serial communication protocol with external devices (e.g. your computer)
  Serial.begin(115200);

  power_external_device();

  Wire.begin(); // initialize I2C bus 
  //Wire.setTimeout(25);
  Wire.setClock(50000); // set I2C clock frequency to 50kHz
  sensor.tcaselect(0); 

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
      deep_sleep_mode(60*60*24*365*10); //sleep during ten years (don't wake up) to avoid damaging the battery
    }
  }
}

void loop(){ //No loop is used 
}

void power_external_device(){
  //Turn on Pin to control the gate of MOSFET for all the sensors, display and microSD card(turned on)
  pinMode(MOSFET_SENSORS_PIN, OUTPUT);
  digitalWrite(MOSFET_SENSORS_PIN, HIGH);
  pinMode(MOSFET_NOTECARD_PIN, OUTPUT);
  digitalWrite(MOSFET_NOTECARD_PIN, LOW);
  delay(50);
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

// Display sensor values
sensor.measure();                 // mesure tous les capteurs

// (facultatif) log sur le port série, sans String
sensor.printSerial(Serial);

// Affichage OLED : on met le texte formaté dans un buffer, puis on l'affiche
char oledbuf[256];
sensor.formatSerialToBuffer(oledbuf, sizeof(oledbuf));

u8x8.clear();
u8x8.setCursor(0, 0);
int line_count = 0;

for (const char* p = oledbuf; *p; ++p) {
  char c = *p;
  if (c == '\n') {
    line_count++;
    if (line_count % 8 == 0) {   // pagination si > 8 lignes
      delay(8000);
      u8x8.clear();
      u8x8.setCursor(0, 0);
    } else {
      u8x8.println();
    }
  } else {
    u8x8.print(c);
  }
}
delay(8000); // attendre 8 s pour lire
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
      // lire INBOUND_PERIOD (4e ligne) si présent, sinon garder la valeur par défaut
      // sauter jusqu'à la fin de ligne actuelle (comme au-dessus)
      while (configFile.peek() != '\n' && configFile.peek() != -1) {
        configFile.seek(configFile.position() + 1);
      }
      if (configFile.peek() == '\n') {
        configFile.seek(configFile.position() + 1); // enlever le '\n'
      }

      // si on a encore des données, lire le 4e champ jusqu'au ';'
      if (configFile.available()) {
        String str_inbound_period = configFile.readStringUntil(';');
        int tmp_inbound = str_inbound_period.toInt();
        if (tmp_inbound > 0) inbound_period = tmp_inbound;
      }

    configFile.close();

    //print values read on the conf.txt file
    Serial.println("Values read from SD Card:");
    Serial.print("- Inbound period (1/N envoie): ");
    Serial.println(inbound_period);
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
       dataFile.println();
       dataFile.println();
       dataFile.println();
       dataFile.print("ID;DateTime;");
       sensor.printFileHeader(dataFile);   // ← plus de String, écriture directe
       dataFile.println();
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
    dataFile.print(data_message);     // "ID;DateTime;" déjà construit
    sensor.printFileData(dataFile);   // ← ajoute "v1;v2;...;" sans String
    dataFile.println();               // fin de ligne
    dataFile.close();
  }
}

void mesure_all_sensors(){
  Serial.println("Start of the measurements");
  sensor.measure(); // measures all sensors and stores the values in an array
  char oledbuf[256];
sensor.formatSerialToBuffer(oledbuf, sizeof(oledbuf));
// tu peux garder ta boucle qui itère caractère par caractère sur oledbuf
for (const char* p = oledbuf; *p; ++p) {
  char c = *p;
  if (c == '\n') {
    // gérer changement de ligne OLED
  } else {
    // u8x8.print(c);
  }
}
// et pour le port série:
sensor.printSerial(Serial);
  save_data_in_SD(); //save measuremetns on the SD card
}


void get_external_parameter(uint8_t max_notes = 8) {
  // S'assurer d'être sur le canal Notecard (et revenir sur 0 à la fin)
  sensor.tcaselect(NOTECARD_I2C_MULTIPLEXER_CHANNEL);

  // Compteurs "nouveaux" (=-1 → pas de changement reçu)
  int new_time_step = -1;
  int new_nb_meas_sent_at_once = -1;
  int new_inbound_period = -1;

  // 1) Demander le nombre de changements pour data.qi (schémas différents possibles)
  int nb_changes = 0;
  {
    J *req = notecard.newRequest("file.changes");
    JAddStringToObject(req, "file", "data.qi");      // <— préciser le fichier
    J *rsp = notecard.requestAndResponse(req);
    if (rsp) {
      // Nouveaux firmwares : "total" à la racine
      nb_changes = (int)JGetInt(rsp, "total");
      // Variantes : dans "info", voire "info" → "data.qi"
      if (nb_changes <= 0) {
        J *info = JGetObjectItem(rsp, "info");
        if (info) {
          int t = (int)JGetInt(info, "total");
          if (t > nb_changes) nb_changes = t;
          J *dq = JGetObjectItem(info, "data.qi");
          if (dq) {
            t = (int)JGetInt(dq, "total");
            if (t > nb_changes) nb_changes = t;
          }
        }
      }
      notecard.deleteResponse(rsp);
    }
  }

  Serial.print("Nb Changes (data.qi): ");
  Serial.println(nb_changes);
  if (nb_changes <= 0) { sensor.tcaselect(0); return; }

  // 2) Tirer au plus 'max_notes' notes de config
  int pulled = 0;
  while (nb_changes > 0 && pulled < max_notes) {
    J *req = notecard.newRequest("note.get");
    JAddStringToObject(req, "file", "data.qi");
    JAddBoolToObject(req, "delete", true);
    J *rsp = notecard.requestAndResponse(req);
    if (!rsp) break;

    J *body = JGetObjectItem(rsp, "body");
    if (!body) { notecard.deleteResponse(rsp); break; }

    // Tolérer les variantes/typos de clés
    int ts = (int)JGetInt(body, "time_step");
    if (ts <= 0) ts = (int)JGetInt(body, "time_setp");   // typo rencontrée
    if (ts <= 0) ts = (int)JGetInt(body, "timestep");
    if (ts > 0) new_time_step = ts;

    int nm = (int)JGetInt(body, "nb_meas");
    if (nm > 0) new_nb_meas_sent_at_once = nm;

    int ip = (int)JGetInt(body, "inbound_period");
    if (ip > 0) new_inbound_period = ip;

    notecard.deleteResponse(rsp);
    nb_changes--;
    pulled++;
  }

  // 3) Appliquer si quelque chose a réellement changé
  bool changed = false;
  if (new_time_step > 0 && new_time_step != time_step) {
    time_step = new_time_step;
    changed = true;
  }
  if (new_nb_meas_sent_at_once > 0 && new_nb_meas_sent_at_once != nb_meas_sent_at_once) {
    nb_meas_sent_at_once = new_nb_meas_sent_at_once;
    changed = true;
  }
  if (new_inbound_period > 0 && new_inbound_period != inbound_period) {
    inbound_period = new_inbound_period;
    changed = true;
  }

  if (changed) {
    Serial.println("👉 Mise à jour paramètres depuis Notehub");
    Serial.print(" - time_step = "); Serial.println(time_step);
    Serial.print(" - nb_meas_sent_at_once = "); Serial.println(nb_meas_sent_at_once);
    Serial.print(" - inbound_period = "); Serial.println(inbound_period);

    // Réécrire conf.txt (robuste aux redémarrages)
    SD.remove(CONFIG_FILENAME);
    File configFile = SD.open(CONFIG_FILENAME, FILE_APPEND);
    if (configFile) {
      configFile.print(String(time_step) + "; //time step in seconds \n");
      configFile.print(String(SetRTC) + "; //set RTC with GSM time (1 or 0) \n");
      configFile.print(String(nb_meas_sent_at_once) + "; //number of measurements to be sent at once via gsm \n");
      configFile.print(String(inbound_period) + "; //inbound sync period (do hub.sync inbound every N send cycles)\n");
      configFile.close();
    }

    // Recaler la base de temps pour que l’échéancier reste aligné
    if (bootCount != 0) {
      DateTime starting_time_dt(
        start_date_time[0], start_date_time[1], start_date_time[2],
        start_date_time[3], start_date_time[4], start_date_time[5]
      );
      // On prend comme nouveau "start" la prochaine échéance **selon l’ancien plan**
      DateTime new_start_time = starting_time_dt + bootCount_since_change * (long)time_step;
      start_time        = (int)new_start_time.unixtime();
      start_date_time[0]= new_start_time.year();
      start_date_time[1]= new_start_time.month();
      start_date_time[2]= new_start_time.day();
      start_date_time[3]= new_start_time.hour();
      start_date_time[4]= new_start_time.minute();
      start_date_time[5]= new_start_time.second();
      bootCount_since_change = 0;
    }

  }

  // 4) Toujours relâcher le bus TCA
  sensor.tcaselect(0);
}

void set_RTC_with_GSM_time(){ //gets the time from GSM and adjust the rtc clock with the swiss winter time (UTC+1)
  Serial.println("Try to use the notehub to request the GSM time ....");
  unsigned int GSM_time = getGSMtime()+2; // I added an offset of 2 seconds since the time was somehow always off by 2 seconds
  rtc.adjust(DateTime(GSM_time));  
  readRTC();
  delay(3000);
}

unsigned int getGSMtime(){
  //power the notecard and switch the multiplexer to its I2c line
  initialize_notecard_safe();

  //inform the user with the display and the RGB led that the notecard is synchronizing
  u8x8.setCursor(0, 4);
  u8x8.print("Get GSM time...");
  tp.DotStar_SetPixelColor(25, 0, 25 ); //LED Purple

  if(sync_in()){
    //get_external_parameter(); //get changes from notehub
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

void send_data_overGSM() {
  // ====== Paramètres généraux ======
  const int MAX_BATCH   = 300;
  int       CHUNK_LINES = 30;      // on peut ajuster si OOM à l’attache du body
  const unsigned long MAX_MS = 200000UL;   // 200 s: time-box global

  unsigned long t0 = millis();
  auto remaining_s = [&]() -> unsigned long {
    unsigned long now = millis();
    if (now - t0 >= MAX_MS) return 0UL;
    return (MAX_MS - (now - t0)) / 1000UL;
  };
  auto timed_out = [&]() -> bool { return (millis() - t0) >= MAX_MS; };

  // ====== Sélection des lignes à envoyer ======
  int want = nb_meas_sent_at_once + (failed_sync_count * nb_meas_sent_at_once);
  int target_lines = min(want, MAX_BATCH);
  if (target_lines <= 0) { Serial.println("Rien à envoyer."); return; }

  File myFile = SD.open(DATA_FILENAME, FILE_READ);
  if (!myFile) { Serial.println("❌ SD open error"); return; }

  long size = myFile.size();
  if (size <= 0) { myFile.close(); Serial.println("Fichier vide."); return; }

  myFile.seek(max(0L, size - 1));
  bool ends_with_nl = (myFile.read() == '\n');

  int needed_newlines = target_lines + (ends_with_nl ? 1 : 0);
  long pos = size - 1; int found = 0;
  while (pos >= 0 && found < needed_newlines) {
    myFile.seek(pos);
    if (myFile.read() == '\n') found++;
    pos--;
  }
  long startOff = (pos < 0) ? 0 : (pos + 1);
  myFile.seek(startOff);
  int c = myFile.peek();
  if (c == '\r') { myFile.read(); c = myFile.peek(); }
  if (c == '\n') { myFile.read(); }

  int effective_lines = found - (ends_with_nl ? 1 : 0);
  if (effective_lines < target_lines) target_lines = effective_lines;
  if (target_lines <= 0) { myFile.close(); Serial.println("Pas assez de lignes."); return; }

  // ====== Préparation capteurs/nom des champs ======
  int ns = sensor.get_nb_values();
  if (ns <= 0) { myFile.close(); Serial.println("Aucun capteur (ns==0)."); return; }
  const char* const* sensor_names = sensor.get_names();

  Serial.printf("📤 Objectif: %d mesures (capées à %d, chunk=%d)\n", want, target_lines, CHUNK_LINES);

  // ====== Initialisation Notecard (time-box + double essai) ======
  bool nc_on = false;
  auto nc_power_cycle = [&]() {
    digitalWrite(MOSFET_NOTECARD_PIN, LOW);
    delay(300);
    digitalWrite(MOSFET_NOTECARD_PIN, HIGH);
    delay(800);
  };

  if (timed_out()) { myFile.close(); Serial.println("⏱️ Timeout avant init NC"); return; }

  if (!initialize_notecard_safe()) {
    Serial.println("⚠️ Init NC: essai #1 KO → power-cycle & retry");
    nc_power_cycle();
    if (!initialize_notecard_safe()) {
      Serial.println("❌ Notecard init KO (après 2 essais)");
      myFile.close();
      // LED rouge pour bien repérer
      tp.DotStar_SetPixelColor(50, 0, 0);
      return;
    }
  }
  nc_on = true;
  // LED jaune (perçu) pendant l’opération
  tp.DotStar_SetPixelColor(25, 25, 0);
  Serial.println("✅ Notecard init OK");

  // ====== Construction/enqueue des chunks ======
  int sent_valid = 0;
  bool aborted_for_time = false;

  while (myFile.available() && sent_valid < target_lines) {
    if (timed_out()) { aborted_for_time = true; break; }

    int chunk_target = min(CHUNK_LINES, target_lines - sent_valid);

    // JSON du chunk
    J *body = JCreateObject();
    J *data = JCreateObject();
    JAddItemToObject(body, "data", data);

    // Tableaux par capteur
    J **arr = (J**) malloc(ns * sizeof(J*));
    if (!arr) {
      Serial.println("❌ OOM chunk arrays");
      JDelete(body);
      break;
    }
    for (int i = 0; i < ns; i++) {
      arr[i] = JAddArrayToObject(data, sensor_names[i]);
      if (!arr[i]) Serial.println("❌ OOM array capteur");
    }

    int chunk_valid = 0;
    int safety_reads = 0;

    while (myFile.available() && chunk_valid < chunk_target && safety_reads < (chunk_target * 4)) {
      if (timed_out()) { aborted_for_time = true; break; }

      safety_reads++;
      String line = myFile.readStringUntil('\n');
      if (line.length() == 0) continue;

      int idx = 0;
      auto nextField = [&](String &out) {
        int sep = line.indexOf(';', idx);
        if (sep < 0) { out = line.substring(idx); idx = line.length(); }
        else { out = line.substring(idx, sep); idx = sep + 1; }
      };

      String fld;
      nextField(fld);  // id
      nextField(fld);  // datetime
      if (fld.indexOf('/') == -1 || fld.indexOf(':') == -1 || fld.length() < 19) continue;

      DateTime dt(
        fld.substring(6,10).toInt(),
        fld.substring(3,5).toInt(),
        fld.substring(0,2).toInt(),
        fld.substring(11,13).toInt(),
        fld.substring(14,16).toInt(),
        fld.substring(17,19).toInt()
      );
      unsigned long epoch = (unsigned long) dt.unixtime() - 3600; // CET/UTC+1 hiver

      bool ok_line = true;
      for (int si = 0; si < ns; si++) {
        String vs; nextField(vs);
        if (vs.length() == 0) { ok_line = false; break; }
        if (vs.length() && vs[vs.length()-1] == '\r') vs.remove(vs.length()-1);

        J *sample = JCreateObject();
        if (!sample) { ok_line = false; break; }
        JAddStringToObject(sample, "value", vs.c_str());
        JAddNumberToObject(sample, "epoch", (double)epoch);
        JAddItemToArray(arr[si], sample);
      }
      if (!ok_line) continue;

      Serial.printf("🕒 %lu  (chunk #%d)\n", epoch, (sent_valid / CHUNK_LINES) + 1);
      chunk_valid++;
      sent_valid++;
    }

    if (aborted_for_time) {
      free(arr);
      JDelete(body);
      break;
    }

    if (chunk_valid == 0) {
      free(arr);
      JDelete(body);
      Serial.println("⚠️ chunk vide, arrêt");
      break;
    }

    // Attache body → note.add
    J *req = notecard.newRequest("note.add");
    JAddStringToObject(req, "file", "data.qo");
    JAddItemToObject(req, "body", body);   // transfert propriété

    if (!JGetObjectItem(req, "body")) {
      Serial.println("⚠️ OOM à l’attache du body → chunk plus petit ensuite");
      JDelete(body);
      free(arr);
      CHUNK_LINES = max(1, CHUNK_LINES / 2);
      break;
    }

    JAddBoolToObject(req, "sync", false);
    notecard.sendRequest(req);     // enqueue sans attendre

    free(arr);
    // ne pas JDelete(body) ici (attaché à req)
  }

  myFile.close();

  // ====== Session hub.sync unique (OUT + éventuellement IN) ======
  int ip = inbound_period; if (ip < 1) ip = 4;
  bool want_in = ((send_success_count + 1) % ip) == 0;

  bool ok = false;
  if (!aborted_for_time && !timed_out()) {
    unsigned long remain = remaining_s();
    if (remain < 10UL) remain = 10UL;            // mini 10 s
    Serial.print("🔄 Sync (OUT"); if (want_in) Serial.print("+IN"); Serial.println(")...");
    ok = sync_once(want_in, remain);
  } else {
    Serial.println("⏱️ Pas de sync: temps imparti écoulé");
  }

  long queued = 0;
  if (ok) {
    J *st = notecard.requestAndResponse(notecard.newRequest("hub.sync.status"));
    if (st) {
      queued = (long) JGetInt(st, "queued");
      notecard.deleteResponse(st);
    }
  }

  if (!ok || queued > 0) {
    Serial.printf("⚠️ Sync partielle: ok=%d queued=%ld\n", (int)ok, queued);
    tp.DotStar_SetPixelColor(50, 20, 0);       // orange
    failed_sync_count = min(failed_sync_count + 1, 3);
  } else {
    Serial.println("✅ Données envoyées !");
    tp.DotStar_SetPixelColor(0, 50, 0);        // vert
    failed_sync_count = 0;
    send_success_count++;
    if (want_in && !timed_out()) {
      get_external_parameter();                // courte, mais si ça traîne, on coupera quand même après
    }
  }

  // ====== Toujours éteindre la Notecard ======
  if (nc_on) {
    powerOffNotecard();                        // MOSFET_NOTECARD_PIN LOW + tcaselect(0)
  }
  delay(80);
}





