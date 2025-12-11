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
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include <Preferences.h>

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

Preferences prefs;
RTC_DATA_ATTR uint32_t rtc_magic = 0;
#define MAGIC 0xA55A1234

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
RTC_DATA_ATTR bool g_nc_inited = false;   // persiste mais on le remettra à true quand on init
RTC_DATA_ATTR bool nc_keep_on = false;
RTC_DATA_ATTR bool keep_nc_next_sleep = false;


// =================== HELPERS ===================

static void ensure_config_valid() {
  if (time_step <= 0 || time_step > 24*3600) {
    Serial.println("⚠️ time_step invalide → fallback 300s");
    time_step = 300;
  }
  if (nb_meas_sent_at_once <= 0 || nb_meas_sent_at_once > 300) {
    Serial.println("⚠️ nb_meas_sent_at_once invalide → fallback 1");
    nb_meas_sent_at_once = 1;
  }
  if (inbound_period <= 0) inbound_period = 4;
  if (start_time <= 0) {
    Serial.println("⚠️ start_time invalide → recalcul");
    get_next_rounded_time();
  }
}
// seconds until next RTC alarm (si sleeping_time==0 on calcule depuis start_time)
static uint32_t seconds_until_next_alarm(int sleeping_time) {
  DateTime now = rtc.now();
  long target = (long)start_time + (long)bootCount_since_change * (long)sleeping_time;
  long delta  = target - (long)now.unixtime();
  if (delta < 5)  delta = 5;                    // marge min
  if (delta > 86400L) delta = 86400L;           // clamp 24 h (sécurité)
  return (uint32_t)delta;
}

// Demande à la Notecard de couper le modem et de dormir pendant N secondes
static void notecard_prepare_for_sleep(uint32_t seconds) {
  sensor.tcaselect(NOTECARD_I2C_MULTIPLEXER_CHANNEL);

  // coupe la radio rapidement
  J *w = notecard.newRequest("card.wireless");
  JAddStringToObject(w, "mode", "off");
  notecard.sendRequest(w);

  // sommeil temporisé côté Notecard (elle restera alimentée mais en veille profonde)
  J *sl = notecard.newRequest("card.sleep");
  JAddNumberToObject(sl, "seconds", (double)seconds);
  notecard.sendRequest(sl);

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
static bool sync_once(bool want_in, unsigned long timeout_s = 300) {
  J *req = notecard.newRequest("hub.sync");
  JAddBoolToObject(req, "out", true);
  if (want_in) JAddBoolToObject(req, "in", true);
  notecard.sendRequest(req);
  return wait_sync_completed(timeout_s);
}

static void releaseNotecardBus() {
  sensor.tcaselect(0);   // on sort du canal TCA3
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
  digitalWrite(MOSFET_NOTECARD_PIN, LOW);
  sensor.tcaselect(NOTECARD_I2C_MULTIPLEXER_CHANNEL);
  delay(30);

  notecard.begin();
  delay(500);

  // Handshake: s'assurer que la carte répond
  J *ver = notecard.requestAndResponse(notecard.newRequest("card.version"));
  if (!ver) {
    Serial.println("❌ Notecard: pas de réponse à card.version");
    sensor.tcaselect(0);
    //digitalWrite(MOSFET_NOTECARD_PIN, LOW);
    return false;
  }
  notecard.deleteResponse(ver);

  // hub + RAT
  J *req_set = notecard.newRequest("hub.set");
  JAddStringToObject(req_set, "product", myProductID);
  JAddStringToObject(req_set, "mode", "minimum");
  notecard.sendRequest(req_set);

  J *w = notecard.newRequest("card.wireless");
  JAddStringToObject(w, "mode", DESIRED_RAT);   // "m" / "nb" / "auto"
  notecard.sendRequest(w);

  // Log bars (optionnel)
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.wireless"));
  if (rsp) {
    J *net = JGetObjectItem(rsp,"net");
    int bars = net ? (int)JGetNumber(net,"bars") : -1;
    Serial.print("Wireless set, bars="); Serial.println(bars);
    notecard.deleteResponse(rsp);
  }

  // en fin de fonction après les réglages
g_nc_inited = true;
nc_keep_on  = true;   // tu l’avais déjà
return true;
}

//********************** MAIN LOOP OF THE PROGRAMM *******************************
//The setup is recalled each time the micrcontroller wakes up from a deepsleep
void setup(){ 

  //Start serial communication protocol with external devices (e.g. your computer)
  Serial.begin(115200);
  delay(150);
  prefs.begin("app", false);

  // How did we wake?
esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
bool was_ds = (cause != ESP_SLEEP_WAKEUP_UNDEFINED) || (rtc_magic == MAGIC);

// Consume the token so a later SW_RESET doesn’t look like deep sleep
rtc_magic = 0;

first_time = !was_ds;

// Restore counters from NVS only if NOT first boot
if (!first_time) {
  bootCount              = prefs.getInt("bootCount", 0);
  bootCount_since_change = prefs.getInt("bootSince", 0);
  send_success_count     = prefs.getULong("sendOK", 0);
  failed_sync_count      = prefs.getInt("failSync", 0);
  inbound_period         = prefs.getInt("inbN", inbound_period);
  start_time             = prefs.getInt("start", start_time);
  for (int i=0;i<6;i++){ char k[8]; sprintf(k,"sd%d",i); start_date_time[i]=prefs.getInt(k,start_date_time[i]); }
}
prefs.end();

// IMPORTANT : au réveil, les GPIO peuvent être "holdés".
// On libère le hold global + celui du pin Notecard avant de le reconfigurer.
   gpio_deep_sleep_hold_dis();
   gpio_hold_dis((gpio_num_t)MOSFET_NOTECARD_PIN);

  power_external_device();

  Wire.begin(); // initialize I2C bus 
  Wire.setTimeout(25);
  Wire.setClock(100000); // set I2C clock frequency to 50kHz

  // ✅ OLED: toujours re-begin car on l’a coupée pendant le deep-sleep
u8x8.begin();
u8x8.setBusClock(50000);
u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);

  if(first_time){ //this snipet is only executed when the microcontroller is turned on for the first time
    tp.DotStar_SetPower(true); //This method turns on the Dotstar power switch, which enables the tinypico LED
    tp.DotStar_SetPixelColor(25, 25, 25 ); //Turn the LED white while SD card is not initialised

    //write Init on the display
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
    ensure_config_valid();

    if (bootCount<=3){ //Turn the LED blue during first 3 measurements ( only to check that the logger works normally)
      tp.DotStar_SetPower(true);
      tp.DotStar_SetPixelColor(0, 0, 25 ); 
    }

    mesure_all_sensors();

    int nsend = (nb_meas_sent_at_once > 0) ? nb_meas_sent_at_once : 1;
    if ((bootCount_since_change % nsend) == 0) {
      send_data_overGSM();
    }

    if(tp.GetBatteryVoltage()>3.4){
      deep_sleep_mode(time_step);
    }
    else{
      digitalWrite(MOSFET_NOTECARD_PIN, HIGH);
      deep_sleep_mode(10800); //sleep during ten years (don't wake up) to avoid damaging the battery
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

  // >>> Désactiver le boost 5V <<<
  pinMode(Enable5VPIN, OUTPUT);
  digitalWrite(Enable5VPIN, LOW);

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
  // --- 0) Calcul de l’instant de réveil ---
  uint32_t wake_epoch = start_time + bootCount_since_change * sleeping_time;

  // --- 1) Assure-toi que le TCA n’est pas sur la Notecard, pour éviter du back-power ---
  sensor.tcaselect(0);

  // --- 2) Programmer la RTC TANT QUE I²C est actif ---
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.clearAlarm(1);
  rtc.disableAlarm(2);
  rtc.setAlarm1(wake_epoch, DS3231_A1_Hour);

  // GPIO du réveil (si pas de pull-up externe, garde INPUT_PULLUP)
  pinMode(GPIO_NUM_15, INPUT);               // ou INPUT_PULLUP si besoin
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 0);

  // --- 3) Couper toutes les charges avant le sleep ---
  // Notecard : OFF à chaque sleep (comme demandé)
  digitalWrite(MOSFET_NOTECARD_PIN, HIGH);

  // Capteurs / OLED / SD / 5V
  u8x8.setPowerSave(1);
  digitalWrite(MOSFET_SENSORS_PIN, LOW);
  digitalWrite(Enable5VPIN, LOW);
  tp.DotStar_SetPower(false);

  // --- 4) Libérer les bus pour traquer les µA résiduels ---
  Wire.end();                // maintenant seulement !
  pinMode(21, INPUT);        // SDA
  pinMode(22, INPUT);        // SCL
  pinMode(5,  INPUT);        // SD-CS
  pinMode(18, INPUT);        // SCK
  pinMode(19, INPUT);        // MISO
  pinMode(23, INPUT);        // MOSI

  // Leave a “deep-sleep token” for next boot
rtc_magic = MAGIC;

// Persist counters/settings so a SW_RESET won’t lose them
prefs.begin("app", false);
prefs.putInt("bootCount",  bootCount);
prefs.putInt("bootSince",  bootCount_since_change);
prefs.putULong("sendOK",   send_success_count);
prefs.putInt("failSync",   failed_sync_count);
prefs.putInt("inbN",       inbound_period);
prefs.putInt("start",      start_time);
for (int i=0;i<6;i++){ char k[8]; sprintf(k,"sd%d",i); prefs.putInt(k,start_date_time[i]); }
prefs.end();

  // --- 5) Dodo ---
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


void get_external_parameter(){   //get parameters from notehub like timestep or nb_meas
  //get the number of changes made on notehub

  int new_inbound_period = 0;
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
      int temp_inbound_period = (int) JGetInt(parameter_changed_body,"inbound_period");
      if (temp_inbound_period > 0) {
      new_inbound_period = temp_inbound_period;
      }
      notecard.deleteResponse(parameter_changed);
      nb_changes-=1;
    }
    if (new_time_step != 0 || new_nb_meas_sent_at_once != 0 || new_inbound_period != 0) { //only do changes if one of the parameters changed on notehub had the correct name
      if(new_time_step==0) new_time_step = time_step; //no change in time step
      if(new_nb_meas_sent_at_once==0) new_nb_meas_sent_at_once = nb_meas_sent_at_once; //no change in nb_meas_sent_at_once
      if(new_inbound_period == 0) new_inbound_period = inbound_period;
      Serial.print("new time step: ");
      Serial.println(new_time_step);
      Serial.print("new number of measurements to send at once: ");
      Serial.println(new_nb_meas_sent_at_once);
      Serial.print("new inbound period: ");
      Serial.println(new_inbound_period);

      //write changes to sd card conf.txt file
      Serial.println("change conf file");
      String new_conf_file_str = String(new_time_step)+"; //time step in seconds \n";
      new_conf_file_str = new_conf_file_str + String(SetRTC) +"; //set RTC with GSM time (1 or 0) \n";
      new_conf_file_str = new_conf_file_str + String(new_nb_meas_sent_at_once) +"; //number of measurements to be sent at once via gsm \n";
      new_conf_file_str += String(new_inbound_period) +"; //inbound sync period (do hub.sync inbound every N send cycles)\n";
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
      inbound_period = new_inbound_period;
    }
  }
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
    //digitalWrite(MOSFET_NOTECARD_PIN, LOW); //turn off the notecard
    sensor.tcaselect(0); //connect the multiplexer to another I2c line to avoid having the notecard interfer with other devices
    releaseNotecardBus();
    return recieved_time+3600; //+3600 for GMT+1 (winter time in switzerland)

  }
  else{
    Serial.println("could not connect the Notecard, we use RTC time instead");
    u8x8.setCursor(0, 6);
    u8x8.print("Failed to connect");
    tp.DotStar_SetPixelColor(50, 0, 0 ); //LED Red
    //digitalWrite(MOSFET_NOTECARD_PIN, LOW); //turn off the notecard
    sensor.tcaselect(0); //connect the multiplexer to another I2c line to avoid having the notecard interfer with other devices
    releaseNotecardBus();
    return (unsigned int)rtc.now().unixtime(); //if notecard couldn't connect just return the current time of the rtc.
  }
}

void send_data_overGSM() {
  // ---- Paramètres d’envoi ----
  const int MAX_BATCH   = 300;
  int CHUNK_LINES       = 30;    // démarre petit pour valider; remonte ensuite (10, 20, …)

  int want = nb_meas_sent_at_once + (failed_sync_count * nb_meas_sent_at_once);
  int target_lines = min(want, MAX_BATCH);
  if (target_lines <= 0) { Serial.println("Rien à envoyer."); return; }

  File myFile = SD.open(DATA_FILENAME, FILE_READ);
  if (!myFile) { Serial.println("❌ SD open error"); return; }

  // ---- Repositionnement sur les N dernières lignes ----
  long size = myFile.size();
  if (size <= 0) { myFile.close(); Serial.println("Fichier vide."); return; }
  myFile.seek(max(0L, size - 1));
  bool ends_with_nl = (myFile.read() == '\n');

  int needed_newlines = target_lines + (ends_with_nl ? 1 : 0);
  long pos = size - 1; int found = 0;
  while (pos >= 0 && found < needed_newlines) { myFile.seek(pos); if (myFile.read() == '\n') found++; pos--; }

  long startOff = (pos < 0) ? 0 : (pos + 1);
  myFile.seek(startOff);
  int c = myFile.peek();
  if (c == '\r') { myFile.read(); c = myFile.peek(); }
  if (c == '\n') { myFile.read(); }

  int effective_lines = found - (ends_with_nl ? 1 : 0);
  if (effective_lines < target_lines) target_lines = effective_lines;
  if (target_lines <= 0) { myFile.close(); Serial.println("Pas assez de lignes."); return; }

  // ---- Préparation ----
  int ns = sensor.get_nb_values();
  if (ns <= 0) { myFile.close(); Serial.println("Aucun capteur (ns==0)."); return; }
  const char* const* sensor_names = sensor.get_names();

  Serial.printf("📤 Objectif: %d mesures (capées à %d, chunk=%d)\n", want, target_lines, CHUNK_LINES);

  // Allumer la notecard maintenant (comme ta version stable)
  if (!initialize_notecard_safe()) {
  Serial.println("❌ Notecard init KO");
  myFile.close();
  return;
}

  tp.DotStar_SetPixelColor(25, 25, 0);

  int sent_valid = 0;

  while (myFile.available() && sent_valid < target_lines) {
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
      if (!arr[i]) { Serial.println("❌ OOM array capteur"); }
    }

    int chunk_valid = 0;
    int safety_reads = 0;

    while (myFile.available() && chunk_valid < chunk_target && safety_reads < (chunk_target * 4)) {
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
      nextField(fld);            // id (ignoré)
      nextField(fld);            // datetime
      if (fld.indexOf('/') == -1 || fld.indexOf(':') == -1 || fld.length() < 19) continue;

      DateTime dt(
        fld.substring(6,10).toInt(),
        fld.substring(3,5).toInt(),
        fld.substring(0,2).toInt(),
        fld.substring(11,13).toInt(),
        fld.substring(14,16).toInt(),
        fld.substring(17,19).toInt()
      );
      unsigned long epoch = (unsigned long) dt.unixtime() - 3600;

      bool ok = true;
      for (int si = 0; si < ns; si++) {
        String vs; nextField(vs);
        if (vs.length() == 0) { ok = false; break; }
        if (vs.length() && vs[vs.length()-1] == '\r') vs.remove(vs.length()-1);

        J *sample = JCreateObject();
        if (!sample) { ok = false; break; }
        JAddStringToObject(sample, "value", vs.c_str());   // garder la précision CSV
        JAddNumberToObject(sample, "epoch", (double)epoch);
        JAddItemToArray(arr[si], sample);
      }
      if (!ok) continue;

      Serial.printf("🕒 %lu  (chunk #%d)\n", epoch, (sent_valid / CHUNK_LINES) + 1);
      chunk_valid++;
      sent_valid++;
    }

    // SI aucun échantillon valide → on libère et on stoppe
    if (chunk_valid == 0) {
      free(arr);
      JDelete(body);
      Serial.println("⚠️ chunk vide, arrêt");
      break;
    }

    // Réduire la charge pendant le TX radio
    u8x8.setPowerSave(1);        // OLED OFF (baisse ~15-20 mA)
    tp.DotStar_SetPower(false);  // LED OFF
    digitalWrite(MOSFET_SENSORS_PIN, LOW); // coupe capteurs + SD (on a fini de lire la SD)
    delay(20);                   // laisse la rail se tasser avant la radio
// … après chunk_valid++ / sent_valid++ et AVANT note.add
// (optionnel : seulement pour le 1er chunk pour éviter d’inonder la console)
static const bool LOG_BODY = false;
if (LOG_BODY) {
  print_json_body_chunk(body, sensor_names, ns);
}
    // note.add avec **body** (aucun JPrint, pas de payload)
    J *req = notecard.newRequest("note.add");
    JAddStringToObject(req, "file", "data.qo");
    JAddItemToObject(req, "body", body);     // transfert de propriété

    // Vérifier que l’attache a réussi (important !)
    if (!JGetObjectItem(req, "body")) {
      Serial.println("⚠️ OOM à l’attache du body → on réessaiera avec un chunk plus petit");
      // on détruit ce qu’on a créé localement
      JDelete(body);
      free(arr);
      // Réduis le chunk pour la suite
      CHUNK_LINES = max(1, CHUNK_LINES / 2);
      break;
    }

    JAddBoolToObject(req, "sync", false);

    // Envoi (sans ACK pour limiter la mémoire, comme dans ta version qui marchait)
    notecard.sendRequest(req);

    free(arr);
    // NE PAS JDelete(body) ici : il est attaché à la requête
  }

  myFile.close();


// → une seule session OUT (+ IN périodique)
int ip = inbound_period; if (ip < 1) ip = 4;
bool want_in = ((send_success_count + 1) % ip) == 0;
Serial.print("🔄 Sync (OUT"); if (want_in) Serial.print("+IN"); Serial.println(")...");
bool ok = sync_once(want_in, 300);
delay(150);


  long queued = 0;
  if (ok) {
    J *st = notecard.requestAndResponse(notecard.newRequest("hub.sync.status"));
    queued = (long) JGetInt(st, "queued");
    notecard.deleteResponse(st);
  }

  if (!ok || queued > 0) {
    Serial.printf("⚠️ Sync partielle: ok=%d queued=%ld\n", (int)ok, queued);
    tp.DotStar_SetPixelColor(50, 25, 0);
    failed_sync_count = min(failed_sync_count + 1, 3);
  } else {
    Serial.println("✅ Données envoyées !");
    tp.DotStar_SetPixelColor(0, 50, 0);
    failed_sync_count = 0;
    send_success_count++;
    if (want_in) get_external_parameter();
  }

  releaseNotecardBus();

  delay(250);
}

