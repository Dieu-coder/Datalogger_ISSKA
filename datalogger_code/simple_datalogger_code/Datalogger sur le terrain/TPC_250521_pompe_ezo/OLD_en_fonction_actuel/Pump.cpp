#include "pump.h"
#include <TinyPICO.h>
#include <EEPROM.h>
// File paths for config and data logging
#define PUMP_CONFIGFILE "/pump.txt"
#define PUMP_DATAFILE "/pumpdata.csv"
#define TEST_FILENAME "/test.csv"

// Constructor
Pump::Pump() {
  // Empty constructor
}




// Persistent variables in RTC memory (survive deep sleep)
RTC_DATA_ATTR InjectionData Pump::data = {
  .heights = {0.0f},     // Initialize all water height readings to 0
  .boot_step = 0,        // Step count since boot
};

RTC_DATA_ATTR int doseTableSize = 0;             // Number of dose configurations
RTC_DATA_ATTR ClassDose doseTable[10];           // Table of dose configurations
RTC_DATA_ATTR int pumpActivationCount = 0;       // Number of times pump was activated
RTC_DATA_ATTR bool Inject = true;                // Whether injection is allowed
RTC_DATA_ATTR int counter = 0;                   // Counter for delay between injections
RTC_DATA_ATTR float water_height_offset = 10;   // offset to correct the water height in function of how deep is the sensor              
float measuredWaterheight =  0;                  // Current measured water level
bool Setoffset = false;
bool Config = false;
bool Filltubes = false;
const int EEPROM_SIZE = 4;  // Enough for a float (4 bytes)
const int EEPROM_INJECTION_START = 4;

// Function to read a response from the pump with a timeout
String Pump::readPumpResponse() {
  String response = "";
  const unsigned long timeout = 120000;  // 2 minutes timeout
  unsigned long startTime = millis();

  while (true) {
    while (Serial2.available() > 0) {
      char inchar = (char)Serial2.read();
      startTime = millis();  // Reset timeout when data is received

      if (inchar == '\r' || inchar == '\n') {
        response.trim();     // Clean whitespace
        if (!response.isEmpty()) {
          return response;   // Return complete response
        }
      } else {
        response += inchar;  // Append character to response
      }
    }

    // Return timeout message if no data is received in time
    if (millis() - startTime > timeout) {
      response = "Timeout";  
      return response;
    }
  }
}

// Function to put the pump into sleep mode
void Pump::pumpSleep() {
  Serial2.print("Sleep\r");  // Send Sleep command
  const unsigned long timeout = 5000;
  unsigned long startTime = millis();

  while (true) {
    String response = readPumpResponse();
    if (!response.isEmpty()) {
      if (response.indexOf("*SL") != -1) {
        Serial.println("Pump response (sl): " + response);
        Serial.println("Pump is now in sleep mode.");
        return;
      } else if (response.indexOf("*ER") != -1){
        Serial.println("Pump couldn't fall asleep");
        handleError(response);
        return;
      }
    }
    if (millis() - startTime > timeout) {
      Serial.println("Pump couldn't fall asleep due to lack of communication");
      return;
    }
  }
}

// Error handling function
bool Pump::handleError(String& response) {
  if (response == "*ER") {
    Serial.println("The pump encountered a problem");
    Serial2.print("X\r");  // Cancel/reset operation
    latestErrorMessage = response;
    return true;
  } else if (response == "Timeout") {
    Serial.print("The pump doesn't answer\n");
    latestErrorMessage = response;
    Serial.print("latesterrormessage : ");
    Serial.println(latestErrorMessage);
    return true;
  } else {
    if (response != "*RS" && response != "*RE" && response != "*OK") {
      latestErrorMessage = response;
      int index = latestErrorMessage.indexOf(',');
      if (index != -1) {
        latestErrorMessage = latestErrorMessage.substring(0, index);
      }
    }
    return false;
  }
}

// Send command to pump and handle responses
void Pump::sendCommand(const String& pumpcommand) {
  bool error = false;
  delay(1000);
  Serial2.print(pumpcommand + "\r");  // Send command to pump

  while (true) {
    String response = readPumpResponse();  // Wait for response

    if (!response.isEmpty()) {
      if (response == "*OK" || response == "*RE" || response == "*RS") {
        continue;  // Ignore generic responses
      } else {
        Serial.println(response);  // Print response
      }

      if (response.indexOf("DONE") != -1) {
        // Successful injection
        latestErrorMessage = "Injected";
        pumpActivationCount++;
        break;
      } else if (response.indexOf("*WA") != -1) {
        // Wait and retry
        latestErrorMessage = "*WA";
        Serial2.print(pumpcommand + "\r");  // Resend command
      } else {
        // Handle errors
        error = handleError(response);
        if (error) break;
        else continue;
      }
    }
  }
  pumpSleep();  // Put pump to sleep after command
} 


// Display current pump configuration on screen and Serial
void Pump::displayConfiguration(U8X8 &u8x8) {
  Serial.printf("%-18s| %-18s| %-5s\n", "Lower class limit", "Upper class limit", "Dose");
  Serial.println("------------------|------------------|------");

  for (int i = 0; i < doseTableSize; i++) {
    Serial.printf("%-18d| %-18d| %-5d\n", doseTable[i].lower, doseTable[i].upper, doseTable[i].dose);
  }

  // Display on OLED
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.println(" L | U | D | I ");
  u8x8.println("---|---|---|--");

  for (int i = 0; i < doseTableSize; i++) {
    if (i % 6 == 0 && i > 0) {
      delay(5000);  // Pause after 6 lines
      u8x8.clear();
      u8x8.setCursor(0, 0);
      u8x8.println(" L | U | D | I ");
      u8x8.println("---|---|---|--");
    }

    // Format values for OLED
    if (doseTable[i].lower < 10) u8x8.print("  ");
    else if (doseTable[i].lower < 100) u8x8.print(" ");
    u8x8.print(doseTable[i].lower); u8x8.print(" ");

    if (doseTable[i].upper < 10) u8x8.print("  ");
    else if (doseTable[i].upper < 100) u8x8.print(" ");
    u8x8.print(doseTable[i].upper); u8x8.print(" ");

    if (doseTable[i].dose < 10) u8x8.print("  ");
    else if (doseTable[i].dose < 100) u8x8.print(" ");
    u8x8.print(doseTable[i].dose); u8x8.print(" ");

    if (doseTable[i].numberOfInjections < 10) u8x8.print("  ");
    else if (doseTable[i].numberOfInjections < 100) u8x8.print(" ");
    u8x8.println(doseTable[i].numberOfInjections);
  }
  delay(5000);
}
// Load configuration from SD card
void Pump::configure(int &time_step) {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } else {
    File pumpconfigFile = SD.open(PUMP_CONFIGFILE);
    int EEPROM_SIZE = sizeof(float) + (countLines(pumpconfigFile) - 1) * sizeof(int);
    EEPROM.begin(EEPROM_SIZE);
    doseTableSize = 0;

    // Blink or reset pump before reading config
    for (int i = 0; i < 3; i++) {
      Serial2.print("L,1\r");
      delay(500);
      Serial2.print("L,0\r");
      delay(500);
    }

    Serial.println("reading config file...");
    
    String str_Config = pumpconfigFile.readStringUntil(';');
    if (str_Config=="0") Config = false;
    else if (str_Config=="1") Config = true;
    Serial.print("debug");
    Serial.println(str_Config);
    while (pumpconfigFile.peek() != '\n'){ //if there are any black space if the config file
      pumpconfigFile.seek(pumpconfigFile.position() + 1);
    }
    pumpconfigFile.seek(pumpconfigFile.position() + 1);
    
    String str_Setoffset = pumpconfigFile.readStringUntil(';');
    if (str_Setoffset=="0") Setoffset = false;
    else if (str_Setoffset=="1") Setoffset = true;
    Serial.print("debug");
    Serial.println(str_Setoffset);
    while (pumpconfigFile.peek() != '\n'){ //if there are any black space if the config file
      pumpconfigFile.seek(pumpconfigFile.position() + 1);
    }
    pumpconfigFile.seek(pumpconfigFile.position() + 1);
    
    String str_Filltubes = pumpconfigFile.readStringUntil(';');
    if (str_Filltubes=="0") Filltubes = false;
    else if (str_Filltubes=="1") Filltubes = true;
    Serial.print("debug");
    Serial.println(str_Filltubes);
    while (pumpconfigFile.peek() != '\n'){ //if there are any black space if the config file
      pumpconfigFile.seek(pumpconfigFile.position() + 1);
    }
    pumpconfigFile.seek(pumpconfigFile.position() + 1);

    while (pumpconfigFile.available()) {
      String line = pumpconfigFile.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;

      int dashPos = line.indexOf('-');
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);

      // Parse lines like "10-20,5,2"
      if (dashPos != -1 && firstComma != -1 && secondComma != -1 && doseTableSize < 10) {
        int lower = line.substring(0, dashPos).toInt();
        int upper = line.substring(dashPos + 1, firstComma).toInt();
        int dose = line.substring(firstComma + 1, secondComma).toInt();
        int numberOfInjections = line.substring(secondComma + 1).toInt();

        if (Config) {
          int addr = EEPROM_INJECTION_START + doseTableSize * sizeof(int);
          EEPROM.put(addr, numberOfInjections);
          EEPROM.commit();
          
          doseTable[doseTableSize++] = {lower, upper, dose, numberOfInjections};
        }

        else if (!Config){
          int addr = EEPROM_INJECTION_START + doseTableSize * sizeof(int);
          EEPROM.get(addr, numberOfInjections);

          doseTable[doseTableSize++] = {lower, upper, dose, numberOfInjections};
          Serial.print("wkjn bdld");
          Serial.print(numberOfInjections);
        }
   
      }
    }
    pumpconfigFile.close();

    // Initialize data file
    File pumpdataFile = SD.open(PUMP_DATAFILE, FILE_APPEND);
    pumpdataFile.print("ID; INJECTION STATUS; PUMP COUNT\n");
    pumpdataFile.close();
    Serial.print("SetOffset:");
    Serial.println(Setoffset);
    Serial.print("Offset");
    Serial.println(water_height_offset);

    if (Setoffset){
      Serial.print("SetOffset:");
      Serial.println(Setoffset);
      water_height_offset = measuredWaterheight; 
      EEPROM.put(0, water_height_offset);  // Save at address 0
      EEPROM.commit();  // Important! Actually write to flash
      Serial.print("Offset saved! EEPROM stored value : ");
      EEPROM.get(0, water_height_offset);
      Serial.println(water_height_offset);
    }
    else if (!Setoffset){
      Serial.print("SetOffset:");
      Serial.println(Setoffset);
      EEPROM.get(0, water_height_offset);
      Serial.print("Offset read from EEPROM! value read : ");
      Serial.println(water_height_offset);
    }



    // Send a starting command
    if (Filltubes){
      sendCommand("D,150");
    }
    

    // Determine how often the device will boot and run (e.g. hourly)
    data.boot_step = (int)((30.0 / (float)time_step) + 0.5);
    if (data.boot_step == 0) data.boot_step = 1;
  }
}

// Log pump status in SD card
void Pump::save_in_SD(int &bootCount) {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } 
  else {
    String pumpdata_message = String(bootCount) + ";" + pump.latestErrorMessage + ";" + String(pumpActivationCount) + ";";
    File pumpdataFile = SD.open(PUMP_DATAFILE, FILE_APPEND);
    pumpdataFile.println(pumpdata_message);
    pumpdataFile.close();
  }
}

// Main function to manage dosing logic
void Pump::handleInjections2(int &bootCount, int &time_step) {
  delay(1000);

  // Shift height readings to make space for the latest one
  for (int i = 0; i < 4; i++) {
    data.heights[i] = data.heights[i + 1];
  }
  data.heights[4] = measuredWaterheight - water_height_offset;
  Serial.print("data.heights[4] and water height offset : ");
  Serial.print(data.heights[4]);
  Serial.print(", ");
  Serial.println(water_height_offset); 
  // Calculate average height
  float heightsMean = 0.0;
  for (int i = 0; i < 5; ++i) {
    heightsMean += data.heights[i] / 5;
  }

  // If allowed to inject
  if (Inject) {
    for (int i = 0; i < doseTableSize; i++) {
      // Check which range the water height falls into
      if (heightsMean >= doseTable[i].lower && heightsMean < doseTable[i].upper && doseTable[i].numberOfInjections > 0) {
        String pumpCommand = "D," + String(doseTable[i].dose);
        Serial.println(i);
        Serial.println(doseTable[i].dose);
        sendCommand(pumpCommand);  // Inject
        Inject = false;            // Disable further injections until cooldown
        Serial.println(pumpCommand);
        doseTable[i].numberOfInjections--;
        int eeprom_addr = EEPROM_INJECTION_START + i * sizeof(int);
        EEPROM.put(eeprom_addr, doseTable[i].numberOfInjections);
        EEPROM.commit();

        break;
      }
    }
  } else {
    // Wait until cooldown is complete before injecting again
    counter++;
    if (counter >= (int)(10800 / time_step + 0.5)) {  // 3 hours
      Inject = true;
      counter = 0;
    }
  }

  // Log result
  save_in_SD(bootCount);

  // Reset error message for next cycle
  Serial.print(latestErrorMessage);
  latestErrorMessage = "*OFF";
  Serial.print(latestErrorMessage);
  delay(1000);
  return;
}

int Pump::countLines(File &file) {
  int lines = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (line.length() > 0) {
      lines++;
    }
  }
  file.seek(0);  // Reset file pointer for re-use
  return lines;
}