/**
 **************************************************
 *
 * @file        singleSensor.ino
 * @brief       See how to init one sensor and make a simple reading of PPM of gas.
 *
 *              To successfully run the sketch:
 *              - Connect the breakout to your Dasduino board via easyC
 *              - Connect LMPEN pin to GND or a GPIO pin so the breakout can be configured
 *              - Run the sketch and open serial monitor at 115200 baud!
 *
 *              Electrochemical Gas Sensor Breakout: solde.red/333218
 *              Dasduino Core: www.solde.red/333037
 *              Dasduino Connect: www.solde.red/333034
 *              Dasduino ConnectPlus: www.solde.red/333033
 *
 * @authors     Robert @ soldered.com
 ***************************************************/

// Include the required library
#include "Electrochemical-Gas-Sensor-SOLDERED.h"
#include <U8x8lib.h>

#define MOSFET_SENSORS_PIN 14

// SGX-OX-ROHS-LP - Oxygen sensor
const sensorType SENSOR_OX = {
  5.0F,                     // nanoAmperesPerPPM (approx, based on 0.1 mA in air -> ~4760 nA per %vol; ≈ 4.76 nA/ppm)
  0.0,                      // internalZeroCalibration (oxygen has small offset; can be tuned after calibration)
  ADS_GAIN_2_048V,          // adsGain (keeps sensor output within ADC range)
  TIA_GAIN_120_KOHM,        // TIA_GAIN_IN_KOHMS (balances sensitivity vs noise)
  RLOAD_10_OHM,             // RLOAD (common for SGX sensors)
  REF_EXTERNAL,             // REF_SOURCE (external 2.5V reference)
  INTERNAL_ZERO_67_PERCENT, // INTERNAL_ZERO (best bias point for -600 mV)
  BIAS_SIGN_NEGATIVE,       // BIAS_SIGN (datasheet specifies -600 mV)
  BIAS_24_PERCENT,          // BIAS (≈ -600 mV with 2.5V ref → 0.24 * 2.5V ≈ 0.6 V)
  FET_SHORT_DISABLED,       // FET_SHORT
  OP_MODE_3LEAD_AMP_CELL,   // OP_MODE
};

// Configurations for each of the sensor types are in sensorConfigData.h in the library's 'src' folder
// Create the sensor object with the according type
ElectrochemicalGasSensor sensor(SENSOR_CO);
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE); //initialise something for the OLED display

// If you are using a custom I2C address (for multiple sensors) use:
// ElectrochemicalGasSensor sensor(SENSOR_SO2, 0x4B, 5);
// For more details on this, see customAddress.ino

void power_external_device(){
  //Turn on Pin to control the gate of MOSFET for all the sensors, display and microSD card
  pinMode(MOSFET_SENSORS_PIN, OUTPUT);
  digitalWrite(MOSFET_SENSORS_PIN, HIGH);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(0x73);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(115200); // For debugging

  power_external_device();

  delay(30);
  tcaselect(7);
  delay(30);

  u8x8.begin();
  u8x8.setBusClock(50000); //Give the I2C frequency information to the display
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_r); //set the font
  u8x8.setCursor(0, 0);

  // Init the breakout
    if (!sensor.begin())
    {
        // Can't init? Notify the user and go to infinite loop
        Serial.println("ERROR: Can't init the sensor! Check connections!");
        u8x8.println("Init fail");
        while (true)
            delay(100);
    }

    Serial.println("Sensor initialized successfully!");
    u8x8.println("Init done");
    delay(2000);
    u8x8.clear();
}

void loop()
{

  delay(30);
  tcaselect(7);
  delay(30);

  double reading = sensor.getPPM();

  // If PPB is more relevant for your sensor, you can use:
  // double reading = sensor.getPPB();

  // Print the reading with 5 digits of precision
  Serial.print("Sensor reading: ");
  Serial.print(reading, 5);
  Serial.println(" PPM");

  // Wait a bit before reading again
  delay(2500);
}