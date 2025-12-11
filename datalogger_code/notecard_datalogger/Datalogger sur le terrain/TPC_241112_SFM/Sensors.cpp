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

#include <Wire.h>
#include <math.h>
#include <TinyPICO.h>
#include <stdlib.h>   // dtostrf on ESP32 is in stdlib/Arduino
#include "Sensors.h"
#include "SHT31.h"
#include "SparkFun_BMP581_Arduino_Library.h"
#include <SensirionI2cSfmSf06.h>

// ====== PCHIP METAS (0°) : v = a + b*(q-q_i) + c*(q-q_i)^2 + d*(q-q_i)^3 ======
// Nœuds de débit (SLM) – incluant l'ancre 0.0
static const float q_breaks[] = {
    0.000000000, 0.192000000, 0.609000000, 1.540000000, 3.330000000, 5.940000000, 8.810000000, 11.660000000,
    15.940000000, 20.440000000, 39.280000000, 73.840000000
};

// Coefficients PCHIP par intervalle [q_i, q_{i+1}] (METAS 0° + ancre 0,0)
// v(q) = a[i] + b[i]*(q-q_i) + c[i]*(q-q_i)^2 + d[i]*(q-q_i)^3
static const float pchip_a[] = {
    0.000000000, 0.154000000, 0.301000000, 0.498000000, 0.799000000, 1.196000000, 1.602000000, 1.993000000,
    2.497000000, 2.991000000, 5.014000000
};
static const float pchip_b[] = {
    0.943818221, 0.514445975, 0.273130461, 0.189676840, 0.160228520, 0.146676497, 0.139292988, 0.127383367,
    0.113660776, 0.108810859, 0.096584972
};
static const float pchip_c[] = {
    0.021706158, -0.586255287, -0.108632088, -0.019616159, -0.004142413, -0.002876564, 0.001968282, -0.003541234,
    -0.001510906, 0.000420757, -0.000210137
};
static const float pchip_d[] = {
    -3.957857809, 0.474673299, 0.045694767, 0.004242224, 0.000394953, 0.000369393, -0.000949167, 0.000301889,
    0.000144004, -0.000026370, -0.000002737
};


// Évalue PCHIP pour un débit q_SLM (SLM). Symétrie: v(-q) = -v(q).
// Comportement hors-plage : on sature au dernier point (pas d'extrapolation).
static float pchipVelocityFromSlm(float q_SLM) {
  float sign = 1.0f;
  if (q_SLM < 0.0f) { sign = -1.0f; q_SLM = -q_SLM; }

  const int N = sizeof(q_breaks) / sizeof(q_breaks[0]); // 12 nœuds => 11 intervalles

  if (q_SLM <= q_breaks[0]) return 0.0f; // garantit v(0)=0

  // Hors plage haute : on renvoie la valeur au dernier nœud (saturation)
  if (q_SLM >= q_breaks[N - 1]) {
    const int iLast = N - 2; // intervalle [N-2, N-1]
    const float dx = q_breaks[N - 1] - q_breaks[N - 2];
    const float v_end = pchip_a[iLast]
                      + pchip_b[iLast] * dx
                      + pchip_c[iLast] * dx * dx
                      + pchip_d[iLast] * dx * dx * dx;
    return sign * v_end;
  }

  // Recherche binaire de l'intervalle i tel que q_breaks[i] <= q < q_breaks[i+1]
  int lo = 0, hi = N - 1;
  while ((hi - lo) > 1) {
    int mid = (lo + hi) >> 1;
    if (q_SLM < q_breaks[mid]) hi = mid; else lo = mid;
  }
  const int i = lo;
  const float dx = q_SLM - q_breaks[i];
  const float v = pchip_a[i]
                + pchip_b[i] * dx
                + pchip_c[i] * dx * dx
                + pchip_d[i] * dx * dx * dx;
  return sign * v;
}
// ====== fin PCHIP ======
// ===== PCB parameters =====
static const uint8_t I2C_MUX_ADDRESS       = 0x73;
static const uint8_t BMP581_SENSOR_ADDRESS = 0x46;
static const uint8_t SHT35_SENSOR_ADDRESS  = 0x44; // (au cas où)


// ===== Instances =====
static TinyPICO tiny;
static SHT31    sht;
static BMP581   bmp;
static SensirionI2cSfmSf06 sfmSf06;


// ===== Names & values (same order everywhere) =====
static const char* const NAMES[] = { "Vbatt", "tempSHT", "humSHT", "tempBMP", "pressBMP","debitSFM","VelSFM","T_SFM" };
static const int   NB_VALUES = sizeof(NAMES) / sizeof(NAMES[0]);
static float       VALUES[NB_VALUES];         // valeurs courantes
static uint8_t     HEALTH_MASK = 0;           // bit0=SHT ok, bit1=BMP ok, etc.
// (optionnel, mais plus lisible)
static const uint8_t HEALTH_SHT   = 0x01;
static const uint8_t HEALTH_BMP   = 0x02;
static const uint8_t HEALTH_SFM = 0x04;


// ===== Per-sensor decimals (modifiable) =====
// CSV: ce qui va dans le fichier ; Serial: ce qui s'affiche sur Serial/OLED
static uint8_t CSV_DECIMALS[NB_VALUES]    = { 2, 3, 2, 3, 1, 7, 7, 3 };
static uint8_t SERIAL_DECIMALS[NB_VALUES] = { 1, 1, 1, 1, 0, 3, 3, 1 };

// ===== Simple calibration (offset + scale) =====
static float CAL_OFFSET[NB_VALUES] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static float CAL_SCALE [NB_VALUES] = { 1, 1, 1, 1, 1, 1, 1, 1 };

// ===== Helpers =====
static inline bool  isValidIndex(uint8_t i) { return i < NB_VALUES; }
static inline float asNaN() { return NAN; }

// ===== Public API =====

const char* const* Sensors::get_names() { return NAMES; }
int Sensors::get_nb_values() { return NB_VALUES; }
const float* Sensors::get_values() const { return VALUES; }
uint8_t Sensors::getHealthMask() const { return HEALTH_MASK; }

// "Vbatt;tempSHT;...;"
void Sensors::printFileHeader(Print& out) {
  for (int i = 0; i < NB_VALUES; ++i) {
    out.print(NAMES[i]);
    out.print(';');
  }
}

// "v1;v2;...;"
void Sensors::printFileData(Print& out) {
  char buf[24];
  for (int i = 0; i < NB_VALUES; ++i) {
    if (isnan(VALUES[i])) {
      out.print(F("nan;"));
    } else {
      dtostrf((double)VALUES[i], 0, CSV_DECIMALS[i], buf);
      out.print(buf);
      out.print(';');
    }
  }
}

// Serial:
// Vbatt: 4.2
// tempSHT: 23.1
void Sensors::printSerial(Print& out) {
  char buf[24];
  for (int i = 0; i < NB_VALUES; ++i) {
    out.print(NAMES[i]);
    out.print(F(": "));
    if (isnan(VALUES[i])) {
      out.println(F("nan"));
    } else {
      dtostrf((double)VALUES[i], 0, SERIAL_DECIMALS[i], buf);
      out.println(buf);
    }
  }
}

// Même contenu que printSerial(), mais dans 'buf' (pour ton OLED)
void Sensors::formatSerialToBuffer(char* buf, size_t bufsize) const {
  if (!buf || bufsize == 0) return;
  size_t used = 0;
  auto append = [&](const char* s){
    if (!s) return;
    size_t n = strnlen(s, 1024);
    size_t room = (used < bufsize) ? (bufsize - used - 1) : 0;
    size_t copy = (n < room) ? n : room;
    if (copy > 0) { memcpy(buf + used, s, copy); used += copy; }
    buf[used] = '\0';
  };
  char num[24];
  for (int i = 0; i < NB_VALUES; ++i) {
    append(NAMES[i]); append(": ");
    if (isnan(VALUES[i])) {
      append("nan");
    } else {
      dtostrf((double)VALUES[i], 0, SERIAL_DECIMALS[i], num);
      append(num);
    }
    append("\n");
  }
}

// Mesure tous les capteurs et remplit VALUES[]
void Sensors::measure() {
  delay(5);               // s'assurer que l'alimentation est stable
  HEALTH_MASK = 0;

  // --- Batterie ---
  VALUES[0] = tiny.GetBatteryVoltage(); // Vbatt

  // --- SHT35 (TCA1) ---
  tcaselect(1);
  delay(3);               // temps de propagation MUX
  sht.begin();            // init (adresse interne lib si besoin)
  bool sht_ok = sht.read();
  if (sht_ok) {
    VALUES[1] = sht.getTemperature();   // tempSHT
    VALUES[2] = sht.getHumidity();      // humSHT
    HEALTH_MASK |= HEALTH_SHT;
  } else {
    VALUES[1] = asNaN();
    VALUES[2] = asNaN();
  }

// --- BMP581 (TCA2) ---
tcaselect(2);
delay(10);  // un peu plus de temps après le switch + power-on capteur

// beginI2C retourne un code (0 = OK)
int8_t rc = bmp.beginI2C(BMP581_SENSOR_ADDRESS);  // 0x46
delay(5);

// petit retry si échec
if (rc != 0) {
  Serial.printf("BMP581 beginI2C(0x%02X) rc=%d, retry...\n", BMP581_SENSOR_ADDRESS, rc);
  delay(10);
  tcaselect(2);
  rc = bmp.beginI2C(BMP581_SENSOR_ADDRESS);
}

// fallback: certaines cartes câblent SDO différemment -> 0x47
if (rc != 0) {
  const uint8_t ALT_ADDR = 0x47;
  Serial.printf("BMP581 retry with ALT_ADDR 0x%02X...\n", ALT_ADDR);
  delay(5);
  tcaselect(2);
  rc = bmp.beginI2C(ALT_ADDR);
}

if (rc == 0) {
  bmp5_sensor_data data = {0, 0};
  int8_t err = bmp.getSensorData(&data);
  // Serial.printf("BMP581 getSensorData err=%d\n", err);
  if (err == 0) {
    VALUES[3] = data.temperature;              // tempBMP
    VALUES[4] = (float)data.pressure * 0.01f;  // mbar
    HEALTH_MASK |= HEALTH_BMP;
  } else {
    Serial.println("BMP581 read failed -> NaN");
    VALUES[3] = asNaN();
    VALUES[4] = asNaN();
  }
} else {
  Serial.println("BMP581 init failed -> NaN");
  VALUES[3] = asNaN();
  VALUES[4] = asNaN();
}


    // --- SFM3003-300-CE (TCA7) ---
  delay(50);
  tcaselect(7);
  delay(500);  // petite marge après le MUX

  // begin() ne renvoie rien (void)
  sfmSf06.begin(Wire, SFM3003_I2C_ADDR_2D);
  delay(500);

  int16_t  flowRaw        = 0;
  int16_t  temperatureRaw = 0;
  uint16_t status         = 0;

  float debitSlm    = asNaN();
  float velMs       = asNaN();
  float t_sfm_degC  = asNaN();

  // La LIB renvoie 0 si OK
  delay(200);
  uint16_t sfmErr = sfmSf06.readMeasurementDataRaw(flowRaw, temperatureRaw, status);
  if (sfmErr != 0) {
    Serial.print(F("SFM3003 read failed, err="));
    Serial.print(sfmErr);
    Serial.print(F(", status=0x"));
    Serial.println(status, HEX);
  } else {
    // Conversion brute -> SLM (ta formule)
    debitSlm   = (float(flowRaw) + 12288.0f) / 120.0f;
    // PCHIP METAS -> vitesse
    velMs      = pchipVelocityFromSlm(debitSlm);
    // Température capteur
    t_sfm_degC = (float)temperatureRaw / 200.0f;

    // Sanity check optionnel
    bool plausible =
      (debitSlm > -100.0f && debitSlm < 100.0f) &&
      (t_sfm_degC > -40.0f && t_sfm_degC < 100.0f);

    if (!plausible) {
      Serial.println(F("SFM3003: mesures hors plage -> NaN"));
      debitSlm   = asNaN();
      velMs      = asNaN();
      t_sfm_degC = asNaN();
    } else {
      HEALTH_MASK |= HEALTH_SFM;  // capteur OK
    }
  }

  // Écriture dans VALUES[] (alignée avec NAMES[])
  VALUES[5] = debitSlm;     // "debitSFM"
  VALUES[6] = velMs;        // "VelSFM"
  VALUES[7] = t_sfm_degC;   // "T_SFM"

  delay(50);
  tcaselect(2);
  delay(50);


  // --- Calibration (offset/scale) ---
  for (int i = 0; i < NB_VALUES; ++i) {
    if (!isnan(VALUES[i])) {
      VALUES[i] = CAL_OFFSET[i] + CAL_SCALE[i] * VALUES[i];
    }
  }
}

// Sélection de canal sur TCA9548A (idempotente)
void Sensors::tcaselect(uint8_t channel) {
  static uint8_t last = 0xFF;
  if (channel > 7 || channel == last) return;
  Wire.beginTransmission(I2C_MUX_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
  last = channel;
  delayMicroseconds(50);
}

// ===== Setters décimales & calibration =====

void Sensors::setCsvDecimals(uint8_t idx, uint8_t d) {
  if (isValidIndex(idx)) CSV_DECIMALS[idx] = d;
}
void Sensors::setSerialDecimals(uint8_t idx, uint8_t d) {
  if (isValidIndex(idx)) SERIAL_DECIMALS[idx] = d;
}
void Sensors::setCsvDecimalsAll(const uint8_t* a, int len) {
  for (int i = 0; i < len && i < NB_VALUES; ++i) CSV_DECIMALS[i] = a[i];
}
void Sensors::setSerialDecimalsAll(const uint8_t* a, int len) {
  for (int i = 0; i < len && i < NB_VALUES; ++i) SERIAL_DECIMALS[i] = a[i];
}
void Sensors::setCalibration(uint8_t idx, float off, float sc) {
  if (isValidIndex(idx)) { CAL_OFFSET[idx] = off; CAL_SCALE[idx] = sc; }
}
void Sensors::setCalibrationAll(const float* off, const float* sc, int len) {
  for (int i = 0; i < len && i < NB_VALUES; ++i) { CAL_OFFSET[i] = off[i]; CAL_SCALE[i] = sc[i]; }
}







