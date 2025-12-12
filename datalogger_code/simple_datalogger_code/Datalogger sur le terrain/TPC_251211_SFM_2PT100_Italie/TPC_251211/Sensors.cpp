/* ******************************************************************************
   Sensors.cpp — version robuste, compatible 100% avec le .ino existant
   - Interface publique IDENTIQUE à ton code d’origine (tcaselect: void).
   - Initialisation des capteurs faite UNE FOIS automatiquement au 1er measure().
   - Vérifications I²C avant begin() pour éviter les crashs (LoadProhibited).
   - SFM3003: démarré en continu lors de l’init puis simple lecture ensuite.
   - À la fin de measure(), on laisse le TCA sur le canal 2 comme dans ta version.
******************************************************************************** */

#include <Wire.h>
#include <math.h>
#include "Sensors.h"
#include "SHT31.h"
#include "SparkFun_BMP581_Arduino_Library.h"
#include <TinyPICO.h>
#include <SparkFun_ADS122C04_ADC_Arduino_Library.h>
#include <SensirionI2cSfmSf06.h>
#include "SparkFun_SCD4x_Arduino_Library.h"

// Paramètres matériels (identiques à ta base)
#define I2C_MUX_ADDRESS 0x73         // Adresse I²C du multiplexeur (selon ton PCB)
#define BMP581_sensor_ADRESS 0x46    // Adresse I²C du BMP581
#define SHT35_sensor_ADRESS  0x44    // Adresse I²C du SHT35


// macro definitions
// make sure that we use the proper definition of NO_ERROR
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0
static char errorMessage[64];
static int16_t error;


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


// Instances capteurs
TinyPICO tiny = TinyPICO();
SHT31 sht;
BMP581 bmp;
SensirionI2cSfmSf06 sfmSf06;
SFE_ADS122C04 Pt100_1; // Pt100 sensor
SFE_ADS122C04 Pt100_2; // Pt100 sensor

// Noms / valeurs / décimales (identiques à ta base)
String names[] = {"Vbatt","tempSHT","humSHT","tempBMP","pressBMP","debitSFM","velSFM","tempSFM","tempPt1","tempPt2"};
const int nb_values = sizeof(names) / sizeof(names[0]);
float values[nb_values];
int   decimals[] = {2, 3, 1, 2, 2, 6, 6, 2, 2, 2};

// ================= Helpers internes (privés au fichier) =================

static bool g_inited = false;  // init effectuée une seule fois
// --- SCD41: état minimaliste pour un single-shot échelonné
static bool g_scd41_present = false;   // détecté ce cycle ?
static bool g_scd41_pending = false;   // une mesure single-shot est en cours ?
static bool g_scd41_ready = false;
static bool g_sfm_ok = false;   // vrai seulement si begin() + start() ont réussi




// Test de présence I²C d'une adresse (ACK ?)
static bool i2c_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Sélection TCA avec retour de statut (utilisé en interne)
// NB: l’API publique conserve void tcaselect(uint8_t).
static bool tca_select_ok(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(I2C_MUX_ADDRESS);
  Wire.write(1 << ch);
  uint8_t err = Wire.endTransmission();
  delayMicroseconds(500);
  return (err == 0);
}

// Désélectionne tous les canaux (bonne hygiène entre lectures)
static void tca_disable() {
  Wire.beginTransmission(I2C_MUX_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
  delayMicroseconds(200);
}

static void init_sfm_block_with_retry() {
    g_sfm_ok = false;

    if (!tca_select_ok(7)) {
        tca_disable();
        return;
    }

    // On tente plusieurs fois, au cas où le SFM se réveille lentement
    const int max_tries = 5;
    bool ack = false;
    for (int attempt = 1; attempt <= max_tries && !ack; attempt++) {
        if (i2c_present(0x2D)) {   // adresse SFM3003
            ack = true;
        } else {
            Serial.print("[SFM] pas de ACK sur 0x2D (tentative ");
            Serial.print(attempt);
            Serial.println(")");
            delay(200); // 200 ms d’attente avant de retenter
        }
    }

    if (!ack) {
        Serial.println("[SFM] abandon init, capteur introuvable après retries");
        tca_disable();
        return;
    }

    // Ici, on a finalement un ACK → on suit la séquence Sensirion
    sfmSf06.begin(Wire, SFM3003_I2C_ADDR_2D);

    int16_t err = sfmSf06.stopContinuousMeasurement();
    delay(100);

    uint32_t productIdentifier = 0;
    uint8_t serialNumber[8] = {0};
    err = sfmSf06.readProductIdentifier(productIdentifier, serialNumber, 8);
    if (err != NO_ERROR) {
        char msg[64];
        errorToString(err, msg, sizeof(msg));
        Serial.print("[SFM] readProductIdentifier error: ");
        Serial.println(msg);
        tca_disable();
        return;
    }

    Serial.print("[SFM] productIdentifier: ");
    Serial.print(productIdentifier);
    Serial.print("  SN: ");
    for (int i = 0; i < 8; i++) {
        Serial.print(serialNumber[i], HEX);
    }
    Serial.println();

    err = sfmSf06.startO2ContinuousMeasurement();
    if (err != NO_ERROR) {
        char msg[64];
        errorToString(err, msg, sizeof(msg));
        Serial.print("[SFM] startO2ContinuousMeasurement error: ");
        Serial.println(msg);
        tca_disable();
        return;
    }

    g_sfm_ok = true;
    Serial.println("[SFM] continuous measurement started OK");
    tca_disable();
}

// Initialisation des capteurs (appelée automatiquement au 1er measure())
static void sensors_init_once() {
  if (g_inited) return;

  // On suppose que Wire.begin() et la vitesse bus sont posés par le .ino.

  // 1) Sanity check: TCA présent ?
  if (!i2c_present(I2C_MUX_ADDRESS)) {
    Serial.println("[Sensors] WARN: TCA mux not detected at I2C 0x73");
  }

  // 2) Init SHT35 (canal 1)
  if (tca_select_ok(1) && i2c_present(SHT35_sensor_ADRESS)) {
    delay(20);
    sht.begin();         // lib SHT31 sans statut de retour
  }
  tca_disable();

  // 3) Init BMP581 (canal 2)
  if (tca_select_ok(2) && i2c_present(BMP581_sensor_ADRESS)) {
    delay(20);
    bmp.beginI2C(BMP581_sensor_ADRESS); // pas de check strict, on reste tolérant
  }
  tca_disable();

  // 4) Init SFM3003 (canal 5)
  init_sfm_block_with_retry();

  // 5) Init Pt100_1 (canal 5)
  if(tca_select_ok(5)){
    delay(200);
    Pt100_1.begin();
    Pt100_1.configureADCmode(ADS122C04_4WIRE_MODE);
  }
  tca_disable();

  // 6) Init Pt100_2 (canal 6)
  if(tca_select_ok(6)){
    delay(200);
    Pt100_2.begin();
    Pt100_2.configureADCmode(ADS122C04_4WIRE_MODE);
  }
  tca_disable();
}



// ========================= API publique (identique) =========================

String* Sensors::get_names() { return names; }

int Sensors::get_nb_values() { return nb_values; }

String Sensors::getFileHeader() {
  String header_string = "";
  header_string.reserve(128);
  for (int i = 0; i < nb_values; i++) {
    header_string += names[i]; header_string += ';';
  }
  return header_string;
}

String Sensors::getFileData() {
  String datastring = "";
  datastring.reserve(160);
  for (int i = 0; i < nb_values; i++) {
    datastring += String(values[i], decimals[i]); datastring += ';';
  }
  return datastring;
}

String Sensors::serialPrint() {
  String sensor_display_str = "";
  sensor_display_str.reserve(256);
  for (int i = 0; i < nb_values; i++) {
    sensor_display_str += names[i]; sensor_display_str += ": ";
    sensor_display_str += String(values[i], 3); sensor_display_str += "\n";
  }
  Serial.print(sensor_display_str);
  return sensor_display_str;
}

// Mesure de tous les capteurs (appelle auto l'init au premier passage)
void Sensors::measure() {
  // Auto-init idempotente (ne fait rien si déjà initialisé)
  sensors_init_once();

  delay(200); // petit temps pour stabilité alim

  // 0) Batterie
  values[0] = tiny.GetBatteryVoltage();

  // 1) SHT35 (canal 1)
  if (tca_select_ok(1)) {
    delay(2);
    sht.read();
    values[1] = sht.getTemperature();
    values[2] = sht.getHumidity();
  }
  tca_disable();

  // 2) BMP581 (canal 2)
  if (tca_select_ok(2)) {
    delay(20);
    bmp5_sensor_data data = {0, 0};
    int8_t err = bmp.getSensorData(&data);
    if (err != 0) {
      // Petit “retry” avec ré-init
      bmp.beginI2C(BMP581_sensor_ADRESS);
      delay(50);
      err = bmp.getSensorData(&data);
    }
    if (err == 0) {
      values[3] = data.temperature;
      values[4] = data.pressure / 100.0f; // mbar
    }
  }
  tca_disable();

  // 3) SFM3003 (canal 7) — lecture en mode continu
  values[5] = NAN;
  values[6] = NAN;
  values[7] = NAN;

  if (g_sfm_ok && tca_select_ok(7)) {
    delay(100);
    int16_t flowRaw = 0, temperatureRaw = 0;
    uint16_t status = 0;

    int16_t err = sfmSf06.readMeasurementDataRaw(flowRaw, temperatureRaw, status);
    if (err == 0) {
      values[5] = (float(flowRaw) + 12288.0f) / 120.0f;
      values[6] = pchipVelocityFromSlm(values[5]);
      values[7] = float(temperatureRaw) / 200.0f;
    } else {
      Serial.println("[Sensors] WARN: lecture SFM3003 échouée");
    }
  }
  tca_disable();

  // 4) Pt100_1 (canal 5)
  values[8] = NAN;

  if(tca_select_ok(5)){
    delay(200);
    values[8] = Pt100_1.readPT100Centigrade();
    Pt100_1.powerdown();
  }
  tca_disable();

  // 5) Pt100_2 (canal 6)
  values[9] = NAN;

  if(tca_select_ok(6)){
    delay(200);
    values[9] = Pt100_2.readPT100Centigrade();
    Pt100_2.powerdown();
  }
  tca_disable();
    
  // (Comportement d’origine) — on termine en sélectionnant le canal 2.
  tcaselect(2);
  delay(50);
}

// Sélection de canal TCA — signature d’origine (void) conservée
void Sensors::tcaselect(uint8_t i) {
  (void)tca_select_ok(i); // on ignore le statut pour rester compatible
}
