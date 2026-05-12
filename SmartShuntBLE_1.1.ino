#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <INA226_WE.h> 
#include <Preferences.h>

// ESP32-C6
#define I2C_SDA 20
#define I2C_SCL 19
#define I2C_ADDRESS 0x40

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_DATA_UUID         "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_CONFIG_UUID       "82250625-fba7-47bc-8034-745ad5a95a40"

INA226_WE ina = INA226_WE(I2C_ADDRESS);
Preferences prefs;

double actual_capacity_ah = 8.0; 
const float SHUNT_RESISTANCE = 0.00025; 
float v_full_charge = 25.2; 

double current_capacity_ah; 
double charged_prev_ah = 0;    
double discharged_prev_ah = 0; 
double current_session_delta = 0; 

unsigned long session_start_ms = 0;
double ema_current = 0;
double dynamic_offset = 0.027; 
bool learning_mode = false; 
bool ina_found = false; 

unsigned long last_micros = 0;
double last_current_a = 0;
const double THRESHOLD_A = 0.015; // 15mA

BLEServer* pServer = NULL;
BLECharacteristic* pDataChar = NULL;

String formatTime(unsigned long sec) {
    int h = sec / 3600;
    int m = (sec % 3600) / 60;
    int s = sec % 60;
    char buf[12];
    sprintf(buf, "%02d:%02d:%02d", h, m, s);
    return String(buf);
}

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("[BLE CMD] "); Serial.println(value);
            
            // 1. Synchronization 100%
            if (value == "sync:1") {
                current_capacity_ah = actual_capacity_ah;
                current_session_delta = 0;
                learning_mode = false;
                prefs.putFloat("rem_ah", (float)current_capacity_ah);
            }
            // 2. Synchronization 0% (Calibration)
            else if (value == "sync:0") {
                current_capacity_ah = 0;
                current_session_delta = 0;
                learning_mode = true;
            }
            // 3. Setting capacity (e.g., cap:60.0)
            else if (value.startsWith("cap:")) {
                actual_capacity_ah = value.substring(4).toDouble();
                prefs.putFloat("act_cap", (float)actual_capacity_ah);
            }
          // 4. Setting voltage to 100% (e.g., volt:14.2)
            else if (value.startsWith("volt:")) {
                v_full_charge = value.substring(5).toFloat();
                prefs.putFloat("v_full", v_full_charge);
                Serial.printf("Target voltage set to: %.2fV\n", v_full_charge);
            }
        }
    }
};

double getPreciseCurrent() {
    if (!ina_found) return 0.0;
    double shunt_v_mv = ina.getShuntVoltage_mV();
    double current = (shunt_v_mv / (SHUNT_RESISTANCE * 1000.0)) + dynamic_offset;
    ema_current = (current * 0.1) + (ema_current * 0.9);
    if (abs(ema_current) < 0.008) { 
        dynamic_offset -= (ema_current * 0.0001);
        return 0.0;
    }
    return ema_current;
}

void setup() {
    Serial.begin(115200);
    delay(5000); 
    Serial.println("\n--- Starting SmartShunt BLE (C6-Pro) ---");

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(50); 

    Wire.beginTransmission(I2C_ADDRESS);
    if (Wire.endTransmission() == 0) {
        if(ina.init()) {
            Serial.println("[OK] INA226 Ready.");
            ina_found = true;
            ina.setResistorRange(SHUNT_RESISTANCE, 300.0);
            ina.setAverage(INA226_AVERAGE_512);
        }
    } else {
        Serial.println("[ERROR] INA226 NOT FOUND!");
    }

    prefs.begin("batt_pro", false);
    actual_capacity_ah = prefs.getFloat("act_cap", 8.0);
    v_full_charge = prefs.getFloat("v_full", 25.2);
    current_capacity_ah = prefs.getFloat("rem_ah", actual_capacity_ah);
    charged_prev_ah = prefs.getFloat("chg_p", 0.0);
    discharged_prev_ah = prefs.getFloat("dis_p", 0.0);
    dynamic_offset = prefs.getFloat("off_val", 0.027);

    // Bluetooth Setup
    BLEDevice::init("SmartShunt-C6");
    pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pDataChar = pService->createCharacteristic(
                    CHAR_DATA_UUID,
                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                );
    BLECharacteristic *pConfigChar = pService->createCharacteristic(
                    CHAR_CONFIG_UUID,
                    BLECharacteristic::PROPERTY_WRITE
                );
    pConfigChar->setCallbacks(new MyCallbacks());
    pService->start();
    BLEDevice::getAdvertising()->start();

    session_start_ms = millis();
    last_micros = micros();
}

void loop() {
    double current_a = getPreciseCurrent();
    unsigned long now_micros = micros();
    double dt = (double)(now_micros - last_micros) / 3600000000.0; 
    last_micros = now_micros;

    if (ina_found) {
        double delta_ah = ((last_current_a + current_a) / 2.0) * dt;
        current_capacity_ah += delta_ah;
        current_session_delta += delta_ah;
    }

    
    // Charging complete
    if (last_current_a > THRESHOLD_A && current_a <= THRESHOLD_A) {
        if (current_session_delta > 0.001) {
            charged_prev_ah = current_session_delta;
            prefs.putFloat("chg_p", (float)charged_prev_ah);
            current_session_delta = 0;
            session_start_ms = millis();
        }
    }
   // Completion of the discharge
    else if (last_current_a < -THRESHOLD_A && current_a >= -THRESHOLD_A) {
        if (abs(current_session_delta) > 0.001) {
            discharged_prev_ah = abs(current_session_delta);
            prefs.putFloat("dis_p", (float)discharged_prev_ah);
            current_session_delta = 0;
            session_start_ms = millis();
        }
    }
   // Woke up from Idle
    else if (abs(last_current_a) <= THRESHOLD_A && abs(current_a) > THRESHOLD_A) {
        current_session_delta = 0;
        session_start_ms = millis();
    }

    float v = ina_found ? ina.getBusVoltage_V() : 0.0;
    
   // Auto-sync 100%
    if (ina_found && v >= (v_full_charge - 0.1) && current_a > 0.005 && current_a < 0.12) {
        if (learning_mode && current_session_delta > 0.5) {
            actual_capacity_ah = current_session_delta; 
            prefs.putFloat("act_cap", (float)actual_capacity_ah);
            learning_mode = false; 
        }
        current_capacity_ah = actual_capacity_ah;
    }

    if (current_capacity_ah > actual_capacity_ah) current_capacity_ah = actual_capacity_ah;
    if (current_capacity_ah < 0) current_capacity_ah = 0;
    
    last_current_a = current_a;

  // BLE Update (Once per second)
    static unsigned long lastBle = 0;
    if (millis() - lastBle > 1000) {
        lastBle = millis();
        if (pServer->getConnectedCount() > 0) {
            char buf[250];
            double soc = (actual_capacity_ah > 0) ? (current_capacity_ah / actual_capacity_ah) * 100.0 : 0.0;
            unsigned long dur = (millis() - session_start_ms) / 1000;
            double since_full = actual_capacity_ah - current_capacity_ah;
            
            // Extended format:
            // V | mA | Ah | % | Mode | SessionDelta | HistoryLC | HistoryLD | SinceFull | Time
            snprintf(buf, sizeof(buf), "%.2fV|%.0fmA|%.3fAh|%.1f%%|%s|S:%.4f|LC:%.3f|LD:%.3f|SF:%.3f|T:%s", 
                     v, 
                     current_a * 1000.0, 
                     current_capacity_ah, 
                     soc, 
                     ina_found ? (learning_mode ? "LEARN" : "OK") : "ERR_I2C",
                     current_session_delta, 
                     charged_prev_ah, 
                     discharged_prev_ah,
                     since_full,
                     formatTime(dur).c_str()
            );
            
            pDataChar->setValue(buf);
            pDataChar->notify();
        }
    }

    // Background Save (Every 30 seconds)
    static unsigned long last_save = 0;
    if (millis() - last_save > 30000) {
        prefs.putFloat("rem_ah", (float)current_capacity_ah);
        prefs.putFloat("off_val", (float)dynamic_offset);
        last_save = millis();
        Serial.println("[MEM] State saved.");
    }
}
