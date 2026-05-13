#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <INA226_WE.h> 
#include <Preferences.h>

// RTC RAM — Data that persists through Deep Sleep
RTC_DATA_ATTR double rtc_current_capacity_ah = -1.0;
RTC_DATA_ATTR double rtc_dynamic_offset = 0.027;
RTC_DATA_ATTR double rtc_charged_prev_ah = 0;
RTC_DATA_ATTR double rtc_discharged_prev_ah = 0;
RTC_DATA_ATTR double rtc_last_measured_current = 0; 
RTC_DATA_ATTR int rtc_sleep_cycles = 0;
RTC_DATA_ATTR bool rtc_learning_mode = false;

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
const double THRESHOLD_A = 0.015; 
const double WAKE_DELTA_A = 0.100; 

double current_session_delta = 0; 
unsigned long session_start_ms = 0;
double ema_current = 0;
bool ina_found = false; 
unsigned long last_micros = 0;
double last_current_a = 0;
bool deviceConnected = false;

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pDataChar = NULL;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) { 
        deviceConnected = false; 
        BLEDevice::startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            if (value == "sync:1") { rtc_current_capacity_ah = actual_capacity_ah; rtc_learning_mode = false; }
            else if (value == "sync:0") { rtc_current_capacity_ah = 0; rtc_learning_mode = true; }
            else if (value.startsWith("cap:")) { 
                actual_capacity_ah = value.substring(4).toDouble(); 
                prefs.putFloat("act_cap", (float)actual_capacity_ah); 
            }
            else if (value.startsWith("volt:")) { 
                v_full_charge = value.substring(5).toFloat(); 
                prefs.putFloat("v_full", v_full_charge); 
            }
        }
    }
};

String formatTime(unsigned long sec) {
    char buf[12];
    sprintf(buf, "%02d:%02d:%02d", (int)(sec/3600), (int)((sec%3600)/60), (int)(sec%60));
    return String(buf);
}

double getPreciseCurrent() {
    if (!ina_found) return 0.0;
    double current = (ina.getShuntVoltage_mV() / (SHUNT_RESISTANCE * 1000.0)) + rtc_dynamic_offset;
    double alpha = (abs(current) > 0.5) ? 0.3 : 0.1;
    ema_current = (current * alpha) + (ema_current * (1.0 - alpha));
    
    if (abs(ema_current) < 0.008) { 
        rtc_dynamic_offset -= (ema_current * 0.0001); 
        return 0.0; 
    }
    return ema_current;
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    if (ina.init()) {
        ina_found = true;
        ina.setResistorRange(SHUNT_RESISTANCE, 300.0);
    }

    prefs.begin("batt_pro", false);
    actual_capacity_ah = prefs.getFloat("act_cap", 8.0);
    v_full_charge = prefs.getFloat("v_full", 25.2);
    
    if (rtc_current_capacity_ah < 0) {
        rtc_current_capacity_ah = prefs.getFloat("rem_ah", actual_capacity_ah);
        rtc_dynamic_offset = prefs.getFloat("off_val", 0.027);
    }

    double current_now = getPreciseCurrent();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        rtc_current_capacity_ah += (current_now * (10.0 / 3600.0));
        rtc_sleep_cycles++;
        bool current_jump = abs(current_now - rtc_last_measured_current) > WAKE_DELTA_A;
        rtc_last_measured_current = current_now;

        if (!current_jump && abs(current_now) < THRESHOLD_A && rtc_sleep_cycles < 30) {
            esp_sleep_enable_timer_wakeup(10 * 1000000);
            esp_deep_sleep_start();
        }
    }
    
    rtc_sleep_cycles = 0; 
    rtc_last_measured_current = current_now;
    ina.setAverage(INA226_AVERAGE_16); 

    BLEDevice::init("SmartShunt-C6");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);

    pDataChar = pService->createCharacteristic(CHAR_DATA_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    
    BLECharacteristic *pConfigChar = pService->createCharacteristic(CHAR_CONFIG_UUID, BLECharacteristic::PROPERTY_WRITE);
    pConfigChar->setCallbacks(new MyCallbacks());
    
    pService->start();
    BLEDevice::getAdvertising()->start();

    last_micros = micros();
    session_start_ms = millis();
}

void loop() {
    double current_a = getPreciseCurrent();
    unsigned long now_micros = micros();
    double dt = (double)(now_micros - last_micros) / 3600000000.0; 
    last_micros = now_micros;

    if (ina_found) {
        double delta_ah = ((last_current_a + current_a) / 2.0) * dt;
        rtc_current_capacity_ah += delta_ah;
        current_session_delta += delta_ah;
    }

    static unsigned long last_activity = millis();
    if (abs(current_a) > THRESHOLD_A || deviceConnected) {
        last_activity = millis();
    }

    if (millis() - last_activity > 20000) {
        prefs.putFloat("rem_ah", (float)rtc_current_capacity_ah);
        prefs.putFloat("off_val", (float)rtc_dynamic_offset);
        rtc_last_measured_current = current_a;
        esp_sleep_enable_timer_wakeup(10 * 1000000); 
        esp_deep_sleep_start();
    }

    if (last_current_a > THRESHOLD_A && current_a <= THRESHOLD_A) {
        if (current_session_delta > 0.001) rtc_charged_prev_ah = current_session_delta;
        current_session_delta = 0; session_start_ms = millis();
    } else if (last_current_a < -THRESHOLD_A && current_a >= -THRESHOLD_A) {
        if (abs(current_session_delta) > 0.001) rtc_discharged_prev_ah = abs(current_session_delta);
        current_session_delta = 0; session_start_ms = millis();
    }

    float v = (ina_found) ? ina.getBusVoltage_V() : 0.0;
    if (ina_found && v >= (v_full_charge - 0.1) && current_a > 0.005 && current_a < 0.12) {
        rtc_current_capacity_ah = actual_capacity_ah;
    }

    if (rtc_current_capacity_ah > actual_capacity_ah) rtc_current_capacity_ah = actual_capacity_ah;
    if (rtc_current_capacity_ah < 0) rtc_current_capacity_ah = 0;
    last_current_a = current_a;

    static unsigned long lastBle = 0;
    if (deviceConnected && millis() - lastBle > 1000) {
        lastBle = millis();
        char buf[250];
        double soc = (actual_capacity_ah > 0) ? (rtc_current_capacity_ah / actual_capacity_ah) * 100.0 : 0.0;
        snprintf(buf, sizeof(buf), "%.2fV|%.0fmA|%.3fAh|%.1f%%|%s|S:%.4f|LC:%.3f|LD:%.3f|T:%s", 
                 v, current_a * 1000.0, rtc_current_capacity_ah, soc, 
                 ina_found ? (rtc_learning_mode ? "LEARN" : "OK") : "ERR",
                 current_session_delta, rtc_charged_prev_ah, rtc_discharged_prev_ah,
                 formatTime((millis() - session_start_ms) / 1000).c_str());
        pDataChar->setValue(buf);
        pDataChar->notify();
    }
}
