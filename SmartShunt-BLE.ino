#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <INA226_WE.h> 
#include <Preferences.h>

// I2C pin configuration for ESP32
#define I2C_SDA 20
#define I2C_SCL 19
#define I2C_ADDRESS 0x40

// Hardware ALERT pin connected from INA226 to ESP32 (Wakeup Pin)
#define INA_ALERT_PIN 4 

// Non-volatile RTC RAM variables (persisting through Deep Sleep)
RTC_DATA_ATTR double rtc_current_capacity_ah = -1.0;
RTC_DATA_ATTR double rtc_dynamic_offset = 0.027; 
RTC_DATA_ATTR double rtc_gain_factor = 1.0; 
RTC_DATA_ATTR double rtc_charged_prev_ah = 0;
RTC_DATA_ATTR double rtc_discharged_prev_ah = 0;
RTC_DATA_ATTR double rtc_last_measured_current = 0; 
RTC_DATA_ATTR int rtc_sleep_cycles = 0;
RTC_DATA_ATTR bool rtc_learning_mode = false;

INA226_WE ina = INA226_WE(I2C_ADDRESS);
Preferences prefs;

// Battery and Shunt settings
double actual_capacity_ah = 8.0;       
const float SHUNT_RESISTANCE = 0.00025; // External heavy-duty shunt: 0.25 mOhm
float v_full_charge = 25.2;             

// Sleep automation constants
const double THRESHOLD_A = 0.015;      // Activity threshold (15 mA)
const double WAKE_DELTA_A = 0.100;     // Current delta to keep awake (100 mA)

// Session logic and filtering variables
double current_session_delta = 0; 
unsigned long session_start_ms = 0;
unsigned long last_integration_micros = 0;
double last_filtered_current_a = 0;
double ema_current = 0;

bool ina_found = false; 
bool deviceConnected = false;
unsigned long lastBleNotify = 0;

// Two-point calibration variables
double p1_raw = 0.0;
double p1_ext = 0.0;
bool p1_ready = false;

// BLE UUIDs
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_DATA_UUID         "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_CONFIG_UUID       "82250625-fba7-47bc-8034-745ad5a95a40"

BLEServer* pServer = NULL;
BLECharacteristic* pDataChar = NULL;

// Direct I2C write for INA226 16-bit registers (bypasses library limits)
void writeInaRegisterDirect(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));   // High byte
    Wire.write((uint8_t)(value & 0xFF));  // Low byte
    Wire.endTransmission();
}

// Direct I2C read for INA226 16-bit registers
uint16_t readInaRegisterDirect(uint8_t reg) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return 0;
    
    Wire.requestFrom((uint8_t)I2C_ADDRESS, (uint8_t)2);
    if (Wire.available() == 2) {
        uint16_t value = Wire.read() << 8;
        value |= Wire.read();
        return value;
    }
    return 0;
}

// Helper function to re-arm INA226 hardware Alert configuration before sleep
void configureInaAlert() {
    if (!ina_found) return;
    
    // 1. Read Mask/Enable register (0x06) to clear any old latch/alert flags
    readInaRegisterDirect(0x06);
    
    // 2. Calculate Shunt Over Voltage limit value:
    // Limit in microvolts divided by 2.5 uV step size
    // 15mA * 0.25 mOhm = 3.75 uV. 3.75 / 2.5 = 1.5 -> rounding up to 2 steps (5 uV)
    uint16_t alert_limit_register = (uint16_t)((THRESHOLD_A * SHUNT_RESISTANCE * 1000000.0) / 2.5);
    if (alert_limit_register == 0) alert_limit_register = 1;
    
    // Write limit into Alert Limit Register (0x07)
    writeInaRegisterDirect(0x07, alert_limit_register);
    
    // 3. Configure Mask/Enable Register (0x06)
    // Bit 15 (SOL) = 1: Shunt Over Voltage configuration
    // Bit 0  (LEN) = 1: Latch Enable (holds the alert pin LOW until read)
    uint16_t mask_enable_config = (1 << 15) | (1 << 0);
    writeInaRegisterDirect(0x06, mask_enable_config);
}

// Get clean raw current from ADC (without any calibration applied)
double getPureRawCurrent() {
    if (!ina_found) return 0.0;
    return (ina.getShuntVoltage_mV() / (SHUNT_RESISTANCE * 1000.0));
}

// Get fully calibrated and filtered current
double getPreciseCurrent() {
    if (!ina_found) return 0.0;
    
    // Calculate current using linear system: (Raw_Current + Offset) * Gain
    double current_calculated = (getPureRawCurrent() + rtc_dynamic_offset) * rtc_gain_factor;
    
    // Adaptive Exponential Moving Average (EMA) filter
    double alpha = (abs(current_calculated) > 0.5) ? 0.4 : 0.2;
    ema_current = (current_calculated * alpha) + (ema_current * (1.0 - alpha));
    
    return ema_current;
}

// Format interval/session time into HH:MM:SS
String formatTime(unsigned long sec) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", (int)(sec/3600), (int)((sec%3600)/60), (int)(sec%60));
    return String(buf);
}

// Format device total uptime since boot into HH:MM:SS
String getSessionTime() {
    unsigned long total_seconds = millis() / 1000;
    unsigned long seconds = total_seconds % 60;
    unsigned long minutes = (total_seconds / 60) % 60;
    unsigned long hours = total_seconds / 3600;
    
    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu", hours, minutes, seconds);
    return String(time_str);
}

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
        if (value.length() == 0) return;

        // POINT 1: Background current measurement (e.g., p1:-0.014)
        if (value.startsWith("p1:")) {
            p1_ext = value.substring(3).toDouble();
            
            double raw_sum = 0.0;
            int samples = 300; 
            for (int i = 0; i < samples; i++) {
                raw_sum += getPureRawCurrent();
                delay(15); 
            }
            p1_raw = raw_sum / samples;
            p1_ready = true;
            
            Serial.print("[CAL] Point 1 Saved. Raw: "); Serial.print(p1_raw, 4);
            Serial.print(" A | Ext: "); Serial.print(p1_ext, 4); Serial.println(" A");
        }
        
        // POINT 2: Loaded current measurement (e.g., p2:-0.749)
        else if (value.startsWith("p2:")) {
            if (!p1_ready) {
                Serial.println("[CAL ERROR] Set point 1 first using p1: command !!!");
                return;
            }
            
            double p2_ext = value.substring(3).toDouble();
            
            double raw_sum = 0.0;
            int samples = 300;
            for (int i = 0; i < samples; i++) {
                raw_sum += getPureRawCurrent();
                delay(15); 
            }
            double p2_raw = raw_sum / samples;
            
            if (abs(p2_raw - p1_raw) > 0.010) { 
                rtc_gain_factor = (p2_ext - p1_ext) / (p2_raw - p1_raw);
                rtc_dynamic_offset = (p1_ext / rtc_gain_factor) - p1_raw;
                
                prefs.putDouble("offset", rtc_dynamic_offset);
                prefs.putDouble("gain_f", rtc_gain_factor);
                
                ema_current = p2_ext; 
                p1_ready = false;     
                
                Serial.println("=========================================");
                Serial.println("[CAL SUCCESS] CALIBRATION COMPLETED");
                Serial.print("Calculated Gain: "); Serial.println(rtc_gain_factor, 6);
                Serial.print("Calculated Offset: "); Serial.println(rtc_dynamic_offset, 6);
                Serial.println("=========================================");
            } else {
                Serial.println("[CAL ERROR] Failed! Current delta between p1 and p2 is too small.");
            }
        }
        else if (value == "sync:1") { 
            rtc_current_capacity_ah = actual_capacity_ah; 
            rtc_learning_mode = false; 
        }
        else if (value == "sync:0") { 
            rtc_current_capacity_ah = 0; 
            rtc_learning_mode = true; 
        }
        else if (value.startsWith("cap:")) {
            actual_capacity_ah = value.substring(4).toDouble();
            prefs.putFloat("act_cap", (float)actual_capacity_ah);
        }
        else if (value.startsWith("volt:")) { 
            v_full_charge = value.substring(5).toFloat(); 
            prefs.putFloat("v_full", v_full_charge); 
        }
        else if (value.startsWith("gain:")) {
            rtc_gain_factor = value.substring(5).toDouble();
            prefs.putDouble("gain_f", rtc_gain_factor);
        }
    }
};

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Configure hardware alert input pin with internal pullup 
    pinMode(INA_ALERT_PIN, INPUT_PULLUP);

    prefs.begin("batt_pro", false);
    actual_capacity_ah = prefs.getFloat("act_cap", 8.0);
    v_full_charge = prefs.getFloat("v_full", 25.2);
    
    if (prefs.isKey("offset")) rtc_dynamic_offset = prefs.getDouble("offset");
    if (prefs.isKey("gain_f")) rtc_gain_factor = prefs.getDouble("gain_f");
    
    if (rtc_current_capacity_ah < 0) {
        rtc_current_capacity_ah = prefs.getFloat("rem_ah", actual_capacity_ah);
    }

    if (ina.init()) {
        ina_found = true;
        ina.setResistorRange(SHUNT_RESISTANCE, 400.0);
        readInaRegisterDirect(0x06); // Direct clear any pending flags on boot
    } else {
        Serial.println("!!! TARGET INA226 BOARD NOT FOUND !!!");
    }

    double current_now = getPreciseCurrent();

    // Wakeup logic handling both Timer and Hardware Pin signals (Compatible with modern ESP32)
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        rtc_current_capacity_ah += (current_now * (10.0 / 3600.0)); 
        rtc_sleep_cycles++;
        
        bool current_jump = abs(current_now - rtc_last_measured_current) > WAKE_DELTA_A;
        rtc_last_measured_current = current_now;

        // If no sudden current activity or active BLE connection, go back to sleep
        if (!current_jump && abs(current_now) < THRESHOLD_A && rtc_sleep_cycles < 30) {
            configureInaAlert();
            
            uint64_t pin_mask = 1ULL << INA_ALERT_PIN;
            esp_sleep_enable_ext1_wakeup(pin_mask, ESP_EXT1_WAKEUP_ANY_LOW); 
            
            esp_sleep_enable_timer_wakeup(10 * 1000000);
            esp_deep_sleep_start();
        }
    }
    
    rtc_sleep_cycles = 0; 
    rtc_last_measured_current = current_now;
    
    // Switch ADC to highest hardware average settings for active runtime session
    if (ina_found) {
        ina.setAverage(INA226_AVERAGE_1024); 
        ina.setConversionTime(INA226_CONV_TIME_1100); 
    }

    // BLE initialization
    BLEDevice::init("SmartShunt-C6");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pDataChar = pService->createCharacteristic(CHAR_DATA_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    BLECharacteristic *pConfigChar = pService->createCharacteristic(CHAR_CONFIG_UUID, BLECharacteristic::PROPERTY_WRITE);
    pConfigChar->setCallbacks(new MyCallbacks());
    pService->start();
    
    // BLE Advertising Configuration
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  
    pAdvertising->setMinPreferred(0x12);
    
    // Properly package Manufacturer Specific Data
    std::string mfgData = "";
    mfgData += (char)0xE5; 
    mfgData += (char)0x02; 
    mfgData += "Alex_Lab"; 
    
    BLEAdvertisementData advData;
    advData.setManufacturerData(String(mfgData.c_str())); 
    pAdvertising->setAdvertisementData(advData);

    pAdvertising->start();
    
    last_integration_micros = micros();
    session_start_ms = millis();
    Serial.println("Battery monitoring system initialized.");
}

void loop() {
    double current_a = getPreciseCurrent();
    float v_bus = (ina_found) ? ina.getBusVoltage_V() : 0.0;
    float v_shunt_mv = (ina_found) ? ina.getShuntVoltage_mV() : 0.0;

    unsigned long now_micros = micros();
    double dt = (double)(now_micros - last_integration_micros) / 3600000000.0; 
    last_integration_micros = now_micros;

    if (ina_found) {
        double delta_ah = ((last_filtered_current_a + current_a) / 2.0) * dt;
        rtc_current_capacity_ah += delta_ah;
        current_session_delta += delta_ah;
    }

    // Process inactivity timeout to go into Deep Sleep
    static unsigned long last_activity = millis();
    if (abs(current_a) > THRESHOLD_A || deviceConnected) {
        last_activity = millis();
    }

    if (millis() - last_activity > 20000) { // 20 seconds of inactivity
        prefs.putFloat("rem_ah", (float)rtc_current_capacity_ah);
        rtc_last_measured_current = current_a;
        
        // Prepare INA and ARM external pin triggers
        configureInaAlert();
        
        uint64_t pin_mask = 1ULL << INA_ALERT_PIN;
        esp_sleep_enable_ext1_wakeup(pin_mask, ESP_EXT1_WAKEUP_ANY_LOW); 
        
        esp_sleep_enable_timer_wakeup(10 * 1000000); 
        esp_deep_sleep_start();
    }

    // Session transitions state logging
    if (last_filtered_current_a > THRESHOLD_A && current_a <= THRESHOLD_A) {
        if (current_session_delta > 0.001) rtc_charged_prev_ah = current_session_delta;
        current_session_delta = 0; session_start_ms = millis();
    } else if (last_filtered_current_a < -THRESHOLD_A && current_a >= -THRESHOLD_A) {
        if (abs(current_session_delta) > 0.001) rtc_discharged_prev_ah = abs(current_session_delta);
        current_session_delta = 0; session_start_ms = millis();
    }

    // Full charge automatic sync processing
    if (ina_found && v_bus >= (v_full_charge - 0.1) && current_a > 0.005 && current_a < 0.12) {
        rtc_current_capacity_ah = actual_capacity_ah;
    }

    if (rtc_current_capacity_ah > actual_capacity_ah) rtc_current_capacity_ah = actual_capacity_ah;
    if (rtc_current_capacity_ah < 0) rtc_current_capacity_ah = 0;
    
    last_filtered_current_a = current_a;

    // Send data update package via BLE
    if (deviceConnected && (millis() - lastBleNotify > 1000)) {
        lastBleNotify = millis();
        char buf[250];
        double soc = (actual_capacity_ah > 0) ? (rtc_current_capacity_ah / actual_capacity_ah) * 100.0 : 0.0;
        String session_time = getSessionTime();
        String st_time = formatTime((millis() - session_start_ms) / 1000);
        
        snprintf(buf, sizeof(buf), "Upt:%s|%.2fV|%.1fmA|%.4fAh|%.2f%%|%s|S_Dlt:%.4f|LC:%.3f|LD:%.3f|ST:%s|Shnt:%.3fmV|Offs:%.5f|Gain:%.5f", 
                 session_time.c_str(), v_bus, current_a * 1000.0, rtc_current_capacity_ah, soc, 
                 ina_found ? (rtc_learning_mode ? "LEARN" : "OK") : "ERR",
                 current_session_delta, rtc_charged_prev_ah, rtc_discharged_prev_ah, st_time.c_str(),
                 v_shunt_mv, rtc_dynamic_offset, rtc_gain_factor);
                 
        pDataChar->setValue(buf); 
        pDataChar->notify();
    }
    delay(15);
}
