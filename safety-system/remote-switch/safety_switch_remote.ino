// Remote Control (ESP32 Huzzah)
#include <esp_now.h>
#include <WiFi.h>

const int SWITCH_PIN = 26;
const int WIFI_CHANNEL = 1;
const int BATT_MONITOR_EN = 13;    // Battery monitor enable pin
const int VBAT_PIN = 35;           // Battery voltage monitoring pin

// Updated MAC Address to match receiver's STA MAC exactly
uint8_t robotAddress[] = {0x08, 0xD1, 0xF9, 0xCC, 0x7E, 0xF0};

typedef struct {
    bool switchState;
    uint32_t sequence;
} SafetyMessage;

uint32_t messageCount = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.printf("Failed to send message #%u\n", messageCount);
    } else {
        Serial.printf("Message #%u sent successfully\n", messageCount);
    }
}

float readBatteryVoltage() {
    float voltage = analogRead(VBAT_PIN);
    voltage *= 2;    // Voltage divider on board
    voltage *= 3.3;  // Reference voltage
    voltage /= 4095.0; // Convert ADC value
    return voltage;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nRemote Starting...");
    
    // Enable battery monitoring
    pinMode(BATT_MONITOR_EN, OUTPUT);
    digitalWrite(BATT_MONITOR_EN, HIGH);
    
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    
    // Set WiFi mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Print own MAC for debugging
    Serial.print("My MAC: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    
    esp_now_register_send_cb(OnDataSent);
    
    // Add peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, robotAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    
    // Print target MAC for verification
    Serial.print("Sending to Robot MAC: ");
    for(int i=0; i<6; i++) {
        if(i > 0) Serial.print(":");
        Serial.printf("%02X", robotAddress[i]);
    }
    Serial.println();
    
    Serial.println("Remote ready!");
}

void loop() {
    static unsigned long lastSend = 0;
    static bool lastSwitchState = false;
    bool currentSwitchState = !digitalRead(SWITCH_PIN);
    
    // Send message every 100ms or immediately on state change
    if (millis() - lastSend >= 100 || currentSwitchState != lastSwitchState) {
        SafetyMessage msg;
        msg.switchState = currentSwitchState;
        msg.sequence = ++messageCount;
        
        esp_err_t result = esp_now_send(robotAddress, (uint8_t*)&msg, sizeof(msg));
        if (result != ESP_OK) {
            Serial.println("Failed to send data");
        }
        
        lastSend = millis();
        lastSwitchState = currentSwitchState;
    }
    
    // Print status occasionally
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus >= 5000) {
        Serial.println("\n--- Status Update ---");
        Serial.printf("Switch state: %s\n", currentSwitchState ? "ON" : "OFF");
        Serial.printf("Messages attempted: %u\n", messageCount);
        Serial.printf("Battery Voltage: %.2fV\n", readBatteryVoltage());
        lastStatus = millis();
    }
}