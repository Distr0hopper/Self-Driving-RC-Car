// Robot Receiver (ESP32 Dev C)
#include <esp_now.h>
#include <WiFi.h>

const int BLUE_LED = 13;
const int GREEN_LED = 26;
const int RED_LED = 14;
const int RELAY_IN1 = 17;
const int RELAY_IN2 = 18;

unsigned long lastMessageTime = 0;
const unsigned long MESSAGE_TIMEOUT = 500;
bool switchState = false;

typedef struct {
    bool switchState;
    uint32_t sequence;
} SafetyMessage;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(SafetyMessage)) {
        SafetyMessage *msg = (SafetyMessage*)data;
        switchState = msg->switchState;
        lastMessageTime = millis();
        
        Serial.printf("Message #%u received - Switch: %s\n", 
            msg->sequence, switchState ? "ON" : "OFF");
            
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                 recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
        Serial.printf("From: %s\n", macStr);
    }
}

void initWiFi() {
    // Set WiFi mode and disconnect from any previous connections
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Print MAC addresses
    Serial.print("STA MAC: "); 
    Serial.println(WiFi.macAddress());
    Serial.print("AP MAC:  "); 
    Serial.println(WiFi.softAPmacAddress());
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nRobot Starting...");
    
    // Initialize pins
    pinMode(BLUE_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(RELAY_IN1, OUTPUT);
    pinMode(RELAY_IN2, OUTPUT);
    
    // Initial LED states
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(RELAY_IN1, LOW);
    digitalWrite(RELAY_IN2, LOW);
    
    // Initialize WiFi and get MAC
    initWiFi();
    
    // Try multiple times to initialize ESP-NOW
    bool espNowInitialized = false;
    for(int i = 0; i < 3; i++) {
        if (esp_now_init() == ESP_OK) {
            espNowInitialized = true;
            Serial.println("ESP-NOW initialized successfully");
            break;
        }
        Serial.println("ESP-NOW init failed, retrying...");
        delay(1000);
    }
    
    if (!espNowInitialized) {
        Serial.println("Failed to initialize ESP-NOW after multiple attempts");
        digitalWrite(RED_LED, HIGH);
        while(1) {
            delay(1000);  // Halt execution
        }
    }
    
    esp_now_register_recv_cb(OnDataRecv);
    
    // Print MAC address again for verification
    Serial.println("\nFinal MAC Addresses:");
    Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
    Serial.print("AP MAC:  "); Serial.println(WiFi.softAPmacAddress());
}

void updateOutputs() {
    bool receiving = (millis() - lastMessageTime) < MESSAGE_TIMEOUT;
    
    digitalWrite(RED_LED, !receiving);
    digitalWrite(GREEN_LED, receiving);
    digitalWrite(BLUE_LED, receiving && switchState);
    
    digitalWrite(RELAY_IN1, receiving && switchState);
    digitalWrite(RELAY_IN2, receiving && switchState);
}

void loop() {
    updateOutputs();
    
    // Print status every 5 seconds
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 5000) {
        Serial.println("\n--- Status Update ---");
        Serial.printf("Time since last message: %lu ms\n", millis() - lastMessageTime);
        Serial.printf("Switch State: %s\n", switchState ? "ON" : "OFF");
        Serial.printf("Receiving: %s\n", (millis() - lastMessageTime < MESSAGE_TIMEOUT) ? "YES" : "NO");
        Serial.print("Current MAC: "); Serial.println(WiFi.macAddress());
        lastPrint = millis();
    }
}