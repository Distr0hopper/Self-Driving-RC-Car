#include <Arduino.h>

const int ENCODER_A = 25;    // Yellow (A Phase)
const int ENCODER_B = 26;    // Green (B Phase)
const int HALL_PIN = 33;     // Hall sensor input

volatile long encoderCount = 0;
volatile long hallCount = 0;
volatile int lastA = 0;

// Debounce for Hall sensor
volatile unsigned long lastHallTime = 0;
const unsigned long DEBOUNCE_TIME = 5; // milliseconds
volatile int lastHallState = HIGH;     // Start with inactive state

void IRAM_ATTR encoderISR() {
    int A = digitalRead(ENCODER_A);
    
    if (A != lastA) {
        encoderCount += (A == digitalRead(ENCODER_B)) ? -1 : 1;
    }
    
    
    lastA = A;
}

void IRAM_ATTR hallISR() {
    unsigned long currentTime = millis();
    if (currentTime - lastHallTime > DEBOUNCE_TIME) {
        int hallState = digitalRead(HALL_PIN);
        if (hallState == LOW && lastHallState == HIGH) {  // Detect falling edge (magnet present)
            hallCount++;
        }
        lastHallState = hallState;
        lastHallTime = currentTime;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(HALL_PIN, INPUT_PULLUP);
    
    lastA = digitalRead(ENCODER_A);
    lastHallState = digitalRead(HALL_PIN);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, CHANGE);
    
    Serial.println("Motor Analysis");
    Serial.println("Encoder\tHall\tPulses/Rev\tGear Ratio");
}

void loop() {
    static long lastEncoderCount = 0;
    static long lastHallCount = 0;
    static unsigned long lastPrint = 0;
    const unsigned long PRINT_INTERVAL = 100; // Print every 100ms
    
    unsigned long currentTime = millis();
    
    if (currentTime - lastPrint >= PRINT_INTERVAL) {
        if (encoderCount != lastEncoderCount || hallCount != lastHallCount) {
            float pulsesPerRev = 0;
            float gearRatio = 0;
            
            if (hallCount > 0) {  // Avoid division by zero
                pulsesPerRev = abs((float)encoderCount / hallCount);
                gearRatio = pulsesPerRev / 44.0;  // 11 PPR * 4 = 44 counts per motor rev
            }
            
            Serial.print(encoderCount);
            Serial.print("\t");
            Serial.print(hallCount);
            Serial.print("\t");
            Serial.print(pulsesPerRev, 2);
            Serial.print("\t");
            Serial.println(gearRatio, 2);
            
            lastEncoderCount = encoderCount;
            lastHallCount = hallCount;
        }
        lastPrint = currentTime;
    }
}