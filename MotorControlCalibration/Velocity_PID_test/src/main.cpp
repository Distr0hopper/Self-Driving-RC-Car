#include <Arduino.h>

// Pin Definitions
// Front Right Motor
const int FR_ENC_A = 36;
const int FR_ENC_B = 39;
const int FR_IN1 = 27;
const int FR_IN2 = 23;
const int HALL_PIN = 14;  // Optional for verification

// Encoder count
volatile long fr_count = 0;

// Motor control value (-255 to 255)
int fr_pwm = 0;

// Fast interrupt handler
void IRAM_ATTR fr_encoder() {
    if (digitalRead(FR_ENC_B)) fr_count--;  // Reversed
    else fr_count++;
}

// Set motor PWM (-255 to 255)
void setMotor(int in1_pin, int in2_pin, int pwm) {
    pwm = constrain(pwm, -255, 255); // Safety limit
    
    if (pwm > 0) {
        analogWrite(in1_pin, pwm);
        analogWrite(in2_pin, 0);
    } else {
        analogWrite(in1_pin, 0);
        analogWrite(in2_pin, -pwm);
    }
}

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    
    // Configure encoder pins with pullup
    pinMode(FR_ENC_A, INPUT_PULLUP);
    pinMode(FR_ENC_B, INPUT_PULLUP);
    pinMode(HALL_PIN, INPUT_PULLUP);
    
    // Configure motor pins as outputs
    pinMode(FR_IN1, OUTPUT);
    pinMode(FR_IN2, OUTPUT);
    
    // Attach interrupt on RISING edge
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), fr_encoder, RISING);
    
    // Initialize motor to stopped
    setMotor(FR_IN1, FR_IN2, 0);
}

void loop() {
    // Send encoder counts every 20ms (50Hz)
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 20) {
        // Format: "E,fr\n"
        Serial.print("E,");
        Serial.println(fr_count);
        lastTime = millis();
    }
    
    // Check for PWM commands
    // Format: "M,pwm\n"
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        if (cmd.startsWith("M,")) {
            fr_pwm = cmd.substring(2).toInt();
            setMotor(FR_IN1, FR_IN2, fr_pwm);
        }
    }
}