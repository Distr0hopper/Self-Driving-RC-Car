#include <Arduino.h>

// Pin Definitions
// Encoders
const int FR_ENC_A = 36;
const int FR_ENC_B = 39;
const int FL_ENC_A = 34;
const int FL_ENC_B = 35;
const int BR_ENC_A = 32;
const int BR_ENC_B = 33;
const int BL_ENC_A = 25;
const int BL_ENC_B = 26;

// Motor Drivers
// Right L298N
const int FR_IN1 = 27;
const int FR_IN2 = 23;
const int BR_IN1 = 22;
const int BR_IN2 = 21;

// Left L298N
const int BL_IN1 = 19;
const int BL_IN2 = 18;
const int FL_IN1 = 17;
const int FL_IN2 = 16;

// Encoder counts
volatile long fr_count = 0;
volatile long fl_count = 0;
volatile long br_count = 0;
volatile long bl_count = 0;

// Motor control values (-255 to 255)
int fr_pwm = 0;
int fl_pwm = 0;
int br_pwm = 0;
int bl_pwm = 0;

// Fast interrupt handlers - reversed counting logic
void IRAM_ATTR fr_encoder() {
    if (digitalRead(FR_ENC_B)) fr_count--;  // Reversed
    else fr_count++;
}

void IRAM_ATTR fl_encoder() {
    if (digitalRead(FL_ENC_B)) fl_count++;  // Reversed
    else fl_count--;
}

void IRAM_ATTR br_encoder() {
    if (digitalRead(BR_ENC_B)) br_count--;  // Reversed
    else br_count++;
}

void IRAM_ATTR bl_encoder() {
    if (digitalRead(BL_ENC_B)) bl_count++;  // Reversed
    else bl_count--;
}

// Set motor PWM (-255 to 255)
void setMotor(int in1_pin, int in2_pin, int pwm, bool isLeftSide) {
    if (isLeftSide) pwm = -pwm; // Reverse for left side
    
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
    pinMode(FL_ENC_A, INPUT_PULLUP);
    pinMode(FL_ENC_B, INPUT_PULLUP);
    pinMode(BR_ENC_A, INPUT_PULLUP);
    pinMode(BR_ENC_B, INPUT_PULLUP);
    pinMode(BL_ENC_A, INPUT_PULLUP);
    pinMode(BL_ENC_B, INPUT_PULLUP);
    
    // Attach interrupts on RISING edge
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), fr_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A), fl_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A), br_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A), bl_encoder, RISING);
    
    // Configure motor pins as outputs
    pinMode(FR_IN1, OUTPUT);
    pinMode(FR_IN2, OUTPUT);
    pinMode(BR_IN1, OUTPUT);
    pinMode(BR_IN2, OUTPUT);
    pinMode(BL_IN1, OUTPUT);
    pinMode(BL_IN2, OUTPUT);
    pinMode(FL_IN1, OUTPUT);
    pinMode(FL_IN2, OUTPUT);
    
    // Initialize all motors to stopped
    setMotor(FR_IN1, FR_IN2, 0, false);
    setMotor(FL_IN1, FL_IN2, 0, true);
    setMotor(BR_IN1, BR_IN2, 0, false);
    setMotor(BL_IN1, BL_IN2, 0, true);
}

void loop() {
    // Send encoder counts every 20ms (50Hz)
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 20) {
        // Format: "E,fr,fl,br,bl\n"
        Serial.print("E,");
        Serial.print(fr_count);
        Serial.print(",");
        Serial.print(fl_count);
        Serial.print(",");
        Serial.print(br_count);
        Serial.print(",");
        Serial.println(bl_count);
        lastTime = millis();
    }
    
    // Check for PWM commands
    // Format: "M,fr,fl,br,bl\n"
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        if (cmd.startsWith("M,")) {
            cmd = cmd.substring(2); // Remove "M,"
            fr_pwm = cmd.substring(0, cmd.indexOf(",")).toInt();
            cmd = cmd.substring(cmd.indexOf(",") + 1);
            fl_pwm = cmd.substring(0, cmd.indexOf(",")).toInt();
            cmd = cmd.substring(cmd.indexOf(",") + 1);
            br_pwm = cmd.substring(0, cmd.indexOf(",")).toInt();
            cmd = cmd.substring(cmd.indexOf(",") + 1);
            bl_pwm = cmd.toInt();
            
            // Apply PWM values with side reversal
            setMotor(FR_IN1, FR_IN2, fr_pwm, false);
            setMotor(FL_IN1, FL_IN2, fl_pwm, true);
            setMotor(BR_IN1, BR_IN2, br_pwm, false);
            setMotor(BL_IN1, BL_IN2, bl_pwm, true);
        }
    }
}