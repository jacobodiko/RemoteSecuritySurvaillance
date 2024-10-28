// Blynk template and Wi-Fi credentials
#define BLYNK_TEMPLATE_ID "TMPL2C9_8E695"
#define BLYNK_TEMPLATE_NAME "Home Surveillance"
#define BLYNK_AUTH_TOKEN "9Dz_fybMUNC5VvqvPqLC0Mn7xW9MG39u"

// Include necessary libraries
#include <BlynkSimpleEsp32.h>
#include <Stepper.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#define BLYNK_PRINT Serial

const char *ssid = "slntgns";
const char *pass = "123456789";

// DC Motor Configuration (L298 Driver)
#define MOTOR_IN1 2
#define MOTOR_IN2 4
#define ENABLE_PIN 16

// Stepper Motor Configuration (L298 Driver)
#define STEPPER_IN1 14
#define STEPPER_IN2 12 // Modified to connect to GPIO 12
#define STEPPER_IN3 13 // Modified to connect to GPIO 13
#define STEPPER_IN4 15
const int stepsPerRevolution = 200;
Stepper stepper(stepsPerRevolution, STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);

// Limit Switches
#define LIMIT_SWITCH_OPEN 32
#define LIMIT_SWITCH_CLOSE 33

// Force Sensor and Pump
#define FORCE_SENSOR_PIN 34
#define PUMP_IN1 19
#define PUMP_IN2 18
int forceSensorThreshold = 500;
bool forceDetected = false;
bool sprayActive = false; // Flag to track if spray is active

// GPS Configuration
TinyGPSPlus gps;
HardwareSerial ss(1); // Initialize UART1 for GPS communication

// Blynk Virtual Pins
#define VPIN_MOTOR V1
#define VPIN_LOCK_UNLOCK_DOOR V2
#define VPIN_SIREN_SPRAY V4
#define VPIN_GPS_LAT_LONG V5
#define VPIN_GPS_SWITCH V6 // GPS switch virtual pin

BlynkTimer timer;

bool isLocked = false;
bool isMotorRunning = false;
bool motorDirection = false;
bool isGPSTracking = false; // Variable to track the GPS switch state

void setup() {
    // Start serial communication
    Serial.begin(115200);

    // Start GPS serial communication on specified pins
    ss.begin(9600, SERIAL_8N1, 3, 1); // ESP32 Rx (GPIO3) to GPS Tx, ESP32 Tx (GPIO1) to GPS Rx

    // Initialize Blynk
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    // Initialize GPIOs
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_OPEN, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(FORCE_SENSOR_PIN, INPUT);
    pinMode(PUMP_IN1, OUTPUT);
    pinMode(PUMP_IN2, OUTPUT);

    // Timer to check force sensor
    timer.setInterval(1000L, checkForceSensor);

    // Set labels in Blynk for easier UI control
    Blynk.setProperty(VPIN_MOTOR, "label", "DC Motor (ON/OFF)");
    Blynk.setProperty(VPIN_LOCK_UNLOCK_DOOR, "label", "Lock/Unlock Door");
    Blynk.setProperty(VPIN_SIREN_SPRAY, "label", "Siren and Spray");
    Blynk.setProperty(VPIN_GPS_LAT_LONG, "label", "GPS Lat/Long");

    Serial.println("Blynk Online");
}

void loop() {
    Blynk.run();  // Always run Blynk
    timer.run();  // Run Blynk timers
    updateGPS();  // Update GPS data

    // Motor operation check
    if (isMotorRunning) {
        operateMotor();
    }
}

// DC Motor ON/OFF control via Blynk
BLYNK_WRITE(VPIN_MOTOR) {
    int value = param.asInt();
    
    if (value == 1) {
        Serial.println("Motor ON: Clockwise");
        motorDirection = false;
        isMotorRunning = true;
    } else if (value == 0) {
        Serial.println("Motor ON: Anticlockwise");
        motorDirection = true;
        isMotorRunning = true;
    }
}

// Function to operate the motor based on direction and limit switches
void operateMotor() {
    if (motorDirection == false) {
        if (digitalRead(LIMIT_SWITCH_OPEN) == LOW) {
            stopMotor();
            Serial.println("Motor stopped at clockwise limit.");
        } else {
            rotateClockwise();
        }
    } else {
        if (digitalRead(LIMIT_SWITCH_CLOSE) == LOW) {
            stopMotor();
            Serial.println("Motor stopped at anticlockwise limit.");
        } else {
            rotateAnticlockwise();
        }
    }
}

void rotateClockwise() {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.println("Motor rotating clockwise...");
}

void rotateAnticlockwise() {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.println("Motor rotating anticlockwise...");
}

void stopMotor() {
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    isMotorRunning = false;
}

// Combined Lock/Unlock Door using a single Blynk button
BLYNK_WRITE(VPIN_LOCK_UNLOCK_DOOR) {
    int value = param.asInt();
    if (value) {
        if (isLocked) {
            unlockDoor();
        } else {
            lockDoor();
        }
    }
}

void lockDoor() {
    stepper.step(-stepsPerRevolution / 2);
    isLocked = true;
    Serial.println("Door Locked");
}

void unlockDoor() {
    stepper.step(stepsPerRevolution / 2);
    isLocked = false;
    Serial.println("Door Unlocked");
}

void checkForceSensor() {
    int forceValue = analogRead(FORCE_SENSOR_PIN);
    if (forceValue > forceSensorThreshold && !forceDetected) {
        forceDetected = true;
        Blynk.logEvent("force_detected", "Force detected on door!");
        triggerSpray();
    } else if (forceValue <= forceSensorThreshold) {
        forceDetected = false;
    }
}

void triggerSpray() {
    if (!sprayActive) {
        sprayActive = true;
        activatePumpAndSiren();
        timer.setTimeout(30000L, deactivatePumpAndSiren);
    }
}

void activatePumpAndSiren() {
    digitalWrite(PUMP_IN1, HIGH);
    digitalWrite(PUMP_IN2, LOW);
    Serial.println("Spray and Siren Activated");
}

void deactivatePumpAndSiren() {
    digitalWrite(PUMP_IN1, LOW);
    digitalWrite(PUMP_IN2, LOW);
    sprayActive = false;
    Serial.println("Spray and Siren Deactivated");
}

BLYNK_WRITE(VPIN_SIREN_SPRAY) {
    int value = param.asInt();
    if (value) {
        triggerSpray();
    }
}

// GPS Switch control via Blynk
BLYNK_WRITE(VPIN_GPS_SWITCH) {
    isGPSTracking = param.asInt(); // Update GPS tracking state based on Blynk switch
    if (isGPSTracking) {
        Serial.println("GPS Switch ON: Tracking...");
    } else {
        Serial.println("GPS Switch OFF: Stopped tracking.");
    }
}

void updateGPS() {
    while (ss.available() > 0) {
        gps.encode(ss.read());
        if (gps.location.isUpdated() && isGPSTracking) { // Check if GPS tracking is active
            String gps_data = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
            Blynk.virtualWrite(VPIN_GPS_LAT_LONG, gps_data);
            Serial.println("GPS Location Sent: " + gps_data);
        }
    }
}
