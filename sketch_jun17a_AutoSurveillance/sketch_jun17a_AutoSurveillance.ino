// Blynk template and Wi-Fi credentials 
#define BLYNK_TEMPLATE_ID "TMPL2AYsdr9zk"
#define BLYNK_TEMPLATE_NAME "Home Security"
#define BLYNK_AUTH_TOKEN "u4DPdSaE7yw7ClcPGOCf5QkfozAlXoZP"

// Include necessary libraries
#include <BlynkSimpleEsp32.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#define BLYNK_PRINT Serial

const char *ssid = "slntgns";
const char *pass = "123456789";

// DC Motor Configuration (L298 Driver)
#define MOTOR_IN1 2
#define MOTOR_IN2 4
#define ENABLE_PIN 16

// Solenoid Lock Configuration (L298 Driver)
#define SOLENOID_IN1 14
#define SOLENOID_IN2 12
#define SOLENOID_ENABLE 27

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
#define VPIN_SIREN_SPRAY V3
#define VPIN_GPS_LATITUDE V0
#define VPIN_GPS_LONGITUDE V4

BlynkTimer timer;

bool isLocked = false;
bool isMotorRunning = false;
bool motorDirection = false;

void reconnectBlynk() {
    if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
        Serial.println("Reconnecting to Blynk...");
        Blynk.connect();
    }
}

void setup() {
    // Start serial communication
    Serial.begin(115200);

    // Start GPS serial communication on specified pins
    ss.begin(9600, SERIAL_8N1, 3, 1); // ESP32 Rx (GPIO3) to GPS Tx, ESP32 Tx (GPIO1) to GPS Rx

    // Connect to Wi-Fi
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to Wi-Fi...");
    }
    Serial.println("Wi-Fi connected.");

    // Initialize Blynk
    Blynk.config(BLYNK_AUTH_TOKEN);

    // Initialize GPIOs for L298N control of DC motor
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    // Initialize GPIOs for L298N control of solenoid lock
    pinMode(SOLENOID_IN1, OUTPUT);
    pinMode(SOLENOID_IN2, OUTPUT);
    pinMode(SOLENOID_ENABLE, OUTPUT);
    digitalWrite(SOLENOID_ENABLE, HIGH); // Enable solenoid driver

    // Initialize other GPIOs
    pinMode(LIMIT_SWITCH_OPEN, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(FORCE_SENSOR_PIN, INPUT);
    pinMode(PUMP_IN1, OUTPUT);
    pinMode(PUMP_IN2, OUTPUT);

    // Timer to check force sensor every 1 second
    timer.setInterval(1000L, checkForceSensor);

    // Timer to send GPS data every 1 minute
    timer.setInterval(60000L, sendGPS);

    // Set labels in Blynk for easier UI control
    Blynk.setProperty(VPIN_MOTOR, "label", "DC Motor (ON/OFF)");
    Blynk.setProperty(VPIN_LOCK_UNLOCK_DOOR, "label", "Lock/Unlock Door");
    Blynk.setProperty(VPIN_SIREN_SPRAY, "label", "Siren and Spray");

    Serial.println("Blynk Online");
}

void loop() {
    if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
        Blynk.run();  // Always run Blynk if connected
    } else {
        reconnectBlynk(); // Attempt to reconnect if disconnected
    }

    timer.run();  // Run Blynk timers
    updateGPS();  // Update GPS data continuously

    // Operate the motor only if it is running
    if (isMotorRunning) {
        operateMotor();
    }
}

// GPS function to send location data to Blynk every 1 minute
void sendGPS() {
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    if (!gps.location.isValid()) {
        Serial.println("Failed to read from GPS Module!");
        return;
    }

    // Get latitude and longitude
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Send latitude and longitude to Blynk
    Blynk.virtualWrite(VPIN_GPS_LATITUDE, String(latitude, 6));
    Blynk.virtualWrite(VPIN_GPS_LONGITUDE, String(longitude, 6));

    // Debugging in Serial Monitor
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
}

// Update GPS data if available
void updateGPS() {
    while (ss.available() > 0) {
        gps.encode(ss.read());
        if (gps.location.isUpdated()) {
            String gps_data = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
            Serial.println("GPS Location Updated: " + gps_data);
        }
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

// Lock/Unlock Door using Solenoid Lock with a Blynk switch
BLYNK_WRITE(VPIN_LOCK_UNLOCK_DOOR) {
    int value = param.asInt();
    if (value == 1) {
        lockDoor();
    } else if (value == 0) {
        unlockDoor();
    }
}

void lockDoor() {
    digitalWrite(SOLENOID_IN1, HIGH);
    digitalWrite(SOLENOID_IN2, LOW);
    isLocked = true;
    Serial.println("Door Locked");
}

void unlockDoor() {
    digitalWrite(SOLENOID_IN1, LOW);
    digitalWrite(SOLENOID_IN2, LOW);  // Deactivate solenoid (unlock the door)
    isLocked = false;
    Serial.println("Door Unlocked");
}

// Ensure solenoid remains in its current state unless controlled by Blynk
void maintainSolenoidState() {
    if (!isLocked) {
        digitalWrite(SOLENOID_IN1, LOW);
        digitalWrite(SOLENOID_IN2, LOW);  // Maintain unlocked state
    }
}

// Check force sensor and send notification if force is detected
void checkForceSensor() {
    int forceValue = analogRead(FORCE_SENSOR_PIN);
    if (forceValue > forceSensorThreshold && !forceDetected) {
        forceDetected = true;
        Blynk.logEvent("force_detected", "Force detected on door!");
    } else if (forceValue <= forceSensorThreshold) {
        forceDetected = false;
    }
}

// Spray and Siren control via Blynk
BLYNK_WRITE(VPIN_SIREN_SPRAY) {
    int value = param.asInt();
    if (value == 1) {
        activatePumpAndSiren();
    } else {
        deactivatePumpAndSiren();
    }
}

// Trigger spray and siren
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
