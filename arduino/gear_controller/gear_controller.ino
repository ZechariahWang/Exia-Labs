#include <Servo.h>

#define GEAR_SERVO_PIN 46
#define LED_PIN 21

#define GEAR_REVERSE_POS 57
#define GEAR_NEUTRAL_POS 80
#define GEAR_HIGH_POS 110

#define MAX_GEAR_RATE 100.0f
#define NUM_GEARS 3
#define HEARTBEAT_INTERVAL_MS 500
#define JETSON_TIMEOUT_MS 2000

const int GEAR_POSITIONS[NUM_GEARS] = {
    GEAR_REVERSE_POS, GEAR_NEUTRAL_POS, GEAR_HIGH_POS
};

Servo gearServo;

int currentGear = 1;
float currentServoPos = GEAR_NEUTRAL_POS;
int lastServoOutput = GEAR_NEUTRAL_POS;

unsigned long lastJetsonRxTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastLoopTime = 0;
bool jetsonConnected = false;

char serialBuffer[32];
int serialBufIndex = 0;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    gearServo.attach(GEAR_SERVO_PIN, 1000, 2000);
    gearServo.write(GEAR_NEUTRAL_POS);

    Serial.begin(115200);
    delay(500);

    lastLoopTime = millis();
    lastJetsonRxTime = millis();
    lastHeartbeatTime = millis();

    Serial.println("GEAR_READY");
}

void processSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        lastJetsonRxTime = millis();
        jetsonConnected = true;

        if (c == '\n' || c == '\r') {
            if (serialBufIndex > 0) {
                serialBuffer[serialBufIndex] = '\0';
                parseCommand(serialBuffer);
                serialBufIndex = 0;
            }
        } else if (serialBufIndex < 31) {
            serialBuffer[serialBufIndex++] = c;
        }
    }
}

void parseCommand(const char* msg) {
    if (msg[0] == 'K') {
        int gear = atoi(msg + 1);
        if (gear >= 0 && gear < NUM_GEARS) {
            currentGear = gear;
            Serial.print("GA");
            Serial.println(currentGear);
        }
    }
}

void updateServo(float dt) {
    float targetPos = (float)GEAR_POSITIONS[currentGear];
    float maxChange = MAX_GEAR_RATE * dt;
    float diff = targetPos - currentServoPos;

    if (diff > maxChange) {
        currentServoPos += maxChange;
    } else if (diff < -maxChange) {
        currentServoPos -= maxChange;
    } else {
        currentServoPos = targetPos;
    }

    int newOutput = (int)(currentServoPos + 0.5f);
    if (newOutput != lastServoOutput) {
        gearServo.write(newOutput);
        lastServoOutput = newOutput;
    }
}

void sendHeartbeat() {
    Serial.print("GH");
    Serial.println(currentGear);
    lastHeartbeatTime = millis();
}

void safeState() {
    currentGear = 1;
    currentServoPos = GEAR_NEUTRAL_POS;
    gearServo.write(GEAR_NEUTRAL_POS);
    lastServoOutput = GEAR_NEUTRAL_POS;
    jetsonConnected = false;
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastLoopTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    lastLoopTime = now;

    processSerial();

    if (jetsonConnected && (now - lastJetsonRxTime > JETSON_TIMEOUT_MS)) {
        safeState();
        Serial.println("GE");
    }

    updateServo(dt);

    if (jetsonConnected) {
        digitalWrite(LED_PIN, HIGH);
    }

    if (now - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
        sendHeartbeat();
    }

    delay(10);
}
