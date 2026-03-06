#include <Servo.h>

#define GEAR_SERVO_PIN 2
#define LED_PIN 21

#define GEAR_REVERSE_POS 0
#define GEAR_NEUTRAL_POS 57
#define GEAR_LOW_POS 90
#define GEAR_HIGH_POS 130

#define NUM_GEARS 4
#define HEARTBEAT_INTERVAL_MS 500

const int GEAR_POSITIONS[NUM_GEARS] = {
    GEAR_REVERSE_POS, GEAR_NEUTRAL_POS, GEAR_LOW_POS, GEAR_HIGH_POS
};

Servo gearServo;

int currentGear = 1;
int lastServoOutput = GEAR_NEUTRAL_POS;

unsigned long lastHeartbeatTime = 0;
unsigned long lastLoopTime = 0;
bool jetsonConnected = false;

char serialBuffer[32];
int serialBufIndex = 0;

void parseCommand(const char* msg);

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    gearServo.attach(GEAR_SERVO_PIN, 1000, 2000);
    gearServo.write(GEAR_NEUTRAL_POS);

    Serial.begin(115200);
    delay(500);

    lastLoopTime = millis();
    lastHeartbeatTime = millis();

    Serial.println("GEAR_READY");
}

void processSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
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
            int targetPos = GEAR_POSITIONS[currentGear];
            if (targetPos < lastServoOutput) {
                gearServo.write(max(targetPos - 15, 0));
                delay(300);
            }
            gearServo.write(targetPos);
            lastServoOutput = targetPos;
            Serial.print("GA");
            Serial.println(currentGear);
        }
    }
}

void sendHeartbeat() {
    Serial.print("GH");
    Serial.println(currentGear);
    lastHeartbeatTime = millis();
}

void loop() {
    unsigned long now = millis();
    lastLoopTime = now;

    processSerial();

    if (jetsonConnected) {
        digitalWrite(LED_PIN, HIGH);
    }

    if (now - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
        sendHeartbeat();
    }

    delay(10);
}
