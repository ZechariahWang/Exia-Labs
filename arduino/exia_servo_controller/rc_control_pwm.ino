#include <Servo.h>

#define CH1_PIN 8
#define CH2_PIN 9
#define THROTTLE_PIN 12
#define BRAKE_PIN 13
#define LED_PIN 4

#define RC_CENTER 1500
#define RC_DEADZONE 60
#define RC_MIN 900
#define RC_MAX 2100

#define THROTTLE_NEUTRAL 90
#define THROTTLE_MAX 150
#define BRAKE_NEUTRAL 90
#define BRAKE_MAX 150

#define HEARTBEAT_MS 100
#define RC_TIMEOUT_MS 500
#define SERIAL_TIMEOUT_MS 500
#define STEERING_CHANGE_THRESHOLD 10

Servo throttle;
Servo brake;

unsigned long lastHeartbeatTime = 0;
unsigned long lastValidRCTime = 0;
unsigned long lastSerialRxTime = 0;
int lastSteeringValue = 0;
bool rcLost = false;
bool jetsonConnected = false;

int currentState = 0;
int odriveError = 0;

char serialBuffer[64];
int bufferIndex = 0;

void setup() {
    pinMode(CH1_PIN, INPUT);
    pinMode(CH2_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);

    throttle.attach(THROTTLE_PIN);
    brake.attach(BRAKE_PIN);

    throttle.write(THROTTLE_NEUTRAL);
    brake.write(BRAKE_NEUTRAL);

    Serial.begin(115200);
    delay(100);

    lastValidRCTime = millis();
    lastHeartbeatTime = millis();
    lastSerialRxTime = millis();

    digitalWrite(LED_PIN, LOW);
    Serial.println("READY");
}

void sendSteering(int value) {
    value = constrain(value, -1000, 1000);
    Serial.print("S");
    Serial.println(value);
    lastHeartbeatTime = millis();
}

void sendHeartbeat() {
    Serial.println("H");
    lastHeartbeatTime = millis();
}

void sendRCLost() {
    Serial.println("L");
}

void sendRCRecovered() {
    Serial.println("R");
}

void sendEmergencyStop() {
    Serial.println("E");
}

void processSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        lastSerialRxTime = millis();

        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                serialBuffer[bufferIndex] = '\0';
                parseJetsonMessage(serialBuffer);
                bufferIndex = 0;
            }
        } else if (bufferIndex < 63) {
            serialBuffer[bufferIndex++] = c;
        }
    }
}

void parseJetsonMessage(const char* msg) {
    if (msg[0] == 'F') {
        jetsonConnected = true;
    }
    else if (msg[0] == 'Z') {
        currentState = atoi(msg + 1);
        jetsonConnected = true;
        if (currentState == 3) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
    }
    else if (msg[0] == 'A') {
        jetsonConnected = true;
    }
}

int readRCChannel(int pin) {
    int pulse = pulseIn(pin, HIGH, 25000);
    if (pulse == 0) {
        return -1;
    }
    return pulse;
}

bool isValidRC(int pulse) {
    return (pulse >= RC_MIN && pulse <= RC_MAX);
}

int rcToSteering(int pulse) {
    int centered = pulse - RC_CENTER;
    if (abs(centered) < RC_DEADZONE) {
        return 0;
    }
    int value = map(pulse, 1000, 2000, -1000, 1000);
    return constrain(value, -1000, 1000);
}

void handleThrottleBrake(int ch2) {
    if (!isValidRC(ch2)) {
        throttle.write(THROTTLE_NEUTRAL);
        brake.write(BRAKE_NEUTRAL);
        return;
    }

    int delta = ch2 - RC_CENTER;

    if (abs(delta) < RC_DEADZONE) {
        throttle.write(THROTTLE_NEUTRAL);
        brake.write(BRAKE_NEUTRAL);
    }
    else if (delta > 0) {
        int t = map(delta, RC_DEADZONE, 500, THROTTLE_NEUTRAL, THROTTLE_MAX);
        throttle.write(constrain(t, THROTTLE_NEUTRAL, THROTTLE_MAX));
        brake.write(BRAKE_NEUTRAL);
    }
    else {
        int b = map(-delta, RC_DEADZONE, 500, BRAKE_NEUTRAL, BRAKE_MAX);
        brake.write(constrain(b, BRAKE_NEUTRAL, BRAKE_MAX));
        throttle.write(THROTTLE_NEUTRAL);
    }
}

void safeState() {
    throttle.write(THROTTLE_NEUTRAL);
    brake.write(BRAKE_MAX);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    unsigned long now = millis();

    processSerialInput();

    int ch1 = readRCChannel(CH1_PIN);
    int ch2 = readRCChannel(CH2_PIN);

    bool ch1Valid = isValidRC(ch1);
    bool ch2Valid = isValidRC(ch2);

    if (ch1Valid || ch2Valid) {
        lastValidRCTime = now;

        if (rcLost) {
            rcLost = false;
            sendRCRecovered();
        }
    }

    if (now - lastValidRCTime > RC_TIMEOUT_MS) {
        if (!rcLost) {
            rcLost = true;
            sendRCLost();
            safeState();
        }
        sendSteering(0);
        delay(20);
        return;
    }

    if (ch1Valid) {
        int steeringValue = rcToSteering(ch1);

        if (abs(steeringValue - lastSteeringValue) > STEERING_CHANGE_THRESHOLD) {
            sendSteering(steeringValue);
            lastSteeringValue = steeringValue;
        }
    }

    if (now - lastHeartbeatTime > HEARTBEAT_MS) {
        sendHeartbeat();
    }

    handleThrottleBrake(ch2);

    delay(20);
}
