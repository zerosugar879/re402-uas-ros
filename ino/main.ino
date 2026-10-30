#include <WiFi.h>
#include <WiFiUdp.h>

// Networking
const char* ssid = "WifiNameXX";
const char* password = "PasswordXX";

// WiFi credentials
WiFiUDP udp;


const int localPort = 8888;
const int remotePort = 8889;

// UDP ports
char packetBuffer[256];
String remoteIp;

// Remote address
// GY-25 IMU
#define IMU_UART_NUM  2
#define IMU_BAUD      115200
#define IMU_RX_PIN    16
#define IMU_TX_PIN    17

float imuYaw   = 0.0f;
float imuPitch = 0.0f;
float imuRoll  = 0.0f;
bool  imuValid = false;

// IMU state
// Motor driver pins
#define ENA 25
#define IN1 26
#define IN2 27

#define IN3 14
#define IN4 12
#define ENB 13

#define PWM_FREQ      5000
#define PWM_RES       8
#define PWM_LEFT_CH   0
#define PWM_RIGHT_CH  1

// PWM channels
// Motor PWM limits 
int pwmMin = 0;
int pwmMax = 0;
int pwmTurn = 0;
int pwmStraight = 0;
bool configReceived = false;

// Config flags
// Robot parameters (configurable via UDP)
float wheelBase   = 0.20f;
float wheelRadius = 0.05f;
float maxWheelVel = 6.0f;

// Robot geometry

// Init motor pins
void motorSetup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcAttachChannel(ENA, PWM_FREQ, PWM_RES, PWM_LEFT_CH);
    ledcAttachChannel(ENB, PWM_FREQ, PWM_RES, PWM_RIGHT_CH);
}


// Set motor direction + PWM, stop if no config
void setMotor(int speedL, int speedR) {
    // Config check
    if (!configReceived) {
        ledcWrite(ENA, 0);
        ledcWrite(ENB, 0);
        return;
    }
    // Clamp speeds
    speedL = constrain(speedL, -pwmMax, pwmMax);
    speedR = constrain(speedR, -pwmMax, pwmMax);

    // Both forward
    if (speedL > 0 && speedR > 0) {
        // Forward
        int mn = min(speedL, speedR);

        // Boost min
        if (mn < pwmMin) {
            int boost = pwmMin - mn;
            speedL = min(pwmMax, speedL + boost);
            speedR = min(pwmMax, speedR + boost);
        }

    // Both reverse
    } else if (speedL < 0 && speedR < 0) {
        // Backward
        int mx = max(speedL, speedR);

        // Boost min
        if (mx > -pwmMin) {
            int boost = -pwmMin - mx;
            speedL = max(-pwmMax, speedL - boost);
            speedR = max(-pwmMax, speedR - boost);
        }

    // Single turn
    } else {
        // 1-wheel turn
        if (speedL > 0)
            speedL = pwmTurn;
        if (speedR > 0)
            speedR = pwmTurn;
    }

    // Left Motor
    if (speedL >= 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(ENA, speedL);
    } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(ENA, -speedL);
    }

    // Right Motor
    if (speedR >= 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(ENB, speedR);
    } else {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(ENB, -speedR);
    }
}


// Convert wheel velocity to PWM value
int velToPwm(float vel, bool useTurnPwm = false) {
    // Near zero
    if (abs(vel) < 0.01f) return 0;
    int maxPwm = useTurnPwm ? pwmTurn : pwmStraight;
    if (maxPwm < 1) maxPwm = 1;
    int pwm = (int)((abs(vel) / maxWheelVel) * maxPwm);
    return (vel >= 0 ? 1 : -1) * constrain(pwm, 0, maxPwm);
}


int lastPwmL = 0, lastPwmR = 0;

// Convert twist (vx, wz) to left/right wheel PWM
void processTwist(float vx, float wz) {
    // Differential drive inverse kinematics
    float left_vel  = (vx - wz * wheelBase / 2.0f) / wheelRadius;
    float right_vel = (vx + wz * wheelBase / 2.0f) / wheelRadius;

    left_vel  = constrain(left_vel,  -maxWheelVel, maxWheelVel);
    right_vel = constrain(right_vel, -maxWheelVel, maxWheelVel);

    bool isTurn = (abs(wz) > 0.05f);
    lastPwmL = velToPwm(left_vel, isTurn);
    lastPwmR = velToPwm(right_vel, isTurn);

    setMotor(lastPwmL, lastPwmR);

    Serial.print("[Twist] vx="); Serial.print(vx, 3);
    Serial.print(" wz="); Serial.print(wz, 3);
    Serial.print(" -> PWM L="); Serial.print(lastPwmL);
    Serial.print(" R="); Serial.println(lastPwmR);
}


// Save received config
void applyConfig(int min, int max, int turn, int straight,
                 float wBase, float wRadius, float maxWVel) {
    pwmMin = min;
    pwmMax = max;
    pwmTurn = turn;
    pwmStraight = straight;
    wheelBase = wBase;
    wheelRadius = wRadius;
    maxWheelVel = maxWVel;
    configReceived = true;
    Serial.print("[CFG] PWM: min="); Serial.print(pwmMin);
    Serial.print(" max="); Serial.print(pwmMax);
    Serial.print(" turn="); Serial.print(pwmTurn);
    Serial.print(" straight="); Serial.print(pwmStraight);
    Serial.print(" wBase="); Serial.print(wheelBase, 3);
    Serial.print(" wRadius="); Serial.print(wheelRadius, 3);
    Serial.print(" maxWVel="); Serial.println(maxWheelVel, 3);
}


// WiFi init
void wifiSetup() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");

    int attempts = 0;

    // Retry connect
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    Serial.println();

    // Check status
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected! IP : ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Failed to connect.");
    }

    udp.begin(localPort);

    Serial.print("UDP started on port ");
    Serial.println(localPort);
}


// GY-25 init
void imuSetup() {
    Serial2.begin(IMU_BAUD, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    Serial.println("[GY25] Serial2 started");
}

// Parse GY-25 IMU serial frame -> yaw/pitch/roll
void parseGy25() {
    static uint8_t buf[32];
    static uint8_t idx = 0;
    static bool inFrame = false;

    // Read bytes
    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        // Find header
        if (!inFrame) {
            if (b == 0xAA) {
                buf[0] = b;
                idx = 1;
                inFrame = true;
            }
            continue;
        }

        buf[idx++] = b;

        // Bad header
        if (idx == 2 && buf[1] != 0x55) {
            inFrame = false;
            idx = 0;
            continue;
        }

        // Length check
        if (idx == 3) {
            // Byte 2 = expected frame length (excluding header)
            // If > 24, invalid
            if (buf[2] > 24 || buf[2] < 4) {
                inFrame = false;
                idx = 0;
            }
            continue;
        }

        // Wait for full frame: header(2) + length + data + checksum
        // buf[2] is the number of bytes after the length byte
        uint8_t frameLen = buf[2] + 3; // total (AA 55 + len + data + sum)
        // Frame complete
        if (idx >= frameLen) {
            // Validate checksum: sum of all bytes except last = last byte
            uint8_t sum = 0;
            for (uint8_t i = 0; i < frameLen - 1; i++) sum += buf[i];
            // Checksum OK
            if (sum == buf[frameLen - 1]) {
                uint8_t type = buf[3];
                // Angle data
                if (type == 0x61 && frameLen >= 11) {
                    // Angle output: bytes 4-9 are Yaw, Pitch, Roll (int16 LE, scale 1/100)
                    int16_t raw_yaw   = (int16_t)(buf[4] | (buf[5] << 8));
                    int16_t raw_pitch = (int16_t)(buf[6] | (buf[7] << 8));
                    int16_t raw_roll  = (int16_t)(buf[8] | (buf[9] << 8));

                    imuYaw   = raw_yaw   / 100.0f;
                    imuPitch = raw_pitch / 100.0f;
                    imuRoll  = raw_roll  / 100.0f;
                    imuValid = true;

                    if (imuYaw < 0) imuYaw += 360.0f;
                }
            }
            inFrame = false;
            idx = 0;
        }

        // Buffer safe
        if (idx >= sizeof(buf)) {
            inFrame = false;
            idx = 0;
        }
    }
}

void setup() {
    Serial.begin(115200);

    motorSetup();
    wifiSetup();
    imuSetup();

    Serial.println("ESP32 Ready");
}


// Main loop
void loop() {
    parseGy25();

    int packetSize = udp.parsePacket();

    // Packet received
    if (packetSize) {
        int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);

        // Null terminate
        if (len > 0)
            packetBuffer[len] = '\0';

        remoteIp = udp.remoteIP().toString();

        udp.beginPacket(udp.remoteIP(), udp.remotePort());

        String data = String(packetBuffer);
        data.trim();

        // Config msg
        if (data.startsWith("C:")) {
            // Config message: C:min,max,turn,straight,wBase,wRadius,maxWVel
            String vals = data.substring(2);
            int c1 = vals.indexOf(',');
            int c2 = vals.indexOf(',', c1 + 1);
            int c3 = vals.indexOf(',', c2 + 1);
            int c4 = vals.indexOf(',', c3 + 1);
            int c5 = vals.indexOf(',', c4 + 1);
            int c6 = vals.indexOf(',', c5 + 1);
            // Parse CSV
            if (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0 && c5 > 0 && c6 > 0) {
                int min = vals.substring(0, c1).toInt();
                int max = vals.substring(c1 + 1, c2).toInt();
                int turn = vals.substring(c2 + 1, c3).toInt();
                int straight = vals.substring(c3 + 1, c4).toInt();
                float wBase = vals.substring(c4 + 1, c5).toFloat();
                float wRadius = vals.substring(c5 + 1, c6).toFloat();
                float maxWVel = vals.substring(c6 + 1).toFloat();
                applyConfig(min, max, turn, straight, wBase, wRadius, maxWVel);
            }
            udp.print("OK\n");
            udp.endPacket();
            return;
        }

        int comma = data.indexOf(',');

        // Twist cmd
        if (comma > 0) {
            float vx = data.substring(0, comma).toFloat();
            float wz = data.substring(comma + 1).toFloat();

            processTwist(vx, wz);
        }

        // Response: pwmL,pwmR,yawDeg
        String resp = String(lastPwmL) + "," + String(lastPwmR);
        // Append yaw
        if (imuValid) {
            resp += "," + String(imuYaw, 2);
        }

        udp.print(resp);
        udp.endPacket();
    }
}
