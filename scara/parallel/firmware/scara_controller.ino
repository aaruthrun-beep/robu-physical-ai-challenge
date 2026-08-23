/*
 * Parallel SCARA Controller
 *
 * 2-link SCARA arm with harmonic drive motors on CAN bus
 * L1 = 300mm, L2 = 390mm
 *
 * Commands: GO <x> <y>     — move to position (mm)
 *           JOINT <j1> <j2> — move joints (degrees)
 *           STATUS           — current position
 *           HOME             — return to home
 */
#include <SPI.h>
#include <mcp_can.h>

// CAN bus
#define CAN_CS_PIN  10
#define MOTOR_L1_ID 0x10
#define MOTOR_L2_ID 0x11

MCP_CAN CAN(CAN_CS_PIN);

// SCARA geometry (mm)
#define L1 300.0
#define L2 390.0

float current_j1 = 0.0;
float current_j2 = 0.0;

void setup() {
    Serial.begin(115200);
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        CAN.setMode(MCP_NORMAL);
        Serial.println("CAN OK");
    } else {
        Serial.println("CAN FAIL");
    }
    home();
    Serial.println("SCARA READY");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        handleCommand(cmd);
    }
}

void handleCommand(String cmd) {
    if (cmd.startsWith("GO ")) {
        float x = cmd.substring(3, cmd.indexOf(' ')).toFloat();
        float y = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
        float j1, j2;
        if (inverseKinematics(x, y, j1, j2)) {
            sendPosition(MOTOR_L1_ID, j1);
            sendPosition(MOTOR_L2_ID, j2);
            current_j1 = j1;
            current_j2 = j2;
            Serial.println("OK " + String(j1, 1) + " " + String(j2, 1));
        } else {
            Serial.println("ERR UNREACHABLE");
        }
    }
    else if (cmd.startsWith("JOINT ")) {
        float j1 = cmd.substring(6, cmd.indexOf(' ')).toFloat();
        float j2 = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
        sendPosition(MOTOR_L1_ID, j1);
        sendPosition(MOTOR_L2_ID, j2);
        current_j1 = j1;
        current_j2 = j2;
        Serial.println("OK");
    }
    else if (cmd == "STATUS") {
        float x, y;
        forwardKinematics(current_j1, current_j2, x, y);
        Serial.println("J1=" + String(current_j1, 1) +
            " J2=" + String(current_j2, 1) +
            " X=" + String(x, 1) +
            " Y=" + String(y, 1));
    }
    else if (cmd == "HOME") {
        home();
        Serial.println("HOME");
    }
}

void home() {
    sendPosition(MOTOR_L1_ID, 0.0);
    sendPosition(MOTOR_L2_ID, 0.0);
    current_j1 = 0.0;
    current_j2 = 0.0;
}

void forwardKinematics(float j1_deg, float j2_deg, float &x, float &y) {
    float t1 = j1_deg * PI / 180.0;
    float t2 = j2_deg * PI / 180.0;
    x = L1 * cos(t1) + L2 * cos(t1 + t2);
    y = L1 * sin(t1) + L2 * sin(t1 + t2);
}

bool inverseKinematics(float x, float y, float &j1, float &j2) {
    float d = sqrt(x * x + y * y);
    if (d > (L1 + L2) || d < fabs(L1 - L2)) return false;
    float cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cos_q2 = constrain(cos_q2, -1.0, 1.0);
    float q2 = acos(cos_q2);
    float k1 = L1 + L2 * cos(q2);
    float k2 = L2 * sin(q2);
    float q1 = atan2(y, x) - atan2(k2, k1);
    j1 = q1 * 180.0 / PI;
    j2 = q2 * 180.0 / PI;
    return true;
}

void sendPosition(uint16_t can_id, float angle_deg) {
    byte data[8] = {0};
    memcpy(data, &angle_deg, 4);
    CAN.sendMsgBuf(can_id, 0, 8, data);
}
