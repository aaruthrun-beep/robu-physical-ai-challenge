/*
 * XM9X6 Heads of Heart — Leg Controller
 *
 * Each leg has 3 DOF: coxa, femur, tibia
 * Controlled via serial bus from coordinator
 *
 * Protocol: L<leg> J<joint> A<angle>
 *           STATUS
 *           HOME
 */
#include <Servo.h>

#define LEG_ID    1       // Set per leg (1-6)
#define COXA_PIN  9
#define FEMUR_PIN 10
#define TIBIA_PIN 11

Servo coxa, femur, tibia;

// Home positions (degrees)
int home_coxa  = 90;
int home_femur = 90;
int home_tibia = 90;

void setup() {
    Serial.begin(115200);
    coxa.attach(COA_PIN);
    femur.attach(FEMUR_PIN);
    tibia.attach(TIBIA_PIN);
    home();
    Serial.println("LEG" + String(LEG_ID) + " READY");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        handleCommand(cmd);
    }
}

void handleCommand(String cmd) {
    if (cmd == "STATUS") {
        Serial.println("LEG" + String(LEG_ID) +
            " C=" + String(coxa.read()) +
            " F=" + String(femur.read()) +
            " T=" + String(tibia.read()));
    }
    else if (cmd == "HOME") {
        home();
        Serial.println("LEG" + String(LEG_ID) + " HOME");
    }
    else if (cmd.startsWith("L")) {
        // Parse: L<leg> J<joint> A<angle>
        int leg = cmd.substring(1, cmd.indexOf('J')).toInt();
        if (leg != LEG_ID) return;
        int joint = cmd.substring(cmd.indexOf('J') + 1, cmd.indexOf('A')).toInt();
        int angle = cmd.substring(cmd.indexOf('A') + 1).toInt();
        angle = constrain(angle, 0, 180);
        switch (joint) {
            case 1: coxa.write(angle); break;
            case 2: femur.write(angle); break;
            case 3: tibia.write(angle); break;
        }
        Serial.println("OK");
    }
}

void home() {
    coxa.write(home_coxa);
    femur.write(home_femur);
    tibia.write(home_tibia);
}
