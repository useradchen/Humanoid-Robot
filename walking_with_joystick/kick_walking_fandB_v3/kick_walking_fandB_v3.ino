#include <DynamixelWorkbench.h>
#include <math.h>
#include "robotController.h"

float stepLength = 0.0f;
float rotationAngle = 0.0f;
float sideStepLength = 0.0f;
bool isWalking = false;

void setup() {
    Serial.begin(1000000);
    delay(2000);

    Serial.println("=== Program Start ===");
    if (!initializePosition()) {
        Serial.println("Initial position setting failed!");
        while(1);
    }
    
    Serial.println("Ready for commands:");
    Serial.println("Input: run,stepLength,rotationAngle,sideStepLength");
    Serial.println("stepLength(>0 forward, <0 backward), rotationAngle(<0 turn right, >0 turn left), sideStepLength(>0 right, <0 left)");
}

void loop() {
    static float stepLength = 0.0f;
    static float rotationAngle = 0.0f;
    static float sideStepLength = 0.0f;
    static bool isWalking = false;
    static bool isKicking = false;
    static bool isLeftLeg = true;

    if (Serial.available() > 32) {
        while (Serial.available()) {
            Serial.read();
        }
    }

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        Serial.println("command: " + input);

        if (input == "q") {
            isWalking = false;
            isKicking = false;
            isLeftLeg = true;
            stepLength = 0.0f;
            rotationAngle = 0.0f;
            sideStepLength = 0.0f;
            Serial.println("command (q)");
            if (!moveToCoordinate(0, 0, initialHeight, 0, true, true, 0.0f)) {
                Serial.println("Left leg reset failed");
            } else {
                Serial.println("Left leg reset successful");
            }
            if (!moveToCoordinate(0, 0, initialHeight, 0, false, true, 0.0f)) {
                Serial.println("Right leg reset failed");
            } else {
                Serial.println("Right leg reset successful");
            }
            Serial.println("Stop");
            while (Serial.available()) {
                Serial.read();
            }
        }
        else if (input == "initialize") {
            isWalking = false;
            isKicking = false;
            isLeftLeg = true;
            stepLength = 0.0f;
            rotationAngle = 0.0f;
            sideStepLength = 0.0f;
            if (!initializePosition()) {
                Serial.println("Initialization failed");
            } else {
                Serial.println("Initialization successful");
            }
            while (Serial.available()) {
                Serial.read();
            }
        }
        else if (input.startsWith("run,")) {
            int firstComma = input.indexOf(',', 4);
            int secondComma = input.indexOf(',', firstComma + 1);
            if (firstComma == -1 || secondComma == -1) {
                Serial.println("error : run,stepLength,rotationAngle,sideStepLength");
                return;
            }
            float newStepLength = input.substring(4, firstComma).toFloat();
            float newRotationAngle = input.substring(firstComma + 1, secondComma).toFloat();
            float newSideStepLength = input.substring(secondComma + 1).toFloat();
            // update
            stepLength = 0.8 * newStepLength + 0.2 * stepLength;
            rotationAngle = 0.8 * newRotationAngle + 0.2 * rotationAngle;
            sideStepLength = 0.8 * newSideStepLength + 0.2 * sideStepLength;
            Serial.println("Start...");
            isWalking = true;
            isKicking = false;
            isLeftLeg = true;
        }
        else if (input.startsWith("kick,")) {
            int firstComma = input.indexOf(',', 5);
            int secondComma = input.indexOf(',', firstComma + 1);
            if (firstComma == -1 || secondComma == -1) {
                Serial.println("error : kick,stepLength,rotationAngle,sideStepLength");
                return;
            }
            stepLength = input.substring(5, firstComma).toFloat();
            rotationAngle = input.substring(firstComma + 1, secondComma).toFloat();
            sideStepLength = input.substring(secondComma + 1).toFloat();
            Serial.println("Start kick...");
            isWalking = false;
            isKicking = true;
            isLeftLeg = true;
        }
    }

    if (isWalking && !isKicking) {
        if (!walking(stepLength, rotationAngle, sideStepLength)) {
            isWalking = false;
            isLeftLeg = true;
            Serial.println("stop walking");
        }
    }
    else if (isKicking) {
        if (!kick(stepLength, rotationAngle, sideStepLength)) {
            Serial.println("kick failed");
        }
        isKicking = false;
    }
}