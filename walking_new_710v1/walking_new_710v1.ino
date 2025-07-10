#include <DynamixelWorkbench.h>
#include <math.h>
#include "robotController.h"

void setup() {
    delay(200);
    Serial.begin(115200);
    delay(200);

    Serial.println("=== Program Start ===");
    Serial.println("Enter x y z (ex 5 5 5) or 'q' to stop.");
    delay(200);

    if (!initializePosition()) {
        Serial.println("Initial position setting failed!");
        // while(1);
    }
    while(Serial.available()){
    Serial.read();
  }
}

String inputString = "";         
bool stringComplete = false;     
bool isWalking = false;         
float inputX = 0, inputY = 0, inputZ = 0;

void loop() {
  // moveToPos(5, 5, 5);
  // moveToPos(5, -5, 5);
  // delay(10000);
    // // 讀取串口輸入
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
  
    if (stringComplete) {
        inputString.trim();

        // 'q' for stop
        if (inputString.equalsIgnoreCase("q")) {
            Serial.println("stop");
            walking(inputX, inputY, inputZ, true);
            isWalking = false;
            // currentStep = 0;
            inputString = "";
            stringComplete = false;
            while (Serial.available()) {
                Serial.read();
            }
            Serial.println("\nerror Enter x y z (ex 5 5 5) or 'q' to stop.");
            return;
        }

        // 解析 x,y,z
        int comma1 = inputString.indexOf(',');
        if (comma1 != -1) {
            int comma2 = inputString.indexOf(',', comma1 + 1);
            if (comma2 != -1) {
                String x_str = inputString.substring(0, comma1);
                String y_str = inputString.substring(comma1 + 1, comma2);
                String z_str = inputString.substring(comma2 + 1);

                // string -> float
                inputX = x_str.toFloat();
                inputY = y_str.toFloat();
                inputZ = z_str.toFloat();

                Serial.print("x=");
                Serial.print(inputX);
                Serial.print(", y=");
                Serial.print(inputY);
                Serial.print(", z=");
                Serial.println(inputZ);

                // 開始或更新步行
                isWalking = true;
                // currentStep = 1; // 從第一步開始
            } else {
                Serial.println("\nerror Enter x y z (ex 5 5 5) or 'q' to stop.");
            }
        } else {
            Serial.println("\nerror Enter x y z (ex 5 5 5) or 'q' to stop.");
        }
        // 重置輸入
        inputString = "";
        stringComplete = false;
        Serial.println("\nerror Enter x y z (ex 5 5 5) or 'q' to stop.");

    }
    // moveToPos(inputX, inputY, inputZ); // for 20,20,20
    // 執行步行邏輯
    if (isWalking) {
        walking(inputX, inputY, inputZ,false);
    }
}