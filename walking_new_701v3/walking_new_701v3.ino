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

String inputString = "";         // 用來儲存接收到的字串
bool stringComplete = false;     // 字串是否接收完成的標誌
bool isWalking = false;         // 標記是否正在步行
float inputX = 0, inputY = 0, inputZ = 0; // 當前步行參數
// int currentStep = 0;           

void loop() {
    // 讀取串口輸入
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        }
    }

    // 處理接收到的輸入
    if (stringComplete) {
        inputString.trim(); // 移除首尾空白

        // 檢查是否為退出指令 'q'
        if (inputString.equalsIgnoreCase("q")) {
            Serial.println("stop");
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

                // 將字串轉換為浮點數
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

    // 執行步行邏輯
    if (isWalking) {
        walking(inputX, inputY, inputZ);
    }
}