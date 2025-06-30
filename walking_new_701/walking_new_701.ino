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
float currentX = 0, currentY = 0, currentZ = 0; // 當前步行參數
int currentStep = 0;           // 當前步行步數 (0: 未開始, 1: 右腳第一步, 2: 右腳-左腳循環)

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
            Serial.println("程式結束。");
            isWalking = false;
            currentStep = 0;
            inputString = "";
            stringComplete = false;
            while (Serial.available()) {
                Serial.read();
            }
            Serial.println("\n請輸入 x,y,z (用逗號分隔) 或輸入 q 離開:");
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
                currentX = x_str.toFloat();
                currentY = y_str.toFloat();
                currentZ = z_str.toFloat();

                Serial.print("接收到: x=");
                Serial.print(currentX);
                Serial.print(", y=");
                Serial.print(currentY);
                Serial.print(", z=");
                Serial.println(currentZ);

                // 開始或更新步行
                isWalking = true;
                currentStep = 1; // 從第一步開始
            } else {
                Serial.println("輸入格式錯誤。請輸入 x,y,z (用逗號分隔)。");
            }
        } else {
            Serial.println("輸入格式錯誤。請輸入 x,y,z (用逗號分隔)。");
        }

        // 重置輸入
        inputString = "";
        stringComplete = false;
        Serial.println("\n請輸入 x,y,z (用逗號分隔) 或輸入 q 離開:");
    }

    // 執行步行邏輯
    if (isWalking) {
        walking(currentX, currentY, currentZ);
    }
}