#include <Arduino.h>
#include "dynamixel_setup.h"
// #include "Walkinggait.h"
#define DBG SerialUsb

// int state;

// String imu_data = "";
bool receiving = false;

void setup() {
  Serial.begin(115200);
  // Serial2.begin(115200);
  // Serial4.begin(115200);
  while (!Serial);

  // Dynamixel init
  dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  setupControl.init(portHandler, packetHandler);

  Serial.println("Start..");
  if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Failed to initialize port.");
    return;
  }
  Serial.println("Port initialized!");

  setupControl.motor_address();
  delay(10);
  setupControl.motor_torque();
  delay(10);
  setupControl.Inverse_kinematic(0, 0, 50, 0, 0); // (x,y,z,theta,RL)
  setupControl.Inverse_kinematic(0, 0, 50, 0, 6);
  setupControl.rad2motor(0);
  setupControl.rad2motor(6);
  for (int i = 0; i < 12; ++i) setupControl.origin_position[i] = setupControl.motor_position[i];
  setupControl.walking_groupSyncWrite(setupControl.motor_position, setupControl.motor_position);

}


String inputString = "";
bool stringComplete = false;
bool isActive = false;
bool moveExecuted = false;
float inputX = 0, inputY = 0, inputZ = 0, inputTheta = 0;
enum ControlMode { MOVE_TO_POS, WALKING }; // 定義模式
ControlMode mode = WALKING;

void loop() {
  // 處理串口輸入
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  if (stringComplete) {
    inputString.trim();
    Serial.print("Received input: "); Serial.println(inputString);

    if (inputString.equalsIgnoreCase("q")) {
      Serial.println("Stopping movement");
      // 執行停止動作：回到初始位置
      setupControl.moveToPos(0, 0, 0, 0, 0); // 左腳回到初始位置
      setupControl.moveToPos(0, 0, 0, 0, 6); // 右腳回到初始位置
      isActive = false;
      moveExecuted = false;
      inputString = "";
      stringComplete = false;
      while (Serial.available()) {
        Serial.read(); // 清空緩衝區
      }
      Serial.println("Enter x, y, z, theta (e.g., 5, 5, 5, 5) or 'q' to stop.");
      return;
    }

    // 解析輸入
    int comma1 = inputString.indexOf(',');
    if (comma1 != -1) {
      int comma2 = inputString.indexOf(',', comma1 + 1);
      if (comma2 != -1) {
        int comma3 = inputString.indexOf(',', comma2 + 1);
        if (comma3 != -1) {
          String x_str = inputString.substring(0, comma1);
          String y_str = inputString.substring(comma1 + 1, comma2);
          String z_str = inputString.substring(comma2 + 1, comma3);
          String theta_str = inputString.substring(comma3 + 1);

          inputX = x_str.toFloat();
          inputY = y_str.toFloat();
          inputZ = z_str.toFloat();
          inputTheta = theta_str.toFloat();

          Serial.print("Parsed: x="); Serial.print(inputX);
          Serial.print(", y="); Serial.print(inputY);
          Serial.print(", z="); Serial.print(inputZ);
          Serial.print(", theta="); Serial.println(inputTheta);

          isActive = true;
          moveExecuted = false; // 重置 moveToPos 執行標誌
        } else {
          Serial.println("Error: Invalid input format. Enter x, y, z, theta (e.g., 5, 5, 5, 5) or 'q' to stop.");
        }
      } else {
        Serial.println("Error: Invalid input format. Enter x, y, z, theta (e.g., 5, 5, 5, 5) or 'q' to stop.");
      }
    } else {
      Serial.println("Error: Invalid input format. Enter x, y, z, theta (e.g., 5, 5, 5, 5) or 'q' to stop.");
    }

    inputString = "";
    stringComplete = false;
    while (Serial.available()) {
      Serial.read(); // 清空緩衝區
    }
  }

  // 根據模式執行動作
  if (isActive) {
    if (mode == MOVE_TO_POS) {
      // moveToPos 模式：只執行一次
      if (!moveExecuted) {
        Serial.println("Executing moveToPos...");
        setupControl.moveToPos(inputX, inputY, inputZ, inputTheta, 0); // 左腳
        // setupControl.moveToPos(inputX, inputY, inputZ, inputTheta, 6); // 右腳
        moveExecuted = true;
        Serial.println("Move completed. Waiting for new input or 'q' to stop.");
      }
    } else if (mode == WALKING) {
      // walking 模式：連續步行
      Serial.println("Executing walking...");
      bool finished = setupControl.walking(inputX, inputY, inputZ, inputTheta, false);
      if (finished) {
        Serial.println("Walking finished.");
        isActive = false;
      }
    }
  }
}
