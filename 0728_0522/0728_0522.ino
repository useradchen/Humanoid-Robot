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

  // Serial.println("Start..");
  if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
    // Serial.println("Failed to initialize port.");
    return;
  }
  // Serial.println("Port initialized!");

  setupControl.motor_address();
  delay(10);
  setupControl.motor_torque();
  delay(10);
  setupControl.Inverse_kinematic(0, 0, 50, 0, 0); // (x,y,z,theta,RL)
  setupControl.Inverse_kinematic(0, 0, 50, 0, 6);
  setupControl.rad2motor(0);
  setupControl.rad2motor(6);
  for (int i = 0; i < 12; ++i) {
    setupControl.origin_position[i] = setupControl.motor_position[i];
    setupControl.last_position[i] = setupControl.origin_position[i];
  }
  setupControl.walking_groupSyncWrite(setupControl.motor_position, setupControl.motor_position);
}

String inputString = "";
bool stringComplete = false;
bool isActive = false;
bool moveExecuted = false;
float inputX = 0, inputY = 0, inputZ = 0, inputTheta = 0;
enum ControlMode { MOVE_TO_POS, WALKING }; // 定義模式
ControlMode mode = WALKING;

// void loop() {
//   // 處理串口輸入
//   while (Serial.available() > 0 && !stringComplete) {
//     char inChar = (char)Serial.read();
//     Serial.print("Received char: "); Serial.println(inChar); // 調試：顯示每個接收字符
//     inputString += inChar;
//     if (inChar == '\n') {
//       stringComplete = true;
//       Serial.println("Command received, processing..."); // 回饋 UI
//     }
//   }

//   if (stringComplete) {
//     Serial.print("Full input received: "); Serial.println(inputString); // 調試：顯示完整指令
//     inputString.trim();
//     Serial.print("Received input (trimmed): "); Serial.println(inputString);

//     if (inputString.equalsIgnoreCase("q")) {
//       Serial.println("Stopping movement");
//       setupControl.moveToPos(0, 0, 0, 0, 0, 0); // 左腳回到初始位置
//       setupControl.moveToPos(0, 0, 0, 0, 6, 0); // 右腳回到初始位置
//       isActive = false;
//       moveExecuted = false; // 重置 moveExecuted
//       for (int i = 0; i < 12; i++)
//         setupControl.last_position[i] = setupControl.origin_position[i];
//       Serial.println("Ready for new command.");
//     }
//     else if (inputString.startsWith("set_yz:")) {
//       String params = inputString.substring(7);
//       int comma1 = params.indexOf(',');
//       int comma2 = params.indexOf(',', comma1 + 1);
//       if (comma1 != -1 && comma2 != -1) {
//         String y_str = params.substring(0, comma1);
//         String z_str = params.substring(comma1 + 1, comma2);
//         String steps_str = params.substring(comma2 + 1);
//         float newYPos = y_str.toFloat();
//         float newZPos = z_str.toFloat();
//         int newSteps = steps_str.toInt();

//         setupControl.initialYPos = newYPos;
//         setupControl.initialZPos = newZPos;
//         setupControl.steps = newSteps;

//         Serial.print("Updated initialYPos: "); Serial.println(setupControl.initialYPos);
//         Serial.print("Updated initialZPos: "); Serial.println(setupControl.initialZPos);
//         Serial.print("Updated steps: "); Serial.println(setupControl.steps);
//       } else {
//         Serial.println("Error: Invalid set_yz format. Use set_yz:initialYPos,initialZPos,steps");
//       }
//       moveExecuted = false; // 重置 moveExecuted 允許後續動作
//       Serial.println("Parameters updated, ready for new command.");
//     }
//     else {
//       int comma1 = inputString.indexOf(',');
//       if (comma1 != -1) {
//         int comma2 = inputString.indexOf(',', comma1 + 1);
//         if (comma2 != -1) {
//           int comma3 = inputString.indexOf(',', comma2 + 1);
//           if (comma3 != -1) {
//             String x_str = inputString.substring(0, comma1);
//             String y_str = inputString.substring(comma1 + 1, comma2);
//             String z_str = inputString.substring(comma2 + 1, comma3);
//             String theta_str = inputString.substring(comma3 + 1);

//             inputX = x_str.toFloat();
//             inputY = y_str.toFloat();
//             inputZ = z_str.toFloat();
//             inputTheta = theta_str.toFloat();

//             Serial.print("Parsed: x="); Serial.print(inputX);
//             Serial.print(", y="); Serial.print(inputY);
//             Serial.print(", z="); Serial.print(inputZ);
//             Serial.print(", theta="); Serial.println(inputTheta);

//             isActive = true;
//             moveExecuted = false; // 重置 moveExecuted 允許重複執行
//           } else {
//             Serial.println("Error: Invalid input format. Enter x,y,z,theta (e.g., 5,5,5,5), set_yz:initialYPos,initialZPos,steps, or 'q' to stop.");
//           }
//         } else {
//           Serial.println("Error: Invalid input format. Enter x,y,z,theta (e.g., 5,5,5,5), set_yz:initialYPos,initialZPos,steps, or 'q' to stop.");
//         }
//       } else {
//         Serial.println("Error: Invalid input format. Enter x,y,z,theta (e.g., 5,5,5,5), set_yz:initialYPos,initialZPos,steps, or 'q' to stop.");
//       }
//     }

//     inputString = "";
//     stringComplete = false;
//     delay(100); // 增加延遲至 100ms 確保緩衝區清空
//     while (Serial.available() > 0) {
//       Serial.read(); // 清空緩衝區
//     }
//   }

//   // 根據模式執行動作
//   if (isActive) {
//     Serial.print("isActive: "); Serial.println(isActive);
//     Serial.print("moveExecuted: "); Serial.println(moveExecuted);
//     if (mode == MOVE_TO_POS) {
//       if (!moveExecuted) {
//         Serial.println("Executing moveToPos...");
//         setupControl.moveToPos(inputX, inputY, inputZ, inputTheta, 0, 0); // 左腳
//         moveExecuted = true;
//         Serial.println("Move completed, ready for new command.");
//       }
//     } else if (mode == WALKING) {
//       Serial.println("Executing walking...");
//       bool finished = setupControl.walking(inputX, inputY, inputZ, inputTheta, false);
//       if (finished) {
//         Serial.println("Walking finished.");
//         isActive = false;
//       }
//     }
//   }
// }
void loop() {
  // 處理串口輸入
  while (Serial.available() > 0 && !stringComplete) {
    char inChar = (char)Serial.read();
    // Serial.print("Received char: "); Serial.println(inChar); // 調試：顯示每個接收字符
    if (inChar == '\n') {
      stringComplete = true;
      // Serial.println("Command received, processing..."); // 回饋 UI
    } else if (inChar != '\r') { // 忽略回車符
      inputString += inChar;
    }
  }

  if (stringComplete) {
    // Serial.print("Full input received: "); Serial.println(inputString); // 調試：顯示完整指令
    inputString.trim();
    // Serial.print("Received input (trimmed): "); Serial.println(inputString);

    if (inputString.equalsIgnoreCase("q")) {
      // Serial.println("Ready for new command.");
      setupControl.moveToPos(0, 0, 0, 0, 0, 0); // 左腳回到初始位置
      setupControl.moveToPos(0, 0, 0, 0, 6, 0); // 右腳回到初始位置
      isActive = false;
      moveExecuted = false; // 重置 moveExecuted
      for (int i = 0; i < 12; i++)
        setupControl.last_position[i] = setupControl.origin_position[i];
    }
    else if (inputString.startsWith("set_yz:")) {
      String params = inputString.substring(7);
      int comma1 = params.indexOf(',');
      int comma2 = params.indexOf(',', comma1 + 1);
      if (comma1 != -1 && comma2 != -1) {
        String y_str = params.substring(0, comma1);
        String z_str = params.substring(comma1 + 1, comma2);
        String steps_str = params.substring(comma2 + 1);
        float newYPos = y_str.toFloat();
        float newZPos = z_str.toFloat();
        int newSteps = steps_str.toInt();

        setupControl.initialYPos = newYPos;
        setupControl.initialZPos = newZPos;
        setupControl.steps = newSteps;

        Serial.println("Parameters updated"); // 明確回饋
        delay(10);
        // Serial.print("Updated initialYPos: "); Serial.println(setupControl.initialYPos);
        // Serial.print("Updated initialZPos: "); Serial.println(setupControl.initialZPos);
        // Serial.print("Updated steps: "); Serial.println(setupControl.steps);
      } else {
        Serial.println("Error: Invalid set_yz format. Use set_yz:initialYPos,initialZPos,steps");
      }
      moveExecuted = false; // 重置 moveExecuted
    }
    else {
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

            // Serial.print("Parsed: x="); Serial.print(inputX);
            // Serial.print(", y="); Serial.print(inputY);
            // Serial.print(", z="); Serial.print(inputZ);
            // Serial.print(", theta="); Serial.println(inputTheta);

            isActive = true;
            moveExecuted = false; // 重置 moveExecuted
          } else {
            Serial.println("Error: Invalid input format. Enter x,y,z,theta (e.g., 5,5,5,5), set_yz:initialYPos,initialZPos,steps, or 'q' to stop.");
          }
        } else {
          Serial.println("Error: Invalid input format. Enter x,y,z,theta (e.g., 5,5,5,5), set_yz:initialYPos,initialZPos,steps, or 'q' to stop.");
        }
      } else {
        Serial.println("Error: Invalid input format. Enter x,y,z,theta (e.g., 5,5,5,5), set_yz:initialYPos,initialZPos,steps, or 'q' to stop.");
      }
    }

    inputString = "";
    stringComplete = false;
    Serial.flush(); // 確保輸出緩衝區清空
    while (Serial.available() > 0) {
      Serial.read(); // 清空輸入緩衝區
    }
  }

  // 根據模式執行動作
  if (isActive && !moveExecuted) {
    // Serial.print("isActive: "); Serial.println(isActive);
    // Serial.print("moveExecuted: "); Serial.println(moveExecuted);
    if (mode == MOVE_TO_POS) {
      // Serial.println("Executing moveToPos...");
      setupControl.moveToPos(inputX, inputY, inputZ, inputTheta, 0, 0); // 左腳
      setupControl.moveToPos(inputX, inputY, inputZ, inputTheta, 6, 0); // 右腳
      moveExecuted = true;
      Serial.println("Move completed"); // 明確回饋
    } else if (mode == WALKING) {
      // Serial.println("Executing walking...");
      bool finished = setupControl.walking(inputX, inputY, inputZ, inputTheta, false);
      if (finished) {
        // Serial.println("Walking finished.");
        isActive = false;
        moveExecuted = true;
        Serial.println("Move completed"); // 明確回饋
      }
    }
  }
}