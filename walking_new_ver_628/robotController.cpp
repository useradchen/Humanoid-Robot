#include "robotController.h"

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3"
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif

#define BAUDRATE  1000000
enum joints{
    left_ankle_roll = 1,
    left_ankle_pitch,
    left_hip_pitch,
    left_hip_roll,
    left_hip_yaw,
    right_ankle_roll,
    right_ankle_pitch,
    right_hip_pitch,
    right_hip_roll,
    right_hip_yaw,
    head_yaw,
    head_pitch,
};
const uint8_t num_actuators = 12;
float LENGTH = 180.0;  // leg length(mm)

DynamixelWorkbench dxl_wb;
uint8_t dxl_id[num_actuators] = {left_ankle_roll, left_ankle_pitch, left_hip_pitch, left_hip_roll, left_hip_yaw, 
                      right_ankle_roll, right_ankle_pitch, right_hip_pitch, right_hip_roll, right_hip_yaw,
                      head_yaw,head_pitch};

const uint8_t handlerIndex = 0;

int32_t initialLegPositions[12] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 512, 512};

int32_t leftLegInitialPos[5] = {2048,2048,2048,2048,2048};   // left initial position

int32_t rightLegInitialPos[5] = {2048,2048,2048,2048,2048};  // right initial position

int32_t heaPos[2] = {512,512};

float initialHeight = 20.0;  // init z

//================================================================================================================

IKAngles calculateInverseKinematics(float x, float y, float z) {
    /*
    	use x(front and back) y(left and right) z(up and down) calculate inverse kinematics
    */
	float l = LENGTH;
  IKAngles angles;
    
  float theta_hp1 = atan2(x, (2*l - z));
  float d = sqrt(pow(2*l - z, 2) + pow(x, 2));
  float theta_hp2 = acos(d / (2*l));
  angles.theta_hp = theta_hp1 + theta_hp2;
  angles.theta_ap = acos(x / d) + theta_hp2 - M_PI/2;
  angles.theta_ar = atan2(y, (2*l - z));
  angles.theta_hr = atan2(y, (2*l - z)); // ar == hr
  return angles;
}

int32_t convertRadianToPosition(float radian) {
	/*
		(Radian) Map the input to 4096 
	*/
    float degree = radian * (180.0/M_PI);
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    return (int32_t)((4096.0/360.0) * degree);
}

int32_t convertDegreeToPosition(float degree) {
    /*
        Map the input degree to 4096 units (0-4095) for FK
    */
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    return (int32_t)((4096.0 / 360.0) * degree);
}

bool readAllActuatorPositions(int32_t* positions) {
    /*
      read and check all actuators Positions
    */
	const char *log;
    
    for (int i = 0; i < num_actuators; i++) {
        int retryCount = 0;
        while (retryCount < num_actuators) {
            if (dxl_wb.itemRead(dxl_id[i], "Present_Position", &positions[i], &log)) {
                break;
            }
            retryCount++;
            delay(20);
        }
        if (retryCount >= num_actuators) {
            return false;
        }
    }
    return true;
}

// bool initializePosition() {
//     const char *log;
//     bool result = false;
//     int32_t middlePositions[num_actuators] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 512, 512};
    
//     result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
//     if (!result) {
//         Serial.println("Initialization failed");
//         return false;
//     }
    
//     for (int i = 0; i < num_actuators; i++) {
//         uint16_t model_number = 0;
//         if (!dxl_wb.ping(dxl_id[i], &model_number, &log)) {
//             Serial.print("Failed to ping Dynamixel ID: ");
//             Serial.println(dxl_id[i]);
//             return false;
//         }
        
//         if (!dxl_wb.jointMode(dxl_id[i], 0, 30, &log)) {
//             Serial.print("Failed to set joint mode for Dynamixel ID: ");
//             Serial.println(dxl_id[i]);
//             return false;
//         }
        
//         if (!dxl_wb.torqueOn(dxl_id[i], &log)) {
//             Serial.print("Failed to turn torque on for Dynamixel ID: ");
//             Serial.println(dxl_id[i]);
//             return false;
//         }
//     }

//     result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
//     if (!result) {
//         Serial.println("Failed to add sync write handler");
//         return false;
//     }
    
//     bool initPos = dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, middlePositions, 1, &log);

//     if (!initPos) {
//         Serial.println("Failed to set middle position");
//         return false;
//     }
    
//     leftLegInitialPos[0] = middlePositions[0];
//     leftLegInitialPos[1] = middlePositions[1];
//     leftLegInitialPos[2] = middlePositions[2];
//     leftLegInitialPos[3] = middlePositions[3];
//     leftLegInitialPos[4] = middlePositions[4];

//     rightLegInitialPos[0] = middlePositions[5];
//     rightLegInitialPos[1] = middlePositions[6];
//     rightLegInitialPos[2] = middlePositions[7];
//     rightLegInitialPos[3] = middlePositions[8];
//     rightLegInitialPos[4] = middlePositions[9];

//     heaPos[0] = middlePositions[10];
//     heaPos[1] = middlePositions[11];

//     delay(1000); 
    
//     IKAngles legAngles = calculateInverseKinematics(0, 0, initialHeight);
//     int32_t realInitialPositions[num_actuators] = {
//         2048,
//         convertRadianToPosition(legAngles.theta_ap-M_PI),     //2
//         convertRadianToPosition(-(legAngles.theta_hp+M_PI)),  //3
//         2048,
//         2048,
//         2048,
//         convertRadianToPosition(-(legAngles.theta_ap-M_PI)),  //7
//         convertRadianToPosition(legAngles.theta_hp+M_PI),     //8
//         2048,
//         2048,
//         512,
//         512,
//     };

//     result = dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, realInitialPositions, 1, &log);
//     if (!result) {
//         Serial.println("Failed to enter real initial position!");
//         return false;
//     }
    
//     return true; 
// }

bool initializePosition() {
    Serial.println("Simulating initializePosition (no motors connected)...");

    int32_t middlePositions[num_actuators] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 512, 512};

    // 更新左腿初始位置
    leftLegInitialPos[0] = middlePositions[0]; // left_ankle_roll
    leftLegInitialPos[1] = middlePositions[1]; // left_ankle_pitch
    leftLegInitialPos[2] = middlePositions[2]; // left_hip_pitch
    leftLegInitialPos[3] = middlePositions[3]; // left_hip_roll
    leftLegInitialPos[4] = middlePositions[4]; // left_hip_yaw

    // 更新右腿初始位置
    rightLegInitialPos[0] = middlePositions[5]; // right_ankle_roll
    rightLegInitialPos[1] = middlePositions[6]; // right_ankle_pitch
    rightLegInitialPos[2] = middlePositions[7]; // right_hip_pitch
    rightLegInitialPos[3] = middlePositions[8]; // right_hip_roll
    rightLegInitialPos[4] = middlePositions[9]; // right_hip_yaw

    // 更新頭部位置
    heaPos[0] = middlePositions[10]; // head_yaw
    heaPos[1] = middlePositions[11]; // head_pitch

    // 計算初始位置 (x=0, y=0, z=20)
    IKAngles legAngles = calculateInverseKinematics(0, 0, initialHeight);
    int32_t realInitialPositions[num_actuators] = {
        2048,
        convertRadianToPosition(legAngles.theta_ap - M_PI),     // left_ankle_pitch
        convertRadianToPosition(-(legAngles.theta_hp + M_PI)),  // left_hip_pitch
        2048,
        2048,
        2048,
        convertRadianToPosition(-(legAngles.theta_ap - M_PI)),  // right_ankle_pitch
        convertRadianToPosition(legAngles.theta_hp + M_PI),     // right_hip_pitch
        2048,
        2048,
        512,
        512,
    };

    // 儲存初始位置到全局變數
    for (int i = 0; i < num_actuators; i++) {
        initialLegPositions[i] = realInitialPositions[i];
    }

    // 模擬 syncWriteHandler 初始化成功
    Serial.println("Simulated syncWriteHandler initialized.");

    return true; // 模擬成功
}

// bool moveToPos(float x, float y, float z) {
//     Serial.println("Start simulating moveToPos!");
//     Serial.print("Input Position: x="); Serial.print(x);
//     Serial.print(", y="); Serial.print(y);
//     Serial.print(", z="); Serial.println(z);

//     // 範圍檢查
//     if (sqrt(x*x + y*y + z*z) > 2 * LENGTH) {
//         Serial.println("Position out of range!");
//         return false;
//     }

//     // 計算目標腳的逆運動學角度
//     IKAngles targetLegAngles = calculateInverseKinematics(x, y, z);

//     // 根據 y 的正負決定哪隻腳移動
//     int32_t targetPositions[num_actuators];

//     // 初始位置的逆運動學角度（x=0, y=0, z=20）
//     IKAngles initialLegAngles = calculateInverseKinematics(0, 0, initialHeight);

//     // 根據 y 的正負決定哪隻腳移動

//     if (y > 0) {
//         // 右腳移動到 (x, y, z)，左腳回到初始位置 (x=0, y=0, z=20)
//         Serial.println("Moving Right Leg to target, Left Leg to initial position");
//         targetPositions[0] = initialLegPositions[0]; // left_ankle_roll
//         targetPositions[1] = initialLegPositions[1]; // left_ankle_pitch
//         targetPositions[2] = initialLegPositions[2]; // left_hip_pitch
//         targetPositions[3] = initialLegPositions[3]; // left_hip_roll
//         targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
//         targetPositions[5] = convertRadianToPosition(M_PI - targetLegAngles.theta_ar); // right_ankle_roll
//         targetPositions[6] = convertRadianToPosition(-(targetLegAngles.theta_ap - M_PI)); // right_ankle_pitch
//         targetPositions[7] = convertRadianToPosition(targetLegAngles.theta_hp + M_PI); // right_hip_pitch
//         targetPositions[8] = convertRadianToPosition(M_PI - targetLegAngles.theta_hr); // right_hip_roll
//         targetPositions[9] = initialLegPositions[9]; // right_hip_yaw
//     } else if (y < 0) {
//         // 左腳移動到 (x, y, z)，右腳回到初始位置 (x=0, y=0, z=20)
//         Serial.println("Moving Left Leg to target, Right Leg to initial position");
//         targetPositions[0] = convertRadianToPosition(M_PI - targetLegAngles.theta_ar); // left_ankle_roll
//         targetPositions[1] = convertRadianToPosition(targetLegAngles.theta_ap - M_PI); // left_ankle_pitch
//         targetPositions[2] = convertRadianToPosition(-(targetLegAngles.theta_hp + M_PI)); // left_hip_pitch
//         targetPositions[3] = convertRadianToPosition(M_PI - targetLegAngles.theta_hr); // left_hip_roll
//         targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
//         targetPositions[5] = initialLegPositions[5]; // right_ankle_roll
//         targetPositions[6] = initialLegPositions[6]; // right_ankle_pitch
//         targetPositions[7] = initialLegPositions[7]; // right_hip_pitch
//         targetPositions[8] = initialLegPositions[8]; // right_hip_roll
//         targetPositions[9] = initialLegPositions[9]; // right_hip_yaw
//     } else {
//         // y == 0，兩腳都回到初始位置
//         Serial.println("y == 0, both legs return to initial position");
//         for (int i = 0; i < num_actuators; i++) {
//             targetPositions[i] = initialLegPositions[i];
//         }
//     }

//     targetPositions[10] = heaPos[0]; // head_yaw
//     targetPositions[11] = heaPos[1]; // head_pitch

//     // 模擬成功設置馬達位置
//     Serial.println("Simulating syncWrite to target positions...");

//     // 更新初始位置數組
//     leftLegInitialPos[0] = targetPositions[0]; // left_ankle_roll
//     leftLegInitialPos[1] = targetPositions[1]; // left_ankle_pitch
//     leftLegInitialPos[2] = targetPositions[2]; // left_hip_pitch
//     leftLegInitialPos[3] = targetPositions[3]; // left_hip_roll
//     leftLegInitialPos[4] = targetPositions[4]; // left_hip_yaw

//     rightLegInitialPos[0] = targetPositions[5]; // right_ankle_roll
//     rightLegInitialPos[1] = targetPositions[6]; // right_ankle_pitch
//     rightLegInitialPos[2] = targetPositions[7]; // right_hip_pitch
//     rightLegInitialPos[3] = targetPositions[8]; // right_hip_roll
//     rightLegInitialPos[4] = targetPositions[9]; // right_hip_yaw

//     // 列印左腿初始位置
//     Serial.println("Left Leg Initial Positions:");
//     Serial.print("left_ankle_roll: "); Serial.println(leftLegInitialPos[0]);
//     Serial.print("left_ankle_pitch: "); Serial.println(leftLegInitialPos[1]);
//     Serial.print("left_hip_pitch: "); Serial.println(leftLegInitialPos[2]);
//     Serial.print("left_hip_roll: "); Serial.println(leftLegInitialPos[3]);
//     Serial.print("left_hip_yaw: "); Serial.println(leftLegInitialPos[4]);

//     // 列印右腿初始位置
//     Serial.println("Right Leg Initial Positions:");
//     Serial.print("right_ankle_roll: "); Serial.println(rightLegInitialPos[0]);
//     Serial.print("right_ankle_pitch: "); Serial.println(rightLegInitialPos[1]);
//     Serial.print("right_hip_pitch: "); Serial.println(rightLegInitialPos[2]);
//     Serial.print("right_hip_roll: "); Serial.println(rightLegInitialPos[3]);
//     Serial.print("right_hip_yaw: "); Serial.println(rightLegInitialPos[4]);

//     // 列印目標角度以便檢查
//     Serial.println("Target Leg Angles (radians):");
//     Serial.print("theta_hp: "); Serial.println(targetLegAngles.theta_hp);
//     Serial.print("theta_ap: "); Serial.println(targetLegAngles.theta_ap);
//     Serial.print("theta_ar: "); Serial.println(targetLegAngles.theta_ar);
//     Serial.print("theta_hr: "); Serial.println(targetLegAngles.theta_hr);

//     return true; // 模擬成功
// }

bool moveToPos(float x, float y, float z) { // for simulation
    Serial.println("Start simulating moveToPos!");
    Serial.print("Input Position: x="); Serial.print(x);
    Serial.print(", y="); Serial.print(y);
    Serial.print(", z="); Serial.println(z);

    // 計算目標腳的逆運動學角度
    IKAngles targetLegAngles = calculateInverseKinematics(x, y, z);
    // 初始位置的逆運動學角度（x=0, y=0, z=20）
    IKAngles initialLegAngles = calculateInverseKinematics(0, 0, initialHeight);

    // 根據 y 的正負決定哪隻腳移動
    int32_t targetPositions[num_actuators];

    if (y > 0) {
        // 右腳移動到 (x, y, z)，左腳回到初始位置 (x=0, y=0, z=20)
        Serial.println("Moving Right Leg to target, Left Leg to initial position");
        targetPositions[0] = initialLegPositions[0]; // left_ankle_roll
        targetPositions[1] = initialLegPositions[1]; // left_ankle_pitch
        targetPositions[2] = initialLegPositions[2]; // left_hip_pitch
        targetPositions[3] = initialLegPositions[3]; // left_hip_roll
        targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
        targetPositions[5] = convertRadianToPosition(M_PI - targetLegAngles.theta_ar); // right_ankle_roll
        targetPositions[6] = convertRadianToPosition(-(targetLegAngles.theta_ap - M_PI)); // right_ankle_pitch
        targetPositions[7] = convertRadianToPosition(targetLegAngles.theta_hp + M_PI); // right_hip_pitch
        targetPositions[8] = convertRadianToPosition(M_PI - targetLegAngles.theta_hr); // right_hip_roll
        targetPositions[9] = initialLegPositions[9]; // right_hip_yaw
    } else if (y < 0) {
        // 左腳移動到 (x, y, z)，右腳回到初始位置 (x=0, y=0, z=20)
        Serial.println("Moving Left Leg to target, Right Leg to initial position");
        targetPositions[0] = convertRadianToPosition(M_PI - targetLegAngles.theta_ar); // left_ankle_roll
        targetPositions[1] = convertRadianToPosition(targetLegAngles.theta_ap - M_PI); // left_ankle_pitch
        targetPositions[2] = convertRadianToPosition(-(targetLegAngles.theta_hp + M_PI)); // left_hip_pitch
        targetPositions[3] = convertRadianToPosition(M_PI - targetLegAngles.theta_hr); // left_hip_roll
        targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
        targetPositions[5] = initialLegPositions[5]; // right_ankle_roll
        targetPositions[6] = initialLegPositions[6]; // right_ankle_pitch
        targetPositions[7] = initialLegPositions[7]; // right_hip_pitch
        targetPositions[8] = initialLegPositions[8]; // right_hip_roll
        targetPositions[9] = initialLegPositions[9]; // right_hip_yaw
    } else {
        // y == 0，兩腳都回到初始位置
        Serial.println("y == 0, both legs return to initial position");
        for (int i = 0; i < num_actuators; i++) {
            targetPositions[i] = initialLegPositions[i];
        }
    }

    targetPositions[10] = heaPos[0]; // head_yaw
    targetPositions[11] = heaPos[1]; // head_pitch

    // 模擬成功設置馬達位置
    Serial.println("Simulating syncWrite to target positions...");

    // 更新初始位置數組
    leftLegInitialPos[0] = targetPositions[0]; // left_ankle_roll
    leftLegInitialPos[1] = targetPositions[1]; // left_ankle_pitch
    leftLegInitialPos[2] = targetPositions[2]; // left_hip_pitch
    leftLegInitialPos[3] = targetPositions[3]; // left_hip_roll
    leftLegInitialPos[4] = targetPositions[4]; // left_hip_yaw

    rightLegInitialPos[0] = targetPositions[5]; // right_ankle_roll
    rightLegInitialPos[1] = targetPositions[6]; // right_ankle_pitch
    rightLegInitialPos[2] = targetPositions[7]; // right_hip_pitch
    rightLegInitialPos[3] = targetPositions[8]; // right_hip_roll
    rightLegInitialPos[4] = targetPositions[9]; // right_hip_yaw

    // 列印左腿初始位置
    Serial.println("Left Leg Initial Positions:");
    Serial.print("left_ankle_roll: "); Serial.println(leftLegInitialPos[0]);
    Serial.print("left_ankle_pitch: "); Serial.println(leftLegInitialPos[1]);
    Serial.print("left_hip_pitch: "); Serial.println(leftLegInitialPos[2]);
    Serial.print("left_hip_roll: "); Serial.println(leftLegInitialPos[3]);
    Serial.print("left_hip_yaw: "); Serial.println(leftLegInitialPos[4]);

    // 列印右腿初始位置
    Serial.println("Right Leg Initial Positions:");
    Serial.print("right_ankle_roll: "); Serial.println(rightLegInitialPos[0]);
    Serial.print("right_ankle_pitch: "); Serial.println(rightLegInitialPos[1]);
    Serial.print("right_hip_pitch: "); Serial.println(rightLegInitialPos[2]);
    Serial.print("right_hip_roll: "); Serial.println(rightLegInitialPos[3]);
    Serial.print("right_hip_yaw: "); Serial.println(rightLegInitialPos[4]);

    return true; // 模擬成功
}