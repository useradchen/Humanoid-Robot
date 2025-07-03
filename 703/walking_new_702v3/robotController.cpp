#include "robotController.h"
#include <math.h>

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

float initialZPos = 20.0;  // init z

int steps = 1000; // linearMovement

int currentStep = 1;

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

bool initializePosition() {
    const char *log;
    bool result = false;
    int32_t middlePositions[num_actuators] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 512, 512};
    
    result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
    if (!result) {
        Serial.println("Initialization failed");
        return false;
    }
    
    for (int i = 0; i < num_actuators; i++) {
        uint16_t model_number = 0;
        if (!dxl_wb.ping(dxl_id[i], &model_number, &log)) {
            Serial.print("Failed to ping Dynamixel ID: ");
            Serial.println(dxl_id[i]);
            return false;
        }
        
        if (!dxl_wb.jointMode(dxl_id[i], 800, 100, &log)) { // id , v , a
            Serial.print("Failed to set joint mode for Dynamixel ID: ");
            Serial.println(dxl_id[i]);
            return false;
        }
        
        if (!dxl_wb.torqueOn(dxl_id[i], &log)) {
            Serial.print("Failed to turn torque on for Dynamixel ID: ");
            Serial.println(dxl_id[i]);
            return false;
        }
    }

    result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
    if (!result) {
        Serial.println("Failed to add sync write handler");
        return false;
    }
    
    bool initPos = dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, middlePositions, 1, &log);

    if (!initPos) {
        Serial.println("Failed to set middle position");
        return false;
    }
    
    leftLegInitialPos[0] = middlePositions[0];
    leftLegInitialPos[1] = middlePositions[1];
    leftLegInitialPos[2] = middlePositions[2];
    leftLegInitialPos[3] = middlePositions[3];
    leftLegInitialPos[4] = middlePositions[4];

    rightLegInitialPos[0] = middlePositions[5];
    rightLegInitialPos[1] = middlePositions[6];
    rightLegInitialPos[2] = middlePositions[7];
    rightLegInitialPos[3] = middlePositions[8];
    rightLegInitialPos[4] = middlePositions[9];

    heaPos[0] = middlePositions[10];
    heaPos[1] = middlePositions[11];

    delay(1000); 
    
    IKAngles legAngles = calculateInverseKinematics(0, 0, initialZPos);
    int32_t realInitialPositions[num_actuators] = {
        2048,
        convertRadianToPosition(legAngles.theta_ap-M_PI),     //2
        convertRadianToPosition(-(legAngles.theta_hp+M_PI)),  //3
        2048,
        2048,
        2048,
        convertRadianToPosition(-(legAngles.theta_ap-M_PI)),  //7
        convertRadianToPosition(legAngles.theta_hp+M_PI),     //8
        2048,
        2048,
        512,
        512,
    };

    result = dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, realInitialPositions, 1, &log);
    if (!result) {
        Serial.println("Failed to enter real initial position!");
        return false;
    }
    
    for (int i = 0; i < num_actuators; i++) {
        initialLegPositions[i] = realInitialPositions[i];
    }
    
    return true; 
}

// bool initializePosition() { // Simulating
//     Serial.println("Simulating initializePosition (no motors connected)...");

//     int32_t middlePositions[num_actuators] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 512, 512};

//     // 更新左腿初始位置
//     leftLegInitialPos[0] = middlePositions[0]; // left_ankle_roll
//     leftLegInitialPos[1] = middlePositions[1]; // left_ankle_pitch
//     leftLegInitialPos[2] = middlePositions[2]; // left_hip_pitch
//     leftLegInitialPos[3] = middlePositions[3]; // left_hip_roll
//     leftLegInitialPos[4] = middlePositions[4]; // left_hip_yaw

//     // 更新右腿初始位置
//     rightLegInitialPos[0] = middlePositions[5]; // right_ankle_roll
//     rightLegInitialPos[1] = middlePositions[6]; // right_ankle_pitch
//     rightLegInitialPos[2] = middlePositions[7]; // right_hip_pitch
//     rightLegInitialPos[3] = middlePositions[8]; // right_hip_roll
//     rightLegInitialPos[4] = middlePositions[9]; // right_hip_yaw

//     // 更新頭部位置
//     heaPos[0] = middlePositions[10]; // head_yaw
//     heaPos[1] = middlePositions[11]; // head_pitch

//     // 計算初始位置 (x=0, y=0, z=20)
//     IKAngles legAngles = calculateInverseKinematics(0, 0, initialZPos);
//     int32_t realInitialPositions[num_actuators] = {
//         2048,
//         convertRadianToPosition(legAngles.theta_ap - M_PI),     // left_ankle_pitch
//         convertRadianToPosition(-(legAngles.theta_hp + M_PI)),  // left_hip_pitch
//         2048,
//         2048,
//         2048,
//         convertRadianToPosition(-(legAngles.theta_ap - M_PI)),  // right_ankle_pitch
//         convertRadianToPosition(legAngles.theta_hp + M_PI),     // right_hip_pitch
//         2048,
//         2048,
//         512,
//         512,
//     };

//     // 儲存初始位置到全局變數
//     for (int i = 0; i < num_actuators; i++) {
//         initialLegPositions[i] = realInitialPositions[i];
//     }

//     // 模擬 syncWriteHandler 初始化成功
//     Serial.println("Simulated syncWriteHandler initialized.");

//     return true; // 模擬成功
// }

bool linearMovement(int32_t startPositions[], int32_t targetPositions[]) {
    const char *log;
    int32_t currentPositions[num_actuators];

    for (int i = 0; i < steps; i++) {
        // float t = (float)(i + 1) / steps;
        float t = 0.5 * (1 - cos(M_PI * i / (steps - 1))); // sin(t * M_PI)
        for (int j = 0; j < num_actuators; j++) {
            currentPositions[j] = startPositions[j] + (targetPositions[j] - startPositions[j]) * t;
        }

        bool result = dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, currentPositions, 1, &log);
        if (!result) {
            Serial.println("Failed to syncWrite positions!");
            return false;
        }
    }

    // 更新初始位置數組
    for (int i = 0; i < num_actuators; i++) {
        initialLegPositions[i] = targetPositions[i];
    }
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
    heaPos[0] = targetPositions[10]; // head_yaw
    heaPos[1] = targetPositions[11]; // head_pitch

    return true;
}

// bool linearMovement(int32_t startPositions[], int32_t targetPositions[]) {  // for simulation
//     int32_t currentPositions[num_actuators];

//     Serial.println("Start Positions before linear movement:");
//     for (int j = 0; j < num_actuators; j++) {
//         // Serial.print("Actuator "); Serial.print(j); Serial.print(": "); Serial.println(startPositions[j]);
//     }
//     Serial.println("Target Positions:");
//     for (int j = 0; j < num_actuators; j++) {
//         // Serial.print("Actuator "); Serial.print(j); Serial.print(": "); Serial.println(targetPositions[j]);
//     }

//     for (int i = 0; i < steps; i++) {
//         // float t = (float)(i + 1) / steps;
//         float t = 0.5 * (1 - cos(M_PI * i / (steps - 1))); // sin(t * M_PI)
//         for (int j = 0; j < num_actuators; j++) {
//             currentPositions[j] = round(startPositions[j] + (targetPositions[j] - startPositions[j]) * t);
//         }

//         // Serial.println("======================================================================");
//         // Serial.print("Simulating linear movement step "); Serial.println(i + 1);
//         // Serial.print("left_ankle_roll: "); Serial.println(currentPositions[0]);
//         // Serial.print("left_ankle_pitch: "); Serial.println(currentPositions[1]);
//         // Serial.print("left_hip_pitch: "); Serial.println(currentPositions[2]);
//         // Serial.print("left_hip_roll: "); Serial.println(currentPositions[3]);
//         // Serial.print("left_hip_yaw: "); Serial.println(currentPositions[4]);
//         // Serial.print("right_ankle_roll: "); Serial.println(currentPositions[5]);
//         // Serial.print("right_ankle_pitch: "); Serial.println(currentPositions[6]);
//         // Serial.print("right_hip_pitch: "); Serial.println(currentPositions[7]);
//         // Serial.print("right_hip_roll: "); Serial.println(currentPositions[8]);
//         // Serial.print("right_hip_yaw: "); Serial.println(currentPositions[9]);
//         // Serial.print("head_yaw: "); Serial.println(currentPositions[10]);
//         // Serial.print("head_pitch: "); Serial.println(currentPositions[11]);
//         // Serial.println("======================================================================");
//     }

//     // 更新初始位置數組
//     for (int i = 0; i < num_actuators; i++) {
//         initialLegPositions[i] = targetPositions[i];
//     }
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
//     heaPos[0] = targetPositions[10]; // head_yaw
//     heaPos[1] = targetPositions[11]; // head_pitch

//     return true;
// }

bool moveToPos(float x, float y, float z) {
    Serial.println("Start moveToPos!");
    Serial.print("Input Position: x="); Serial.print(x);
    Serial.print(", y="); Serial.print(y);
    Serial.print(", z="); Serial.println(z);

    int32_t startPositions[num_actuators];
    for (int i = 0; i < num_actuators; i++) {
        startPositions[i] = initialLegPositions[i];
    }

    // 範圍檢查
    if (sqrt(x*x + y*y + z*z) > 2 * LENGTH) {
        Serial.println("Position out of range!");
        return false;
    }

    // 計算目標腳的逆運動學角度
    z = z + initialZPos;
    IKAngles targetLegAngles = calculateInverseKinematics(x, y, z);

    // 初始位置的逆運動學角度（x=0, y=0, z=20）
    IKAngles initialLegAngles = calculateInverseKinematics(0, 0, initialZPos);

    int32_t targetPositions[num_actuators];

    if (y > 0) {
        // 右腳移動到 (x, y, z)，左腳回到初始位置 (x=0, y=0, z=20) swing
        targetPositions[0] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // left_ankle_roll
        targetPositions[1] = convertRadianToPosition(initialLegAngles.theta_ap - M_PI); // left_ankle_pitch
        targetPositions[2] = convertRadianToPosition(-(initialLegAngles.theta_hp + M_PI)); // left_hip_pitch
        targetPositions[3] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // left_hip_roll
        targetPositions[4] = 2048; // left_hip_yaw
        targetPositions[5] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // right_ankle_roll
        targetPositions[6] = convertRadianToPosition(-(targetLegAngles.theta_ap - M_PI)); // right_ankle_pitch
        targetPositions[7] = convertRadianToPosition(targetLegAngles.theta_hp + M_PI); // right_hip_pitch
        targetPositions[8] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // right_hip_roll
        targetPositions[9] = 2048; // right_hip_yaw
    } else if (y < 0) {
        // 左腳移動到 (x, y, z)，右腳回到初始位置 (x=0, y=0, z=20)
        Serial.println("Moving Left Leg to target, Right Leg to initial position");
        targetPositions[0] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // left_ankle_roll
        targetPositions[1] = convertRadianToPosition(targetLegAngles.theta_ap - M_PI); // left_ankle_pitch
        targetPositions[2] = convertRadianToPosition(-(targetLegAngles.theta_hp + M_PI)); // left_hip_pitch
        targetPositions[3] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // left_hip_roll
        targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
        targetPositions[5] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // right_ankle_roll
        targetPositions[6] = convertRadianToPosition(-(initialLegAngles.theta_ap - M_PI)); // right_ankle_pitch
        targetPositions[7] = convertRadianToPosition(initialLegAngles.theta_hp + M_PI); // right_hip_pitch
        targetPositions[8] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // right_hip_roll
        targetPositions[9] = 2048; // right_hip_yaw
    } else if (y == 0 && z != 0){
        Serial.println("y == 0 and z == 0, both legs return to ground (z=0)");
        targetPositions[0] = convertRadianToPosition(M_PI + initialLegAngles.theta_ar); // left_ankle_roll
        targetPositions[1] = convertRadianToPosition(initialLegAngles.theta_ap - M_PI); // left_ankle_pitch
        targetPositions[2] = convertRadianToPosition(-(initialLegAngles.theta_hp + M_PI)); // left_hip_pitch
        targetPositions[3] = convertRadianToPosition(M_PI + initialLegAngles.theta_hr); // left_hip_roll
        targetPositions[4] = 2048; // left_hip_yaw
        targetPositions[5] = convertRadianToPosition(M_PI + initialLegAngles.theta_ar); // right_ankle_roll
        targetPositions[6] = convertRadianToPosition(-(initialLegAngles.theta_ap - M_PI)); // right_ankle_pitch
        targetPositions[7] = convertRadianToPosition(initialLegAngles.theta_hp + M_PI); // right_hip_pitch
        targetPositions[8] = convertRadianToPosition(M_PI + initialLegAngles.theta_hr); // right_hip_roll
        targetPositions[9] = 2048; // right_hip_yaw
    }else {
        // y == 0，兩腳都回到初始位置
        Serial.println("y == 0, both legs return to initial position");
        for (int i = 0; i < num_actuators; i++) {
            targetPositions[i] = initialLegPositions[i];
        }
    }

    targetPositions[10] = heaPos[0]; // head_yaw
    targetPositions[11] = heaPos[1]; // head_pitch

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

    // // 列印左腿初始位置
    // Serial.println("Left Leg Initial Positions:");
    // Serial.print("left_ankle_roll: "); Serial.println(leftLegInitialPos[0]);
    // Serial.print("left_ankle_pitch: "); Serial.println(leftLegInitialPos[1]);
    // Serial.print("left_hip_pitch: "); Serial.println(leftLegInitialPos[2]);
    // Serial.print("left_hip_roll: "); Serial.println(leftLegInitialPos[3]);
    // Serial.print("left_hip_yaw: "); Serial.println(leftLegInitialPos[4]);

    // Serial.println("Right Leg Initial Positions:");
    // Serial.print("right_ankle_roll: "); Serial.println(rightLegInitialPos[0]);
    // Serial.print("right_ankle_pitch: "); Serial.println(rightLegInitialPos[1]);
    // Serial.print("right_hip_pitch: "); Serial.println(rightLegInitialPos[2]);
    // Serial.print("right_hip_roll: "); Serial.println(rightLegInitialPos[3]);
    // Serial.print("right_hip_yaw: "); Serial.println(rightLegInitialPos[4]);

    bool result = linearMovement(startPositions, targetPositions);

    return true;
}

// bool moveToPos(float x, float y, float z) { // for simulation
   
//     Serial.println("Start simulating moveToPos!");
//     Serial.print("Input Position: x="); Serial.print(x);
//     Serial.print(", y="); Serial.print(y);
//     Serial.print(", z="); Serial.println(z);
    
//     int32_t startPositions[num_actuators];
//     for (int i = 0; i < num_actuators; i++) {
//         startPositions[i] = initialLegPositions[i];
//     }
//     // 計算目標腳的逆運動學角度
//     IKAngles targetLegAngles = calculateInverseKinematics(x, y, z);
//     // 初始位置的逆運動學角度（x=0, y=0, z=20）
//     IKAngles initialLegAngles = calculateInverseKinematics(0, 0, initialZPos);
  
//     // 根據 y 的正負決定哪隻腳移動
//     int32_t targetPositions[num_actuators];

//     if (y > 0) {
//         // 右腳移動到 (x, y, z)，左腳回到初始位置 (x=0, y=0, z=20) swing
//         targetPositions[0] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // left_ankle_roll
//         targetPositions[1] = convertRadianToPosition(initialLegAngles.theta_ap - M_PI); // left_ankle_pitch
//         targetPositions[2] = convertRadianToPosition(-(initialLegAngles.theta_hp + M_PI)); // left_hip_pitch
//         targetPositions[3] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // left_hip_roll
//         targetPositions[4] = 2048; // left_hip_yaw
//         targetPositions[5] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // right_ankle_roll
//         targetPositions[6] = convertRadianToPosition(-(targetLegAngles.theta_ap - M_PI)); // right_ankle_pitch
//         targetPositions[7] = convertRadianToPosition(targetLegAngles.theta_hp + M_PI); // right_hip_pitch
//         targetPositions[8] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // right_hip_roll
//         targetPositions[9] = 2048; // right_hip_yaw
//     } else if (y < 0) {
//         // 左腳移動到 (x, y, z)，右腳回到初始位置 (x=0, y=0, z=20)
//         Serial.println("Moving Left Leg to target, Right Leg to initial position");
//         targetPositions[0] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // left_ankle_roll
//         targetPositions[1] = convertRadianToPosition(targetLegAngles.theta_ap - M_PI); // left_ankle_pitch
//         targetPositions[2] = convertRadianToPosition(-(targetLegAngles.theta_hp + M_PI)); // left_hip_pitch
//         targetPositions[3] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // left_hip_roll
//         targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
//         targetPositions[5] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // right_ankle_roll
//         targetPositions[6] = convertRadianToPosition(-(initialLegAngles.theta_ap - M_PI)); // right_ankle_pitch
//         targetPositions[7] = convertRadianToPosition(initialLegAngles.theta_hp + M_PI); // right_hip_pitch
//         targetPositions[8] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // right_hip_roll
//         targetPositions[9] = 2048; // right_hip_yaw
//     } else {
//         // y == 0，兩腳都回到初始位置
//         Serial.println("y == 0, both legs return to initial position");
//         for (int i = 0; i < num_actuators; i++) {
//             targetPositions[i] = initialLegPositions[i];
//         }
//     }

//     targetPositions[10] = heaPos[0]; // head_yaw
//     targetPositions[11] = heaPos[1]; // head_pitch

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
//     // Serial.println("Left");
//     // Serial.print("left_ankle_roll: "); Serial.println(leftLegInitialPos[0]);
//     // Serial.print("left_ankle_pitch: "); Serial.println(leftLegInitialPos[1]);
//     // Serial.print("left_hip_pitch: "); Serial.println(leftLegInitialPos[2]);
//     // Serial.print("left_hip_roll: "); Serial.println(leftLegInitialPos[3]);
//     // Serial.print("left_hip_yaw: "); Serial.println(leftLegInitialPos[4]);

//     // // 列印右腿初始位置
//     // Serial.println("Right");
//     // Serial.print("right_ankle_roll: "); Serial.println(rightLegInitialPos[0]);
//     // Serial.print("right_ankle_pitch: "); Serial.println(rightLegInitialPos[1]);
//     // Serial.print("right_hip_pitch: "); Serial.println(rightLegInitialPos[2]);
//     // Serial.print("right_hip_roll: "); Serial.println(rightLegInitialPos[3]);
//     // Serial.print("right_hip_yaw: "); Serial.println(rightLegInitialPos[4]); 

//     // 執行線性插值運動（模擬版本）
//     bool result = linearMovement(startPositions, targetPositions); // 10 步
    
//     // 更新init
//     for (int i = 0; i < num_actuators; i++) {
//         initialLegPositions[i] = targetPositions[i];
//     }
//     return true; // 模擬成功
// }

// bool walking(float x, float y, float z) { // no 1/2 stop
//     Serial.print("Walking with: x="); Serial.print(x);
//     Serial.print(", y="); Serial.print(y);
//     Serial.print(", z="); Serial.println(z);

//     float oneStep = x / 2;

//     if (currentStep == 1) {
//       // 第一步：右腳
//       moveToPos(oneStep, y, z);  // 抬腳
//       moveToPos(oneStep, y, 0);  // 放下
//       currentStep = 2; // 進入主循環
//       return false; // 繼續下一次循環
//   } else if (currentStep == 2) {
//       // 主循環：右腳-左腳交替
//       Serial.println("Left");
//       moveToPos(x, -y, z);
//       moveToPos(x, -y, 0);

//       Serial.println("Right");
//       moveToPos(x, y, z);
//       moveToPos(x, y, 0);

//       return false; // 繼續無限循環
//   }

//     return true;
// }

bool leftMovement(float x, float y, float z, bool stop){
    // moveToPos(x, -y, z);  // left
    if (stop){
        // moveToPos(x, y, 0);  // right
        moveToPos(x, -y, z);  // left
        moveToPos(x, 0, 0);  // left
    }else{
        moveToPos(x, -y, z);  // left
        moveToPos(x, -y, 0);  // left
    }
    return true;
}

bool rightMovement(float x, float y, float z, bool stop){
    // moveToPos(x, y, z);  // right
    if (stop){
        // moveToPos(x, -y, 0);  // left
        moveToPos(x, y, z);  // right
        moveToPos(x, 0, 0);  // right
    }else{
        moveToPos(x, y, z);  // right
        moveToPos(x, y, 0);  // right
    }
return true;
}

bool walking(float x, float y, float z, bool stop) {
    Serial.print("Walking with: x="); Serial.print(x);
    Serial.print(", y="); Serial.print(y);
    Serial.print(", z="); Serial.println(z);

    float oneStep = x / 2;

    if (stop) {
        // 1/2x
        if (currentStep == 1 || (currentStep == 2 && y > 0)) {
            Serial.println("Stopping: Moving Left Leg for final 1/2x step");
            leftMovement(oneStep, y, z, true);
        } else if (currentStep == 2 && y < 0) {
            // 1/2x
            Serial.println("Stopping: Moving Right Leg for final 1/2x step");
            // moveToPos(oneStep, y, z);
            // moveToPos(oneStep, 0, 0);
            rightMovement(oneStep, y, z, true);
        }
        currentStep = 1; // for next step
        return true; 
    }

    if (currentStep == 1) {
        // first step
        Serial.println("First step: Right Leg");
        // moveToPos(oneStep, y, z);  // 抬右腳
        // moveToPos(oneStep, y, 0);  // 放右腳
        rightMovement(oneStep, y, z, false);
        currentStep = 2;
        return false; // next circle
    } else if (currentStep == 2) {
        Serial.println("Left");
        // moveToPos(x, -y, z);  // left
        // moveToPos(x, -y, 0);  // left
        leftMovement(x, y, z, false);

        Serial.println("Right");
        // moveToPos(x, y, z);   // right
        // moveToPos(x, y, 0);   // right
        rightMovement(x, y, z, false);

        return false;
    }

    return true;
}