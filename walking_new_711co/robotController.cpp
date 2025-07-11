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

float stance = 5;

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

bool leftMovement(float x, float y, float z, bool stop) {
    Serial.println("Executing leftMovement with coordinate interpolation");

    // 定義起始座標（假設從 (0, -y, initialZPos) 或當前位置開始）
    float startX = 0;
    float startY = -y; // 左腳移動時，y 為負
    float startZ = initialZPos;

    // 如果停止，執行最終步驟
    if (stop) {
        // 先移動到目標位置 (x, -y, z)
        moveToPos(x, -y, z);
        // 回到地面 (x, 0, 0)
        moveToPos(x, 0, 0);
        return true;
    }

    // 座標線性插值
    // const int steps = 1000; // 使用與原 linearMovement 相同的步數
    for (int i = 0; i < steps; i++) {
        // 使用 cosine 插值公式以平滑移動
        float t = 0.5 * (1 - cos(M_PI * i / (steps - 1)));
        
        // 計算中間座標
        float x_t = startX + (x - startX) * t;
        float y_t = startY + (-y - startY) * t;
        float z_t = startZ + (z - startZ) * t;

        // 呼叫 moveToPos 移動到中間座標
        if (!moveToPos(x_t, y_t, z_t)) {
            Serial.println("Failed to move to intermediate position in leftMovement!");
            return false;
        }
    }

    // 移動到地面 (x, -y, 0)
    if (!moveToPos(x, -y, 0)) {
        Serial.println("Failed to move to ground position in leftMovement!");
        return false;
    }

    return true;
}

bool rightMovement(float x, float y, float z, bool stop) {
    Serial.println("Executing rightMovement with coordinate interpolation");

    // 定義起始座標（假設從 (0, y, initialZPos) 或當前位置開始）
    float startX = 0;
    float startY = y; // 右腳移動時，y 為正
    float startZ = initialZPos;

    // 如果停止，執行最終步驟
    if (stop) {
        // 先移動到目標位置 (x, y, z)
        moveToPos(x, y, z);
        // 回到地面 (x, 0, 0)
        moveToPos(x, 0, 0);
        return true;
    }

    // 座標線性插值
    // const int steps = 1000; // 使用與原 linearMovement 相同的步數
    for (int i = 0; i < steps; i++) {
        // 使用 cosine 插值公式以平滑移動
        float t = 0.5 * (1 - cos(M_PI * i / (steps - 1)));
        
        // 計算中間座標
        float x_t = startX + (x - startX) * t;
        float y_t = startY + (y - startY) * t;
        float z_t = startZ + (z - startZ) * t;

        // 呼叫 moveToPos 移動到中間座標
        if (!moveToPos(x_t, y_t, z_t)) {
            Serial.println("Failed to move to intermediate position in rightMovement!");
            return false;
        }
    }

    // 移動到地面 (x, y, 0)
    if (!moveToPos(x, y, 0)) {
        Serial.println("Failed to move to ground position in rightMovement!");
        return false;
    }

    return true;
}

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

    if (y > 0) { // (20,10,20)
        if (z == initialZPos){
            // IKAngles initialLegAnglesY0 = calculateInverseKinematics(x, 0, initialZPos);
            targetPositions[0] = convertRadianToPosition(M_PI + initialLegAngles.theta_ar); // left_ankle_roll
            targetPositions[1] = convertRadianToPosition(initialLegAngles.theta_ap - M_PI); // left_ankle_pitch
            targetPositions[2] = convertRadianToPosition(-(initialLegAngles.theta_hp + M_PI)); // left_hip_pitch
            targetPositions[3] = convertRadianToPosition(M_PI + initialLegAngles.theta_hr); // left_hip_roll
            targetPositions[4] = 2048; // left_hip_yaw
            targetPositions[5] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // right_ankle_roll
            targetPositions[6] = convertRadianToPosition(-(targetLegAngles.theta_ap - M_PI)); // right_ankle_pitch
            targetPositions[7] = convertRadianToPosition(targetLegAngles.theta_hp + M_PI); // right_hip_pitch
            targetPositions[8] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr); // right_hip_roll
            targetPositions[9] = 2048; // right_hip_yaw
        }else{
            // 右腳移動到 (x, y, z)
            IKAngles leftPreviousPos = calculateInverseKinematics(x, -y, initialZPos); // Keep the left foot in the previous position
            targetPositions[0] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // left_ankle_roll
            targetPositions[1] = convertRadianToPosition(leftPreviousPos.theta_ap - M_PI); // left_ankle_pitch
            targetPositions[2] = convertRadianToPosition(-(leftPreviousPos.theta_hp + M_PI)); // left_hip_pitch
            targetPositions[3] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr) + stance; // left_hip_roll
            targetPositions[4] = 2048; // left_hip_yaw
            targetPositions[5] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // right_ankle_roll
            targetPositions[6] = convertRadianToPosition(-(targetLegAngles.theta_ap - M_PI)); // right_ankle_pitch
            targetPositions[7] = convertRadianToPosition(targetLegAngles.theta_hp + M_PI); // right_hip_pitch
            targetPositions[8] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr) - stance; // right_hip_roll
            targetPositions[9] = 2048; // right_hip_yaw
        }
    } else if (y < 0) {
        if (z == initialZPos){
            // IKAngles initialLegAnglesY0 = calculateInverseKinematics(x, 0, initialZPos);
            targetPositions[0] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // left_ankle_roll
            targetPositions[1] = convertRadianToPosition(targetLegAngles.theta_ap - M_PI); // left_ankle_pitch
            targetPositions[2] = convertRadianToPosition(-(targetLegAngles.theta_hp + M_PI)); // left_hip_pitch
            targetPositions[3] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr) + stance; // left_hip_roll
            targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
            targetPositions[5] = convertRadianToPosition(M_PI + initialLegAngles.theta_ar); // right_ankle_roll
            targetPositions[6] = convertRadianToPosition(-(initialLegAngles.theta_ap - M_PI)); // right_ankle_pitch
            targetPositions[7] = convertRadianToPosition(initialLegAngles.theta_hp + M_PI); // right_hip_pitch
            targetPositions[8] = convertRadianToPosition(M_PI + initialLegAngles.theta_hr); // right_hip_roll
            targetPositions[9] = 2048; // right_hip_yaw
        }else{
            // 左腳移動到 (x, y, z)，右腳回到初始位置 (x=0, y=0, z=20)
            Serial.println("Moving Left Leg to target, Right Leg to initial position");
            IKAngles rightPreviousPos = calculateInverseKinematics(x, y, initialZPos); // Keep the right foot in the previous position
            targetPositions[0] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // left_ankle_roll
            targetPositions[1] = convertRadianToPosition(targetLegAngles.theta_ap - M_PI); // left_ankle_pitch
            targetPositions[2] = convertRadianToPosition(-(targetLegAngles.theta_hp + M_PI)); // left_hip_pitch
            targetPositions[3] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr) + stance; // left_hip_roll
            targetPositions[4] = initialLegPositions[4]; // left_hip_yaw
            targetPositions[5] = convertRadianToPosition(M_PI + targetLegAngles.theta_ar); // right_ankle_roll
            targetPositions[6] = convertRadianToPosition(-(rightPreviousPos.theta_ap - M_PI)); // right_ankle_pitch
            targetPositions[7] = convertRadianToPosition(rightPreviousPos.theta_hp + M_PI); // right_hip_pitch
            targetPositions[8] = convertRadianToPosition(M_PI + targetLegAngles.theta_hr) - stance; // right_hip_roll
            targetPositions[9] = 2048; // right_hip_yaw
        }
    } else if (y == 0 && z == initialZPos){
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

    // bool result = linearMovement(startPositions, targetPositions);

    return true;
}

bool walking(float x, float y, float z, bool stop) {
    Serial.print("Walking with: x="); Serial.print(x);
    Serial.print(", y="); Serial.print(y);
    Serial.print(", z="); Serial.println(z);

    float oneStep = x;

    if (stop) {
        // 停止時執行最終步驟
        if (currentStep == 1 || (currentStep == 2 && y > 0)) {
            Serial.println("Stopping: Moving Left Leg for final step");
            leftMovement(oneStep, y, z, true);
        } else if (currentStep == 2 && y < 0) {
            Serial.println("Stopping: Moving Right Leg for final step");
            rightMovement(oneStep, y, z, true);
        }
        currentStep = 1; // 重置為下一次步行
        return true;
    }

    if (currentStep == 1) {
        // 第一步：右腳
        Serial.println("First step: Right Leg");
        rightMovement(oneStep, y, z, false);
        currentStep = 2;
        return false; // 繼續下一個循環
    } else if (currentStep == 2) {
        // 第二步：左腳，然後右腳
        Serial.println("Left Leg");
        leftMovement(oneStep * 2, y, z, false);
        Serial.println("Right Leg");
        rightMovement(oneStep * 2, y, z, false);
        return false; // 繼續下一個循環
    }

    return true;
}