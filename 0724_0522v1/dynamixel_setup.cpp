#include "delay.h"
#include "dynamixel_setup.h"
#include <math.h>
#include <Arduino.h>

control::control(){

}
void control::init(dynamixel::PortHandler *ph, dynamixel::PacketHandler *pkh) {
  portHandler = ph;
  packetHandler = pkh;
}


void control::motor_address(){
  // define porthandler 
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  // for X-series use differen id to separate two kind motor
  // dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_TORQUE, 1);
  for (int id = 1; id <= 15; ++id) {
    // setup indrectadress
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_TORQUE, ADDR_PRO_X_TORQUE_ENABLE);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_X_PROFILE_VELOCITY + 0);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_X_PROFILE_VELOCITY + 1);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_X_GOAL_POSITION + 0);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_X_GOAL_POSITION + 1);

  }
  for (int id : {DXL20_ID, DXL21_ID, DXL26_ID, DXL27_ID}) {
    // setup indrectadress
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_TORQUE, ADDR_PRO_X_TORQUE_ENABLE);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_X_PROFILE_VELOCITY + 0);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_X_PROFILE_VELOCITY + 1);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_X_GOAL_POSITION + 0);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_X_GOAL_POSITION + 1);

  }
  // for pro_series
  for (int id : {DXL16_ID, DXL17_ID, DXL18_ID, DXL19_ID, DXL22_ID, DXL23_ID, DXL24_ID, DXL25_ID}) {
    // setup indrectadress
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_TORQUE, ADDR_PRO_H_TORQUE_ENABLE);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_H_PROFILE_VELOCITY + 0);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_H_PROFILE_VELOCITY + 1);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_H_PROFILE_VELOCITY + 2);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_H_PROFILE_VELOCITY + 3);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_H_GOAL_POSITION + 0);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_H_GOAL_POSITION + 1);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 12, ADDR_PRO_H_GOAL_POSITION + 2);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 14, ADDR_PRO_H_GOAL_POSITION + 3);
  }
}

void control::motor_torque(){
  // Initialize GroupSyncWrite instance to initial data adress
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_TORQUE, LEN_PRO_GOAL_AND_VELOCITY);
  uint8_t dxl_error = 0;
  uint8_t param[1] = {TORQUE_ENABLE};;
  // for X-series, the adress is velocity-position
  for (int id = 0;id < 27;id++) {
    if (!groupSyncWrite.addParam(id+1, param)){
      //serial.print("Failed to add parameter for ID: ");
      //serial.println(id);
    } 
  }
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //serial.print("Failed to send Sync write packet: ");
    //serial.println(packetHandler->getTxRxResult(dxl_comm_result));
  }
  groupSyncWrite.clearParam();
}


void control::walking_groupSyncWrite(int profile_velocity[12], int goal_position[12]){
  // Initialize GroupSyncWrite instance to initial data adress
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_WRITE, LEN_PRO_GOAL_AND_VELOCITY);
  uint8_t dxl_error = 0;
  int id = 16;
  // for X-series, the adress is velocity-position
  for (int i = 0;i < 12;i++) {
    // add information for every motor
    id = i + 16;
    uint8_t param_goal_position_velocity[LEN_PRO_GOAL_AND_VELOCITY];
    param_goal_position_velocity[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity[i]));
    param_goal_position_velocity[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity[i]));
    param_goal_position_velocity[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity[i]));
    param_goal_position_velocity[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity[i]));
    param_goal_position_velocity[4] = DXL_LOBYTE(DXL_LOWORD(goal_position[i]));
    param_goal_position_velocity[5] = DXL_HIBYTE(DXL_LOWORD(goal_position[i]));
    param_goal_position_velocity[6] = DXL_LOBYTE(DXL_HIWORD(goal_position[i]));
    param_goal_position_velocity[7] = DXL_HIBYTE(DXL_HIWORD(goal_position[i]));
    if (!groupSyncWrite.addParam(id, param_goal_position_velocity)){
      //serial.print("Failed to add parameter for ID: ");
      //serial.println(id);
    } 
  }
  // Send the packet to all Dynamixels
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //serial.print("Failed to send Sync write packet: ");
    //serial.println(packetHandler->getTxRxResult(dxl_comm_result));
    return;
  }
  groupSyncWrite.clearParam();
}


void control::Inverse_kinematic(float end_point_x, float end_point_y, float end_point_z, float end_point_theta, int RL){
  float L6 = -5.8;
  float l = 25.0;
  makeTransformMatrix(0, 0, end_point_theta, end_point_x, end_point_y, end_point_z);
  // 拆解 T 變數
  float nx = T[0][0];
  float ny = T[1][0];
  float nz = T[2][0];

  float ox = T[0][1];
  float oy = T[1][1];
  float oz = T[2][1];

  float ax = T[0][2];
  float ay = T[1][2];
  float az = T[2][2];

  float px = T[0][3];
  float py = T[1][3];
  float pz = T[2][3];
  float px2 = px + L6 * ax;
  float py2 = py + L6 * ay;
  float pz2 = pz + L6 * az;
  float L = sqrt(px2 * px2 + py2 * py2 + pz2 * pz2);
  theta[RL+3] = acos((L * L) / (2 * l * l) - 1);
  float a = acos(L / (2 * l));

  theta[RL+5] = atan2(py + l * ay, pz + l * az);
  theta[RL+4] = -atan2(px + l * ax, sqrt((py + l * ay) * (py + l * ay) + (pz + l * az) * (pz + l * az))) - a;

  float s6 = sin(theta[RL+5]);
  float c6 = cos(theta[RL+5]);
  float c45 = cos(theta[RL+3] + theta[RL+4]);
  float s45 = sin(theta[RL+3] + theta[RL+4]);
  float R_21 = ny * c45 + oy * s6 * s45 + ay * c6 * s45;
  float R_22 = oy * c6 - ay * s6;
  float R_13 = -nx * s45 + ox * s6 * c45 + ax * c6 * c45;
  float R_23 = -ny * s45 + oy * s6 * c45 + ay * c6 * c45;
  float R_33 = -nz * s45 + oz * s6 * c45 + az * c6 * c45;

  theta[RL+0] = atan2(R_13, R_33);
  float s1 = sin(theta[RL+0]);
  float c1 = cos(theta[RL+0]);

  theta[RL+1] = atan2(-R_23, R_13*s1+R_33*c1);
  theta[RL+2] = atan2(R_21, R_22);
  for (int i = 0; i < 6; i++) {
    Serial.print("theta["); Serial.print(RL + i); Serial.print("] = "); Serial.println(theta[RL + i]);
  }
}

void control::makeTransformMatrix(float roll, float pitch, float yaw, float px, float py, float pz) {
  // 三角函數
  float cx = cos(roll);
  float sx = sin(roll);
  float cy = cos(pitch);
  float sy = sin(pitch);
  float cz = cos(yaw);
  float sz = sin(yaw);

  // ZYX rotation matrix
  float R[3][3] = {
    {cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx},
    {sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx},
    {-sy,     cy * sx,                cy * cx}
  };

  // 填入旋轉矩陣
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      T[i][j] = R[i][j];
    }
  }

  // 位置向量 px, py, pz
  T[0][3] = px;
  T[1][3] = py;
  T[2][3] = pz;

  // 最下面那一列
  T[3][0] = 0;
  T[3][1] = 0;
  T[3][2] = 0;
  T[3][3] = 1;
}

void control::rad2motor(int RL){
  float X_pi2output = 4096/(2*M_PI);
  float H_pi2output = 303750/(2*M_PI);
  float l = 5.7;
  int d = 4;
  float theta5_origin = theta[RL+4];
  float theta6_origin = theta[RL+5];
  float x = (d*sin(theta6_origin));
  float theta56_new = asin(x/l);
  if (RL == 0) {
    theta[4] = theta56_new-theta5_origin;
    theta[5] = theta56_new+theta5_origin;
  }
  else {
    theta[RL+4] = -theta56_new-theta5_origin;
    theta[RL+5] = -theta56_new+theta5_origin;
  }
  for (int i = 0;i < 4;i++){
    motor_position[RL+i] = round(theta[RL+i]*H_pi2output);
  }
  for (int i = 4;i < 6;i++){
    motor_position[RL+i] = round(theta[RL+i]*X_pi2output);
  }
  if (RL == 0) {
    motor_position[0] = relative_position[0]-motor_position[0];
    motor_position[1] = relative_position[1]+motor_position[1];
    motor_position[3] = relative_position[3]-motor_position[3];
    motor_position[4] = relative_position[4]+2048+motor_position[4];
    motor_position[5] = relative_position[5]+2048+motor_position[5];
  }
  else {
    motor_position[RL+0] = relative_position[RL+0]+motor_position[RL+0];
    motor_position[RL+1] = relative_position[RL+1]+motor_position[RL+1];
    motor_position[RL+3] = relative_position[RL+3]+motor_position[RL+3];
    motor_position[RL+4] = relative_position[RL+4]+2048-motor_position[RL+4];
    motor_position[RL+5] = relative_position[RL+5]+2048-motor_position[RL+5];
  }
  motor_position[RL+2] = relative_position[RL+2]-motor_position[RL+2];
}

void control::linearMovement(int startPositions[], int targetPositions[]) {
  // 定義速度和位置陣列
  int profile_velocity[12] = {0}; // 速度可以根據需求設定，預設為 0 或根據你的應用調整
  int currentPositions[12];

  // 線性插值步數
  for (int i = 0; i < steps; i++) {
    // 使用餘弦插值實現平滑運動，與 robotController.cpp 的方式一致
    // float t = 0.5 * (1 - cos(M_PI * i / (steps - 1)));
    float t = (float)(i + 1) / steps;
    Serial.print("start");Serial.println(startPositions[0]);
    Serial.print("target");Serial.println(targetPositions[0]);
    // 計算當前插值位置
    for (int j = 0; j < 12; j++) {
      currentPositions[j] = startPositions[j] + (targetPositions[j] - startPositions[j]) * t;
      // Serial.println(currentPositions[j]);
    }

    walking_groupSyncWrite(profile_velocity, currentPositions);
    // delay(20);
    // Serial.println("next");

  }
}

void control::moveToPos(float x, float y, float z, float theta, int RL,int swing) {
    Serial.println("Start Start moveToPos!");
    Serial.print("Input Position: x="); Serial.print(x);
    Serial.print(", y="); Serial.print(y);
    Serial.print(", z="); Serial.println(z);
    Serial.print(", theta="); Serial.println(theta);

    // 儲存起始位置
    
    // startPositions[i] = origin_position[i];
    for (int i = 0; i < 12; i++) {
        startPositions[i] = last_position[i];
    }
    
    // float L = 25.0; // 腿長
    // if (sqrt(x * x + y * y + z * z) > 2 * L) {
    //     Serial.println("Position out of range!");
    //     return;
    // }

    // 計算初始位置
    int leftInitialPos[6];
    Inverse_kinematic(0, 0, initialZPos, 0, 0); // 左腳初始位置
    rad2motor(0);
    for (int i = 0; i < 6; i++) {
        leftInitialPos[i] = motor_position[i];
    }

    int rightInitialPos[6];
    Inverse_kinematic(0, 0, initialZPos, 0, 6); // 右腳初始位置
    rad2motor(6);
    for (int i = 0; i < 6; i++) {
        rightInitialPos[i] = motor_position[i + 6];
    }

    // 計算目標位置
    int targetPositions[12];
    x = x + initialXPos;
    y = y + initialYPos; // y>0 left, y<0
    z = initialZPos - z;

    if (RL == 0) { // 左
        if (y == initialYPos && z == initialZPos){
          Inverse_kinematic(x, initialYPos, initialZPos, theta, 0);
          rad2motor(0);
          for (int i = 0; i < 6; i++) {
              targetPositions[i] = motor_position[i];
          }

          // 右
          Inverse_kinematic(initialXPos, initialYPos, initialZPos, theta, 6);
          rad2motor(6);
          for (int i = 0; i < 6; i++) {
              targetPositions[i + 6] = motor_position[i + 6];
          }
        }else{
          if (z == initialZPos) {
            // 左
            Inverse_kinematic(x, 0, initialZPos, theta, 0);
            rad2motor(0);
            for (int i = 0; i < 6; i++) {
                targetPositions[i] = motor_position[i];
            }
            // 右
            Inverse_kinematic(-x, 0, initialZPos, theta, 6);
            rad2motor(6);
            for (int i = 0; i < 6; i++) {
                targetPositions[i + 6] = motor_position[i + 6];
            }
        } else {
            // 左
            Inverse_kinematic(x, y, z, theta, 0);
            rad2motor(0);
            for (int i = 0; i < 6; i++) {
                targetPositions[i] = motor_position[i];
            }
            // Serial.println(targetPositions[0]);
            // Serial.println(motor_position[0]);
            // 右
            Inverse_kinematic(x, y, initialZPos, theta, 6);
            rad2motor(6);
            for (int i = 0; i < 6; i++) {
                targetPositions[i + 6] = motor_position[i + 6];
            }
          }
        }
    } else if (RL == 6) { // 右
        if (y == initialYPos && z == initialZPos){
          Inverse_kinematic(initialXPos, initialYPos, initialZPos, theta, 0);
          rad2motor(0);
          for (int i = 0; i < 6; i++) {
              targetPositions[i] = motor_position[i];
          }
          // 右
          Inverse_kinematic(x, initialYPos, initialZPos, theta, 6);
          rad2motor(6);
          for (int i = 0; i < 6; i++) {
              targetPositions[i + 6] = motor_position[i + 6];
          }
        }else{
          if (z == initialZPos) {
              // 左
              Inverse_kinematic(-x, 0, initialZPos, theta, 0);
              rad2motor(0);
              for (int i = 0; i < 6; i++) {
                  targetPositions[i] = motor_position[i];
              }
              // 右
              Inverse_kinematic(x, 0, initialZPos, theta, 6);
              rad2motor(6);
              for (int i = 0; i < 6; i++) {
                  targetPositions[i + 6] = motor_position[i + 6];
              }
          } else {
              // 左
              Inverse_kinematic(x, y, initialZPos, theta, 0);
              rad2motor(0);
              for (int i = 0; i < 6; i++) {
                  targetPositions[i] = motor_position[i];
              }
              // 右
              Inverse_kinematic(x, y, z, theta, 6);
              rad2motor(6);
              for (int i = 0; i < 6; i++) {
                  targetPositions[i + 6] = motor_position[i + 6];
              }
          }
        }
    }

    for (int i = 0; i < 12; i++) {
      last_position[i] = targetPositions[i];
    }
    linearMovement(startPositions, targetPositions);    
}

bool control::leftMovement(float x, float y, float z, float theta, bool stop){
    if (theta > 0){
      theta = theta;
    }else{
      theta = 0;
    }
  // left y>0
    if (stop){
        // float x, float y, float z, float theta, int RL,int swing
        moveToPos(x, -y, z, 0, 0,0);  // right
        moveToPos(x, 0, 0, 0, 0,1);  // right
    }else{
        moveToPos(0, y, z, theta, 0,0);  // left
        moveToPos(x, y, 0, theta, 0,1);  // left
        // delay(5000);
    }
    return true; 
}

bool control::rightMovement(float x, float y, float z, float theta, bool stop){
  // right y<0
    if (theta > 0){
      theta = 0;
    }else{
      theta = theta;
    }

    if (stop){
      //  float x, float y, float z, float theta, int RL,int swing
        moveToPos(x, y, z, 0, 6,0);  // right
        moveToPos(x, 0, 0, 0, 6,1);  // right
    }else{
        moveToPos(0, -y, z, theta, 6,0);  // right
        // delay(5000);
        moveToPos(x, -y, 0, theta, 6,1);  // right
        // delay(5000);
    }
    return true; 
}

int currentStep = 1;

bool control::walking(float x, float y, float z, float theta, bool stop) {
    Serial.print("Walking - currentStep: "); Serial.println(currentStep);
    Serial.print("x="); Serial.print(x);
    Serial.print(", y="); Serial.print(y);
    Serial.print(", z="); Serial.println(z);
    Serial.print(", theta="); Serial.println(theta);

    float oneStep = x;

    if (stop) {
        Serial.println("Stopping...");
        if (currentStep == 1 || (currentStep == 2 && y > 0)) {
            Serial.println("Stopping: Moving Left Leg for final 1/2x step");
            leftMovement(oneStep, y, z, theta, true);
        } else if (currentStep == 2 && y < 0) {
            Serial.println("Stopping: Moving Right Leg for final 1/2x step");
            rightMovement(oneStep, y, z, theta, true);
        }
        currentStep = 1; // 重置為下一次步行
        return true;
    }

    if (currentStep == 1) {
        Serial.println("First step: Right Leg");
        rightMovement(oneStep, y, z, theta, false);
        currentStep = 2;
        Serial.println("Switching to currentStep = 2");
        return false; // 繼續步行
    } 
    if (currentStep == 2) {
        Serial.println("Second step: Left Leg");
        leftMovement(x, y, z, theta, false);
        Serial.println("Second step: Right Leg");
        rightMovement(x, y, z, theta, false);
        return false; // 繼續步行
    }

    return true;
}

control setupControl = control();