#ifndef DYNAMIXEL_SETUP_H
#define DYNAMIXEL_SETUP_H

#include <DynamixelSDK.h>

//indirect address setup
// Control table address
#define ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE      578 // X-series
#define ADDR_PRO_X_INDIRECTADDRESS_FOR_READ       594
#define ADDR_PRO_X_INDIRECTADDRESS_FOR_TORQUE     602
#define ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE      49  // Pro-series
#define ADDR_PRO_H_INDIRECTADDRESS_FOR_READ       65
#define ADDR_PRO_H_INDIRECTADDRESS_FOR_TORQUE     73
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE           634
#define ADDR_PRO_INDIRECTDATA_FOR_READ            642
#define ADDR_PRO_INDIRECTDATA_FOR_TORQUE          646


//X-Serise
// Control table address
#define ADDR_PRO_X_TORQUE_ENABLE          64
#define ADDR_PRO_X_PROFILE_VELOCITY       112
#define ADDR_PRO_X_GOAL_POSITION          116
#define ADDR_PRO_X_PRESENT_POSITION       132
//===================================================================================================

// Pro-Series
// Control table address
#define ADDR_PRO_H_TORQUE_ENABLE          562
#define ADDR_PRO_H_GOAL_POSITION          596
#define ADDR_PRO_H_PROFILE_VELOCITY       600
#define ADDR_PRO_H_PRESENT_POSITION       611
//====================================================================================================

// Data Byte Length
#define LEN_PRO_GOAL_AND_VELOCITY       8
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0

// Default settings
#define BAUDRATE                        1000000
#define DEVICENAME                      "OpenCR_DXL_Port"  // Port name used by OpenCR

// Torque control
#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

// Dynamixel IDs
#define DXL1_ID                         1
#define DXL2_ID                         2
#define DXL3_ID                         3
#define DXL4_ID                         4
#define DXL5_ID                         5
#define DXL6_ID                         6
#define DXL7_ID                         7
#define DXL8_ID                         8
#define DXL9_ID                         9
#define DXL10_ID                        10
#define DXL11_ID                        11
#define DXL12_ID                        12
#define DXL13_ID                        13
#define DXL14_ID                        14
#define DXL15_ID                        15
#define DXL16_ID                        16
#define DXL17_ID                        17
#define DXL18_ID                        18
#define DXL19_ID                        19
#define DXL20_ID                        20
#define DXL21_ID                        21
#define DXL22_ID                        22
#define DXL23_ID                        23
#define DXL24_ID                        24
#define DXL25_ID                        25
#define DXL26_ID                        26
#define DXL27_ID                        27

class control{
// Function declarations
  public:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    control();
    void init(dynamixel::PortHandler *ph, dynamixel::PacketHandler *pkh);
    void motor_address();
    void motor_torque();
    void walking_groupSyncWrite(int profile_velocity[12], int goal_position[12]);

    void linearMovement(int startPositions[], int targetPositions[]);
    void moveToPos(float x, float y, float z, float theta, int RL,int swing);
    bool leftMovement(float x, float y, float z, float theta, bool stop);
    bool rightMovement(float x, float y, float z, float theta, bool stop);
    bool walking(float x, float y, float z, float theta, bool stop);

    void Inverse_kinematic(float end_point_x, float end_point_y, float end_point_z, float end_point_theta, int RL);
    void makeTransformMatrix(float roll, float pitch, float yaw, float px, float py, float pz);
    void rad2motor(int RL);
    float theta[12];
    float past_theta[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int motor_position[12];
    int profile_velocity[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int relative_position[12] = {6000, 0, 0, 0, 0, 0, -6000, 0, 0, 0, 0, 0};
    int origin_position[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int last_position[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int startPositions[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float T[4][4];
    int steps = 300;
// xyz 初始位置
    float initialXPos = 0.0;
    float initialYPos = 0.0;
    float initialZPos = 50.0;
};



extern control setupControl;

#endif // DYNAMIXEL_SETUP_H
