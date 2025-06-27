#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <DynamixelWorkbench.h>
#include <math.h>

// Structure definitions
struct IKAngles {
    float theta_hp;
    float theta_ap;
    float theta_ar;
    float theta_hr; // ar == hr
};

struct Position {
    float x;
    float y;
    float z;
};

// Global variables declaration
extern DynamixelWorkbench dxl_wb;
extern float LENGTH;
extern uint8_t dxl_id[12];
extern const uint8_t num_actuators;
extern const uint8_t handlerIndex;
extern float initialHeight;

extern int32_t initialLegPositions[12];
extern int32_t leftLegInitialPos[5];
extern int32_t rightLegInitialPos[5];
extern int32_t heaPos[2];


// Function declarations
IKAngles calculateInverseKinematics(float x, float y, float z);
int32_t convertRadianToPosition(float radian);
int32_t convertDegreeToPosition(float degree);
bool readAllActuatorPositions(int32_t* positions);
bool initializePosition();
bool moveToPos(float x, float y, float z);
// bool walking();


#endif // ROBOT_CONTROLLER_H