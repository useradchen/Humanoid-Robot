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
extern Position currentleftPos;  // current position
extern Position targetleftPos;   // target position
extern Position currentRightPos;  // current position
extern Position targetRightPos;   // target position
extern const Position INIT_LEFT_POS;
extern const Position INIT_RIGHT_POS;
extern int32_t leftLegInitialPos[5];
extern int32_t rightLegInitialPos[5];
extern int32_t heaPos[2];
extern float initialHeight;
extern float balanceVar;
extern float MAX_ROLL_ANGLE;
extern int numInterpolationPoints;


// Function declarations
IKAngles calculateInverseKinematics(float x, float y, float z);
int32_t convertRadianToPosition(float radian);
int32_t convertDegreeToPosition(float degree);
bool readAllActuatorPositions(int32_t* positions);
float smoothInterpolation(float t);
bool moveToCoordinate(float x, float y, float z, float rotationAngle, bool isLeftLeg, bool lift, float stepLength);
bool kickMoveToCoordinate(float x, float y, float z, float rotationAngle, bool isLeftLeg, bool lift, float stepLength);
bool initializePosition();
float walking(float& stepLength, float& rotationAngle, float& sideStepLength);
float kick(float& stepLength, float& rotationAngle, float& sideStepLength);

// Linear trajectory class
class LinearLegTrajectory {
public:    
    static bool generateSmoothTrajectory(
        Position currentPositionLeft, 
        Position targetPositionLeft, 
        Position currentPositionRight, 
        Position targetPositionRight,
        float rotationAngle, 
        bool isLeftLeg,
        bool lift, 
        float stepLength,
        void (*moveCallback)(float, float, float, float, float, float, float, bool, bool, float)
    );
};

#endif // ROBOT_CONTROLLER_H