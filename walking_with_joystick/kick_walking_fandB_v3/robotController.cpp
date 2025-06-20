#include "robotController.h"
#include <algorithm>

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

// Global variables definition
DynamixelWorkbench dxl_wb;
float LENGTH = 180.0;  // leg length
float MAX_ROLL_ANGLE = 3.0f; // swing 3
float balanceVar = 5.0f;
int numInterpolationPoints = 200;//180
float initialHeight = 20.0;  // init z
float liftHeight = 120; // (liftHeight = x - baseZ) => liftHeight > x

uint8_t dxl_id[num_actuators] = {left_ankle_roll, left_ankle_pitch, left_hip_pitch, left_hip_roll, left_hip_yaw, 
                      right_ankle_roll, right_ankle_pitch, right_hip_pitch, right_hip_roll, right_hip_yaw,
                      head_yaw,head_pitch};

const uint8_t handlerIndex = 0;
float checkX = 0.0f;
Position currentleftPos = {0, 0, 20};  // current position
Position targetleftPos = {0, 0, 0};   // target position
Position currentRightPos = {0, 0, 20};  // current position
Position targetRightPos = {0, 0, 0};   // target position

int32_t leftLegInitialPos[5] = {2048,2048,2048,2048,2048};   // left initial position

int32_t rightLegInitialPos[5] = {2048,2048,2048,2048,2048};  // right initial position

int32_t heaPos[2] = {512,512};

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
  // angles.theta_ap = acos(fabs(x) / d) + theta_hp2 - M_PI/2;
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
        Map the input degree to 4096 units (0-4095).
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

bool LinearLegTrajectory::generateSmoothTrajectory(
    Position currentPositionLeft, 
    Position targetPositionLeft, 
    Position currentPositionRight, 
    Position targetPositionRight,
    float rotationAngle, 
    bool isLeftLeg,
    bool lift, 
    float stepLength,
    void (*moveCallback)(float, float, float, float, float, float, float, bool, bool, float)
) {
    float baseZ = initialHeight; // 改basedz
    currentleftPos = currentPositionLeft;
    targetleftPos = targetPositionLeft;
    currentRightPos = currentPositionRight;
    targetRightPos = targetRightPos;

    currentleftPos.z = baseZ;
    targetleftPos.z = baseZ;
    currentRightPos.z = baseZ;
    targetRightPos.z = baseZ;

    float deltaLeftX = targetleftPos.x - currentleftPos.x;
    float deltaLeftY = targetleftPos.y - currentleftPos.y;
    float deltaRightX = targetRightPos.x - currentRightPos.x;
    float deltaRightY = targetRightPos.y - currentRightPos.y;

    float delayTime = 0.0f;

    for (int i = 0; i <= numInterpolationPoints; ++i) {
        float t = (float)i / numInterpolationPoints;
        float sinFactor = sin(t * M_PI);
        float cosFactor = (1.0f - cos(t * M_PI)) / 2.0f;

        float interpolatedLeftX = currentleftPos.x + cosFactor * deltaLeftX;
        float interpolatedRightX = currentRightPos.x + cosFactor * deltaRightX;

        float interpolatedLeftY = currentleftPos.y + sinFactor * deltaLeftY;
        float interpolatedRightY = currentRightPos.y + sinFactor * deltaRightY;

        float interpolatedLeftZ = baseZ;
        float interpolatedRightZ = baseZ;
        if (lift) {
            if (isLeftLeg) {
                interpolatedLeftZ = baseZ + sinFactor * (liftHeight - baseZ);
            } else {
                interpolatedRightZ = baseZ + sinFactor * (liftHeight - baseZ);
            }
        }
        if (t > 0.5 && !lift) {
            float remainingT = (t - 0.5) / 0.5;
            interpolatedLeftY = targetleftPos.y - (1.0f - remainingT) * (targetleftPos.y - currentleftPos.y);
            interpolatedRightY = targetRightPos.y - (1.0f - remainingT) * (targetRightPos.y - currentRightPos.y);
        }

        if (t > 0.99) {
            interpolatedLeftZ = baseZ;
            interpolatedRightZ = baseZ;
        }        
        if (t > 0.99) {
            interpolatedLeftZ = baseZ;
            interpolatedRightZ = baseZ;
        }

        if (interpolatedLeftZ <= 0.0f || interpolatedRightZ <= 0.0f) {
            interpolatedLeftZ = baseZ;
            interpolatedRightZ = baseZ;
        }

        float appliedRotationAngle = 0.0f;
        if (t >= 0.5) {
            if (isLeftLeg && rotationAngle > 0) {
                appliedRotationAngle = rotationAngle;
            } else if (!isLeftLeg && rotationAngle < 0) {
                appliedRotationAngle = rotationAngle;
            }
        }

        moveCallback(
            interpolatedLeftX,
            interpolatedLeftY,
            interpolatedLeftZ,
            interpolatedRightX,
            interpolatedRightY,
            interpolatedRightZ,
            appliedRotationAngle,
            isLeftLeg,
            lift,
            t
        );

        delay(delayTime);
    }

    currentleftPos = {targetleftPos.x, targetleftPos.y, baseZ};
    currentRightPos = {targetRightPos.x, targetRightPos.y, baseZ};
    targetleftPos.z = baseZ;
    targetRightPos.z = baseZ;

    return true;
}

bool moveToCoordinate(float x, float y, float z, float rotationAngle, bool isLeftLeg, bool lift, float stepLength) {
    const float baseX = 0.0f;
    const float baseY = 0.0f;
    const float baseZ = initialHeight;

    using MoveCallback = void (*)(float, float, float, float, float, float, float, bool, bool, float);
    
    static MoveCallback internalMoveCallback = [](float leftX, float leftY, float leftZ,
                                                  float rightX, float rightY, float rightZ,
                                                  float rotationAngle, bool isLeftLeg, bool lift, float t) {

        IKAngles leftLegAngles = calculateInverseKinematics(leftX, leftY, leftZ);
        IKAngles rightLegAngles = calculateInverseKinematics(rightX, rightY, rightZ);

        int32_t targetPositions[num_actuators];
        static int32_t lastTargetPositions[num_actuators] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 512, 512};

        float ankleSupportRollAngleDeg = 0.0f;
        if (lift) {
            ankleSupportRollAngleDeg = isLeftLeg ? MAX_ROLL_ANGLE : -MAX_ROLL_ANGLE;
        }

        float currentSinFactor = sin(t * M_PI);
        float ankleBalanceRollOffsetDeg = 0.0f;
        if (lift) {
            if (!isLeftLeg) {
                ankleBalanceRollOffsetDeg = MAX_ROLL_ANGLE * currentSinFactor;
            } else {
                ankleBalanceRollOffsetDeg = -MAX_ROLL_ANGLE * currentSinFactor;
            }
        }
        
        targetPositions[head_yaw - 1] = 512;
        targetPositions[head_pitch - 1] = 568;
 
        targetPositions[right_ankle_roll - 1] = convertDegreeToPosition(180.0f + ankleBalanceRollOffsetDeg);
        targetPositions[left_ankle_roll - 1] = convertDegreeToPosition(180.0f + ankleBalanceRollOffsetDeg);
       
        targetPositions[left_ankle_pitch -1] = convertRadianToPosition(leftLegAngles.theta_ap - M_PI);
        targetPositions[left_hip_pitch -1]   = convertRadianToPosition(-(leftLegAngles.theta_hp + M_PI));

        targetPositions[right_ankle_pitch -1] = convertRadianToPosition(-(rightLegAngles.theta_ap - M_PI));
        targetPositions[right_hip_pitch -1]   = convertRadianToPosition(rightLegAngles.theta_hp + M_PI);

        float balanceRollOffsetDeg = 0.0f;

        if (lift) {
            if (!isLeftLeg) {
                balanceRollOffsetDeg = MAX_ROLL_ANGLE * currentSinFactor;
            } else {
                balanceRollOffsetDeg = -MAX_ROLL_ANGLE * currentSinFactor;
            }
        }

        float leftLeg_ik_hr_deg = leftLegAngles.theta_hr * (180.0f / M_PI);
        float rightLeg_ik_hr_deg = rightLegAngles.theta_hr * (180.0f / M_PI);

        targetPositions[left_hip_roll -1] = convertDegreeToPosition(180.0f + balanceRollOffsetDeg + leftLeg_ik_hr_deg)-convertDegreeToPosition(balanceVar);
        targetPositions[right_hip_roll -1] = convertDegreeToPosition(180.0f + balanceRollOffsetDeg - rightLeg_ik_hr_deg)+convertDegreeToPosition(balanceVar);

        targetPositions[left_hip_yaw - 1] = 2048;
        targetPositions[right_hip_yaw - 1] = 2048;

        if (rotationAngle != 0.0f) {
            int32_t rotationPositionValue = convertDegreeToPosition(180 + rotationAngle);
            if (rotationAngle > 0) {
                if (isLeftLeg) {
                    targetPositions[right_hip_yaw - 1] = rotationPositionValue;
                } else {
                    targetPositions[left_hip_yaw - 1] = rotationPositionValue;
                }
            } else if (rotationAngle < 0) {
                if (!isLeftLeg) {
                    targetPositions[left_hip_yaw - 1] = rotationPositionValue;
                } else {
                    targetPositions[right_hip_yaw - 1] = rotationPositionValue;
                }
            }
        }

        for (int i = 0; i < 10; i++) {
            if (i == (left_hip_roll-1) || i == (right_hip_roll-1)) { // hip roll joints
                targetPositions[i] = 0.5 * lastTargetPositions[i] + 0.5 * targetPositions[i];
            } else {
                targetPositions[i] = 0.6 * targetPositions[i] + 0.4 * lastTargetPositions[i];
            }
            lastTargetPositions[i] = targetPositions[i];
        }

        const char *log_sync;
        dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, targetPositions, 1, &log_sync);
    };

    if (z != initialHeight) {
        Serial.println("Warning: Input Z incorrect in moveToCoordinate!");
        Serial.print("z: "); Serial.println(z);
        z = initialHeight;
    }

    if (currentleftPos.z != initialHeight || currentRightPos.z != initialHeight) {
        currentleftPos.z = initialHeight;
        currentRightPos.z = initialHeight;
    }

    targetleftPos.z = baseZ;
    targetRightPos.z = baseZ;

    if (targetleftPos.z != initialHeight || targetRightPos.z != initialHeight) {
        targetleftPos.z = initialHeight;
        targetRightPos.z = initialHeight;
    }

    Position currentPositionLeft = {currentleftPos.x, currentleftPos.y, baseZ};
    Position currentPositionRight = {currentRightPos.x, currentRightPos.y, baseZ};

  if (isLeftLeg) {
        targetleftPos = {x, y, baseZ};
        targetRightPos = {0, 0, baseZ};
    } else {
        targetRightPos = {x, y, baseZ};
        targetleftPos = {0, 0, baseZ};
    }

    Position targetPositionLeft = {targetleftPos.x, targetleftPos.y, baseZ};
    Position targetPositionRight = {targetRightPos.x, targetRightPos.y, baseZ};

    bool result = LinearLegTrajectory::generateSmoothTrajectory(
        currentPositionLeft, 
        targetPositionLeft, 
        currentPositionRight,
        targetPositionRight,
        rotationAngle,
        isLeftLeg,
        lift,
        stepLength,
        internalMoveCallback
    );

    if (result) {
        currentleftPos = {targetleftPos.x, targetleftPos.y, baseZ};
        currentRightPos = {targetRightPos.x, targetRightPos.y, baseZ};
        targetleftPos.z = baseZ;
        targetRightPos.z = baseZ;
    } else {
        Serial.println("Trajectory execution failed!");
        return false;
    }

    Serial.print("left x = ");
    Serial.println(currentleftPos.x);
    Serial.print("left y = ");
    Serial.println(currentleftPos.y);
    Serial.print("left z = ");
    Serial.println(currentleftPos.z);
    Serial.println("--------------------------------");
    Serial.print("right x = ");
    Serial.println(currentRightPos.x);
    Serial.print("right y = ");
    Serial.println(currentRightPos.y);
    Serial.print("right z = ");
    Serial.println(currentRightPos.z);
    Serial.println("--------------------------------");

    return result;
}

bool kickMoveToCoordinate(float x, float y, float z, float rotationAngle, bool isLeftLeg, bool lift, float stepLength) {
    const float baseX = 0.0f;
    const float baseY = 0.0f;
    const float baseZ = initialHeight;

    using MoveCallback = void (*)(float, float, float, float, float, float, float, bool, bool, float);
    
    static MoveCallback internalMoveCallback = [](float leftX, float leftY, float leftZ,
                                                  float rightX, float rightY, float rightZ,
                                                  float rotationAngle, bool isLeftLeg, bool lift, float t) {
        float adjustedLeftX = leftX;
        float adjustedRightX = rightX;
        if (lift) {
            if (isLeftLeg) {
                if (t < 0.3f) {
                    adjustedLeftX = -0.1f; // back 0.1 
                } else if (t < 0.46f) {
                    adjustedLeftX = leftX;
                }
            } else {
                if (t < 0.3f) {
                    adjustedRightX = -0.1f; // back 0.1
                } else if (t < 0.46f) {
                    adjustedRightX = rightX;
                }
            }
        }

        IKAngles leftLegAngles = calculateInverseKinematics(adjustedLeftX, leftY, leftZ);
        IKAngles rightLegAngles = calculateInverseKinematics(adjustedRightX, rightY, rightZ);

        int32_t targetPositions[num_actuators];
        static int32_t lastTargetPositions[num_actuators] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 512, 512};

        float currentSinFactor = sin(t * M_PI); 
        float kickPhaseFactor = 0.0f;
        if (t < 0.3f) {
            kickPhaseFactor = t / 0.3f;
        } else if (t < 0.46f) {
            kickPhaseFactor = (0.46f - t) / 0.16f;
            kickPhaseFactor = pow(1.0f - kickPhaseFactor, 3.0f);
        } else {
            kickPhaseFactor = 0.0f;
        }

        float ankleBalanceRollOffsetDeg = 0.0f;
        if (lift) {
            ankleBalanceRollOffsetDeg = isLeftLeg ? -MAX_ROLL_ANGLE * currentSinFactor : MAX_ROLL_ANGLE * currentSinFactor;
        }

        targetPositions[head_yaw - 1] = 512;
        targetPositions[head_pitch - 1] = 760; // head 
        targetPositions[left_ankle_roll - 1] = convertDegreeToPosition(180.0f + ankleBalanceRollOffsetDeg);
        targetPositions[right_ankle_roll - 1] = convertDegreeToPosition(180.0f + ankleBalanceRollOffsetDeg);

        const float kickHipPitchOffsetDeg = 30.0f; // hip pitch offset
        const float kickAnklePitchOffsetDeg = 35.0f; // ankle pitch offset
        float hipPitchOffsetRad = 0.0f;
        float anklePitchOffsetRad = 0.0f;
        if (lift) {
            if (t < 0.3f) {
                // Pull phase: Hip back, ankle slightly down
                hipPitchOffsetRad = kickHipPitchOffsetDeg * kickPhaseFactor * M_PI / 180.0f;
                anklePitchOffsetRad = -15.0f * kickPhaseFactor * M_PI / 180.0f;
            } else if (t < 0.46f) {
                // Front kick phase: hip forward, ankle forward
                hipPitchOffsetRad = -kickHipPitchOffsetDeg * kickPhaseFactor * M_PI / 180.0f;
                anklePitchOffsetRad = kickAnklePitchOffsetDeg * (1.0f - kickPhaseFactor) * M_PI / 180.0f;
            }
        }

        if (isLeftLeg) {
            targetPositions[left_ankle_pitch - 1] = convertRadianToPosition(leftLegAngles.theta_ap - M_PI + anklePitchOffsetRad);
            targetPositions[left_hip_pitch - 1] = convertRadianToPosition(-(leftLegAngles.theta_hp + M_PI + hipPitchOffsetRad));
            targetPositions[right_ankle_pitch - 1] = convertRadianToPosition(-(rightLegAngles.theta_ap - M_PI));
            targetPositions[right_hip_pitch - 1] = convertRadianToPosition(rightLegAngles.theta_hp + M_PI);
        } else {
            targetPositions[right_ankle_pitch - 1] = convertRadianToPosition(-(rightLegAngles.theta_ap - M_PI + anklePitchOffsetRad));
            targetPositions[right_hip_pitch - 1] = convertRadianToPosition(rightLegAngles.theta_hp + M_PI + hipPitchOffsetRad);
            targetPositions[left_ankle_pitch - 1] = convertRadianToPosition(leftLegAngles.theta_ap - M_PI);
            targetPositions[left_hip_pitch - 1] = convertRadianToPosition(-(leftLegAngles.theta_hp + M_PI));
        }

        // hip roll
        float balanceRollOffsetDeg = 0.0f;
        if (lift) {
            balanceRollOffsetDeg = isLeftLeg ? -MAX_ROLL_ANGLE * currentSinFactor : MAX_ROLL_ANGLE * currentSinFactor;
        }

        float leftLeg_ik_hr_deg = leftLegAngles.theta_hr * (180.0f / M_PI);
        float rightLeg_ik_hr_deg = rightLegAngles.theta_hr * (180.0f / M_PI);

        targetPositions[left_hip_roll - 1] = convertDegreeToPosition(180.0f + balanceRollOffsetDeg + leftLeg_ik_hr_deg) - convertDegreeToPosition(balanceVar);
        targetPositions[right_hip_roll - 1] = convertDegreeToPosition(180.0f + balanceRollOffsetDeg - rightLeg_ik_hr_deg) + convertDegreeToPosition(balanceVar);

        // hip yaw
        targetPositions[left_hip_yaw - 1] = 2048;
        targetPositions[right_hip_yaw - 1] = 2048;

        if (rotationAngle != 0.0f) {
            int32_t rotationPositionValue = convertDegreeToPosition(180 + rotationAngle);
            if (rotationAngle > 0) {
                if (isLeftLeg) {
                    targetPositions[right_hip_yaw - 1] = rotationPositionValue;
                } else {
                    targetPositions[left_hip_yaw - 1] = rotationPositionValue;
                }
            } else if (rotationAngle < 0) {
                if (!isLeftLeg) {
                    targetPositions[left_hip_yaw - 1] = rotationPositionValue;
                } else {
                    targetPositions[right_hip_yaw - 1] = rotationPositionValue;
                }
            }
        }

        for (int i = 0; i < 10; i++) {
            if (i == (left_hip_roll - 1) || i == (right_hip_roll - 1)) {
                targetPositions[i] = 0.5 * lastTargetPositions[i] + 0.5 * targetPositions[i];
            } else {
                targetPositions[i] = 0.7 * targetPositions[i] + 0.3 * lastTargetPositions[i];
            }
        }

        const char *log_sync;
        dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, targetPositions, 1, &log_sync);
    };

    if (z != initialHeight) {
        Serial.println("Warning: Input Z incorrect in kickMoveToCoordinate!");
        z = initialHeight;
    }

    if (currentleftPos.z != initialHeight || currentRightPos.z != initialHeight) {
        currentleftPos.z = initialHeight;
        currentRightPos.z = initialHeight;
    }

    targetleftPos.z = baseZ;
    targetRightPos.z = baseZ;

    if (targetleftPos.z != initialHeight || targetRightPos.z != initialHeight) {
        targetleftPos.z = initialHeight;
        targetRightPos.z = initialHeight;
    }

    Position currentPositionLeft = {currentleftPos.x, currentleftPos.y, baseZ};
    Position currentPositionRight = {currentRightPos.x, currentRightPos.y, baseZ};

    if (isLeftLeg) {
        targetleftPos = {x, y, baseZ};
        targetRightPos = {0, 0, baseZ};
    } else {
        targetRightPos = {x, y, baseZ};
        targetleftPos = {0, 0, baseZ};
    }

    Position targetPositionLeft = {targetleftPos.x, targetleftPos.y, baseZ};
    Position targetPositionRight = {targetRightPos.x, targetRightPos.y, baseZ};

    bool result = LinearLegTrajectory::generateSmoothTrajectory(
        currentPositionLeft, 
        targetPositionLeft, 
        currentPositionRight,
        targetPositionRight,
        rotationAngle,
        isLeftLeg,
        lift,
        stepLength,
        internalMoveCallback
    );

    if (result) {
        currentleftPos = {targetleftPos.x, targetleftPos.y, baseZ};
        currentRightPos = {targetRightPos.x, targetRightPos.y, baseZ};
        targetleftPos.z = baseZ;
        targetRightPos.z = baseZ;
    } else {
        Serial.println("Kick trajectory execution failed!");
        return false;
    }

    Serial.print("left x = "); Serial.println(currentleftPos.x);
    Serial.print("left y = "); Serial.println(currentleftPos.y);
    Serial.print("left z = "); Serial.println(currentleftPos.z);
    Serial.println("--------------------------------");
    Serial.print("right x = "); Serial.println(currentRightPos.x);
    Serial.print("right y = "); Serial.println(currentRightPos.y);
    Serial.print("right z = "); Serial.println(currentRightPos.z);
    Serial.println("--------------------------------");

    return result;
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
        
        if (!dxl_wb.jointMode(dxl_id[i], 0, 0, &log)) {
            Serial.print("Failed to set joint mode for Dynamixel ID: ");
            Serial.println(dxl_id[i]);
            return false;
        }
        
        if (!dxl_wb.torqueOn(dxl_id[i], &log)) {
            Serial.print("Failed to turn torque on for Dynamixel ID: ");
            Serial.println(dxl_id[i]);
            return false;
        }

        dxl_wb.itemWrite(dxl_id[i], "Position_P_Gain", 1200);
        dxl_wb.itemWrite(dxl_id[i], "Position_I_Gain", 200);
        dxl_wb.itemWrite(dxl_id[i], "Position_D_Gain", 800);
        dxl_wb.itemWrite(dxl_id[i], "Moving_Speed", 100); 
        dxl_wb.itemWrite(dxl_id[i], "Moving_Threshold", 15); 
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
    
    IKAngles legAngles = calculateInverseKinematics(0, 0, initialHeight);
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
        568,
    };

    result = dxl_wb.syncWrite(handlerIndex, dxl_id, num_actuators, realInitialPositions, 1, &log);
    if (!result) {
        Serial.println("Failed to enter real initial position!");
        return false;
    }
    
    return true; 
}

float walking(float& stepLength, float& rotationAngle, float& sideStepLength) {
  static bool isLeftLeg = true;
  static bool isFirstStep = true;
  static float lastStepLength = 0.0f;

  // reset isFirstStep
  if (lastStepLength != stepLength) {
    isFirstStep = true;
    lastStepLength = stepLength;
  }

  bool forwardNow = (stepLength >= 0);
  float absStepLength = fabs(stepLength);
  float stepDistance = isFirstStep ? absStepLength / 2.0 : absStepLength;
  stepDistance = forwardNow ? stepDistance : -stepDistance;
  float secondStepDistance = stepDistance * 2;

  Serial.print("Walking ");
  Serial.println(forwardNow ? "forward" : "backward");
  Serial.print("stepDistance=");
  Serial.println(stepDistance);
  Serial.print("secondStepDistance=");
  Serial.println(secondStepDistance);
  Serial.print("rotationAngle=");
  Serial.println(rotationAngle);
  Serial.print("sideStepLength=");
  Serial.println(sideStepLength);
  Serial.print("isFirstStep=");
  Serial.println(isFirstStep ? "true" : "false");
  Serial.print("isLeftLeg=");
  Serial.println(isLeftLeg ? "true" : "false");

  bool success = false;
  if (isLeftLeg) {
    success = moveToCoordinate(stepDistance, sideStepLength, initialHeight, rotationAngle, true, true, absStepLength);
    if (!success) {
      Serial.println("Left leg step error!");
      return 0.0;
    }
  } else {
    success = moveToCoordinate(secondStepDistance, sideStepLength, initialHeight, rotationAngle, false, true, absStepLength);
    if (!success) {
      Serial.println("Right leg step error!");
      return 0.0;
    }
  }

  isLeftLeg = !isLeftLeg;
  isFirstStep = false;
  return 1.0; 
}

float kick(float& stepLength, float& rotationAngle, float& sideStepLength) {
    static bool isLeftLeg = true;
    bool forwardNow = (stepLength >= 0);
    stepLength = fabs(stepLength);
    float forward = stepLength;
    float back = -forward;

    Serial.print("Kicking ");
    Serial.println(forwardNow ? "forward" : "backward");
    Serial.print("forward/back = ");
    Serial.println(forwardNow ? forward : back);
    Serial.print("rotationAngle(degree) = ");
    Serial.println(rotationAngle);
    Serial.print("sideStepLength = ");
    Serial.println(sideStepLength);
    Serial.print("isLeftLeg = ");
    Serial.println(isLeftLeg ? "true" : "false");

    if (stepLength == 0 && rotationAngle == 0 && sideStepLength == 0) {
        Serial.println("No kick movement requested (stepLength=0, rotationAngle=0, sideStepLength=0)");
        return 1.0; 
    }

    if (forwardNow) {
        if (!kickMoveToCoordinate(forward, sideStepLength, initialHeight, rotationAngle, isLeftLeg, true, stepLength)) {
            Serial.println(isLeftLeg ? "Left leg kick error!" : "Right leg kick error!");
            return 0.0;
        }
    } else {
        if (!kickMoveToCoordinate(back, sideStepLength, initialHeight, rotationAngle, isLeftLeg, true, stepLength)) {
            Serial.println(isLeftLeg ? "Left leg kick error!" : "Right leg kick error!");
            return 0.0;
        }
    }

    isLeftLeg = !isLeftLeg;
    return 1.0; 
}