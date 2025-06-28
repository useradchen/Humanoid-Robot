#include <DynamixelWorkbench.h>
#include <math.h>
#include "robotController.h"

void setup() {
    delay(200);
    Serial.begin(115200);
    delay(200);

    Serial.println("=== Program Start ===");
    if (!initializePosition()) {
        Serial.println("Initial position setting failed!");
        // while(1);
    }
}

void loop() {
  delay(200);
  // put your main code here, to run repeatedly:
  Serial.println("right");
  moveToPos(5,5,5);

  Serial.println("left");
  moveToPos(5,-5,5);
  delay(5000);
}
