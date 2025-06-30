#include <DynamixelWorkbench.h>
#include <math.h>
#include "robotController.h"

void setup() {
    delay(200);
    Serial.begin(115200);
    delay(200);

    Serial.println("=== Program Start ===");
    delay(200);

    if (!initializePosition()) {
        Serial.println("Initial position setting failed!");
        // while(1);
    }
}

void loop() {
  delay(200);
  // put your main code here, to run repeatedly:
  Serial.println("right");
  
  moveToPos(20,10,40);
  delay(200);
  moveToPos(20,10,0);
  delay(100);

  while(1){
    Serial.println("left");
    moveToPos(40,-10,40);
    delay(200);
    moveToPos(40,-10,0);
    delay(100);

    moveToPos(10,10,30);
    delay(200);
    moveToPos(10,10,0);
    delay(100);
    }

}