#include <Servo.h>

Servo servoX;  // Pan
Servo servoY;  // Tilt

int xAngle = 90;
int yAngle = 90;
String inputString = "";

void setup() {
  Serial.begin(115200);
  servoX.attach(7);
  servoY.attach(8);

  servoX.write(xAngle);
  servoY.write(yAngle);
  Serial.println("Ready to track...");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      // Parse "x,y"
      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        xAngle = inputString.substring(0, commaIndex).toInt();
        yAngle = inputString.substring(commaIndex + 1).toInt();

        // Clamp values (0–180)
        xAngle = constrain(xAngle, 0, 180);
        yAngle = constrain(yAngle, 0, 180);

        servoX.write(xAngle);
        servoY.write(yAngle);
        Serial.print("X: ");
        Serial.print(xAngle);
        Serial.print(" Y: ");
        Serial.println(yAngle);
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }
}
