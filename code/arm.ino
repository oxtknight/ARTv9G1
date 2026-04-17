#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>
SoftwareSerial blub(11, 10); 
Servo base, shoulder, elbow, gripper;
const float L1 = 12.0; 
const float L2 = 15.0;
const float bh = 9.0; 
const float pi = 3.14159265;
int offset1 = 0; 
int offset2 = 0; 

void setup() {
    Serial.begin(9600); 
    blub.begin(9600);
    base.attach(3);//theta1
    shoulder.attach(5);//theta2
    elbow.attach(6);//theta3
    gripper.attach(9);
   //homing code
    base.write(90);
    shoulder.write(90);
    elbow.write(90);
    gripper.write(90);
  Serial.println("sys ready ,use sliders for challenge 1 and 2, use textboxes for challenge 3!!");

}

void loop() {
  //this code for challenge 3
    if (blub.available() > 0) {
        char header = blub.read();
        String data = blub.readStringUntil('\n');
        data.trim();
        if (header == 'K') {
            float tx, ty, tz;
            if (sscanf(data.c_str(), "%f,%f,%f", &tx, &ty, &tz) == 3) {
                calculateIK(tx, ty, tz);
            }
        }
        else {
            int value = data.toInt();
            int safevalue = constrain(value, 0, 180);
            if (header == 'B') base.write(safevalue);
            if (header == 'S') shoulder.write(safevalue);
            if (header == 'E') elbow.write(safevalue);
            if (header == 'G') movegripper(safevalue);
        }
    }
}
void calculateIK(float x, float y, float z) {
    float theta1 = atan2(y, x);
    float r = sqrt(x*x + y*y);
    float zr = z - bh;
    float s = sqrt(r*r + zr*zr);
  //this to check if our arm can reach the target 
    if (s > (L1 + L2) || s < abs(L1 - L2)) {
        Serial.println("the target is unreachable since the distance is not in the limits");
        return;
    }
    float costheta3 = (L1*L1 + L2*L2 - s*s) / (2.0 * L1 * L2);
    float theta3 = acos(constrain(costheta3, -1, 1)); 
    float alpha = atan2(zr, r);
    float cosbeta = (L1*L1 + s*s - L2*L2) / (2.0 * L1 * s);
    float beta = acos(constrain(cosbeta, -1, 1));
    float theta2 = alpha + beta;
// here converted to degrees brr
    int bmove = (theta1 * 180.0 / pi) + 90;
    int smove = (theta2 * 180.0 / pi) + offset1;
    int emove = (180 - (theta3 * 180.0 / pi)) + offset2; 
    base.write(constrain(bmove, 0, 180));
    shoulder.write(constrain(smove, 0, 180));
    elbow.write(constrain(emove, 0, 180));
    Serial.print("coordinates: "); Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(","); Serial.println(z);
    Serial.print("servos angles: B:"); Serial.print(bmove); Serial.print(" S:"); Serial.print(smove); Serial.print(" E:"); Serial.println(emove);
}

void movegripper(int angle) {
    int cangle = constrain(angle, 20, 160); 
    gripper.write(cangle);
    if (cangle < 50) Serial.println("Gripper: CLOSED");
    else Serial.println("Gripper: OPEN");
}
