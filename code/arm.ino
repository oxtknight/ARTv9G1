#include<stdio.h>
#include<SoftwareSerial.h>
#include<Servo.h>
#include<math.h>


SoftwareSerial blub(10,11);
Servo base , shoulder, elbow , gripper;
const float L1 = 12;
const float L2 =15;
const float bh =9;
const float pi = 3.14159265359;
int offset1 = 0 ;
int offset2 = 0;

void setup(){
Serial.begin(115200);
blub.begin(9600);
base.attach(3);//theta 1 
shoulder.attach(5);//theta 2
elbow.attach(6);//theta 3
gripper.attach(9);//theta 4
//homing code
base.write(90);
shoulder.write(90);
elbow.write(90);
gripper.write(90);
Serial.println("sys ready , use sliders for challenge 1 & 2 , textboxes for challenge 3!!");

}

void loop(){
  //for cchallenge 1 and 2 
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
      value = constrain(value,0,180);
      if (header == 'B') base.write(value);
      if (header == 'S') shoulder.write(value);
      if (header == 'E') elbow.write(value);
      if (header == 'G') movegripper(value);
    }
  }
}
//this is for challenge 3 
void calculateIK(float x, float y, float z){
float theta1 = atan2(y,x);
float r = sqrt(x*x + y*y);
float zr = z - bh;
float s = sqrt(r*r + zr*zr);
if (s> (L1+L2) || s < abs(L1-L2)){
  Serial.println("the target is unreachable since the distance is not in the limits");
  return;
}
float costheta3 = (L1*L1 + L2*L2 -s*s)/(2*L1*L2);
float theta3 = acos(constrain(costheta3,-1,1));
float alpha = atan2(zr,r);
float cosbeta = (L1*L1 + s*s - L2*L2)/(2*L1*s);
float beta =  acos(constrain(cosbeta,-1,1));
float theta2 = alpha + beta;
int bmove = (theta1* 180.0 / pi) + 90;
int smove = (theta2* 180.0 /pi) + offset1;//gotta figure out the offsets
int emove = (180 -(theta3* 180.0 /pi)) + offset2;
 base.write(constrain(bmove, 0 , 180));
 shoulder.write(constrain(smove, 0, 180));
 elbow.write(constrain(emove,0,180));
 Serial.println("Arm goes to target:"); 
 Serial.print(x);
 Serial.print(",");
 Serial.print(y);
 Serial.print(",");
 Serial.print(z);
}

void movegripper(int angle){
  int cangle = constrain(angle,20,160); //cangle cuz of constrained angle . get it :>?
  gripper.write(cangle);
  if (cangle < 50){
    Serial.println("gripper is closed");
  } else {
    Serial.println("gripper is open");
  }
}