
#include<SoftwareSerial.h>
#include<Servo.h>
#include<math.h>
SoftwareSerial blub(11,10);
constexpr float L1 = 12.0;
constexpr float L2 = 15.0;
constexpr float bh = 9.0;
constexpr float pi = 3.14159265;
float storedx = 0, storedy=0,storedz=0;
struct joint{
  Servo motor;
  byte pin;
  float currentangle;
  float targetangle;
  float offset;
};
joint b = {Servo(),3,90,90,0};
joint sh = {Servo(),5,90,90,0};
joint e = {Servo(),6,90,90,0};
joint g = {Servo(),9,90,90,0};
unsigned long laststeptime= 0;
const int stepdelay=10;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  blub.begin(9600);
  b.motor.attach(b.pin);
  sh.motor.attach(sh.pin);
  e.motor.attach(e.pin);
  g.motor.attach(g.pin);
  Serial.println("sys ready ,use sliders for challenge 1 and 2, use textboxes for challenge 3!!");
}
void loop() {
    handleserial(); 
    updatemotion(); 
    
}
void handleserial() {
  //so this function is to clear buffer so that we have and save the data from the app 
    static String inputBuffer = "";
    
    while (blub.available() > 0) {
        char c = blub.read();
        if (c == '\n') {
            processcmd(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}
//this the function that processes the recieved data from the app and just put it in a buffer to recheck it later on
void processcmd(String cmd) {
    //i be listening to love dramatic "misterr oh misterrr ゆみじゃないならきかせて "
    cmd.trim();
    if (cmd.length() < 2)
    return;

    char header = cmd[0];
    if (header == 'K'){
        char axis = cmd[1];
        float value = cmd.substring(2).toFloat();
        //this time i be listening to heartache :) pretty cool
        if (axis == 'x'){
             storedx = value;
             Serial.print("Detected X axis = "); Serial.print(storedx);
             }
        else if (axis == 'y') {
             storedy = value;
             Serial.print("Detected Y axis ="); Serial.print(storedy);
             }
        else if (axis == 'z')  {
            storedz = value;
            Serial.print("Detected Z axis ="); Serial.print(storedz);
            Serial.println("inverse kinematic started: ");
            Serial.print(storedx);
            Serial.print(",");
            Serial.print(storedy);
            Serial.print(",");
            Serial.print(storedz);
            
            calculateIK(storedx, storedy, storedz);
        }
        }
     else {
        int val = cmd.substring(1).toInt();
        if (header == 'B') b.targetangle = constrain(val, 0, 180);
        if (header == 'S') sh.targetangle = constrain(val, 0, 180);
        if (header == 'E') e.targetangle = constrain(val, 0, 180);
        if (header == 'G') g.targetangle = constrain(val, 75, 140);
    }
}
//obv this  is the inverse kinematics function 
void calculateIK(float x, float y, float z) {
    float theta1 = atan2(y, x);
    float r = sqrt(x*x + y*y);
    float zr = z - bh;
    float s = sqrt(r*r + zr*zr);

    if (s > (L1 + L2) || s < abs(L1 - L2)) {
        Serial.println(F("the target is unreachable since the distance is not the limits"));
        return;
    }

    float costheta3 = (L1*L1 + L2*L2 - s*s) / (2.0 * L1 * L2);
    float theta3 = acos(constrain(costheta3, -1, 1));
    float alpha = atan2(zr, r);
    float cosbeta = (L1*L1 + s*s - L2*L2) / (2.0 * L1 * s);
    float beta = acos(constrain(cosbeta, -1, 1));
    float theta2 = alpha + beta;
// here i converted to degree brr
    b.targetangle = (theta1 * 180.0 / pi) + 90;
    sh.targetangle = (theta2 * 180.0 / pi) + sh.offset;
    e.targetangle = (180 - (theta3 * 180.0 / pi)) + e.offset;
    
}
//this is the functio that checks what loop finished and edit the time that way it be fasterrr brrr
void updatemotion() {
    if (millis() - laststeptime > stepdelay) {
        laststeptime = millis();

        moveto(b);
        moveto(sh);
        moveto(e);
        moveto(g);
    
}
}
//this is the idea i suggested in the groupchat abt using steps to move that way it goes smoothly 
void moveto(joint &j) {
    if (j.currentangle < j.targetangle) {
        j.currentangle++;
        j.motor.write(j.currentangle);
    } else {if (j.currentangle > j.targetangle) {
        j.currentangle--;
        j.motor.write(j.currentangle);
    }
    }
}
//sscanf(buffer, sizeof(buffer),"")

//chaima's idea : K:x:12

/*String data = Serial.read();
int separatorIndex = data.indexOf(':');  
String part1 = data.substring(0, separatorIndex); 
String part2 = data.substring(separatorIndex,separatorIndex); 
String part3 = data.substring(separatorIndex + 1); 
int value = part3.toInt();*/