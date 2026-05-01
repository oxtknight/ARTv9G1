
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
    handlehardserial();
    handlesoftserial();
    updatemotion(); 
    
}
void handlesoftserial() {//this will have only the slides control brr
//listening to asgore while coding is funn :>>
  //so this function is to clear buffer so that we have and save the data from the app 
    static String softbuffer = "";
    
    while (blub.available() > 0) {
        char c = blub.read();
        if (c == '\n') {
            processcmd(softbuffer);
            softbuffer = "";
        } else {
            softbuffer += c;
        }
    }
}
//the same for hardware serial
void handlehardserial(){
    static String hardbuffer ="";
    while(Serial.available()>0){
        char c = Serial.read();
        if (c == '\n'){
            processcmd(hardbuffer);
            hardbuffer="";
        } else{
            hardbuffer +=c;
        }
    }
}

//this the function that processes the recieved data from the app and just put it in a buffer to recheck it later on
void processcmd(String cmd) {
    cmd.trim();
    if (cmd.length() < 2) 
    return;
    char header = cmd[0];
    if (header == 'K'){
        Serial.println("Detected IK command");   /////// aaaaaaaaaaaaaaaaaaaa debuuuuuuggg aaaa deebuug aahh ohhh noooo wnaodawawd
        char axis = cmd[1];
        float value = cmd.substring(2).toFloat();
        //frfr tho i be using all brain juice available aaaa rahh 
        //if ts doesnt end up work imma tweak out fr this time .mark my words.
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
        
    } else {
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

    Serial.print("Theta 1 = "); ///this and the line before it are for deeeebuuuuuuuuuuuuugg woowoooooooooooooooooooooooooooooo
    Serial.println(theta1);

    Serial.print("R = ");
    Serial.println(r);

    Serial.print("Zr = ");
    Serial.println(zr);

    Serial.print("S = ");
    Serial.println(s);

    
    if (s > (L1 + L2) || s < abs(L1 - L2)) {
        Serial.println(F("The distance is either too far or too close to reach safely"));
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
    Serial.print("Angles in rad [B,S,E] ="); Serial.print(theta1); Serial.print(", "); Serial.print(theta2); Serial.print(", "); Serial.println(theta3);
    Serial.print("Angles in degrees [B,S,E] ="); Serial.print(b.targetangle); Serial.print(", "); Serial.print(sh.targetangle); Serial.print(", "); Serial.println(e.targetangle);
    Serial.print("lucky shot: "); Serial.print(x); Serial.print(", ");Serial.print(y);Serial.println(", "); Serial.println(z);
    Serial.println(" ");

}
//this is the functio~ that checks what loop finished and edit the time that way it be fasterrr brrr
void updatemotion() {
    if (millis() - laststeptime > stepdelay) {
        laststeptime = millis();

        Move(b);
        Move(sh);
        Move(e);
        Move(g);
    
}
}
void Move(joint &J)
{
    float Distance = abs(J.currentangle - J.targetangle);

    if (Distance < 15)
    {
        J.motor.write(J.targetangle);
        J.currentangle = J.targetangle;
    }
    else
    {
            if (J.currentangle < J.targetangle)
        {
            J.motor.write(  J.currentangle + Distance/8  );
            J.motor.write(  J.currentangle + Distance/4  );
            J.motor.write(  J.currentangle + Distance/2  );
            J.motor.write(  J.targetangle - Distance/4  );
            J.motor.write(  J.targetangle - Distance/8  );
            J.motor.write(  J.targetangle  );

            J.currentangle = J.targetangle; 
        }
        else 
         {
            J.motor.write(  J.currentangle - Distance/8  );
            J.motor.write(  J.currentangle - Distance/4  );
            J.motor.write(  J.targetangle + Distance/2  );
            J.motor.write(  J.targetangle + Distance/4  );
            J.motor.write(  J.targetangle + Distance/8);
            J.motor.write(  J.targetangle);

            J.currentangle = J.targetangle;
        }
    }               //// heyy so uhh i know this looks a lil sus and ineffective but lemme cook aye!!! if this works it gonna be so lovelyy, otherwise... rip!
}


