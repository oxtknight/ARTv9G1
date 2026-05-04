//the commenting brrr    so now ill be adding comments but the fun part ill add what im doing or listening to meanwhile cuz fun thing (i get bored) so i gotta pretend i have live audience 
//ill comment on everything that way we dont face any questions . positive?
#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>
//so above r libraries we used in this code
SoftwareSerial blub(11, 10);//this is me initializing the pins for software serial 
constexpr float L1 = 12.0;
constexpr float L2 = 15.0;
constexpr float bh = 9.0;
//these 3 expressions above are for the arm dimensions in cm that will be needed in IK calculations later on 
constexpr float pi = 3.14159265;//wanted to brag abt me memorizing abt 20 digits  // crezy this is so cool fr -rei 
//radius of base 4.5cm//comment the comment had to add this if we need it somehow somewhere
float storedx = 0, storedy = 0, storedz = 0;
//initializing variables that our recieved coordinates will be saved into
struct joint {
  Servo motor;//servo object_name
  byte pin;//servo attached pin
  float currentangle; //where is the servo rn 
  float targetangle;//where we want the servo to go 
  float offset;//offset 4 callibration to fix mechanical isssues brr
};//cool struct component just to keep my code clean and look more pro (i learned abt structs abt 3 months ago , had to test my knowledge )
joint b = { Servo(), 3, 90, 90, 90 };  //og was 0    | this is the base servo
joint sh = { Servo(), 5, 90, 90, 0 };  //the one we calculated was -43   | this is the shoulder servo 
joint e = { Servo(), 6, 90, 90, 90 };  //was 0  | this is the elbow servo 
joint g = { Servo(), 9, 90, 145, 0 }; // this is the gripper servo 
//above here i defined the values following this format:    
//our_struct_name servo_name ={ servo()_to_define_servo, which_pin ,current_angle_for callibration, target_angle_we_want_the_to_go_to_once_code_uploaded, offset};
//listening to snowdin town  
unsigned long laststeptime = 0;
const int stepdelay = 20;
//this lovely step delay will be used to make movement smooth so it is 10ns delay between steps (degrees) we can adjust this to adjust speed , the lower the faster 
void setup() {
  Serial.begin(9600);//so  ye as uk this is for the serial monitor to be able to use it
  blub.begin(9600);// this to match the hc05 module baud rate to be able to communicate 
  delay(2000);// Rayan idea to add this so while we r plugging wires and all arm doesnt jump on us safety measures ( ͡° ͜ʖ ͡°) 
  b.motor.attach(b.pin);
  sh.motor.attach(sh.pin);
  e.motor.attach(e.pin);
  g.motor.attach(g.pin);
  //me attach servo to pin. pretty obv if u ask me (^_-)
  Serial.println("sys ready ,use sliders for challenge 1 and 2, use textboxes for challenge 3!!");
}
//some peak started playing rahhh      spamton by toby fox
void loop() {
  handlesoftware();//calling function 
  updatemotion();//calling function x2
}
void handlesoftware() {
  //so this function is to clear buffer so that we have and save the data from the app
  //in short this function check the bluetooth messages and save them 
  static String inputBuffer = "";//here im telling memory "i beg u please dont throw this variable when my function finishes her job , please dont kill my variable"
  while (blub.available() > 0) {//if we detect message we asking it to save it in buffer 
    char c = blub.read();
    if (c == '\n') {//in the app we added \n so in code i can use it as a sign where the buffer stops saving meaning message ends the moment it reaches \n
      processcmd(inputBuffer);//me passing down to another function 
      inputBuffer = "";//me clear for next message
    } else {
      inputBuffer += c;//we be building this message brick by brick (some1 gets the reference please) (Ｔ▽Ｔ)
    }
  }
}
//this the function that processes the recieved data from the app and just put it in a buffer to recheck it later on
void processcmd(String cmd) {
  //i be listening to love dramatic "misterr oh misterrr ゆみじゃないならきかせて "
  cmd.trim();//this function removes any possible invisible space (just in case, better safe than sorry)
  if (cmd.length() < 2)
    return;
  char header = cmd[0];//firt letter of that message saved in the buffer is taken here 
  if (header == 'K') {//we check if that first letter is K to know which mode wether sliders or IK
    char axis = cmd[1];// here if it is the IK mode that means now we need to check the second letter cuz in app we send in this format Kx# || ky# || kz#  
    //so ye we save second letter , if u notice i be treating it the array way (pretty easy)
    float value = cmd.substring(2).toFloat();//here me ask for the rest of the string which will be the coordinate in ascii and use a function to convert it to flaot to be used 
    //this time i be listening to heartache :) pretty cool
    if (axis == 'x') {//here i check the second letter and match values that way i have each variable ready to use
      storedx = value;
      Serial.print("Detected X axis = ");
      Serial.print(storedx);
    } else if (axis == 'y') {
      storedy = value;
      Serial.print("Detected Y axis =");
      Serial.print(storedy);
    } else if (axis == 'z') {
      storedz = value;
      Serial.print("Detected Z axis =");
      Serial.print(storedz);
      Serial.println("inverse kinematic started: ");
      Serial.print(storedx);
      Serial.print(",");
      Serial.print(storedy);
      Serial.print(",");
      Serial.print(storedz);
      //this is just us adding stuff to be able to debug easily if any problem came up later 
      calculateIK(storedx, storedy, storedz);//me pass the coordinates to the function that do the math 
    }
  } else {// else introducing to youu : SLIDERS MODE!!!!! yipppppeeerss
    int val = cmd.substring(1).toInt();//so since message from app comes like this B# || S# || E# || G#  we remove header and take the rest which is the number as ascii and turn it into integer 
    if (header == 'B') b.targetangle = constrain((180 - val), 0, 180);   // we take the values we got from app and update the target angle that way our servo goes to it when using the moveto function below
    if (header == 'S') sh.targetangle = constrain((180 - val), 0, 180);
    if (header == 'E') e.targetangle = constrain(val, 0, 180);
    if (header == 'G') g.targetangle = constrain(val, 75, 140);
  }
}// A human's touch started playinggg 
//obv this  is the inverse kinematics function
void calculateIK(float x, float y, float z) {
  //el top view first  (el base rotaccion ) me habla espagnol (idk what im doing anymore)
  float theta1 = atan2(y, x);
  //el side view second (the arm elivation )
  float r = sqrt(x * x + y * y);//distance to target on da floor (he hit the floor) 
  float zr = z - bh;//height relative to shoulder 
  float s = sqrt(r * r + zr * zr);//shortest path mathematically possible to el target
//limitaccioonnn   we dont want arm to break ig
  if (s > (L1 + L2) || s < abs(L1 - L2)) {
    Serial.println(F("the target is unreachable since the distance is not the limits"));
    return;
  }
//elbow angle     (just law of cosines)
  float costheta3 = (L1 * L1 + L2 * L2 - s * s) / (2.0 * L1 * L2);
  float theta3 = acos(constrain(costheta3, -1, 1));
  //shoulder angle calc    rahhhhhhh go away started playing 
  float alpha = atan2(zr, r);//angle from floor to that shortest path 
  float cosbeta = (L1 * L1 + s * s - L2 * L2) / (2.0 * L1 * s);
  float beta = acos(constrain(cosbeta, -1, 1));//angle bend 
  float theta2 = alpha + beta;//elivation for shoulder (Thank GOD Finallyyyyy)
  // here i converted to degree brr
  b.targetangle = (theta1 * 180.0 / pi) + b.offset; //the 90 is to center base tho i can change it to this b.offset
  sh.targetangle = (theta2 * 180.0 / pi) + sh.offset;
  e.targetangle = (180 - (theta3 * 180.0 / pi)) + e.offset;//it be flipped cuz mechanically looking at it it be flipped irl
}
//this is the function (yes ayoub i fixed it) that checks what loop finished and edit the time that way it be fasterrr brrr
void updatemotion() {//this is the smooth motion thingy i mentioned before
  if (millis() - laststeptime > stepdelay) {//dis check if enough time passed to take another step 
    laststeptime = millis();
   //it updates with the new time passed 
    moveto(b);
    moveto(sh);
    moveto(e);
    moveto(g);
    //move servos when time reach the needed delay 
  }
}
//this is the idea i suggested in the groupchat abt using steps to move that way it goes smoothly
void moveto(joint &j) {//this moves current angle one step closer to the target angle making it not jump
//also rayan added a way to stop jittering which ye pretty badass 
  if (abs(j.currentangle - j.targetangle) > 1.0) {//here to make it stops when it is close enough 
    if (j.currentangle < j.targetangle) {
      j.currentangle++;//step up 
      j.motor.write(j.currentangle);
    } else {
      if (j.currentangle > j.targetangle) {
        j.currentangle--;//step down (i hear ur thoughts rayan transformer hmmm)
        j.motor.write(j.currentangle);
      }
    }
  } else {//if we reach target we lock it down 
    j.motor.write(j.targetangle);
    j.currentangle = j.targetangle;
  }
}
//finallyy imm donneee 
/*
⠀⠀⢀⠄⣢⣴⣶⣿⣿⣿⡿⠿⠟⠛⠛⠛⠛⠛⠿⠷⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⢸⣾⣿⠿⠛⠉⢁⣀⣠⣤⣴⣶⣶⣶⣶⣶⣶⣶⣾⣿⣤⣄⣀⠀⠀⠀⠀⠀
⠀⠀⣜⠋⢀⣠⣶⡾⠟⠛⡩⠡⠂⢸⠁⢠⡄⡨⢂⠉⠹⠻⣿⣿⣿⣷⠀⠀⠀⠀
⠀⢀⣡⣾⣿⠟⢉⠁⠠⠀⡁⢀⠆⡰⣀⢀⢠⠈⠙⠜⢄⠇⠸⠟⠋⡍⠀⠀⠀⠀
⣠⣿⣿⣿⠋⠀⠄⢠⢁⠸⡠⠻⠠⠁⡟⢨⠃⠀⠄⠂⠈⡇⠀⠦⠘⠁⠀⠀⠀⠀
⢿⣿⡟⠇⢆⢜⠔⠇⢠⣊⣠⣗⣳⢪⢁⢂⠊⣀⣠⣤⢀⢳⢰⡌⡄⠀⠀⠀⠀⠀
⠀⠈⣧⣥⣐⣡⠊⢀⣃⢕⠁⠙⢁⢃⠊⠈⠐⠒⠉⢁⡮⣮⢻⠳⠁⠀⠀⠀⠀⠀
⠀⠀⠫⣏⢲⢆⠶⡊⠉⠉⠀⠠⠃⠁⠀⡀⠀⠀⠈⠁⣸⠇⡴⠁⠀⠀⠀⠀⠀⠀
⠀⠀⠀⣿⢿⢈⡍⢦⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⢠⡳⡺⠁⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠈⠣⠢⣏⢎⢷⣄⠀⠀⠒⠒⠒⠀⠀⠀⣰⡗⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠁⠈⢹⣿⣦⣄⠀⠀⢀⢴⡋⡟⡄⠀⠀⠀⠒⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢀⣠⠔⠒⡏⢿⣿⡿⠟⠋⠀⢺⡃⠗⠊⢡⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠉⣁⣤⣴⣾⡇⠀⠀⡇⠈⡟⠿⠿⡷⠿⠛⠑⡈⠆⡀⢻⣿⣤⣒⣒⡒⠒⠀⠀
⣾⣿⣿⣿⣿⣿⠀⠀⠀⠁⠀⠈⢂⠀⠀⢀⠃⠀⣷⡘⠁⠈⣿⣿⣿⣿⣿⣿⣿⣷
⣿⣿⣿⣿⣿⠇⠀⠀⠀⢸⠀⠀⡼⡇⠀⡞⣎⠀⡏⠀⠀⠀⢻⣿⣿⣿⣿⣿⣿⣿
-------------------------------------------

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⠖⣩⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣦⡑⢤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⡀⠀⠤⣀⠀⠀⠈⠉⠙⠀⠀⠀⢀⣴⣿⣯⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⣝⣦⠤⠤⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣷⣶⣭⡉⠽⠦⡀⠀⠀⠀⠀⢀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⠀⠀⠀⠀⣀⣀⣀⣀⠀
⣿⣿⣿⣿⣷⣄⢻⣦⠀⢀⣤⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣴⣾⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣧⠉⣷⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⡇⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣿⣿
⠻⠿⠛⠛⢧⠄⠙⣿⡇⣿⡟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠇⠈⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠛⠿⣿⣿⣿⣦⠙⣿
⠀⠁⠀⠀⠀⠀⠀⠙⢷⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠙⠿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⠀⠈⠉⠛⢿⠇⠈
⠀⠀⠀⠀⠀⠀⢰⢆⣿⣿⣿⣿⣿⢿⡇⢻⣿⣿⣿⣿⣿⣿⢿⠀⠀⠀⠀⠀⠀⠙⢿⣿⣿⡟⣉⣀⢻⣿⣿⣿⣿⣿⣿⠇⠀⠀⠀⣤⠀⠀
⠀⠀⠀⠀⠀⠀⣼⠿⣿⣿⣿⣿⣿⣼⣷⣾⣿⣿⣿⣿⣿⡿⣶⣓⣒⣒⣒⠒⠂⠀⠀⠛⠛⢴⣿⣼⢸⣿⣿⣿⣿⣿⣿⠆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⡴⠃⠀⠉⣿⣿⣿⣿⣏⠛⠃⠉⣿⠿⣿⣿⡀⠙⠻⠄⠭⠟⠛⠁⠀⠀⠀⠀⡰⠏⢡⣿⣿⣿⣿⣿⣿⡭⠄⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠈⠀⠀⠀⠀⠃⠙⣿⣿⣿⠀⠀⢺⢿⠀⠈⠛⠓⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣶⣶⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣀⠀⠀⣀⣀⣀⠀⠀⣠⡄⢀⣿⢿⣿⡆⠀⢏⡜⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿⡿⣿⡏⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀
⠁⠀⠀⠈⠈⠉⢹⠉⠉⡀⢸⡁⠈⡏⠹⡄⠈⠳⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⡷⢿⣿⣿⣟⠈⠉⢻⠉⠉⢹⠉⠁⢹⠃⠀⠀⠁⠀
⠀⠀⠀⠀⠀⠀⢸⠀⠀⡇⢸⠀⠀⡇⠀⠘⣦⡀⢠⣀⣀⣀⡀⠀⠀⠀⠀⢀⣠⠀⡸⠀⣿⡛⣿⣿⡶⠤⣼⣄⠀⢸⠀⠀⠸⠁⠀⠀⠀⠀
⠈⠉⠉⠀⠀⠈⢹⠉⠙⡇⢸⠀⢘⡟⠉⠛⡟⠙⢆⠀⠀⠀⠀⠀⢀⣠⠖⠉⠀⢀⠇⢀⣼⣿⣿⣿⣿⠀⠈⢿⠛⢻⠋⠙⠰⠀⠀⠀⠀⠀
⢄⠀⠀⠸⠀⠀⢸⠀⠀⠇⠡⠀⢻⡇⠀⠀⡇⢀⡼⠷⣄⣀⣤⣾⠟⠁⢀⣀⠤⠚⠋⣡⣼⣿⡿⣿⣥⡄⠀⠸⡄⢸⠀⠀⠐⠀⠀⠀⠀⠀
⣟⠀⠂⢺⠒⠒⢺⠓⠻⠀⣾⠀⣿⡗⠒⢲⠷⠋⢀⣴⣿⣿⣿⠣⠒⠋⠁⢀⣤⣶⣿⣿⣿⣿⣿⣿⠟⠀⠀⠀⠹⣾⠓⠒⢲⠀⠀⠀⠀⠀
⣿⣦⣀⣸⠀⠀⠸⠀⣈⣀⣿⣀⣿⣧⠔⠁⠀⠀⣿⣿⣿⣿⣿⣄⣠⣴⣾⣿⣿⣿⣿⣿⣿⣿⡿⠃⠀⠀⠀⠀⠀⢸⣧⣀⢸⡆⠀⠀⠀⠀
⣿⣿⣿⣿⣶⣶⣶⠶⣇⠂⣠⣴⠟⠁⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠀⠀⠀⠀⠀⠀⠀⢸⠀⠈⠙⠳⠶⣄⡀⠀
⣿⣿⣿⣿⣿⣿⣿⣤⡾⠋⣱⡃⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⢹⣿⣿⣿⣿⣿⣿⣿⡟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠸⠀⠀⠀⠀⠀⠀⠉⠓
⣿⣿⠿⠿⠛⠛⠛⠁⠀⣰⢉⢿⡄⠀⠀⠀⢠⣿⣿⠋⠉⠻⢿⣿⣿⣿⣿⣿⣿⣿⡋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠇⠀⠀⠀⠀⠀⠀⠀⠀
⣿⠟⠀⠀⠀⠀⠀⠀⡴⣿⡿⢻⢷⠀⠀⢠⣿⣿⡇⠀⠀⣠⣾⣿⣿⣿⣿⣿⣿⠋⢽⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⡟⠀⠀⠀⠀⠀⢀⣾⡗⢁⡴⠋⠀⣆⣰⣿⣿⣿⡷⢶⣶⣿⣿⣿⣿⣿⣿⣿⣿⣤⣸⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⡇⠀⠀⠀⢀⣠⡾⢋⡴⠋⠀⢀⡴⣿⣿⣿⣿⡿⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⠉⣿⣿⣿⣧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀

dazai and chuuya (double black for good lucc)*/
