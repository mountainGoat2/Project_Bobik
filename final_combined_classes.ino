#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Create an instance of the PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVOMIN = 150;
const int SERVOMAX = 600;
// Servo parameters
class Leg {
public:
    // Constructor
    Leg(int servoA, int servoB, int servoC, int offsetA, int offsetB, int offsetC)
        : servoA(servoA), servoB(servoB), servoC(servoC), offsetA(offsetA), offsetB(offsetB), offsetC(offsetC) {}

    // Function to set new angles for the servo motors
    void setAngles(float newA_angle, int newB_angle, int newC_angle) {
        pwm.setPWM(servoA, 0, map(newA_angle + offsetA, 0, 180, SERVOMIN, SERVOMAX));
        pwm.setPWM(servoB, 0, map(newB_angle + offsetB, 0, 180, SERVOMIN, SERVOMAX));
        pwm.setPWM(servoC, 0, map(newC_angle + offsetC, 0, 180, SERVOMIN, SERVOMAX));
    }

private:
    int servoA;
    int servoB;
    int servoC;
    int offsetA;
    int offsetB;
    int offsetC;
};


// Instantiate the legs with offsets
Leg legA(0, 1, 8, -7, 0, -6);  // Adjust offsets as needed
Leg legB(2, 3, 9, 4, -6, -9);
Leg legC(4, 5, 10, -2, 9, 0);
Leg legD(6, 7, 11, 3, -2, -4);
float limbA = 12;
float limbB = 12;
float limbC = 5.7;
float h;
float hnew;
float h_z_new;
float deltax;
float newAangle;
float newBangle;
float newCangle;
float prevA = 135;
float prevB = 90;
float y;
float angle_h_hnew;
float limb_hnew;
float i_step = 0.1;
int height_limit = 20;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50); // Set the PWM frequency to 50Hz (typical for servos)
  legA.setAngles(prevA, prevB, 90);
  legB.setAngles(prevA, prevB, 90);
  legC.setAngles(prevA, prevB, 90);
  legD.setAngles(prevA, prevB, 90);
}


void Up_Down(float d_h) {
  h = sqrt(2 * sq(limbA) * (1 - cos(prevB * PI / 180)));
  hnew = h + d_h;
  if(hnew > height_limit){
    Serial.println("Cannot go higher, try smaller value");
  } else {
   newAangle = acos(hnew / (2 * limbA)) * 180 / PI; // radians??
   newBangle = acos((2 * sq(limbA) - sq(hnew)) / (2 * sq(limbA))) * 180 / PI;
   prevA = newAangle; //stores the angle value hip lmbA
   prevB = newBangle; //stores the angle value feet limbB
   legA.setAngles(newAangle+90, newBangle, 90);
   legD.setAngles(180 - newAangle, 180 - newBangle, 90);
   legB.setAngles(180 - newAangle, 180 - newBangle, 90);
   legC.setAngles(newAangle+90, newBangle, 90);
         }

}

void newangles_Front_Back(float target_height, float z, int x){
    angle_h_hnew = asin(z/target_height)*180/PI;
    limb_hnew = acos(target_height/(2*limbA))*180/PI;
    newAangle = limb_hnew - (angle_h_hnew * x);
    newBangle = acos(1 - (sq(target_height/limbA)/2))*180/PI;
}
// Calculate_Left_Right() function pass down z axis reference distance, y_axis=1 if y!=0, c_coefficient=direction.
void calculate_Left_Right(float min_new_distance, bool y_axis, int c_coefficient, float& hnew, float& h_z_new, float& newAangle, float& newBangle, float& newCangle) {
    h_z_new = sqrt(sq(h - (y_axis * y)) + sq(min_new_distance)); // Update h_z_new
    hnew = sqrt(sq(h_z_new) - sq(limbC)); // Update hnew
    newAangle = acos(hnew / (2 * limbA)) * 180 / PI; // radians??
    newBangle = acos((2 * sq(limbA) - sq(hnew)) / (2 * sq(limbA))) * 180 / PI;
    newCangle = (asin(hnew / h_z_new) + (c_coefficient * asin(min_new_distance / h_z_new))) * 180 / PI;
}

void loop() {
  Serial.print("new test please enter new movement:\n" );
  while (Serial.available() == 0) {
  }
  char action = Serial.read(); 
  deltax = Serial.parseInt();
  h = sqrt( 2 * sq(limbA) * (1-cos(prevB * PI/180))); //correct
  switch(action){

    case 'u': case 'd':
    float y_corrected;
    y_corrected = deltax / 1.12;
     Up_Down(y_corrected);
    break;

    case 'f':

     for(float i=i_step; i<=deltax; i+=i_step) { 
     y = sqrt(i*(deltax-i)); //y coordinate for the circle formula
     h = sqrt( 2 * sq(limbA) * (1-cos(prevB * PI/180)));
     hnew = sqrt(sq(h-y)+sq(i)); //x=1 z=i
     newangles_Front_Back(hnew, i, 1);
    legA.setAngles(newAangle+90, newBangle, 90);
    legD.setAngles(180 - newAangle, 180 - newBangle, 90);

    }
    delay(10);
    for(float i=i_step; i <=deltax; i+=i_step) {   
    hnew = sqrt(sq(h) + sq(deltax-i)); 
    newangles_Front_Back(hnew, deltax -i, 1); 
    legA.setAngles(newAangle+90, newBangle, 90); 
    legD.setAngles(180 - newAangle, 180 - newBangle, 90);
 
    hnew = sqrt(sq(i)+sq(h)); //x=-1 z=k
    newangles_Front_Back(hnew, i, -1);
    legB.setAngles(180 - newAangle, 180 - newBangle, 90);
    legC.setAngles(newAangle+90, newBangle, 90);

   }
   delay(10);
   for(float j=i_step; j <=deltax; j=j+i_step){

    y = sqrt(j*(deltax-j));
    hnew = sqrt(sq(deltax - j)+sq(h - y)); //x=-1 z=deltax-j
    newangles_Front_Back(hnew, deltax-j, -1);
    legB.setAngles(180 - newAangle, 180 - newBangle, 90);
    legC.setAngles(newAangle+90, newBangle, 90);
   
   }
   break;

   case 'b':
    for (float i=i_step; i<=deltax; i+=i_step) { //legA and legD move back from home
     y = sqrt(i * (deltax - i)); // y coordinate for the circle formula
     h = sqrt( 2 * sq(limbA) * (1-cos(prevB * PI/180)));
     hnew = sqrt(sq(h-y)+sq(i)); //x=-1 z=i
     newangles_Front_Back(hnew, i, -1);
     legA.setAngles(newAangle+90, newBangle, 90);
     legD.setAngles(180 - newAangle, 180 - newBangle, 90);
    }
   for (float i=i_step; i<=deltax; i+=i_step) {
    hnew = sqrt(sq(h)+sq(deltax-i)); //x=-1 z=deltax-i
    newangles_Front_Back(hnew, deltax - i, -1);
    legA.setAngles(newAangle+90, newBangle, 90);
    legD.setAngles(180 - newAangle, 180 - newBangle, 90);

    hnew = sqrt(sq(i)+sq(h));
    newangles_Front_Back(hnew, i, 1);
    legB.setAngles(180 - newAangle, 180 - newBangle, 90);
    legC.setAngles(newAangle +90, newBangle, 90); 
    delay(2);

   } 
    for (float i=i_step; i<=deltax; i+=i_step) {
    y = sqrt(i * (deltax - i));
    hnew = sqrt(sq(h-y)+sq(deltax-i)); //x=-1 z=deltax-i
    newangles_Front_Back(hnew, deltax-i, 1);
    legB.setAngles(180 - newAangle, 180 - newBangle, 90);
    legC.setAngles(newAangle+90, newBangle, 90);    
   }
   break;

   case 'l':
   for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); //correct
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(limbC - i, 1, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legA.setAngles(newAangle_local + 90, newBangle_local, newCangle_local);
            

  }
    for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); //correct
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(limbC + i, 1, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legD.setAngles(180 - newAangle_local, 180 - newBangle_local, newCangle_local);

  }

   for (float i= i_step; i<=deltax; i += i_step){
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;
    calculate_Left_Right(deltax - limbC - i, 0, -1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legA.setAngles(newAangle_local + 90, newBangle_local, newCangle_local);
  
    calculate_Left_Right(deltax + limbC - i, 0, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legD.setAngles(180 - newAangle_local, 180 - newBangle_local, newCangle_local);

    calculate_Left_Right(limbC - i, 0, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legB.setAngles(180 - newAangle_local, 180 - newBangle_local, 180 - newCangle_local);

    calculate_Left_Right(limbC + i, 0, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legC.setAngles(newAangle_local + 90, newBangle_local, 180 - newCangle_local);

   }
   for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); //correct
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(deltax - limbC - i, 1, -1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legB.setAngles(180 - newAangle_local, 180 - newBangle_local, 180 - newCangle_local);

  }
    for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); 
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(deltax + limbC - i, 1, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legC.setAngles(90+ newAangle_local, newBangle_local, 180 - newCangle_local);
  }

  break;

  case 'r':
   for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); //correct
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(limbC + i, 1, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legA.setAngles(newAangle_local + 90, newBangle_local, newCangle_local);

  }

    for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); //correct
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(limbC - i, 1, -1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legD.setAngles(180 - newAangle_local, 180 - newBangle_local, newCangle_local);

  }

   for (float i= i_step; i<=deltax; i += i_step){
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;
    calculate_Left_Right(deltax + limbC - i, 0, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legA.setAngles(newAangle_local + 90, newBangle_local, newCangle_local);

    calculate_Left_Right(deltax - limbC - i, 0, -1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legD.setAngles(180 - newAangle_local, 180 - newBangle_local, newCangle_local);
 
    calculate_Left_Right(limbC + i, 0, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legB.setAngles(180 - newAangle_local, 180 - newBangle_local, 180 - newCangle_local);
    
    calculate_Left_Right(limbC - i, 0, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legC.setAngles(newAangle_local + 90, newBangle_local, 180 - newCangle_local);


   }
 
   for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); //correct
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(deltax + limbC - i, 1, 1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legB.setAngles(180 - newAangle_local, 180 - newBangle_local, 180 - newCangle_local);

  }

  for (float i = i_step; i <= deltax; i += i_step) {
    y = sqrt(i * (deltax - i)); 
    float hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local;

    calculate_Left_Right(deltax -limbC - i, 1, -1, hnew_local, h_z_new_local, newAangle_local, newBangle_local, newCangle_local);
    legC.setAngles(90+ newAangle_local, newBangle_local, 180 - newCangle_local);
  }

  break;
  }

}


