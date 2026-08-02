#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

// Robotic arm parameters.
// These will need to be modified based on your arm
////// Theta 1 //////
#define J0_min 112  // min pulselength
#define J0_max 428  // max pulselength
#define d0_min -90  // corresponding limit in degrees (min)
#define d0_max 90   // max degrees

////// Theta 2 //////
#define J1_min 240  // min pulselength
#define J1_max 424  // max pulselength
#define d1_min 0    // corresponding limit in degrees (min)
#define d1_max 120  // max degrees

////// Theta 3 //////
#define J2_min 72   // min pulselength
#define J2_max 516  // max pulselength
#define d2_min -90  // corresponding limit in degrees (min)
#define d2_max 120  // max degrees

////// End Effector //////
// #define J3_min 140  // pulselength in open position
// #define J3_max 295  // pulselength in closed position
// #define d3_min 0    // corresponding min distance in mm
// #define d3_max 10   // max distance in mm


char incomingByte = 0;  // for incoming serial data

// Reference configuration of the robotic arm
double x = 200;  // mm
double y = 0;    // mm
double z = 200;  //mm

// variables used to calculate the inverse kinematics
float t1 = 0;
int d1 = 110;

float t2 = 0;
int d2 = 105;

float t3 = 0;
int d3 = 150;

int grip = 200;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // Move the robotic arm to the reference configuration
  pwm.setPWM(4, 0, map(-60, d0_min, d0_max, J0_min, J0_max));
  pwm.setPWM(6, 0, J1_min);
  pwm.setPWM(7, 0, J2_min);
  // pwm.setPWM(3, 0, grip);
}

void loop() {

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.println(incomingByte);

    // print the current x, y, z value
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.print(y);
    Serial.print(", z: ");
    Serial.println(z);
    switch (incomingByte) {
      case 'q':  // move in x
        x += 5;
        break;
      case 'a':
        x -= 5;
        break;

      case 'w':  // move in y
        y += 5;
        break;
      case 's':
        y -= 5;
        break;

      case 'e':  // move in z
        z += 5;
        break;
      case 'd':
        z -= 5;
        break;

      case 'r':  // open/close the end effector
        grip += 5;
        break;
      case 'f':
        grip -= 5;
        break;

      case 'h':  // move the robot to the reference configuration
        x = 200;
        y = 0;
        z = 200;
        grip = 200;
        break;
    }
    // print the modified x, y, z value
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.print(y);
    Serial.print(", z: ");
    Serial.println(z);


    /////////////// Compute the inverse kinematics of the robotic arm ///////////////
    ////// inverse kinematics (joint 1) //////
    t1 = atan2(y, x);          // [radians]
    t1 = t1 * 180.0 / 3.1415;  // [degrees]
    Serial.println("t1 is:");
    Serial.println(t1);
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(4, 0, map(t1, d0_min, d0_max, J0_min, J0_max));
    Serial.println(mapped_angle);
    ////// inverse kinematics (joint 2) //////
    float r = sqrt(pow(z - d1, 2) + pow(x, 2) + pow(y, 2));
    Serial.print("r is:");
    Serial.println(r);
    float cos_beta = ((pow(r, 2) + pow(d2, 2) - pow(d3, 2)) / (2 * r * d2));
    Serial.print("cos is:");
    Serial.println(cos_beta);
    float sin_beta = sqrt(1 - pow(cos_beta, 2));
    // sqrt(1 - pow(cos_beta, 2))
    t2 = atan2(sin_beta, cos_beta) + atan2(z - d1, sqrt(pow(x, 2) + pow(y, 2)));  // [radians]
    t2 = t2 * 180.0 / 3.1415;                                                     // [degrees]
    Serial.println("t2 is:");
    Serial.println(t2);
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(6, 0, map(t2, d1_min, d1_max, J1_min, J1_max));

    ////// inverse kinematics (joint 3) //////
    float cos_alpha = (pow(d3, 2) + pow(d2, 2) - pow(r, 2)) / (2 * d3 * d2);
    float sin_alpha = sqrt(1 - pow(cos_alpha, 2));
    t3 = 3.1415 - atan2(sin_alpha, cos_alpha);  // [radians]
    t3 = t3 * 180.0 / 3.1415;                   // [degrees]
    Serial.println("t3 is:");
    Serial.println(t3);
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(7, 0, map(t3, d2_min, d2_max, J2_min, J2_max));

  }
}
