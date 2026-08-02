#include <Pixy2.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// called this way, it uses the default address 0x40
Pixy2 pixy;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates
// Robotic arm parameters.
// These will need to be modified based on your arm
////// Theta 1 //////
#define J0_min 112  // min pulselength
#define J0_max 408  // max pulselength
#define d0_min 0    // corresponding limit in degrees (min)
#define d0_max 180  // max degrees

////// Theta 2 //////
#define J1_min 240  // min pulselength
#define J1_max 472  // max pulselength
#define d1_min 0    // corresponding limit in degrees (min)
#define d1_max 120  // max degrees

////// Theta 3 //////
#define J2_min 88   // min pulselength
#define J2_max 464  // max pulselength
#define d2_min -90  // corresponding limit in degrees (min)
#define d2_max 90   // max degrees

////// Theta 4 //////
#define J3_min 124  // min pulselength
#define J3_max 512  // max pulselength
#define d3_min 0    // corresponding limit in degrees (min)
#define d3_max 180  // max degrees

////// Theta 5 //////
#define J4_min 240  // min pulselength
#define J4_max 516  // max pulselength
#define d4_min -45  // corresponding limit in degrees (min)
#define d4_max 90

// Calibration data
int ball_pixels[] = { 172, 155, 139, 127, 117, 110, 103, 95, 89, 85 };     // Update with your measurements
float ball_distances[] = { 50, 60, 70, 80, 90, 100, 110, 120, 130, 140 };  // Update with your measurements
int gate_pixels[] = { 78, 74, 70, 64, 62, 60, 56, 54, 52, 48, 46 };
float gate_distances[] = { 300, 320, 340, 360, 380, 400, 420, 440, 460, 480, 500 };
int numDataPoints = 10;  // Replace with the actual number of data points

// varibales for storing distances and angles of obejcts
float ball_slope, ball_intercept, gate_slope, gate_intercept;
float ball_distance, gate_distance, ball_angle, gate_ball_distance, gate_ball_angle;
float gate_angle = 0;
int angle;
int ball_r = 3;
int ball_final_pixels, gate_final_pixels;
int counter = 0;
int maxObserving = 0;  // Variable to store the maximum value
bool ball = false;
bool gate = false;
// variables used to calculate the inverse kinematics
float t1, t2, t3, t4 = 0;
float d1 = 110;
float d2 = 105;
float d3 = 50;
float x, y, z;
int camera_constant = 75;

void setup() {
  pixy.init();
  Serial.begin(115200);
  LinearRegression(ball_pixels, ball_distances, numDataPoints, ball_slope, ball_intercept);
  LinearRegression(gate_pixels, gate_distances, numDataPoints, gate_slope, gate_intercept);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  angle = 30;
  pwm.setPWM(4, 0, map(angle, d0_min, d0_max, J0_min, J0_max));
  pwm.setPWM(6, 0, map(80, d1_min, d1_max, J1_min, J1_max));
  pwm.setPWM(8, 0, map(0, d2_min, d2_max, J2_min, J2_max));
  pwm.setPWM(12, 0, map(90, d3_min, d3_max, J3_min, J3_max));
  pwm.setPWM(15, 0, map(90, d4_min, d4_max, J4_min, J4_max));

  while (!ball && angle < 150) {
    angle += 2;
    delay(100);
    pwm.setPWM(4, 0, map(angle, d0_min, d0_max, J0_min, J0_max));
    if (pixy.ccc.getBlocks() > 0) {
      for (int i = 0; i < pixy.ccc.getBlocks(); i++) {
        if (pixy.ccc.blocks[i].m_signature == 3) {
          delay(100);
          if (pixy.ccc.blocks[i].m_width > maxObserving) {
            maxObserving = pixy.ccc.blocks[i].m_width;  // Update the maximum value
            ball_final_pixels = maxObserving;
            ball_angle = angle;
          }
        }
      }
    }
  }
  ball = true;
  maxObserving = 0;
  angle = 30;
  pwm.setPWM(4, 0, map(angle, d0_min, d0_max, J0_min, J0_max));
  while (!gate && angle < 150) {
    angle += 2;
    delay(100);
    pwm.setPWM(4, 0, map(angle, d0_min, d0_max, J0_min, J0_max));
    if (pixy.ccc.getBlocks() > 0) {
      for (int i = 0; i < pixy.ccc.getBlocks(); i++) {
        if (pixy.ccc.blocks[i].m_signature == 2) {
          delay(100);
          if (pixy.ccc.blocks[i].m_width > maxObserving) {
            maxObserving = pixy.ccc.blocks[i].m_width;  // Update the maximum value
            gate_final_pixels = maxObserving;
            gate_angle = angle;
          }
        }
      }
    }
  }
  gate = true;
  angle = 30;
  pwm.setPWM(4, 0, map(angle, d0_min, d0_max, J0_min, J0_max));
  gate_distance = getDistance(gate_final_pixels, gate_slope, gate_intercept);
  Serial.println("Finally Gate:");
  Serial.println(gate_distance);
  Serial.println(gate_angle);
  ball_distance = getDistance(ball_final_pixels, ball_slope, ball_intercept);
  Serial.println("Ball:");
  Serial.println(ball_distance);
  Serial.println(ball_angle);
  ball_distance += camera_constant;
  gate_distance += camera_constant;

  // Tuning angles
  if (ball_angle < 80) {
    ball_angle -= 10;
    Serial.println("angle changed");
    Serial.println(ball_angle);
  }
  if (ball_angle > 100) {
    ball_angle += 10;
    Serial.println("angle changed");
    Serial.println(ball_angle);
  }

  // Inverse Kinematics
  x = (ball_distance - d3) * cos(ball_angle * 3.1415 / 180);
  y = (ball_distance - d3) * sin(ball_angle * 3.1415 / 180);
  t1 = atan2(y, x);          // [radians]
  t1 = t1 * 180.0 / 3.1415;  // [degrees]

  t2 = acos((ball_distance - d3) / d2);
  t2 = t2 * 180 / 3.1415;
  //
  t3 = t2;
  // orientation of shooting
  gate_ball_angle = gate_angle - ball_angle;
  gate_ball_distance = sqrt(pow(gate_distance, 2) + pow(ball_distance, 2) - 2 * ball_distance * gate_distance * cos(abs(gate_ball_angle)));
  t4 = (pow(ball_distance, 2) - pow(gate_ball_distance, 2) - pow(gate_distance, 2)) / (-2 * gate_distance * gate_ball_distance);
  t4 = acos(t4) * 180 / 3.1415;
  if (gate_ball_angle > 0) {
    t4 = t4;
  } else {
    t4 = 180 - t4;
  }
  Serial.println("t4 is:");
  Serial.println(t4);
  // map degrees to pulselength and send value to robotic arm
  pwm.setPWM(12, 0, map(t4, d3_min, d3_max, J3_min, J3_max));
  pwm.setPWM(15, 0, map(-30, d4_min, d4_max, J4_min, J4_max));
  delay(1000);
  pwm.setPWM(4, 0, map(t1, d0_min, d0_max, J0_min, J0_max));
  delay(1000);
  pwm.setPWM(6, 0, map(t2, d1_min, d1_max, J1_min, J1_max));
  pwm.setPWM(8, 0, map(t3, d2_min, d2_max, J2_min, J2_max));
  delay(500);
  pwm.setPWM(15, 0, map(80, d4_min, d4_max, J4_min, J4_max));
}
void loop() {}

void LinearRegression(int pixels[], float distances[], int numDataPoints, float &slope, float &intercept) {
  float sum_xy = 0;
  float sum_x = 0;
  float sum_y = 0;
  float sum_x2 = 0;

  for (int i = 0; i < numDataPoints; i++) {
    sum_xy += pixels[i] * distances[i];
    sum_x += pixels[i];
    sum_y += distances[i];
    sum_x2 += pixels[i] * pixels[i];
  }
  slope = (numDataPoints * sum_xy - sum_x * sum_y) / (numDataPoints * sum_x2 - sum_x * sum_x);
  intercept = (sum_y - slope * sum_x) / numDataPoints;
}
// Linear approximation function
float getDistance(int pixels, float slope, float intercept) {
  //Estimate distance based on the linear relationship
  return slope * pixels + intercept;
}
