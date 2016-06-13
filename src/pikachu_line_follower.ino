#include <SPI.h>
#include <Pixy.h>
#include <QTRSensors.h>
// IR Sensors
#define NUM_SENSORS             8   // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4   // average 4 analog samples per sensor reading
#define EMITTER_PIN             2   // emitter is controlled by digital pin 2
#define BLACK_THRESHOLD         700 // More THAN 700 is BLACK [0-1000]
#define BLOCK_AREA_THRESHOLD    500 // Area = height * width > 700 pixel
#define BLOCK_MINIMUM_HEIGHT    60
// Constant and MOTOR PINS
    const int MotorL_IN1  = 5;
    const int MotorL_IN2  = 4;
    const int MotorL_PWM  = 9;
    const int MotorR_IN1  = 7;
    const int MotorR_IN2  = 6;
    const int MotorR_PWM  = 10;
    const int BaseSpeed   = 160;
    const int TopSpeed    = 160;//220
    const int rightBaseSpeed= 150; // this is the speed at which the motors should spin when the robot is perfectly on the line
    const int leftBaseSpeed = 150;
    const int rightMaxSpeed = 160; //220 this is the speed at which the motors should spin when the robot is perfectly on the line
    const int leftMaxSpeed = 160;
// Driving Library ( Must be Under Constant )
    #include "pikachu_driving.h"

// BUTTONS
    #define start_pin 12
// Globar Var
    unsigned int sensors[NUM_SENSORS];
    unsigned int line_array[NUM_SENSORS];
    uint8_t line = 0b00000000;
    unsigned int POS;
    #define REF 700
    int camera_wait_interval =0;
// Pid Variable
    int error= 0;
    int lastError = 0;
// Pixy Global Variable
    Pixy pixy;
// Sensor Array
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

void setup()
{
  pinMode(MotorL_IN1,OUTPUT);
  pinMode(MotorL_IN2,OUTPUT);
  pinMode(MotorL_PWM,OUTPUT);
  pinMode(MotorR_IN1,OUTPUT);
  pinMode(MotorR_IN2,OUTPUT);
  pinMode(MotorR_PWM,OUTPUT);
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pinMode(12, INPUT);
  pinMode(13, OUTPUT);
  Serial.print("Press the Start Button\n");
  while(digitalRead(start_pin) == HIGH) { };
  Serial.print("Starting : Calibration State \n");
  calibration();

  pixy.init();

}

void loop()
{
  POS = qtra.readLine(sensors);
  int flag = read_camera();
  follow_line_pos_pid();
  if(flag > 0){
    //while(digitalRead(start_pin) == HIGH) { FastStop(); };
    if(flag == 1){ //LEFT ONLY
      Forward(200);
      delay(100);
      TurnLeft(200);
      delay(500);
      POS += 1000;
    }else if(flag == 2){ //RIGHT ONLY
      Forward(200);
      delay(100);
      TurnRight(200);
      delay(500);
      POS -= 1000;
    }else if(flag == 3){ //BOTH
      Forward(200);
      delay(300);
    }
  }
}


void follow_line_pos_pid(){
  //PID Calculation
  error = POS - 3500;
  float kp = 0.7;
  float kd = 1.5;
  int motorSpeed = int(float(kp * error) + float(kd * (error - lastError)));
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;

  // Serial.print(leftMotorSpeed);
  // Serial.print(' ');
  // Serial.print(motorSpeed);
  // Serial.print(' ');
  // Serial.print(rightMotorSpeed);
  // Serial.print(' ');
  // Serial.println();

  // Bounderings
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

  // Set Target Value
  digitalWrite(MotorR_IN1, HIGH);
  digitalWrite(MotorR_IN2, LOW);
  analogWrite(MotorR_PWM, rightMotorSpeed);
  digitalWrite(MotorL_IN1, HIGH);
  digitalWrite(MotorL_IN2, LOW);
  analogWrite(MotorL_PWM, leftMotorSpeed);
}

int read_camera(){
  // Blocks of color desired (Configured in PixyMon)
    uint16_t blocks;
    int valid_block_count = 0;
    int flag = 0;
    bool left_detect = false;
    bool right_detect= false;
  // Get the Blocks
    blocks = pixy.getBlocks();
    if(blocks){
      camera_wait_interval++;

      // do this (print) every 50 frames because printing every
      // frame would bog down the Arduino
      if (camera_wait_interval>20)
      {
        camera_wait_interval = 0;
        Serial.print("Detect : ");
        Serial.print(blocks);
        Serial.println(" blocks");
        // Condition 1 : Block detected !
        if(blocks >0){
          for(int i = 0 ; i < blocks ; i++){
            // Filter the 'large enough ' enough
              int block_area = pixy.blocks[i].height * pixy.blocks[i].width;
              int block_height = pixy.blocks[i].height;
              Serial.print("Height : ");
              Serial.println(block_height);
              int block_x = pixy.blocks[i].x;
              int block_y = pixy.blocks[i].y;
              if(block_area > BLOCK_AREA_THRESHOLD && block_height > BLOCK_MINIMUM_HEIGHT){
                // Valid Block Classification Here
                  valid_block_count++;
                  if(block_x < 160){
                    left_detect = true;

                  }else{
                    right_detect = true;
                  }
              }

          }
        Serial.print("Valid block Count : ");
        Serial.println(valid_block_count);
        }
      }
    }
    if(left_detect && !right_detect){
      Serial.println("LEFT BLOCK DETECT");
      return 1;
    }
    if(!left_detect && !right_detect){
      //Serial.println("NO DETECTION");
      return 0;
    }
    if(!left_detect && right_detect){
      Serial.println("RIGHT BLOCK DETECT");
      return 2;
    }
    if(left_detect && right_detect){
      Serial.println("BOTH (L-R) BLOCK DETECT");
      return 3;

    }


}


void calibration()
{

  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
}
