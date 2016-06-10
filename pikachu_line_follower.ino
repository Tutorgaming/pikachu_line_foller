/*
  Line Tracking Car 

*/
#include <QTRSensors.h>

// IR Sensors
    #define NUM_SENSORS             8  // number of sensors used
    #define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
    #define EMITTER_PIN             2  // emitter is controlled by digital pin 2
    QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7}, 
      NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
    #define BLACK_THRESHOLD   700 // More THAN 700 is BLACK [0-1000]
// MOTOR PINS      
    #define MotorL_IN1 5
    #define MotorL_IN2 4
    #define MotorL_PWM 9
    #define MotorR_IN1 7
    #define MotorR_IN2 6
    #define MotorR_PWM 10
    #define BaseSpeed 170
    #define TopSpeed 220
    #define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
    #define leftBaseSpeed 150
    #define rightMaxSpeed 220 // this is the speed at which the motors should spin when the robot is perfectly on the line
    #define leftMaxSpeed 220
// BUTTONS
    #define start_pin 12

// Globar Var
    unsigned int sensors[NUM_SENSORS];
    unsigned int line_array[NUM_SENSORS];
    uint8_t line = 0b00000000;
    unsigned int POS;
     int left_red = 0;
     int left_green = 0;
      int left_blue = 0;
      int right_red = 0;
      int right_green = 0;
      int right_blue = 0;

    
// TCS230 Pins front left
    const int left_s0 = 51;
    const int left_s1 = 53;
    const int left_s2 = 45;
    const int left_s3 = 43;
    const int left_led_pin = 49;
    const int left_colorOutPin = 47;
    
    const int right_s0 = 50;
    const int right_s1 = 48;
    const int right_s2 = 44;
    const int right_s3 = 46;
    const int right_led_pin = 52;
    const int right_colorOutPin = 42;

// PID Var 
    int error=0;
    int lastError=0;

//Color Detection Function
void sendColorRead() {
    //Read Red Color Command
    digitalWrite(left_s2, LOW);
    digitalWrite(left_s3, LOW);
    int new_red = pulseIn(left_colorOutPin, LOW);
    new_red = map(new_red, 29,87,255,0);
    new_red = min(255, max(0,new_red));
    
    //Read Blue Color Command
    digitalWrite(left_s3, HIGH);
    int new_blue = pulseIn(left_colorOutPin, LOW);
    new_blue = map(new_blue, 24,78,255,0);
    new_blue = min(255, max(0,new_blue));

    //Read Green Color Command
    digitalWrite(left_s2, HIGH);
    int new_green = pulseIn(left_colorOutPin,LOW);
    new_green = map(new_green, 33,104,255,0);
    new_green = min(255, max(0,new_green));
    
    //Process Color
    left_red = new_red;
    left_green = new_green;
    left_blue = new_blue;

    //Read Red Color Command
    digitalWrite(right_s2, LOW);
    digitalWrite(right_s3, LOW);
    int new2_red = pulseIn(right_colorOutPin, LOW);
    new2_red = map(new2_red, 29,87,255,0);//new2_red = map(new2_red, 32,196,255,0);
    new2_red = min(255, max(0,new2_red));
    //Read Blue Color Command
    digitalWrite(right_s3, HIGH);
    int new2_blue = pulseIn(right_colorOutPin, LOW);
    new2_blue = map(new2_blue, 24,78,255,0);//new2_blue = map(new2_blue, 25,170,255,0);
    new2_blue = min(255, max(0,new2_blue));
    //Read Green Color Command
    digitalWrite(right_s2, HIGH);
    int new2_green = pulseIn(right_colorOutPin, LOW);
    new2_green = map(new2_green, 33,104,255,0);//new2_green = map(new2_green, 35,190,255,0);
    new2_green = min(255, max(0,new2_green));
    //Process Color
    right_red = new2_red;
    right_green = new2_green;
    right_blue = new2_blue;

}

void calibration(){
  // Do the calibration 
      // Turn The LED On
      digitalWrite(13,HIGH);
      for (int i = 0; i < 200; i++){
         unsigned int div = i / 40;
         Serial.print("[");
         Serial.print(div);
         Serial.println("/5 Sec]");
         qtra.calibrate();   // 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)    
      }
      // Turn The LED Off
      digitalWrite(13,LOW);

      // Print Minimum bound
      for (int i = 0; i < NUM_SENSORS; i++){
        Serial.print(qtra.calibratedMinimumOn[i]);
        Serial.print(' ');
      }
      Serial.println();
      
      // Print Maximum bound
      for (int i = 0; i < NUM_SENSORS; i++){
        Serial.print(qtra.calibratedMaximumOn[i]);
        Serial.print(' ');
      }
      Serial.println();
}


bool leftDetect(){
  if(left_green - left_red > 80 && left_green - left_blue > 50)return true;
  else return false;
}
bool rightDetect(){
  if(right_green >0 && right_red <10 && right_blue<10)return true;
  if(right_green - right_red > 60 && right_green - right_blue > 30)return true;
  else return false;
}

void printColorRead(){
   Serial.print("[L]");
   if(leftDetect()){Serial.print("[1]");}
   Serial.print(left_red);
   Serial.print(' ');
   Serial.print(left_green);
   Serial.print(' ');
   Serial.print(left_blue);
   Serial.print(" ===[R]  ");
    if(rightDetect()){Serial.print("[1]");}
   Serial.print(right_red);
   Serial.print(' ');
   Serial.print(right_green);
   Serial.print(' ');
   Serial.print(right_blue);
   Serial.print(' ');
   Serial.println();
}

// ISO MOTION

void forward(int MotorSpeed)
{
    digitalWrite(MotorL_IN1,HIGH);
    digitalWrite(MotorL_IN2,LOW);
    digitalWrite(MotorR_IN1,HIGH);
    digitalWrite(MotorR_IN2,LOW);
    analogWrite(MotorL_PWM,MotorSpeed);
    analogWrite(MotorR_PWM,MotorSpeed);
}
void backward(int MotorSpeed)
{
    digitalWrite(MotorL_IN1,LOW);
    digitalWrite(MotorL_IN2,HIGH);
    digitalWrite(MotorR_IN1,LOW);
    digitalWrite(MotorR_IN2,HIGH);
    analogWrite(MotorL_PWM,MotorSpeed);
    analogWrite(MotorR_PWM,MotorSpeed);
}
void turn_left(int MotorSpeed)
{
    digitalWrite(MotorL_IN1,LOW);
    digitalWrite(MotorL_IN2,LOW);
    digitalWrite(MotorR_IN1,HIGH);
    digitalWrite(MotorR_IN2,LOW);
    analogWrite(MotorL_PWM,MotorSpeed);
    analogWrite(MotorR_PWM,MotorSpeed);
}
void turn_right(int MotorSpeed)
{
    digitalWrite(MotorL_IN1,HIGH);
    digitalWrite(MotorL_IN2,LOW);
    digitalWrite(MotorR_IN1,LOW);
    digitalWrite(MotorR_IN2,LOW);
    analogWrite(MotorL_PWM,MotorSpeed);
    analogWrite(MotorR_PWM,MotorSpeed);
}
void spin_left(int MotorSpeed)
{
    digitalWrite(MotorL_IN1,LOW);
    digitalWrite(MotorL_IN2,HIGH);
    digitalWrite(MotorR_IN1,HIGH);
    digitalWrite(MotorR_IN2,LOW);
    analogWrite(MotorL_PWM,MotorSpeed);
    analogWrite(MotorR_PWM,MotorSpeed);
}
void spin_right(int MotorSpeed)
{
    digitalWrite(MotorL_IN1,HIGH);
    digitalWrite(MotorL_IN2,LOW);
    digitalWrite(MotorR_IN1,LOW);
    digitalWrite(MotorR_IN2,HIGH);
    analogWrite(MotorL_PWM,MotorSpeed);
    analogWrite(MotorR_PWM,MotorSpeed);
}

void fastStop()
{
    digitalWrite(MotorL_IN1,HIGH);
    digitalWrite(MotorL_IN2,HIGH);
    digitalWrite(MotorR_IN1,HIGH);
    digitalWrite(MotorR_IN2,HIGH);
    digitalWrite(MotorL_PWM,HIGH);
    digitalWrite(MotorR_PWM,HIGH);
}


void lineDetect(){
  for (int i = 0; i < NUM_SENSORS; i++){
        if(sensors[i]>BLACK_THRESHOLD){
           line |= 1 << (7-i);
           line_array[i] = 1;
        }else{
          line &= ~(1 << (7-i));
          line_array[i] =0;
        }
      }
}

void follow_line(){

    if(line == 0b11111111){
      fastStop();
      return;
    }
    
    if(line == 0b00011000 || line == 0b00111000 || line == 0b00011100 || line == 0b00001000 || line == 0b00010000){
      forward(100);
    }else if ( line == 0b00000001 || line == 0b00000011 || line == 0b00000111 || line == 0b00001111 || line == 0b00011111 ){
      spin_right(100); 
    }else if ( line == 0b00000100 || line == 0b00001100 || line == 0b00001110 || line == 0b00011110 ){
      turn_right(100);
    }else if ( line == 0b10000000 || line == 0b11000000 || line == 0b11100000 || line == 0b11110000 || line == 0b11111000 ){
      spin_left(100); 
    }else if ( line == 0b00100000 || line == 0b00110000 || line == 0b01110000 || line == 0b01111000 ){
      turn_left(100);
    }else{
      forward(100);
    }
}

void follow_line_pos(){
  // Normal Controller 
  if(POS >= 0 && POS <= 1000)
  {
    spin_left(TopSpeed);
  }
  if(POS >= 1000 && POS <= 2000)
  {
    turn_left(TopSpeed);
  }
  if(POS >= 2000 && POS <= 3000)
  {
    turn_left(BaseSpeed);
  }
  if(POS >= 3000 && POS <= 4000)
  {
    forward(BaseSpeed);
  }
  if(POS >= 4000 && POS <= 5000)
  {
    turn_right(BaseSpeed);
  }
  if(POS >= 5000 && POS <= 6000)
  {
    turn_right(TopSpeed);
  }
  if(POS >= 5000 && POS <= 6000)
  {
    spin_right(TopSpeed);
  }
}

void follow_line_pos_pid(){
  //PID Calculation
  error = POS - 3500;
  float kp = 2;
  float kd = 0;
  int motorSpeed = int(float(kp * error) + float(kd * (error - lastError)));
  lastError = error;
  
  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;

  Serial.print(leftMotorSpeed);
  Serial.print(' ');
  Serial.print(motorSpeed);
  Serial.print(' ');
  Serial.print(rightMotorSpeed);
  Serial.print(' ');
  Serial.println();

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


void setup() {
  // Init Serial Connection 
      Serial.begin(9600);
      Serial.println("Start the Calibration Process . . .");
  // Line Sensor Calibration
      calibration();
      Serial.println("Finish the Calibration Process . . .");
  
  // Color Sensor 
      pinMode(left_s0, OUTPUT);
      pinMode(left_s1, OUTPUT);
      pinMode(left_s2, OUTPUT);
      pinMode(left_s3, OUTPUT);
      pinMode(left_led_pin, OUTPUT);
      digitalWrite(left_led_pin,HIGH);
      pinMode(left_colorOutPin, INPUT);
      
      pinMode(right_s0, OUTPUT);
      pinMode(right_s1, OUTPUT);
      pinMode(right_s2, OUTPUT);
      pinMode(right_s3, OUTPUT);
      pinMode(right_led_pin, OUTPUT);
      digitalWrite(right_led_pin,HIGH);
      pinMode(right_colorOutPin, INPUT);
          
      // ColorPicker TC320 Scale Initialize
      digitalWrite(right_s0, HIGH);
      digitalWrite(right_s1, LOW);//HIGH
      digitalWrite(left_s0, HIGH);
      digitalWrite(left_s1, LOW);//HIGH

  // Wait For D13
      pinMode(start_pin, INPUT);
      while(digitalRead(start_pin) == HIGH) { };
}

void loop() {
  
  //Read Line Sensors
    POS = qtra.readLine(sensors);
    lineDetect();
  
  //Read Color Sensors
    sendColorRead();
    printColorRead();

  //Detect Color
    bool left_detect = leftDetect();
    bool right_detect = rightDetect();
  // Exception Handler
    if(left_detect && right_detect){
      Serial.println("2FOUND");
      fastStop();
      delay(2000);
      forward(255);
      delay(200);
    }else if(!left_detect && right_detect){
      Serial.println("RIGHT FOUND");
      fastStop();
      delay(2000);
      forward(200);
      delay(100);
      turn_right(200);
      delay(400);
    }else if(left_detect && !right_detect){
      Serial.println("LEFT FOUND");
      fastStop();
      delay(2000);
      forward(200);
      delay(100);
      turn_left(200);
      delay(400);
    }
  //Do the Following 
    follow_line_pos_pid();
    
    delay(20);
}
