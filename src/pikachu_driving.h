
void Forward(int MotorSpeed)
{

  digitalWrite(MotorL_IN1,HIGH);
  digitalWrite(MotorL_IN2,LOW);
  digitalWrite(MotorR_IN1,HIGH);
  digitalWrite(MotorR_IN2,LOW);
  analogWrite(MotorL_PWM,MotorSpeed);
  analogWrite(MotorR_PWM,MotorSpeed);

}
void Backward(int MotorSpeed)
{
  digitalWrite(MotorL_IN1,LOW);
  digitalWrite(MotorL_IN2,HIGH);
  digitalWrite(MotorR_IN1,LOW);
  digitalWrite(MotorR_IN2,HIGH);
  analogWrite(MotorL_PWM,MotorSpeed);
  analogWrite(MotorR_PWM,MotorSpeed);
}
void TurnLeft(int MotorSpeed)
{
  digitalWrite(MotorL_IN1,LOW);
  digitalWrite(MotorL_IN2,LOW);
  digitalWrite(MotorR_IN1,HIGH);
  digitalWrite(MotorR_IN2,LOW);
  analogWrite(MotorL_PWM,MotorSpeed);
  analogWrite(MotorR_PWM,MotorSpeed);
}
void TurnRight(int MotorSpeed)
{
  digitalWrite(MotorL_IN1,HIGH);
  digitalWrite(MotorL_IN2,LOW);
  digitalWrite(MotorR_IN1,LOW);
  digitalWrite(MotorR_IN2,LOW);
  analogWrite(MotorL_PWM,MotorSpeed);
  analogWrite(MotorR_PWM,MotorSpeed);
}
void SpinLeft(int MotorSpeed)
{
  digitalWrite(MotorL_IN1,LOW);
  digitalWrite(MotorL_IN2,HIGH);
  digitalWrite(MotorR_IN1,HIGH);
  digitalWrite(MotorR_IN2,LOW);
  analogWrite(MotorL_PWM,MotorSpeed);
  analogWrite(MotorR_PWM,MotorSpeed);
}
void SpinRight(int MotorSpeed)
{
  digitalWrite(MotorL_IN1,HIGH);
  digitalWrite(MotorL_IN2,LOW);
  digitalWrite(MotorR_IN1,LOW);
  digitalWrite(MotorR_IN2,HIGH);
  analogWrite(MotorL_PWM,MotorSpeed);
  analogWrite(MotorR_PWM,MotorSpeed);
}

void FastStop()
{
  digitalWrite(MotorL_IN1,HIGH);
  digitalWrite(MotorL_IN2,HIGH);
  digitalWrite(MotorR_IN1,HIGH);
  digitalWrite(MotorR_IN2,HIGH);
  digitalWrite(MotorL_PWM,HIGH);
  digitalWrite(MotorR_PWM,HIGH);
}


void SoftStart(int MotorSpeed)
{
  for(int i=0 ; i<MotorSpeed ; i=i+10)
  {
    Forward(i);
    delay(50);
  }
}

void SoftStart2(int MotorSpeed)
{
  for(int i=0 ; i<MotorSpeed ; i=i+10)
  {
    Forward(i);
    delay(10);
  }
}