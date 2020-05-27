#include <Servo.h>
const int APin=2;
const int BPin=7;
const int in1=3;
const int in2=4;
const int in3=5;
const int in4=6;
int n4,n5;
int n1,n2;
int trigPin=12;
int echoPin=11;
long duration, distance;
Servo myservo;
void motorForward() 
{
  analogWrite(APin, 255);
  analogWrite(BPin, 250);
  digitalWrite(in1, HIGH);  
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void motorBackward() 
{
  analogWrite(APin, 255);
  analogWrite(BPin, 250);
  digitalWrite(in1, LOW);  
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(20);
}
void Stop() 
{
  analogWrite(APin, 0);
  analogWrite(BPin, 0);
  digitalWrite(in1, LOW);  
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void motorRight() 
{
  analogWrite(APin, 120);
  analogWrite(BPin, 120);
  digitalWrite(in1, HIGH);  
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void motorLeft() 
{
  analogWrite(APin, 120);
  analogWrite(BPin, 120);
  digitalWrite(in1, LOW);  
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
int calculateDistance()
{ 
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2;
  return distance;
}
int sonarLeft1()
{
  int n2=0;
  for(int i=0;i<=90;i++)
  {  
    myservo.write(i);
    delay(30);
    distance = calculateDistance();
    if(distance<15)
    {
      n2++;
    }
  }
  delay(20);
  myservo.write(90);
  return n2;
}
int sonarLeft2()
{
  int n1=0;
  for(int i=90;i<=179;i++)
  {  
    myservo.write(i);
    delay(30);
    distance = calculateDistance();
    if(distance<15)
    {
      n1++;
    }
  }
  delay(20);
  myservo.write(90);
  return n1;
}
int sonarRight1()
{
  for(int i=90;i>0;i--)
  {  
    myservo.write(i);
    delay(30);
  }
  delay(20);
  myservo.write(90);
}
int sonarRight2()
{
  for(int i=179;i>90;i--)
  {  
    myservo.write(i);
    delay(30);
  }
  delay(20);
  myservo.write(90);
}
void setup() 
{
  Serial.begin(9600);
  pinMode(APin, OUTPUT);
  pinMode(BPin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo.attach(10);
  myservo.write(90);
}
void loop() 
{
  motorForward();
  delay(30);
  Stop();
  delay(30);
  distance = calculateDistance();
  Serial.print(distance);
  Serial.print("\n");
  if(distance<15)
  {
    delay(10);
    Stop();
    sonarRight1();
    n2=sonarLeft1();
    n1=sonarLeft2();
    sonarRight2();
    if(n1>10 && n2<10)
    {
      motorRight();
      delay(800);
      Stop();
    }
    else if(n2>10 && n1<10)
    {
      motorLeft();
      delay(800);
      Stop();
    }
    else if(n1>10 && n2>10)
    {
      motorBackward();
      delay(200);
      Stop();
      motorRight();
      delay(800);
      Stop();
    }
    else if(n1<10 && n2<10)
    {
      motorBackward();
      delay(400);
      Stop();
      motorRight();
      delay(1600);
      Stop();
    }
    else;
  }
  else;
}
