 const int motor1speed = 4;   // H-bridge enable pin
 const int motor1dir1 = 2;    // H-bridge leg 1 
 const int motor1dir2 = 3;    // H-bridge leg 2 

 const int motor2speed = 5;   // H-bridge enable pin
 const int motor2dir1 = 6;    // H-bridge leg 1 
 const int motor2dir2 = 7;    // H-bridge leg 2 
 
 const int revPin = 8; //tell that robot is reversing.
 const int fwdPin = 9; //tell that robot is moving forward
 
 int delay_in_sec = 500; 
 
 const int flex_threshold = 500;

void fwd_direction()
{ //move in forward direction with set speed
      digitalWrite(fwdPin, HIGH);
      digitalWrite(revPin, LOW);

      digitalWrite(motor1dir1, LOW);   // set leg 1 of the H-bridge low
      digitalWrite(motor1dir2, HIGH);  // set leg 2 of the H-bridge high
      digitalWrite(motor2dir1, LOW);   // set leg 1 of the H-bridge low
      digitalWrite(motor2dir2, HIGH);  // set leg 2 of the H-bridge high 
   
      analogWrite(motor1speed, 255);   //A value between 0 - 255
      analogWrite(motor2speed, 255);   //A value between 0 - 255   
}

void rev_turn()
{ //move in reverse direction and turn
     Serial.print("Reverse direction and turn  \n"); //debugging
      
      digitalWrite(fwdPin, LOW);
      digitalWrite(revPin, HIGH);
      
      digitalWrite(motor1dir1, HIGH);   // set leg 1 of the H-bridge low
      digitalWrite(motor1dir2, LOW);  // set leg 2 of the H-bridge high
      digitalWrite(motor2dir1, HIGH);   // set leg 1 of the H-bridge low
      digitalWrite(motor2dir2, LOW);  // set leg 2 of the H-bridge high 
      
      analogWrite(motor1speed, 255);   //A value between 0 - 255
      analogWrite(motor2speed, 0);   //A value between 0 - 255
}


void rev_direction()
{
  //move in reverse direction with set speed
      Serial.print("Reverse direction \n");  //debugging
      
      digitalWrite(fwdPin, LOW);
      digitalWrite(revPin, HIGH);   
     
      digitalWrite(motor1dir1, HIGH);   // set leg 1 of the H-bridge low
      digitalWrite(motor1dir2, LOW);  // set leg 2 of the H-bridge high
      digitalWrite(motor2dir1, HIGH);   // set leg 1 of the H-bridge low
      digitalWrite(motor2dir2, LOW);  // set leg 2 of the H-bridge high 
      
      analogWrite(motor1speed, 250);   //A value between 0 - 255
      analogWrite(motor2speed, 250);   //A value between 0 - 255      
}


 
void setup()  { 
    // set all the other pins you're using as outputs:
    Serial.begin(9600);  //initialize serial communication 
    
    pinMode(motor1speed, OUTPUT); 
    pinMode(motor1dir1, OUTPUT); 
    pinMode(motor1dir2, OUTPUT);
    
    pinMode(motor2speed, OUTPUT); 
    pinMode(motor2dir1, OUTPUT); 
    pinMode(motor2dir2, OUTPUT);
    
    pinMode(fwdPin, OUTPUT);
    pinMode(revPin, OUTPUT); 
 
} 

void loop()  { 
  
  fwd_direction();
  delay(delay_in_sec);
  
  int flex = analogRead(A1);
  
  Serial.println(flex);  //debugging
  
   if ( flex > flex_threshold)
    {
      rev_direction();
      delay(4*delay_in_sec);
      rev_turn();
      delay(2*delay_in_sec);    
    }
    
}
