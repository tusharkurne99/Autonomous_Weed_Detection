const int pulPin1 = 3; 
const int dirPin1 = 4; 
const int enPin1 = 5;
const int pulPin2 = 6; 
const int dirPin2 = 7; 
const int enPin2 = 8;
const int pulPin3 = 9; 
const int dirPin3 = 10; 
const int enPin3 = 11;
float stp1;
float stp2;
float stp3;
float deg[]={45,45,45};
#define res 1.8


void setup() {
  pinMode(pulPin1,OUTPUT); 
  pinMode(dirPin1,OUTPUT);
  pinMode(enPin1,OUTPUT);
  digitalWrite(enPin1,LOW);
  
  pinMode(pulPin2,OUTPUT); 
  pinMode(dirPin2,OUTPUT);
  pinMode(enPin2,OUTPUT);
  digitalWrite(enPin2,LOW);
  
  pinMode(pulPin3,OUTPUT); 
  pinMode(dirPin3,OUTPUT);
  pinMode(enPin3,OUTPUT);
  digitalWrite(enPin3,LOW);
  
  Serial.begin(9600);
 
}

void loop() 
{  
   
  stp1=deg[0]/res;  
  digitalWrite(dirPin1,LOW);
  
  for(float x=0; x<stp1; x++)
  {
    digitalWrite(pulPin1,HIGH); 
    delayMicroseconds(5000); 
    digitalWrite(pulPin1,LOW); 
    delayMicroseconds(5000);
    Serial.println(stp1);
  }
   
  stp2=deg[1]/res;  
  digitalWrite(dirPin2,LOW);
  for(float x=0; x<stp2; x++)
  {
    digitalWrite(pulPin2,HIGH); 
    delayMicroseconds(5000); 
    digitalWrite(pulPin2,LOW); 
    delayMicroseconds(5000);
    Serial.println(stp2);
  }  

  stp3=deg[2]/res;  
  digitalWrite(dirPin3,HIGH);
  for(float x=0; x<stp3; x++)
  {
    digitalWrite(pulPin3,HIGH); 
    delayMicroseconds(5000); 
    digitalWrite(pulPin3,LOW); 
    delayMicroseconds(5000);
     Serial.println(stp3);
  }  
 


  ///////////////////////////////////////////////
  for(float x=0; x<1000000; x++)
  {
    digitalWrite(pulPin1,LOW); 
    delayMicroseconds(5000); 
    digitalWrite(pulPin1,LOW); 
    delayMicroseconds(5000);
  } 
    for(float x=0; x<1000000; x++)
  {
    digitalWrite(pulPin1,LOW); 
    delayMicroseconds(5000); 
    digitalWrite(pulPin1,LOW); 
    delayMicroseconds(5000);
  } 
  
 
  for(float x=0; x<1000000; x++)
  {
    digitalWrite(pulPin1,LOW); 
    delayMicroseconds(5000); 
    digitalWrite(pulPin1,LOW); 
    delayMicroseconds(5000);
  }  
  delay(15000);
}
