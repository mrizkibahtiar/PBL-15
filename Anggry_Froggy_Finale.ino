#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h> 
#include "pitches.h"
#include <NewPing.h>

#define triger 16
#define echo 16
NewPing sonar(triger, echo);
int uS;
int jarak;

int melody[] = {
  NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4, 
  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4, 
  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,
  NOTE_A4
};

int durations[] = {
  8, 8, 4, 4,
  8, 8, 4, 4,
  8, 8, 4, 4,
  2
};


int iot= 0 ;
//Motor
const int pwmL = 5;
const int pwmR = 4;
const int rotL = 0;
const int rotR = 2;

//sensor garis
int L_sens = 14;
int C_sens = 12;
int R_sens = 13;

String command;             //String to store app command state.
int vspeed = 255;         // 400 - 1023.
int halfspeed = 250;         // 400 - 1023.
//int speed_Coeff = 3;
int turnSpeed = 170;

const char* ssid = "IoT";
const char* password = "12344321";

// Piezo Buzzer
const int buzzerPin = 15; // Connect the buzzer to pin 16
ESP8266WebServer server(80);




//Void 
void setup() {


 // HC-SR04 Sensor
 pinMode(triger, OUTPUT);
  pinMode(echo, INPUT);
 

 // Conection
  pinMode(pwmL, OUTPUT);    // PWM motor L
  pinMode(pwmR, OUTPUT);    // PWM motor R
  pinMode(rotL, OUTPUT);    // Rotation motor L
  pinMode(rotR, OUTPUT);    // Rotation motor R
  
  Serial.begin(115200);
  
//Sensor line conection
pinMode(L_sens, INPUT); //Left Sensor
pinMode(C_sens, INPUT); //Center Sensor
pinMode(R_sens, INPUT); //Right Sensor


// Connecting WiFi

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
 
 // Starting WEB-server 
     server.on ( "/", HTTP_handleRoot );
     server.onNotFound ( HTTP_handleRoot );
     server.begin();

 pinMode(buzzerPin, OUTPUT);

}


//ManualControler
void goAhead(){ 

    analogWrite(pwmL, vspeed);
    analogWrite(pwmR, vspeed);
    digitalWrite(rotL, HIGH);
    digitalWrite(rotR, HIGH);
  }

void goBack(){ 
    analogWrite(pwmL, vspeed);
    analogWrite(pwmR, vspeed);
    digitalWrite(rotL, LOW);
    digitalWrite(rotR, LOW);
  }

void goRight(){ 
    analogWrite(pwmL, vspeed);
    analogWrite(pwmR, 0);
    digitalWrite(rotL, HIGH);
    digitalWrite(rotR, LOW);
  }

void goLeft(){

    analogWrite(pwmL, 0);
    analogWrite(pwmR, vspeed);
    digitalWrite(rotL, LOW);
    digitalWrite(rotR, HIGH);
  }

void goAheadRight(){
      
    analogWrite(pwmL, vspeed);
    analogWrite(pwmR, halfspeed);
    digitalWrite(rotL, HIGH);
    digitalWrite(rotR, HIGH);
   }

void goAheadLeft(){
      
    analogWrite(pwmL, halfspeed);
    analogWrite(pwmR, vspeed);
    digitalWrite(rotL, HIGH);
    digitalWrite(rotR, HIGH);
  }

void goBackRight(){ 

    analogWrite(pwmL, halfspeed);
    analogWrite(pwmR, vspeed);
    digitalWrite(rotL, LOW);
    digitalWrite(rotR, LOW);
  }

void goBackLeft(){ 

    analogWrite(pwmL, vspeed);
    analogWrite(pwmR, halfspeed);
    digitalWrite(rotL, LOW);
    digitalWrite(rotR, LOW);
  }

void stopRobot(){  

    analogWrite(pwmL, 0);
    analogWrite(pwmR, 0);
    digitalWrite(rotL, LOW);
    digitalWrite(rotR, LOW);
 }

 
void ManualMode(){
    server.handleClient();
    
      command = server.arg("State");
      if (command == "F") goAhead();
      else if (command == "B") goBack();
      else if (command == "L") goLeft();
      else if (command == "R") goRight();
      else if (command == "G") goAheadRight();
      else if (command == "I") goAheadLeft();
      else if (command == "J") goBackRight();
      else if (command == "H") goBackLeft();
      else if (command == "0") vspeed = 0;
      else if (command == "1") vspeed = 30;
      else if (command == "2") vspeed = 50;
      else if (command == "3") vspeed = 70;
      else if (command == "4") vspeed = 90;
      else if (command == "5") vspeed = 120;
      else if (command == "6") vspeed = 140;
      else if (command == "7") vspeed = 160;
      else if (command == "8") vspeed = 190;
      else if (command == "9") vspeed = 255;
      else if (command == "S") stopRobot();
      else if (command == "W") iot= 1;
      else if (command == "w") iot= 0;
      else if (command == "V") beep();
      else if (command == "v") iot = 0;
      else if (command == "Z") Fobject ();
      else if (command == "z") Following ();
}

void HTTP_handleRoot(void) {

if( server.hasArg("State") ){
       Serial.println(server.arg("State"));
  }
  server.send ( 200, "text/html", "" );
  delay(1);
}



//LineMode
void LineMode(){
lineSensor();
Serial.println("");
delay(500);
}

void lineSensor()
{
int L_sens_state = digitalRead(L_sens);
int C_sens_state = digitalRead(C_sens);
int R_sens_state = digitalRead(R_sens);

if ((L_sens_state == 0 && C_sens_state == 1 && R_sens_state == 0) || (L_sens_state == 1 && C_sens_state == 0 && R_sens_state == 1))
{
Serial.println("forward");

    analogWrite(pwmL, 250);
    analogWrite(pwmR, 250);
    digitalWrite(rotL, HIGH);
    digitalWrite(rotR, HIGH);
}

else if ((L_sens_state == 1 && C_sens_state == 0 && R_sens_state == 0) ||
(L_sens_state == 1 && C_sens_state == 1 && R_sens_state == 0))
{
Serial.println("left");

    analogWrite(pwmL, 0);
    analogWrite(pwmR, 200);
    digitalWrite(rotL, LOW);
    digitalWrite(rotR, HIGH);

}
else if ((L_sens_state == 0 && C_sens_state == 0 && R_sens_state == 1) ||
(L_sens_state == 0 && C_sens_state == 1 && R_sens_state == 1))
{
Serial.println("right");

    analogWrite(pwmL, 200);
    analogWrite(pwmR, 0);
    digitalWrite(rotL, HIGH);
    digitalWrite(rotR, LOW);
}

else if ((L_sens_state == 0 && C_sens_state == 0 && R_sens_state == 0) ||
(L_sens_state == 0 && C_sens_state == 0 && R_sens_state == 0)){
Serial.println("reverse");
analogWrite(pwmL, vspeed );
analogWrite(pwmR, halfspeed);
digitalWrite(rotL, HIGH);
digitalWrite(rotR, LOW);
}





}

void beep() {
   int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[note];
    tone(buzzerPin, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    
    //stop the tone playing:
    noTone(buzzerPin);
  }



}


void loop()
 {
  server.handleClient(); // Handle web server requests

  if (iot == 0) {
   ManualMode();
  } 
  
  else{
   LineMode();
   iot = 0 ;
  }  
}


void Fobject()
{
uS = sonar.ping();
jarak = uS / US_ROUNDTRIP_CM;
deteksi();
Serial.println("");
delay(500);
}


void deteksi()
{
Serial.print("Jarak: ");
Serial.println(jarak);
if (jarak <= 15)
{
Serial.println("turnRight");
analogWrite(pwmL, turnSpeed);
analogWrite(pwmR, 0);
digitalWrite(rotL, HIGH);
digitalWrite(rotR, LOW);
Serial.println("Obyek terlalu dekat");
}

else if (jarak >= 15)
{
Serial.println("forward");
analogWrite(pwmL, vspeed);
analogWrite(pwmR, vspeed);
digitalWrite(rotL, HIGH);
digitalWrite(rotR, HIGH);
Serial.println("Obyek terlalu jauh");
}
}

void Following (){
  Fo();
  delay (100);
}


void Fo() {
  int distance = sonar.ping_cm();

  Serial.print("Jarak: ");
  Serial.println(distance);

  if (distance >= 50) {
    Serial.println("Stop");
    motorStop();
  } else if (distance < 50 && distance > 15) {
    Serial.println("Forward");
    motorForward();
  } else if (distance <= 15) {
    Serial.println("Stop");
    motorStop();
  }

  delay(500);
}

void motorStop() {
  analogWrite(pwmL, 0);
  analogWrite(pwmR, 0);
  digitalWrite(rotL, LOW);
  digitalWrite(rotR, LOW);
}

void motorForward() {
  analogWrite(pwmL, 255); // Adjust the speed as needed
  analogWrite(pwmR, 255); // Adjust the speed as needed
  digitalWrite(rotL, HIGH);
  digitalWrite(rotR, HIGH);
}
