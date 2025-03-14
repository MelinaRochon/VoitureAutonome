// #include <AFMotor.h>
// #include <TaskScheduler.h>

// // Motor initialization with motor numbers 
// AF_DCMotor leftmotor(3); 
// AF_DCMotor rightmotor(4); 

// // Callback methods
// void tCapteurCallback();
// void tDriveCallback();
// void tLaneChangeCallback();

// // Cree les taches
// Task tCapteur(); // lis les capteurs
// Task tDrive(); 
// Task tLaneChange(); // change de ligne selon la direction 

// Scheduler runner;

// void tCapteurCallback() {

// }

// void tDriveCallback() {

// }

// void tLaneChange() {

// }


// void setup() {  
//   leftmotor.setSpeed(200);  // Set the speed of the left motor   
//   rightmotor.setSpeed(200); // Set the speed of the right motor   

//   runner.init() // initialise le runner
//   Serial.begin(9600);
// } 

// void loop() {   
//   runner.execute()
//   // Move forward   
//   /*leftmotor.run(FORWARD);   
//   rightmotor.run(FORWARD);   
//   Serial.println("Moving forward");   
//   delay(2000); // Move forward for 2 seconds
  
//   // Stop   
//   leftmotor.run(RELEASE);   
//   rightmotor.run(RELEASE);   
//   Serial.println("Stopping");   
//   delay(1000); // Stop for 1 second
  
//   // Move backward   
//   leftmotor.run(BACKWARD);   
//   rightmotor.run(BACKWARD);   
//   Serial.println("Moving backward");   
//   delay(2000); // Move backward for 2 seconds
  
//   // Stop   
//   leftmotor.run(RELEASE);   
//   rightmotor.run(RELEASE);   
//   Serial.println("Stopping");   
//   delay(1000); // Stop for 1 second*/
// }

#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>
#include <TaskScheduler.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // Sets the speed of DC motors
#define MAX_SPEED_OFFSET 20
 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
 
AF_DCMotor leftMotor(3, MOTOR34_1KHZ);
AF_DCMotor rightMotor(4, MOTOR34_1KHZ);
Servo myservo;  

// Callback methods
void tCapteurCallback();
void tDriveCallback();
void tLaneChangeCallback();

// Cree les taches
Task tCapteur(); // lis les capteurs
Task tDrive(); 
Task tLaneChange(); 

Scheduler runner;

void tCapteurCallback() {

}

void tDriveCallback() {

}

void tLaneChange() {

}

 
boolean goesForward = false;
int distance = 100;
int speedSet = 0;
 
// Function prototypes
int readPing();
void moveStop();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
int lookRight();
int lookLeft();
 
void setup() {
  myservo.attach(10);  
  myservo.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}
 
void loop() {
  int distanceR = 0;
  int distanceL = 0;
  delay(40);
 
  if (distance <= 15) {
    moveStop();
    delay(100);
    moveBackward();
    delay(300);
    moveStop();
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);
 
    if (distanceR >= distanceL) {
      turnRight();
      moveStop();
    } else {
      turnLeft();
      moveStop();
    }
  } else {
    moveForward();
  }
  distance = readPing();
}
 
int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}
 
void moveStop() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}
 
void moveForward() {
  if (!goesForward) {
    goesForward = true;
    leftMotor.run(FORWARD);      
    rightMotor.run(FORWARD);
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
      leftMotor.setSpeed(speedSet);
      rightMotor.setSpeed(speedSet);
      delay(5);
    }
  }
}
 
void moveBackward() {
  goesForward = false;
  leftMotor.run(BACKWARD);      
  rightMotor.run(BACKWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(speedSet);
    rightMotor.setSpeed(speedSet);
    delay(5);
  }
}  
 
void turnRight() {
  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);    
  delay(500);
  moveForward(); // resume forward motion after turning
}
 
void turnLeft() {
  leftMotor.run(BACKWARD);    
  rightMotor.run(FORWARD);  
  delay(500);
  moveForward(); // resume forward motion after turning
}  
 
int lookRight() {
    myservo.write(50);
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115);
    return distance;
}
 
int lookLeft() {
    myservo.write(170);
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115);
    return distance;
}