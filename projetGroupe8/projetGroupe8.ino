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

boolean goesForward = false;
int distance;
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


// Callback methods
void tCapteurCallback();
void tDriveCallback();
void tLaneChangeCallback();

// Cree les taches
Task tCapteur(2000, TASK_FOREVER, &tCapteurCallback); // lis les capteurs
Task tDrive(3000, TASK_FOREVER, &tDriveCallback); 
Task tLaneChange(1000, TASK_FOREVER, &tLaneChangeCallback); 

Scheduler runner;

void tCapteurCallback() {
  //int distanceR = 0;
  //int distanceL = 0;  
  //while (1) {
    distance = readPing();
    Serial.println("In tCapteurCallback: distance = ");
        Serial.print(distance);

moveForward();
    delay(10);
  
    // if (distance <= 30) { /* 30 cm de distance */
        if (distance <= 10) { /* 30 cm de distance */

    Serial.println("In distance <= 30");
      tDrive.disable();
      tLaneChange.enable(); // va au changement de voie
      
    } else {
      Serial.println("In distance > 30");
      tLaneChange.disable(); // va au changement de voie
      tDrive.enable();

    }
  //}
  //else {
    //moveForward();
  //}
}

void tDriveCallback() {
    Serial.println("In tDriveCallback");

  moveForward();

}

void tLaneChangeCallback() {
  Serial.println("In tLaneChangeCallback");

  int distanceR = 0;
  int distanceL = 0;  
  //moveStop();
  delay(2000);
  // while ((distanceR < 40) || (distanceL < 40)) {
      while ((distanceR < 10) || (distanceL < 10)) {

    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);
  }

  // check max distance
  if (distanceR >= distanceL) {
    turnRight();

    // check if is passed obstacle
    // distanceL = lookLeft();
    // if (distanceL)
    // 
    //moveStop();
  } else {
    turnLeft();
    //moveStop();
  }

  //moveBackward();
  delay(300);
}

 
void setup() {
        Serial.begin(9600); // open the serial port at 9600 bps:

  myservo.attach(10);  
  myservo.write(120);
  delay(2000);
  //runner.execute();
  // runner.init();
  // //runner.addTask(tCapteur);
  // //runner.addTask(tDrive);
  // //runner.addTask(tLaneChange);

  // delay(5000);
  // //tCapteur.enable();
  // tDrive.enable();

  runner.init();  // Initialize the scheduler
  runner.addTask(tCapteur);  // Add tasks to the scheduler
  runner.addTask(tDrive);
  runner.addTask(tLaneChange);

  // Enable tasks to start running
  tCapteur.enable();
  tDrive.enable();
  tLaneChange.enable();

  delay(5000);  // Add a small delay to give things time to initialize
  //runner.enableAll();
  // distance = readPing();
  // delay(100);
  // distance = readPing();
  // delay(100);
  // distance = readPing();
  // delay(100);
  // distance = readPing();
  // delay(100);
}
 
void loop() {
  runner.execute();
  //Serial.println("OK");
}
 
/* Lis la distance entre l'auto et l'objet */
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
  delay(100);
  
  //moveForward(); // resume forward motion after turning
}
 
void turnLeft() {
  leftMotor.run(BACKWARD);    
  rightMotor.run(FORWARD);  
  delay(500);
  moveForward(); // resume forward motion after turning
}  
 
int lookRight() {
  int distanceR;
  float radDistanceAdj;
  for (int degree = 50; degree<90; degree=+5){
    myservo.write(degree);
    delay(20);
    distanceR = readPing();
    delay(100);
    radDistanceAdj = cos((degree * PI) / 180.0) * distanceR;

    // if (radDistanceAdj > 40) {
          if (radDistanceAdj > 10) {

      break;
    }
  }
  myservo.write(90);
  return radDistanceAdj;

}
 
int lookLeft() {
  int distanceL;
  float radDistanceAdj;
  for (int degree = 130; degree>90; degree=-5){
    myservo.write(degree);
    delay(20);
    distanceL = readPing();
    delay(100);
    radDistanceAdj = cos((degree * PI) / 180.0) * distanceL;

    // if (radDistanceAdj > 40) {
          if (radDistanceAdj > 10) {

      break;
    }
  }
  myservo.write(90);
  return radDistanceAdj;
}