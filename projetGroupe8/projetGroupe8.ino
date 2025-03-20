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

// Cree les taches
Task tCapteur(2000, TASK_FOREVER, &tCapteurCallback); // lis les capteurs
Task tDrive(3000, TASK_FOREVER, &tDriveCallback); 

Scheduler runner;

void tCapteurCallback() {
  distance = readPing();
  Serial.print("In tCapteurCallback: distance = ");
  Serial.println(distance);

  if (distance <= 30) { // Obstacle detected within 30 cm
      Serial.println("Obstacle detected. Enabling lane change.");
      tDrive.disable();    // Disable the driving task
      laneChange();
  } else {
      Serial.println("Path clear. Resuming forward motion.");
      tDrive.enable();       // Enable the driving task
  }
}

void tDriveCallback() {
    Serial.println("In tDriveCallback");

  if(distance <= 10) {
    moveStop();
  }
  else {
    moveForward();
    delay(1000);
    moveStop();
    delay(2000);
  }

}

void laneChange() {
  Serial.println("In tLaneChangeCallback: Changing lane.");

  // Stop the car
  moveStop();
  delay(500);

  // Look for the best direction to turn
  int distanceR = lookRight();
  int distanceL = lookLeft();

  // Turn in the direction with more space
  if (distanceR >= distanceL) {
      Serial.println("Turning right.");
      turnRight();
  } else {
      Serial.println("Turning left.");
      turnLeft();
  }

  // Advance a short distance
  //advanceShortDistance();

  // Turn back to the original direction
  if (distanceR >= distanceL) {
      Serial.println("Turning back left.");
      turnLeft();
  } else {
      Serial.println("Turning back right.");
      turnRight();
  }

  // Resume forward motion
  moveForward();
  delay(500);
  tDrive.enable(); // Re-enable driving task
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

  // Enable tasks to start running
  tCapteur.enable();
  tDrive.enable();

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
  //if (!goesForward) {
    goesForward = true;
    leftMotor.run(FORWARD);      
    rightMotor.run(FORWARD);
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
      leftMotor.setSpeed(speedSet);
      rightMotor.setSpeed(speedSet);
      delay(5);
    }
  //}
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





  delay(500); // We should try to change this value to depend on the angle at which the sensor found an opening. Maybe too difficult tho...




  moveStop();
  //moveForward(); // resume forward motion after turning
}
 
void turnLeft() {
  leftMotor.run(BACKWARD);    
  rightMotor.run(FORWARD);  
  delay(500);
  moveStop();
  //moveForward(); // resume forward motion after turning
}  

void advanceShortDistance() {
  Serial.println("Advancing a short distance.");
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
  leftMotor.setSpeed(150);
  rightMotor.setSpeed(150);
  delay(2000); // Move forward for 1 second
  moveStop();
}
 
int lookRight() {
  int distanceR = 0;
  for (int degree = 120; degree >= 50; degree -= 5) {
      myservo.write(degree);
      delay(20);
      distanceR = readPing();
      if (distanceR > 10) { // Stop scanning if a clear path is found
          break;
      }
  }
  myservo.write(120); // Reset servo to center
  return distanceR;
}
 
int lookLeft() {
  int distanceL = 0;
  for (int degree = 120; degree <= 190; degree += 5) {
      myservo.write(degree);
      delay(20);
      distanceL = readPing();
      if (distanceL > 10) { // Stop scanning if a clear path is found
          break;
      }
  }
  myservo.write(120); // Reset servo to center
  return distanceL;
}