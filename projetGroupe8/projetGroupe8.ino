#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>
#include <TaskScheduler.h>

#define TRIG_PIN A4
#define ECHO_PIN A5
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // Sets the speed of DC motors
#define MAX_SPEED_OFFSET 20
 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
 
AF_DCMotor leftMotor(4, MOTOR34_1KHZ);
AF_DCMotor rightMotor(3, MOTOR34_1KHZ);

struct Range
{
  int startDegree;
  int endDegree;
};

Servo myservo;  

boolean goesForward = false;
boolean seenObstacle = false;
int distance;
int degreeMaxR;
int degreeMaxL;
int speedSet = 0;
 
// Function prototypes
int readPing();
void moveStop();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
Range lookLeft();
Range lookRight();

// Callback methods
void tCapteurCallback();
void tDriveCallback();

// Cree les taches
Task tCapteur(2000, TASK_FOREVER, &tCapteurCallback); // lis les capteurs
Task tDrive(3000, TASK_FOREVER, &tDriveCallback); 

Scheduler runner;

void tCapteurCallback() {
  distance = readPing();
  Serial.print("Distance....: ");
  Serial.println(distance);

  if (distance <= 50) { 
    // if (seenObstacle){
      Serial.println("Obejct too close, stopping...");
      tDrive.disable();
      laneChange();
  } else {
    tDrive.enable();  
  }
}


void tDriveCallback() {
  
  if(distance <= 50) {
    moveStop();
  }
  else {
    moveForward();
  }
}

void laneChange() {
  moveStop();
  delay(500);
  tCapteur.disable();

  Range rangeL = lookLeft();
  Range rangeR = lookRight();

  
  int widthL = abs(rangeL.endDegree - rangeL.startDegree);
  int widthR = abs(rangeR.startDegree - rangeR.endDegree);

  Serial.println("espace à la gauche: ");
  Serial.println(widthL);
  Serial.println("espace à la droite: ");
  Serial.println(widthR);

  int tmpDistance = readPing(); // Measure the distance using the ultrasonic sensor
  
  if (widthL > 15 || widthR > 15) {
    if (widthL > widthR) {
      int delai = calculateDelay(degreeMaxL);
      Serial.print("Delai calculee a la gauche: ");
      Serial.println(delai);

      turnLeft();
      delay(delai);
      // moveStop();
    } else {
      //if (distanceR >= distanceL) {
      int delai = calculateDelay(degreeMaxR);
      Serial.print("Delai calculee a la droite: ");
      Serial.println(delai);

      turnRight();
      delay(delai);
      // moveStop();
    }
  } else if (tmpDistance > 50) {
    // tDrive.enable();
    tCapteur.enable();
    return;
  } else {
    // Check if object is forward?
    // Go backwards
    moveBackward();
    delay(500);
    laneChange();
  }
  
  // check si cleared object
  moveStop();
  delay(500);
  tCapteur.enable();
  // tDrive.enable();
}

int calculateDelay(int degree) {
  // converti temps en milisecondes
  float vitesseRoues = 0.19895; // en m/s
  float radian = radians(degree);
  float largeurEntreRoues = 0.11; // en metre
  Serial.println("degree a claucler");
  Serial.println(degree);
  Serial.println("radian a claucler");

  Serial.println(radian);
  float temps = (radian * largeurEntreRoues) / (2 * vitesseRoues); // temps en secondes

  // converti temps en milisecondes
  int returnVal = round(temps * 500);
  return returnVal;
}
 
void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  myservo.attach(10);  
  myservo.write(90);
  delay(2000);

  runner.init();  // Initialize the scheduler
  runner.addTask(tCapteur);  // Add tasks to the scheduler
  runner.addTask(tDrive);

  // Enable tasks to start running
  tCapteur.enable();
  tDrive.enable();

  delay(500);  // Add a small delay to give things time to initialize
}
 
void loop() {
  runner.execute();
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
  goesForward = true;
  leftMotor.run(FORWARD);      
  rightMotor.run(FORWARD);
    leftMotor.setSpeed(MAX_SPEED);
    rightMotor.setSpeed(MAX_SPEED);
}
 
void moveBackward() {
  goesForward = false;
  leftMotor.run(BACKWARD);      
  rightMotor.run(BACKWARD);
 // Slowly goes backwards
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(speedSet);
    rightMotor.setSpeed(speedSet);
    delay(5);
  }
}  
 
void turnRight() {
  rightMotor.run(BACKWARD);    
  leftMotor.run(FORWARD);
    leftMotor.setSpeed(MAX_SPEED);
    rightMotor.setSpeed(MAX_SPEED);
}
 
void turnLeft() {
  leftMotor.run(BACKWARD);    
  rightMotor.run(FORWARD);  
    leftMotor.setSpeed(MAX_SPEED);
    rightMotor.setSpeed(MAX_SPEED);
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
 
Range lookRight() {
  int distanceR = 0; // Initialize the maximum distance
  int startDegree = -1; // Start of the range (initialize to -1 to indicate no range found yet)
  int endDegree = -1;   // End of the range
  int rangeMax = -1;
  degreeMaxR = 45;      // Default to 45° if no valid range is found
  Range range = {90, 90};

    Serial.print("Look right: ");
  for (int degree = 90; degree >= 45; degree -= 5) {

  // Scan from 90° (center) to 45° (right) in steps of 5°
    myservo.write(degree); // Move the servo to the current angle
    delay(20);             // Wait for the servo to stabilize

    int tmpDistance = readPing(); // Measure the distance using the ultrasonic sensor
    Serial.print(tmpDistance);
    Serial.print(" - degree: ");
    Serial.println(degree);

    // Check if the distance is greater than 100 cm
    if (tmpDistance > 50) {
      // If this is the first degree in the range, set startDegree
      if (startDegree == -1) {
        startDegree = degree;
      }
      // Update endDegree to the current degree
      endDegree = degree;
      int tmpRangeMax = abs(startDegree - endDegree);
      if(tmpRangeMax > rangeMax){
        rangeMax = tmpRangeMax;
        range.startDegree = startDegree;
        range.endDegree = endDegree;
      }
    } 
    else {
      startDegree = -1;
      endDegree = -1;
    }
  }

  // Reset the servo to the center (90°)
  myservo.write(90);

  // Calculate the widest range
  if (range.startDegree != -1 && range.endDegree != -1) {
    degreeMaxR = round((range.startDegree + range.endDegree) / 2);
    Serial.print("Widest range on the right: ");
    Serial.print("Start = ");
    Serial.print(range.startDegree);
    Serial.print(", End = ");
    Serial.println(range.endDegree);
  } else {
    Serial.println("No range found with distance > 20 cm.");
  }

  return range;
}

Range lookLeft() {
  int startDegree = -1; // Start of the range (initialize to -1 to indicate no range found yet)
  int endDegree = -1;   // End of the range
  int rangeMax = -1;
  degreeMaxL = 135;     // Default to 135° if no valid range is found
  Range range = {90, 90};
    Serial.print("Look left: ");

  // Scan from 90° (center) to 135° (left) in steps of 5°
    for (int degree = 90; degree <= 135; degree += 5) {

    myservo.write(degree); // Move the servo to the current angle
    delay(20);             // Wait for the servo to stabilize

    int tmpDistance = readPing(); // Measure the distance using the ultrasonic sensor
    Serial.print(tmpDistance);
    Serial.print(" - degree: ");
    Serial.println(degree);

    // Check if the distance is greater than 100 cm
    if (tmpDistance > 50) {
      // If this is the first degree in the range, set startDegree
      if (startDegree == -1) {
        startDegree = degree;
      }
      // Update endDegree to the current degree
      endDegree = degree;
      int tmpRangeMax = abs(startDegree - endDegree);
      if(tmpRangeMax > rangeMax){
        rangeMax = tmpRangeMax;
        range.startDegree = startDegree;
        range.endDegree = endDegree;
      }
    } 
    else {
      startDegree = -1;
      endDegree = -1;
    }
  }

  // Reset the servo to the center (90°)
  myservo.write(90);

  // Calculate the widest range
  if (range.startDegree != -1 && range.endDegree != -1) {
    degreeMaxL = round((range.startDegree + range.endDegree) / 2);
    Serial.print("Widest range on the left: ");
    Serial.print("Start = ");
    Serial.print(range.startDegree);
    Serial.print(", End = ");
    Serial.println(range.endDegree);
  } else {
    Serial.println("No range found with distance > 20 cm.");
  }

  // Return the range as a struct
  return range;
}
