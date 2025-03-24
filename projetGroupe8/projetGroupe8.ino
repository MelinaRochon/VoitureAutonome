#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>
#include <TaskScheduler.h>

#define TRIG_PIN A4
#define ECHO_PIN A5
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // Sets the speed of DC motors
#define MAX_SPEED_OFFSET 20
#define DIRECTION_LEFT 0
#define DIRECTION_RIGHT 1
 
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
    // } else {
    //   seenObstacle = true;
    // }

  } else {
    tDrive.enable();  
    // seenObstacle = false;   
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

  Range rangeL = lookDirection(DIRECTION_LEFT);
  Range rangeR = lookDirection(DIRECTION_RIGHT);

  int widthL = abs(rangeL.endDegree - rangeL.startDegree);
  int widthR = abs(rangeR.startDegree - rangeR.endDegree);

  Serial.println("Space on the left: ");
  Serial.println(widthL);
  Serial.println("Space on the right: ");
  Serial.println(widthR);

  int tmpDistance = readPing(); // Measure the distance using the ultrasonic sensor

  if (widthL > 15 || widthR > 15) {
    if (widthL > widthR) {
      int delai = calculateDelay((rangeL.startDegree + rangeL.endDegree) / 2);
      Serial.print("Calculated delay to the left: ");
      Serial.println(delai);

      turnLeft();
      delay(delai);
    } else {
      int delai = calculateDelay((rangeR.startDegree + rangeR.endDegree) / 2);
      Serial.print("Calculated delay to the right: ");
      Serial.println(delai);

      turnRight();
      delay(delai);
    }
  } else if (tmpDistance > 50) {
    tCapteur.enable();
    return;
  } else {
    moveBackward();
    delay(500);
    laneChange();
  }

  moveStop();
  delay(500);
  tCapteur.enable();
}


int calculateDelay(int degree) {
  // float vitAngulaire = 0.19895;
  // Serial.println("degree a claucler");
  // Serial.println(radians(degree));
  // float temps = radians(degree) / vitAngulaire; // temps en secondes

  // // converti temps en milisecondes
  // int returnVal = round(temps * 1000);
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
  // delay(2000);
  // myservo.write(135);
  // delay(2000);
  // myservo.write(45);
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
  // for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(MAX_SPEED);
    rightMotor.setSpeed(MAX_SPEED);
  //   delay(5);
  // }
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
  rightMotor.run(BACKWARD);    
  leftMotor.run(FORWARD);
  // for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(MAX_SPEED);
    rightMotor.setSpeed(MAX_SPEED);
    // delay(5);
  // }
}
 
 // Possible error! Motors are inverted
void turnLeft() {
  leftMotor.run(BACKWARD);    
  rightMotor.run(FORWARD);  
  // for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(MAX_SPEED);
    rightMotor.setSpeed(MAX_SPEED);
  //   delay(5);
  // }
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

// General function to look in a direction based on the given parameters
Range lookDirection(int direction) {
  int startDegree, endDegree, rangeMax = -1;
  int degreeMax = (direction == DIRECTION_RIGHT) ? 45 : 135; // Default degrees if no valid range is found
  int degreeStep = (direction == DIRECTION_RIGHT) ? -5 : 5;  // Direction of degree change
  int startScan = (direction == DIRECTION_RIGHT) ? 90 : 90;
  int endScan = (direction == DIRECTION_RIGHT) ? 45 : 135;
  Range range = {90, 90};  // Default range starting point

  Serial.print(direction == DIRECTION_RIGHT ? "Look right: " : "Look left: ");

  // Scan from center to the specified direction
  for (int degree = startScan; (direction == DIRECTION_RIGHT ? degree >= endScan : degree <= endScan); degree += degreeStep) {
    myservo.write(degree);
    delay(20);

    int tmpDistance = readPing();
    Serial.print(tmpDistance);
    Serial.print(" - degree: ");
    Serial.println(degree);

    if (tmpDistance > 50) {
      if (startDegree == -1) {
        startDegree = degree;
      }
      endDegree = degree;
      int tmpRangeMax = abs(startDegree - endDegree);
      if (tmpRangeMax > rangeMax) {
        rangeMax = tmpRangeMax;
        range.startDegree = startDegree;
        range.endDegree = endDegree;
      }
    } else {
      startDegree = -1;
      endDegree = -1;
    }
  }

  myservo.write(90);

  if (range.startDegree != -1 && range.endDegree != -1) {
    int middleDegree = round((range.startDegree + range.endDegree) / 2);
    Serial.print("Widest range: Start = ");
    Serial.print(range.startDegree);
    Serial.print(", End = ");
    Serial.println(range.endDegree);
    degreeMax = middleDegree;
  } else {
    Serial.println("No range found with distance > 20 cm.");
  }

  return range;
}

 
