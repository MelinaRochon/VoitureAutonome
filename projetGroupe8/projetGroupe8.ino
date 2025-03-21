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
 
AF_DCMotor leftMotor(4, MOTOR34_1KHZ);
AF_DCMotor rightMotor(3, MOTOR34_1KHZ);

Servo myservo;  

boolean goesForward = false;
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
  if (distance <= 50) { 
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
  int distanceL = lookLeft();
  int distanceR = lookRight();

  Serial.println("distance gauche: ");
  Serial.println(distanceL);
  Serial.println("distance droite: ");
  Serial.println(distanceR);


  if (distanceL > 20 || distanceR > 20) {
    if (distanceL > distanceR) {
      int delai = calculateDelay(degreeMaxL);
      Serial.print("Delai calculee a la gauche: ");
      Serial.println(delai);

      turnLeft();
      delay(delai);
    } else {
      //if (distanceR >= distanceL) {
      int delai = calculateDelay(degreeMaxR);
      Serial.print("Delai calculee a la droite: ");
      Serial.println(delai);

      turnRight();
      delay(delai);
    }
  } else {
    // Check if object is forward?
    // Go backwards
    moveBackward();
    delay(1000);
    laneChange();
  }
  
  // check si cleared object
  tCapteur.enable();
  tDrive.enable();
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
  int returnVal = round(temps * 1000);
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
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(speedSet);
    rightMotor.setSpeed(speedSet);
    delay(5);
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
  rightMotor.run(BACKWARD);    
  leftMotor.run(FORWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(speedSet);
    rightMotor.setSpeed(speedSet);
    delay(5);
  }
}
 
 // Possible error! Motors are inverted
void turnLeft() {
  leftMotor.run(BACKWARD);    
  rightMotor.run(FORWARD);  
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    leftMotor.setSpeed(speedSet);
    rightMotor.setSpeed(speedSet);
    delay(5);
  }
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
  degreeMaxR = 45; // initialisation
  // for (int degree = 90; degree >= 0; degree -= 5) {
  for (int degree = 90; degree >= 45; degree -= 5) {

    myservo.write(degree);
    delay(20);
    int tmpDistance = readPing();
    if (tmpDistance > distanceR) {
      distanceR = tmpDistance;
      degreeMaxR = degree;
    }
  }
  myservo.write(90); // Reset servo to center
  return distanceR;
}
 
int lookLeft() {
  int distanceL = 0;
  degreeMaxL = 135;
  // for (int degree = 90; degree <= 180; degree += 5) {
  for (int degree = 90; degree <= 135; degree += 5) {

    myservo.write(degree);
    delay(20);
    int tmpDistance = readPing();
    if (tmpDistance > distanceL) {
      distanceL = tmpDistance;
      degreeMaxL = degree;
    }
  }
  myservo.write(90); // Reset servo to center
  return distanceL;
}






