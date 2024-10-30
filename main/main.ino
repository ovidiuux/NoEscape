/*
-----------------------------------------------------------------------
sySTEMatic Robotics: NoEscape: Ovidiu Căpraru & Pasere Sebastian
-----------------------------------------
UNIVERSITATEA NAȚIONALĂ DE ȘTIINTĂ ȘI TEHNOLOGIE POLITEHNICA BUCUREȘTI
-----------------------------------------------------------------------
*/

#include <HCSR04.h>
#include <StackArray.h>

#define PWM1 3
#define AIN1 4
#define AIN2 5
#define PWM2 6
#define BIN1 7
#define BIN2 8
#define DEBUG 0  // DEBUG
#define STST 9  // START-STOP

/* Valori generale */
int detectionThreshold = 50;
int NORMAL_SPEED = 200;
int HIGH_SPEED = 255;
int LOW_SPEED = 150;
const int maxGridSize = 20;
int grid[maxGridSize][maxGridSize];
int posX = 0, posY = 0;
int direction = 0;
bool STST_STOPPED = false;
bool DRUP_MAZE = false;


/* Definiți intervale */
unsigned long lastActionTime = 0;
unsigned long actionInterval = 100;
unsigned long lastCorrectionTime = 0;
const int correctionInterval = 100;


void POSITION() {
  switch (direction) {
    case 0: posY -= 1; break;
    case 1: posX += 1; break;
    case 2: posY += 1; break;
    case 3: posX -= 1; break;
  }
}

class TB6612FNG {
public:
  TB6612FNG() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
  }

  void moveForward(int speed) { controlMotors(speed, HIGH, LOW, HIGH, LOW); }
  void turnLeft(int speed) { controlMotors(speed, LOW, HIGH, HIGH, LOW); delay(800); direction = (direction + 3) % 4; POSITION(); }
  void turnRight(int speed) { controlMotors(speed, HIGH, LOW, LOW, HIGH); delay(800); direction = (direction + 1) % 4; POSITION(); }
  void turnAround(int speed) { controlMotors(speed, LOW, HIGH, HIGH, LOW); delay(800); (direction + 2) % 4; POSITION(); }
  void stopMotors() { analogWrite(PWM1, 0); analogWrite(PWM2, 0); }

private:
  void controlMotors(int speed, int a1, int a2, int b1, int b2) {
    analogWrite(PWM1, speed);
    analogWrite(PWM2, speed);
    digitalWrite(AIN1, a1);
    digitalWrite(AIN2, a2);
    digitalWrite(BIN1, b1);
    digitalWrite(BIN2, b2);
  }
};


HCSR04 HCLEFT(2, 3);
HCSR04 HCRIGHT(2, 3);
HCSR04 HCFRONT(2, 3);
StackArray<int> backtrackStackX;
StackArray<int> backtrackStackY;
TB6612FNG sySTEMatic;

void UNDO_BACKTRACK() {
  if (!backtrackStackX.isEmpty() && !backtrackStackY.isEmpty()) {
    posX = backtrackStackX.pop();
    posY = backtrackStackY.pop();
    if (DEBUG) {
      Serial.print("Backtracking la poziția: ");
      Serial.print(posX);
      Serial.print(", ");
      Serial.println(posY);
    }
    sySTEMatic.turnRight(LOW_SPEED);
    delay(1000);
  }
}

void AI_GATES() {
  int openPaths = 0;
  int directions[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (int i = 0; i < 4; i++) {
    int newX = posX + directions[i][0];
    int newY = posY + directions[i][1];
    if (newX >= 0 && newX < maxGridSize && newY >= 0 && newY < maxGridSize && grid[newX][newY] == 0) {
      openPaths++;
    }
  }
  if (openPaths > 1) {
    backtrackStackX.push(posX);
    backtrackStackY.push(posY);
  }
}

void OBSTACLE(int dir) {
  int newX = posX, newY = posY;
  
  switch (dir) {
    case 0: newY -= 1; break; // N
    case 1: newX += 1; break; // E
    case 2: newY += 1; break; // S
    case 3: newX -= 1; break; // V
  }
  
  if (newX >= 0 && newX < maxGridSize && newY >= 0 && newY < maxGridSize) {
    grid[newX][newY] = 1; 
  }
}



long readDistance(HCSR04 &sensor) {
  long totalDistance = 0;
  int validReadings = 0;
  for (int i = 0; i < 3; i++) {
    long distance = sensor.dist();
    if (distance > 0) {
      totalDistance += distance;
      validReadings++;
    }
    delay(20);
  }
  return (validReadings > 0) ? totalDistance / validReadings : -1;
}

void RFC(long distanceFront, long distanceLeft, long distanceRight) {
  grid[posX][posY] = 2;

  if (distanceFront < 20) {
    OBSTACLE(direction);
  }

  if (distanceLeft < 20) {
    OBSTACLE((direction + 3) % 4);
  }

  if (distanceRight < 20) {
    OBSTACLE((direction + 1) % 4);
  }
  
  AI_GATES();
}

void setup() {
  pinMode(STST, INPUT);
  if (DEBUG) Serial.begin(9600);
  for (int i = 0; i < maxGridSize; i++) {
    for (int j = 0; j < maxGridSize; j++) {
      grid[i][j] = 0;
    }
  }
  if (DEBUG) Serial.println("sySTEMatic  " + String(maxGridSize) + " x " + String(maxGridSize));
  delay(1000);
}

bool canMove(int currentX, int currentY) {
  int directions[4][2] = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 } };
  for (int i = 0; i < 4; i++) {
    int newX = currentX + directions[i][0];
    int newY = currentY + directions[i][1];
    if (newX >= 0 && newX < maxGridSize && newY >= 0 && newY < maxGridSize) {
      if (grid[newX][newY] == 0) {
        return true;
      }
    }
  }
  return false;
}

void moveForward(long distanceLeft, long distanceRight) {
    int correctionSpeed = 50;
    int targetDistance = 15;
    if (millis() - lastCorrectionTime >= correctionInterval) {
        if (distanceLeft > targetDistance + 5) sySTEMatic.turnLeft(correctionSpeed);
        else if (distanceRight > targetDistance + 5) sySTEMatic.turnRight(correctionSpeed);
        else sySTEMatic.moveForward(NORMAL_SPEED);
        lastCorrectionTime = millis();
    }
}

void loop() {
  unsigned long currentMillis = millis();

  if (digitalRead(STST) == HIGH) {
        sySTEMatic.stopMotors();
        while (digitalRead(STST) == HIGH) {
            delay(100);
        }
  }

  long distanceFront = readDistance(HCFRONT);
  long distanceLeft = readDistance(HCLEFT);
  long distanceRight = readDistance(HCRIGHT);

  RFC(distanceFront, distanceLeft, distanceRight);

  if (grid[posX][posY] == 0) {
    grid[posX][posY] = 2;
  }
  
  if (distanceLeft < detectionThreshold && distanceRight < detectionThreshold && abs(distanceLeft - distanceRight) < 5) {
    sySTEMatic.moveForward(NORMAL_SPEED); 
    if (!DRUP_MAZE) { sySTEMatic.turnLeft(LOW_SPEED); DRUP_MAZE = true; }
  } else if (!canMove(posX, posY)) {
      UNDO_BACKTRACK();
  } else if (distanceFront < detectionThreshold) {
    if (distanceLeft > distanceRight) {
      sySTEMatic.turnLeft(NORMAL_SPEED);
    } else {
      sySTEMatic.turnRight(NORMAL_SPEED);
    }
  } else {
    moveForward(distanceLeft, distanceRight);
  }

  if (DEBUG) {
    Serial.println("sySTEMatic Robotics: HCSR04: " + String(distanceFront) + ", " + String(distanceLeft) + ", " + String(distanceRight));
  }

  if (DEBUG) {
    for (int i = 0; i < maxGridSize; i++) {
      for (int j = 0; j < maxGridSize; j++) {
        Serial.print(grid[i][j]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}

void gridPosition() {
  if (direction == 0) posY--;
  else if (direction == 1) posX++;
  else if (direction == 2) posY++;
  else if (direction == 3) posX--;
}

