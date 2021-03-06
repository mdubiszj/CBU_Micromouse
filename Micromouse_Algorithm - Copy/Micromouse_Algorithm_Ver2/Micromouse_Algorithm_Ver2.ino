// NOTE: adjacent/neighboring cells mean direct, not diagonal\
// TO DO: Transpose southIsOpen to fit with whenever we rotate or move
// TO DO: Create closed-loop control for MoveFwd to keep it moving straight
#include <StackArray.h>
#include <MKRMotorCarrier.h>
#include <PID.h>
#include "BNO055_support.h"
#include <Wire.h>
#define INTERRUPT_PIN 6
// IR SENSOR VARIABLES/CONSTANTS
// Analog reading of about 11cm distance (largest distance from IR sensor to wall if there is a wall next to the mouse)
#define IR_THRESHOLD  350
#define IR_LEFT_PIN A6      // IN1 on Motor Carrier
#define IR_RIGHT_PIN A1     // IN2 on Motor Carrier
#define IR_FRONT_PIN A2     // IN4 on Motor Carrier

static int batteryVoltage;    // Variable to store the battery voltage

int m1Duty = 50, m2Duty = 50, mDuty = 50;   // for adjusting duty cycle, m1 stays constant & m2 adjusts to match
                      // minimum duty cycle: 25
int cellDistance = 18;    // in cm
int targetEncoderCount;
bool hasMovedFwd = false, hasTurnedLeft = false, hasTurnedRight = false;

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData;    // structure to hold the Euler data
int theta = 90;
unsigned long lastTime = 0;

struct MazeCell {   // Struct holds maze coordinates (x,y) and the corresponding cell value (distance)
  int xStack;
  int yStack;
  int cellValue;

  bool hasNWall;
  bool hasSWall;
  bool hasWWall;
  bool hasEWall;

  void clearData();
};

void MazeCell::clearData() {
  xStack = 0;
  yStack = 0;
  cellValue = 0;

  hasNWall = false;
  hasSWall = false;
  hasWWall = false;
  hasEWall = false;
}

StackArray <MazeCell> floodFillStack;   // holds a stack of type Mazecell

// micromouse would start at maze[4][0]

bool isInStack[5][5] = { false };

MazeCell currentPush;                     // to push current
MazeCell currentLocation = { 4, 0, 4, false, true, true, true };   // current location of robot
MazeCell currentPop;                      // to read top of stack
MazeCell updateXYPeek;                    // peek top of the stack for updating xVirtual and yVirtual
MazeCell adjacentCell[4];               // array to store adjacent cells and their information
MazeCell currentLocAdj[4];                //*** Consider moving declaration to main() (variable isn't used anywhere else)
MazeCell nextLocation;                  //*** Consider moving declaration to main() (variable isn't used anywhere else)
MazeCell maze[5][5];

int currentLocAdjCellValue[4];              //*** Consider renaming and moving declaration to main() (variable isn't used anywhere else)
int adjacentVal[4];         // array to store adjacent cell value
int md = 2;                       // minimum distance of adjacent cells
int top, bottom, left, right;
int x = 4;                        // current location looking at in the stack (ROW)
int y = 0;                        // current location looking at in the stack (COL)
int xVirtual = 3, yVirtual = 0;   // current x and y location of cell to be analyzed
                  // (first occurrence is at [3][0]
int xNext = 2, yNext = 0;               //*** Consider moving declaration to main() (variable isn't used anywhere else)
int arrayCounter = 0;             // counts array size for finding md
int j;                          //*** Consider moving declaration to main() (variable isn't used anywhere else)
int currentLocMD = 2;                 //*** Consider moving declaration to main() (variable isn't used anywhere else)
int tempX, tempY;                   //*** Consider deleting declaration (variable is never used)
bool isAtCenter = false, stackIsEmpty = true, northIsOpen = true, southIsOpen = true, eastIsOpen = true, westIsOpen = true;

// **********************************************************************
// **********************************************************************
// **********************----------------------**************************
// **********************| MOVEMENT FUNCTIONS |**************************
// **********************----------------------**************************
// **********************************************************************
// **********************************************************************

void moveFwd(float numCells, int dutyCycleM1, int dutyCycleM2) {
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
  targetEncoderCount = numCells * (7800 / 2);

  Serial.println("Moving Forward...");
  M1.setDuty(dutyCycleM1);
  M2.setDuty(dutyCycleM2);

  while (abs(encoder1.getRawCount()) < targetEncoderCount) {
    // Get encoder data
    double encoder1Count = encoder1.getRawCount() * -1;
    double encoder2Count = encoder2.getRawCount();
    double encoderDif = encoder1Count - encoder2Count;

    //    Serial.print("Encoder1 Pos [counts]: ");
    //    Serial.println(encoder1Count);
    //    Serial.print("Encoder2 Pos [counts]: ");
    //    Serial.println(encoder2Count);
    //    Serial.print("Dif: ");
    //    Serial.println(encoderDif);
    //    Serial.println();
  }

  M1.setDuty(0);
  M2.setDuty(0);
  hasMovedFwd = true;
}

void turnLeft(int dutyCycleM1, int dutyCycleM2) {
  theta = theta + 90;
  if (theta == 360)
    theta = theta - 360;
  bno055_read_euler_hrp(&myEulerData);
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw - 75; //used to be 90
  float adjustTargetYaw = targetYaw;

  if (targetYaw < 0) {   // may not need this
    adjustTargetYaw = targetYaw + 360;
  }

  Serial.println("Turning Left...");
  M1.setDuty(-(dutyCycleM1 - 20));
  M2.setDuty(dutyCycleM2 - 20);

  if (adjustTargetYaw < 360 && adjustTargetYaw > 270) {         // i.e. 0 < initialYaw < 90
    while (float(myEulerData.h) / 16.00 < adjustTargetYaw) {    // For when current yaw is 45 - 0 degrees
      //    delay(5); 
      //     Serial.print("Heading(Yaw): ");                // To read out the Heading (Yaw)
      //    Serial.println(float(myEulerData.h) / 16.00);   // Convert to degrees

      bno055_read_euler_hrp(&myEulerData);          // Update Euler data into the structure
    }

    while (float(myEulerData.h) / 16.00 > adjustTargetYaw) {    // For when current yaw is 359.99 - adjustedTargetYaw 
      //     delay(5); 
      //     Serial.print("Heading(Yaw): ");                    // To read out the Heading (Yaw)
      //     Serial.println(float(myEulerData.h) / 16.00);      // Convert to degrees

      bno055_read_euler_hrp(&myEulerData);          // Update Euler data into the structure
    }
  }

  else {
    while (float(myEulerData.h) / 16.00 > targetYaw) {
      //    delay(5); 
      //    Serial.print("Heading(Yaw): ");                   // To read out the Heading (Yaw)
      //     Serial.println(float(myEulerData.h) / 16.00);    // Convert to degrees

      bno055_read_euler_hrp(&myEulerData);          // Update Euler data into the structure
    }
  }

  M1.setDuty(0);
  M2.setDuty(0);
  hasTurnedLeft = true;
}


// Turn the Mouse Right
void turnRight(int dutyCycleM1, int dutyCycleM2) {
  theta = theta - 90;
  if (theta < 0)
    theta = theta + 360;

  Serial.println("Begin Right Turn Function");

  // declare needed variables
  bno055_read_euler_hrp(&myEulerData);
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw + 78;
  float adjustTargetYaw = targetYaw;

  if (targetYaw >= 360)
    adjustTargetYaw = targetYaw - 360;

  // turn on motors to turn right
  Serial.println("Turning Right...");
  M1.setDuty(dutyCycleM1 - 20);
  M2.setDuty(-(dutyCycleM2 - 20));

  // loop as long the motors need to be on
  if (adjustTargetYaw < 90 && adjustTargetYaw > 0) {        // i.e. 270 < initialYaw < 360
    while (float(myEulerData.h) / 16.00 > adjustTargetYaw) {  // For when current yaw is between 270 & 359.99 degrees

      //    Serial.print("Heading(Yaw): ");                   // To read out the Heading (Yaw)
      //      Serial.println(float(myEulerData.h) / 16.00);   // Convert to degrees
      bno055_read_euler_hrp(&myEulerData);                    // Update Euler data into the structure
    }

    while (float(myEulerData.h) / 16.00 < adjustTargetYaw) {  // For when current yaw is between 0 & adjustedTargetYaw    
      //      Serial.print("Heading(Yaw): ");                 // To read out the Heading (Yaw)
      //      Serial.println(float(myEulerData.h) / 16.00);   // Convert to degrees
      bno055_read_euler_hrp(&myEulerData);          // Update Euler data into the structure
    }
  }
  else {
    while (float(myEulerData.h) / 16.00 < targetYaw) {
      //      Serial.print("Heading(Yaw): ");                 // To read out the Heading (Yaw)
      //      Serial.println(float(myEulerData.h) / 16.00);   // Convert to degrees
      bno055_read_euler_hrp(&myEulerData);                    // Update Euler data into the structure
    }
  }

  // turn motors off
  M1.setDuty(0);
  M2.setDuty(0);
  hasTurnedRight = true;
}

void turn180Deg(int dutyCycleM1, int dutyCycleM2) {   // this rotates 180 degrees clockwise 

  bno055_read_euler_hrp(&myEulerData);
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw + 180;
  float adjustTargetYaw = targetYaw;

  if (targetYaw > 360) {
    adjustTargetYaw = targetYaw - 360;
  }

  M1.setDuty(dutyCycleM1);
  M2.setDuty(-dutyCycleM2);

  if (adjustTargetYaw < 180) {     // i.e. 180 < initialYaw < 270
    while (float(myEulerData.h) / 16.00 - 360 < targetYaw - 360) {        // NOT SURE IF THIS IS RIGHT
      //      delay(5); 

      Serial.print("Heading(Yaw): ");                 // To read out the Heading (Yaw)
      Serial.println(float(myEulerData.h) / 16.00);   // Convert to degrees

      bno055_read_euler_hrp(&myEulerData);            // Update Euler data into the structure
    }
  }

  else {
    while (float(myEulerData.h) / 16.00 < targetYaw) {
      //      delay(5); 
      Serial.print("Heading(Yaw): ");                 // To read out the Heading (Yaw)
      Serial.println(float(myEulerData.h) / 16.00);   // Convert to degrees

      bno055_read_euler_hrp(&myEulerData);            // Update Euler data into the structure
    }
  }

  M1.setDuty(0);
  M2.setDuty(0);
}

// *******************-----------------------------**********************
// *******************| END OF MOVEMENT FUNCTIONS |**********************
// *******************-----------------------------**********************

// sense walls with IR sensors
void senseWalls(int theta) {
  // Read the input on analog pin
  int leftSensorValue = analogRead(IR_LEFT_PIN);
  int rightSensorValue = analogRead(IR_RIGHT_PIN);
  int frontSensorValue = analogRead(IR_FRONT_PIN);

  //  Serial.print("Left IR Value (raw): ");
  //  Serial.println(leftSensorValue);
  //
  //  Serial.print("Right IR Value (raw): ");
  //  Serial.println(rightSensorValue);
  //
  //  Serial.print("Front IR Value (raw): ");
  //  Serial.println(frontSensorValue);
  //
  //  Serial.println();

  switch (theta) {
    // mouse facing east
  case 0:
    northIsOpen = (leftSensorValue > IR_THRESHOLD) ? false : true; // these bools are with respect to global frame
    eastIsOpen = (frontSensorValue > IR_THRESHOLD) ? false : true;
    southIsOpen = (rightSensorValue > IR_THRESHOLD) ? false : true;
    westIsOpen = true;
    break;

    // mouse facing north
  case 90:
    northIsOpen = (frontSensorValue > IR_THRESHOLD) ? false : true;
    eastIsOpen = (rightSensorValue > IR_THRESHOLD) ? false : true;
    southIsOpen = true;
    westIsOpen = (leftSensorValue > IR_THRESHOLD) ? false : true;
    break;

    // mouse facing west
  case 180:
    northIsOpen = (rightSensorValue > IR_THRESHOLD) ? false : true;
    eastIsOpen = true;
    southIsOpen = (leftSensorValue > IR_THRESHOLD) ? false : true;
    westIsOpen = (frontSensorValue > IR_THRESHOLD) ? false : true;
    break;

    // mouse facing south
  case 270:
    northIsOpen = true;
    eastIsOpen = (leftSensorValue > IR_THRESHOLD) ? false : true;
    southIsOpen = (frontSensorValue > IR_THRESHOLD) ? false : true;
    westIsOpen = (rightSensorValue > IR_THRESHOLD) ? false : true;
    break;
  }

  // assign walls to cell
  maze[x][y].hasNWall = !northIsOpen;
  maze[x][y].hasSWall = !southIsOpen;
  maze[x][y].hasWWall = !westIsOpen;
  maze[x][y].hasEWall = !eastIsOpen;

  // assign walls to adjacent cells (ex. current cell north wall = above cell south wall)
  if (y >= 1) {
    maze[x][y - 1].hasEWall = !westIsOpen;
  }
  if (y <= 3) {
    maze[x][y + 1].hasWWall = !eastIsOpen;
  }
  if (x >= 1) {
    maze[x - 1][y].hasSWall = !northIsOpen;
  }
  if (x <= 3) {
    maze[x + 1][y].hasNWall = !southIsOpen;
  }
}


// **********************************************************************
// **********************************************************************
// ******************-------------------------------*********************
// ******************| MOVEMENT TRACKING FUNCTIONS |*********************
// ******************-------------------------------*********************
// **********************************************************************
// **********************************************************************

void moveNorth() {
  Serial.println("Moving north.");
  currentLocation.xStack--;
  currentLocation.cellValue = maze[currentLocation.xStack][y].cellValue;
  x--;
  xVirtual = x;
}

void moveSouth() {
  Serial.println("Moving south.");
  currentLocation.xStack++;
  currentLocation.cellValue = maze[currentLocation.xStack][y].cellValue;
  x++;
  xVirtual = x;
}

void moveWest() {
  Serial.println("Moving west.");
  currentLocation.yStack--;
  currentLocation.cellValue = maze[x][currentLocation.yStack].cellValue;
  y--;
  yVirtual = y;
}

void moveEast() {
  Serial.println("Moving east.");
  currentLocation.yStack++;
  currentLocation.cellValue = maze[x][currentLocation.yStack].cellValue;
  y++;
  yVirtual = y;
}

// ***************--------------------------------------*****************
// ***************| END OF MOVEMENT TRACKING FUNCTIONS |*****************
// ***************--------------------------------------*****************


// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************
// *********************---------------------------**********************
// *********************| MAIN FLOODFILL FUNCTION |**********************
// *********************---------------------------**********************
// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************

void floodFill(bool stackIsEmpty) {
  currentPush = { xVirtual, yVirtual, maze[xVirtual][yVirtual].cellValue, maze[xVirtual][yVirtual].hasNWall,
          maze[xVirtual][yVirtual].hasSWall, maze[xVirtual][yVirtual].hasWWall, maze[xVirtual][yVirtual].hasEWall };
  Serial.print("Current Push: x=");
  Serial.print(currentPush.xStack);
  Serial.print(", y=");
  Serial.print(currentPush.yStack);
  Serial.print(", cell value=");
  Serial.println(currentPush.cellValue);

  if (stackIsEmpty) {                       // only true for the 1st time
    floodFillStack.push(currentPush);             // push current physical cell location
    isInStack[currentPush.xStack][currentPush.yStack] = true;   // for boolean maze array
  }

  currentPop = floodFillStack.pop();                // pop    
  Serial.print("Current Pop: x=");
  Serial.print(currentPop.xStack);
  Serial.print(", y=");
  Serial.print(currentPop.yStack);
  Serial.print(", cell value=");
  Serial.println(currentPop.cellValue);
  isInStack[currentPop.xStack][currentPop.yStack] = false;

  MazeCell top = { xVirtual - 1, yVirtual, maze[xVirtual - 1][yVirtual].cellValue, maze[xVirtual - 1][yVirtual].hasNWall,
          maze[xVirtual - 1][yVirtual].hasSWall, maze[xVirtual - 1][yVirtual].hasWWall, maze[xVirtual - 1][yVirtual].hasEWall };

  MazeCell bottom = { xVirtual + 1, yVirtual, maze[xVirtual + 1][yVirtual].cellValue, maze[xVirtual + 1][yVirtual].hasNWall,
          maze[xVirtual + 1][yVirtual].hasSWall, maze[xVirtual + 1][yVirtual].hasWWall, maze[xVirtual + 1][yVirtual].hasEWall };

  MazeCell left = { xVirtual, yVirtual - 1, maze[xVirtual][yVirtual - 1].cellValue, maze[xVirtual][yVirtual - 1].hasNWall,
          maze[xVirtual][yVirtual - 1].hasSWall, maze[xVirtual][yVirtual - 1].hasWWall, maze[xVirtual][yVirtual - 1].hasEWall };

  MazeCell right = { xVirtual, yVirtual + 1, maze[xVirtual][yVirtual + 1].cellValue, maze[xVirtual][yVirtual + 1].hasNWall,
          maze[xVirtual][yVirtual + 1].hasSWall, maze[xVirtual][yVirtual + 1].hasWWall, maze[xVirtual][yVirtual + 1].hasEWall };

  if (((xVirtual - 1 >= 0) && (xVirtual - 1 <= 4)) && (maze[xVirtual - 1][yVirtual].cellValue != 0) && !maze[xVirtual][yVirtual].hasNWall && !isInStack[xVirtual][yVirtual]) {   // adjacent OPEN top cell
    adjacentCell[arrayCounter] = top;               // these are arrays
    adjacentVal[arrayCounter] = top.cellValue;    // these are arrays
    arrayCounter++;
    Serial.print("Top cell will be compared: ");
    Serial.print(xVirtual - 1);
    Serial.print(",");
    Serial.print(yVirtual);
    Serial.println();
    Serial.print("Array counter: ");
    Serial.print(arrayCounter);
    Serial.println();
  }

  if (((xVirtual + 1 >= 0) && (xVirtual + 1 <= 4)) && (maze[xVirtual + 1][yVirtual].cellValue != 0) && !maze[xVirtual][yVirtual].hasSWall && !isInStack[xVirtual][yVirtual]) {   // adjacent OPEN bottom cell
    adjacentCell[arrayCounter] = bottom;
    adjacentVal[arrayCounter] = bottom.cellValue;
    arrayCounter++;
        Serial.print("Bottom cell will be compared: ");
    Serial.print(xVirtual + 1);
    Serial.print(",");
    Serial.print(yVirtual);
    Serial.println();
    Serial.print("Array counter: ");
    Serial.print(arrayCounter);
    Serial.println();
  }

  if (((yVirtual - 1 >= 0) && (yVirtual - 1 <= 4)) && (maze[xVirtual][yVirtual - 1].cellValue != 0) && !maze[xVirtual][yVirtual].hasWWall && !isInStack[xVirtual][yVirtual]) {    // adjacent OPEN left cell
    adjacentCell[arrayCounter] = left;
    adjacentVal[arrayCounter] = left.cellValue;
    arrayCounter++;
    Serial.print("Left cell will be compared: ");
    Serial.print(xVirtual);
    Serial.print(",");
    Serial.print(yVirtual - 1);
    Serial.println();
    Serial.print("Array counter: ");
    Serial.print(arrayCounter);
    Serial.println();
  }

  if (((yVirtual + 1 >= 0) && (yVirtual + 1 <= 4)) && (maze[xVirtual][yVirtual + 1].cellValue != 0) && !maze[xVirtual][yVirtual].hasEWall && !isInStack[xVirtual][yVirtual]) {    // adjacent OPEN right cell
    adjacentCell[arrayCounter] = right;
    adjacentVal[arrayCounter] = right.cellValue;
    arrayCounter++;
    Serial.print("Right cell will be compared: ");
    Serial.print(xVirtual);
    Serial.print(",");
    Serial.print(yVirtual + 1);
    Serial.println();
    Serial.print("Array counter: ");
    Serial.print(arrayCounter);
    Serial.println();
  }

  md = adjacentVal[0]; // sets md to first vector value

  // find minimum value (distance) of adjacent cells
  for (int i = 0; i < arrayCounter; i++) {
    if (adjacentVal[i] < md) {
      md = adjacentVal[i];
      Serial.print("md in for loop: ");
      Serial.print(md);
      Serial.println();
    }
  }

  Serial.print("New md: ");
  Serial.print(md);
  Serial.println();

  // *********************************************
  // *********************************************
  // *********************************************
  // *********************************************
  // *********************************************
  // **********ACTUAL START OF FLOODFILL**********
  // *****************(RECURSION)*****************
  // *********************************************
  // *********************************************
  // *********************************************
  // *********************************************

  if (currentPop.cellValue - 1 != md) {
    
    maze[currentPop.xStack][currentPop.yStack].cellValue = md + 1; // update current cell value
    Serial.print("New Cell Value: ");
    Serial.print(maze[currentPop.xStack][currentPop.yStack].cellValue);
    Serial.println();
    
    for (int i = 0; i < arrayCounter; i++) {    // push all adjacent squares into stack
      if (adjacentVal[i] != 0) {      //*** THIS IF STATEMENT IS UNNECESSARY? (already checks if it's center cell above, line 430)
        floodFillStack.push(adjacentCell[i]);
        isInStack[adjacentCell[i].xStack][adjacentCell[i].yStack] = true;
        Serial.print("Pushing values into stack: i = ");
        Serial.print(i);
        
        Serial.println();
      }
    }

    //    // reset all array values for next recursion    
    //    for (int i = 0; i < 4; i++) {
    //      adjacentCell[i].clearData();
    //    }
    //
    //    for (int i = 0; i < 4; i++) {
    //      adjacentVal[i] = 0;
    //    }
    //
    //    arrayCounter = 0;
    //
    //    stackIsEmpty = floodFillStack.isEmpty();
    //
    //    updateXYPeek = floodFillStack.peek();
    //    xVirtual = updateXYPeek.xStack;
    //    Serial.print("xVirtual: ");
    //    Serial.print(xVirtual);
    //    Serial.println();
    //    yVirtual = updateXYPeek.yStack;
    //    Serial.print("yVirtual: ");
    //    Serial.print(yVirtual);
    //    Serial.println();
    //
    //    floodFill(stackIsEmpty);
  }

  stackIsEmpty = floodFillStack.isEmpty();
  Serial.print("Stack is empty: ");
  Serial.print(stackIsEmpty);
  Serial.println();

  if (!stackIsEmpty) {
    updateXYPeek = floodFillStack.peek(); // getting x and y for next recursion
    xVirtual = updateXYPeek.xStack;
    Serial.print("xVirtual: ");
    Serial.print(xVirtual);
    Serial.println();
    yVirtual = updateXYPeek.yStack;
    Serial.print("yVirtual: ");
    Serial.print(yVirtual);
    Serial.println();

    // reset all array values for next recursion    
    for (int i = 0; i < 4; i++) {
      adjacentCell[i].clearData();
    }

    for (int i = 0; i < 4; i++) {
      adjacentVal[i] = 0;
    }

    arrayCounter = 0;

    floodFill(stackIsEmpty);
  }
    // reset all array values for next recursion    
    for (int i = 0; i < 4; i++) {
      adjacentCell[i].clearData();
    }

    for (int i = 0; i < 4; i++) {
      adjacentVal[i] = 0;
    }

    arrayCounter = 0;
}

// *******************-----------------------------*********************
// *******************| END OF FLOODFILL FUNCTION |*********************
// *******************-----------------------------*********************


// **********************************************************************
// **********************************************************************
// *****************************---------********************************
// *****************************| SETUP |********************************
// *****************************---------********************************
// **********************************************************************
// **********************************************************************

void setup() {        // setup code runs once
  Serial.begin(115200); //Serial port initialization

  // Establishing the communication with the motor shield
  if (controller.begin()) {
    Serial.print("MKR Motor Shield connected, firmware version ");
    Serial.println(controller.getFWVersion());
  }
  else {
    Serial.println("Couldn't connect! Is the red led blinking? You may need to update the firmware with FWUpdater sketch");
    while (1);
  }

  // Reboot the motor controller; brings every value back to default
  Serial.println("reboot");
  controller.reboot();
  delay(500);

  // Reset the encoder internal counter to zero (can be set to any initial value)
  Serial.println("reset counters");
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);

  // Read the battery status
  float batteryVoltage = (float)battery.getConverted();
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage);
  Serial.print("V, Raw ");
  Serial.println(battery.getRaw());

  Wire.begin();

  // Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  // Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  // Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);

  // disgusting
  // initial cell values for maze distance to center
  maze[0][0].cellValue = 4;
  maze[0][1].cellValue = 3;
  maze[0][2].cellValue = 2;
  maze[0][3].cellValue = 3;
  maze[0][4].cellValue = 4;

  maze[1][0].cellValue = 3;
  maze[1][1].cellValue = 2;
  maze[1][2].cellValue = 1;
  maze[1][3].cellValue = 2;
  maze[1][4].cellValue = 3;

  maze[2][0].cellValue = 2;
  maze[2][1].cellValue = 1;
  maze[2][2].cellValue = 0;
  maze[2][3].cellValue = 1;
  maze[2][4].cellValue = 2;

  maze[3][0].cellValue = 3;
  maze[3][1].cellValue = 2;
  maze[3][2].cellValue = 1;
  maze[3][3].cellValue = 2;
  maze[3][4].cellValue = 3;

  maze[4][0].cellValue = 4;
  maze[4][1].cellValue = 3;
  maze[4][2].cellValue = 2;
  maze[4][3].cellValue = 3;
  maze[4][4].cellValue = 4;

  maze[4][0].hasWWall = true;
  maze[4][0].hasSWall = true;
  maze[4][0].hasEWall = true;
}


// **********************************************************************
// **********************************************************************
// *****************************--------*********************************
// *****************************| MAIN |*********************************
// *****************************--------*********************************
// **********************************************************************
// **********************************************************************

void loop() {
  // main code; runs repeatedly
  float batteryVoltage = (float)battery.getConverted(); // Reads the battery status

  // Reset to the default values if the battery level is lower than 11V
  if (batteryVoltage < 8) {
    Serial.println(" ");
    Serial.println("WARNING: LOW BATTERY");
    Serial.println("ALL SYSTEMS DOWN");
    M1.setDuty(0);
    M2.setDuty(0);
    M3.setDuty(0);
    M4.setDuty(0);
    while (batteryVoltage < 8) {
      batteryVoltage = (float)battery.getConverted();
    }
  }
  else {
    delay(5000);
    // create empty stack to record movements from start position
    moveFwd(1, mDuty, mDuty); // 1 cell 
    moveNorth();        // update first movement
    Serial.print("Current location: ");
    Serial.print(xVirtual);
    Serial.print(", ");
    Serial.print(yVirtual);
    Serial.println();
    
    // all movements after the initial one
    while (isAtCenter == false) {

      xVirtual = x;   // in Floodfill function, x and y coordinates are all virtual
      yVirtual = y;
      md = currentLocMD;

      delay(1000);  // wait to let micromouse settle down, get good reading from IR sensors

      // Sense walls
      Serial.println("Sensing walls...");
      senseWalls(theta);

      // output location & surrounding wall info
      Serial.println("*********************");
      Serial.print("Location: [");
      Serial.print(x);
      Serial.print("][");
      Serial.print(y);
      Serial.println("]");
      Serial.print("North Open: ");
      Serial.println(northIsOpen);
      Serial.print("East Open: ");
      Serial.println(eastIsOpen);
      Serial.print("South Open: ");
      Serial.println(southIsOpen);
      Serial.print("West Open: ");
      Serial.println(westIsOpen);

      Serial.println("floodfill maze #'s:");
      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
          Serial.print(maze[i][j].cellValue);
          Serial.print(' ');
        }
        Serial.println();
      }

      Serial.println("*********************");

      Serial.println("Executing flood-fill");
      floodFill(true);

      // determine where cell with md is
      Serial.println("Finding minimum distance of open neighbors");
      j = 0;

      if (((x - 1 >= 0) && (x - 1 <= 4)) && !(maze[x][y].hasNWall)) {    // check north cell
        Serial.println("Adjacent North Cell found.");
        currentLocAdj[j] = { x - 1, y, maze[x - 1][y].cellValue };
        currentLocAdjCellValue[j] = maze[x - 1][y].cellValue;
        j++;
      }

      if (((x + 1 >= 0) && (x + 1 <= 4)) && !(maze[x][y].hasSWall)) {    // check south cell
        Serial.println("Adjacent South Cell found.");
        currentLocAdj[j] = { x + 1, y, maze[x + 1][y].cellValue };
        currentLocAdjCellValue[j] = maze[x + 1][y].cellValue;
        j++;
      }

      if (((y - 1 >= 0) && (y - 1 <= 4)) && !(maze[x][y].hasWWall)) {     // check west cell
        Serial.println("Adjacent West Cell found.");
        currentLocAdj[j] = { x, y - 1, maze[x][y - 1].cellValue };
        currentLocAdjCellValue[j] = maze[x][y - 1].cellValue;
        j++;
      }

      if (((y + 1 >= 0) && (y + 1 <= 4)) && !(maze[x][y].hasEWall)) {     // check east cell
        Serial.println("Adjacent East Cell found.");
        currentLocAdj[j] = { x, y + 1, maze[x][y + 1].cellValue };
        currentLocAdjCellValue[j] = maze[x][y + 1].cellValue;
        j++;
      }
      
      Serial.print("j (# of adjacent cells to check) = ");
      Serial.print(j);
      Serial.println();
      
      currentLocMD = currentLocAdjCellValue[0];   // sets md to first array value
      nextLocation = currentLocAdj[0];

      // find minimum value (distance) of adjacent cells
      for (int i = 0; i < j; i++) {
        if (currentLocAdjCellValue[i] < currentLocMD) {
          currentLocMD = currentLocAdjCellValue[i];
          nextLocation = currentLocAdj[i];
          Serial.println("THIS IF STATEMENT HAS BEEN EXECUTED");
        }
        xNext = nextLocation.xStack;
        yNext = nextLocation.yStack;
        
        Serial.print("CurrentLocAdj[");
        Serial.print(i);
        Serial.print("]: x=");
        Serial.print(currentLocAdj[i].xStack);
        Serial.print(", y=");
        Serial.print(currentLocAdj[i].yStack);
        Serial.print(", cell value=");
        Serial.println(currentLocAdj[i].cellValue);
        Serial.print("currentLocMD = ");
        Serial.println(currentLocMD);
      }

      Serial.print("xNext: ");
      Serial.print(xNext);
      Serial.println("");
      Serial.print("yNext: ");
      Serial.print(yNext);
      Serial.println("");

      Serial.println("Found minimum distance");
      Serial.print("md: ");
      Serial.print(md);

      for(int i = 0; i < 4; i++){
        currentLocAdjCellValue[i] = 500; // reset cell values
      }
      
      // POSSIBLE IMRPOVEMENT FOR NEXT SEMESTER: TAKE INTO ACCOUNT PRIORITIZING NEXT CELL IF THERE'S MORE THAN 1 MD CELL

      // x changes; y doesn't change
      Serial.println("Executing movement based on md");
      // west cell; move west
      if (yNext == currentLocation.yStack - 1) {
        if (theta == 0) {     // theta = 0
          turnLeft(mDuty, mDuty);
          delay(500);
          turnLeft(mDuty, mDuty);
          delay(500);
        }
        else if (theta == 90) {   // theta = 90
          turnLeft(mDuty, mDuty);
        }
        else if (theta == 180) {  // theta = 180
        // do not turn
        }
        else {                    // theta = 270  
          turnRight(mDuty, mDuty);
        }
        moveFwd(1, mDuty, mDuty);
        moveWest();
        eastIsOpen = true;
      }

      // east cell; move east
      else if (yNext == currentLocation.yStack + 1) {
        if (theta == 0) {     // theta = 0
          //do not turn
        }
        else if (theta == 90) {   // theta = 90
          turnRight(mDuty, mDuty);
        }
        else if (theta == 180) {  // theta = 180
          turnLeft(mDuty, mDuty);
          delay(500);
          turnLeft(mDuty, mDuty);
          delay(500);
        }
        else {                    // theta = 270  
          turnLeft(mDuty, mDuty);
        }
        moveFwd(1, mDuty, mDuty);
        moveEast();
        westIsOpen = true;
      }

      // x doesn't change; y changes
      else {
        // north cell; move north
        if (xNext == currentLocation.xStack - 1) {
          if (theta == 0) {     // theta = 0            
            turnLeft(mDuty, mDuty);
          }
          else if (theta == 90) {   // theta = 90
            // do not turn
          }
          else if (theta == 180) {  // theta = 180
            turnRight(mDuty, mDuty);
          }
          else {            // theta = 270    
            turnLeft(mDuty, mDuty);
            delay(500);
            turnLeft(mDuty, mDuty);
            delay(500);
          }
          moveFwd(1, mDuty, mDuty);
          moveNorth();
          southIsOpen = true;
        }

        // south cell; move south
        else if (xNext == currentLocation.xStack + 1) {
          if (theta == 0) {     // theta = 0            
            turnRight(mDuty, mDuty);
          }
          else if (theta == 90) {   // theta = 90           
            turnLeft(mDuty, mDuty);
            delay(500);
            turnLeft(mDuty, mDuty);
            delay(500);
          }
          else if (theta == 180) {  // theta = 180            
            turnLeft(mDuty, mDuty);
          }
          else {            // theta = 270            
            // do not turn
          }
          moveFwd(1, mDuty, mDuty);
          moveSouth();
          northIsOpen = true;
        }
      }
      // end of movement

      // check if robot reached center
      if (currentLocation.cellValue == 0) {
        isAtCenter = true;
        Serial.println("Reached the center!!");
      }
    }

    // You made it to the center, congratulations! You can take a break :)
    while (1);
  }
}
