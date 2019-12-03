// NOTE: adjacent/neighboring cells mean direct, not diagonal\
// TO DO: add incrementing to movement functions based on direction
// TO DO: incorporate way to sense walls and tweak function based on wall sensors
#include <StackArray.h>
#include <MKRMotorCarrier.h>
#include <PID.h>
#include "BNO055_support.h"
#include <Wire.h>
// #include "Academic_Open_House_Demo.ino" // movement functions from demo routine ??? pseudo code
#define INTERRUPT_PIN 6

// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************

//Variable to store the battery voltage
static int batteryVoltage;


//****************************//
//***START OF MOVEMENT CODE***//
//****************************//

//Variable to change the motor speed and direction
int m1Duty = -30;   //for adjusting duty cycle, m1 stays constant & m2 adjusts to match
int m2Duty = -30;   // minimum duty cycle: 25
int cellDistance = 18; // in cm
int targetEncoderCount;

bool hasMovedFwd = false;
bool hasTurnedLeft = false;
bool hasTurnedRight = false;

  //This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
int theta = 90;
unsigned long lastTime = 0;

//***FUNCTIONS***//

void moveFwd(float numCells, int dutyCycleM1, int dutyCycleM2) 
{
  // theta doesn't change
  
  encoder1.resetCounter(0);
  encoder2.resetCounter(0); 
  targetEncoderCount = numCells * (8400 / 2);
  
  M1.setDuty(dutyCycleM1);
  M2.setDuty(dutyCycleM2);
  
  while (abs(encoder1.getRawCount()) < targetEncoderCount) 
  {
    delay(5);

        //Get encoder data
    double encoder1Count = encoder1.getRawCount() * -1;
    double encoder2Count = encoder2.getRawCount();
    double encoderDif = encoder1Count - encoder2Count;
    
    Serial.print("Encoder1 Pos [counts]: ");
    Serial.println(encoder1Count);
    Serial.print("Encoder2 Pos [counts]: ");
    Serial.println(encoder2Count);
    
    Serial.print("Dif: ");
    Serial.println(encoderDif);
    Serial.println();
    }
    
  M1.setDuty(0);
  M2.setDuty(0);

  hasMovedFwd = true;
}

void turnLeft(int dutyCycleM1, int dutyCycleM2) 
{
  theta = theta - 90;
    if(theta < 0)
  theta = theta + 360;
  bno055_read_euler_hrp(&myEulerData);
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw - 70; //used to be 90
  float adjustTargetYaw = targetYaw;

  if (targetYaw < 0)    // may not need this
  {
    adjustTargetYaw = targetYaw + 360;
  }
  
  M1.setDuty(-dutyCycleM1);
  M2.setDuty(dutyCycleM2); 

if (adjustTargetYaw < 360 && adjustTargetYaw >= 270) {     // i.e. 0 < initialYaw < 90
      while (float(myEulerData.h) / 16.00 < adjustTargetYaw)      // For when current yaw is 45 - 0 degrees
      {
   //     delay(5); 
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }

      while (float(myEulerData.h) / 16.00 > adjustTargetYaw)    // For when current yaw is 359.99 - adjustedTargetYaw
      {
  //      delay(5); 
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }
    }

    else
    {
      while (float(myEulerData.h) / 16.00 > targetYaw) {
     //   delay(5); 
        
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
       }      
    }

  M1.setDuty(0);
  M2.setDuty(0);

  hasTurnedLeft = true;
}



//Turn the Mouse Right
void turnRight(int dutyCycleM1, int dutyCycleM2, int angleAdjust = 0) 
{
  theta = theta + 90;
  if(theta > 360)
  theta = theta - 360;
  
  Serial.println("Begin Right Turn Function");
  //declare needed variables
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw + 70 + angleAdjust;
  float adjustTargetYaw = targetYaw;

  if (targetYaw >= 360)
    adjustTargetYaw = targetYaw - 360;
  
  //turn on motors to turn right
  M1.setDuty(dutyCycleM1);
  M2.setDuty(-dutyCycleM2); 

  //loop as long as you need the motors on
  if (adjustTargetYaw < 90)     // i.e. 270 < initialYaw < 360
  {     
    while ( (float(myEulerData.h)/16.00 >= 270) && (float(myEulerData.h)/16.00 < 360) )   // For when current yaw is between 270 & 359.99 degrees
    {       
      delay(10); 
      Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
      Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees
      bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
    }

    while ( float(myEulerData.h)/16.00 < adjustTargetYaw)             // For when current yaw is between 0 & adjustedTargetYaw
    {       
      delay(10); 
      Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
      Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees
      bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
    }
  }
  else
  {
    while (float(myEulerData.h) / 16.00 < targetYaw) {
      delay(10); 
      Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
      Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees
      bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
     }      
  }

  //turn motors off
  M1.setDuty(0);
  M2.setDuty(0);

  hasTurnedRight = true;
}

void turn180Deg(int dutyCycleM1, int dutyCycleM2) {    // this rotates 180 degrees clockwise 

  bno055_read_euler_hrp(&myEulerData);
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw + 180;
  float adjustTargetYaw = targetYaw;

    if (targetYaw > 360)    
  {
    adjustTargetYaw = targetYaw - 360;
  }
  
  M1.setDuty(dutyCycleM1);
  M2.setDuty(-dutyCycleM2); 

    if (adjustTargetYaw < 180) {     // i.e. 180 < initialYaw < 270
      while (float(myEulerData.h) / 16.00 - 360 < targetYaw - 360) {        // NOT SURE IF THIS IS RIGHT
  //      delay(5); 
        
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }      
    }

    else
    {
      while (float(myEulerData.h) / 16.00 < targetYaw) {
  //      delay(5); 
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }      
    }
    
  M1.setDuty(0);
  M2.setDuty(0);
}

//***END OF FUNCTIONS***//

//**************************//
//***END OF MOVEMENT CODE***//
//**************************//


// This struct holds maze coordinates (x,y) and the corresponding cell value (distance)
struct MazeCell {
    int xStack;
    int yStack;
    int cellValue;

    void clearData();
};

void MazeCell::clearData() {
  xStack = 0;
  yStack = 0;
  cellValue = 0;
}

StackArray <MazeCell> floodFillStack; // Holds a stack of type Mazecell

int maze[5][5] = { {4, 3, 2, 3, 4},     // micromouse would start at maze[0][4]
                   {3, 2, 1, 2, 3}, 
                   {2, 1, 0, 1, 2},
                   {3, 2, 1, 2, 3},
                   {4, 3, 2, 3, 4} };

bool isInStack[5][5] = {false};

MazeCell currentPush; // to push current
MazeCell currentLocation = {0, 4, 4}; // current location of robot
MazeCell currentPop; // to read top of stack
MazeCell adjacentValues[4]; // array to store adjacent cells and their information
MazeCell currentLocAdj[4];
int currentLocAdjCellValue[4];
MazeCell nextLocation;
int structCellValue[4]; // array to store adjacent cell value
int md = 2; //minimum distance of adjacent cells
int top;
int bottom;
int left;
int right;
int x = 0;  // current location looking at in the stack
int y = 4;  // current location looking at in the stack
int xVirtual = 0; // current x location of cell to be analyzed
int yVirtual = 0; // current y location of cell to be analyzed
int xNext;
int yNext;
int arrayCounter = 0; // counts array size for finding md
//int theta = 90;
int j;
int currentLocMD;
bool isAtCenter = false;
bool stackIsEmpty = true;
bool northIsOpen = true;
bool southIsOpen = true;
bool eastIsOpen = true;
bool westIsOpen = true;


void moveNorth() {
  currentLocation.yStack--;
  currentLocation.cellValue = maze[x][currentLocation.yStack];
}

void moveSouth() {
  currentLocation.yStack++; 
  currentLocation.cellValue = maze[x][currentLocation.yStack];
}

void moveWest() {
  currentLocation.xStack--; 
  currentLocation.cellValue = maze[currentLocation.xStack][y];
}

void moveEast() {
  currentLocation.xStack++; 
  currentLocation.cellValue = maze[currentLocation.xStack][y];
}

// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************

// MAIN FLOODFILL FUNCTION
void floodFill(bool stackIsEmpty) {
    currentPush = {x, y, maze[x][y]};
    
    if (stackIsEmpty) {
    floodFillStack.push(currentPush);
    isInStack[currentPush.xStack][currentPush.yStack] = true;   // for boolean maze array
    }

    currentPop = floodFillStack.pop();
    isInStack[currentPop.xStack][currentPop.yStack] = false;

    if (currentPop.cellValue - 1 != md) {
        maze[currentPop.xStack][currentPop.yStack] = md + 1; // update current cell value

        MazeCell top = {x, y-1, maze[x][y-1]};
        MazeCell bottom = {x, y+1, maze[x][y+1]};
        MazeCell left = {x-1, y, maze[x-1][y]};
        MazeCell right = {x+1, y, maze[x+1][y]};

        if ( (y-1 >= 0) || (y-1 <=4) || (maze[x][y-1] != 0) && northIsOpen){   // adjacent top cell
            for (int i = 0; i < floodFillStack.count(); i++) {
               if ( isInStack[x][y-1] == false) {
                   adjacentValues[arrayCounter] = top; // these are arrays
                   structCellValue[arrayCounter] = top.cellValue; // these are arrays
                   arrayCounter++;

                   isInStack[x][y-1] = true;
               }
            }    
         }

       if ( (y+1 >= 0) || (y+1 <=4) || (maze[x][y+1] != 0) && southIsOpen) {   // adjacent bottom cell
          for (int i = 0; i < floodFillStack.count(); i++) {
               if ( isInStack[x][y+1] == false) {
                  adjacentValues[arrayCounter] = bottom;
                  structCellValue[arrayCounter] = bottom.cellValue;  
                  arrayCounter++;

                  isInStack[x][y+1] = true;
              }
          }           
       }
        
      if ( (x-1 >= 0) || (x-1 <= 4) || (maze[x-1][y] != 0) && westIsOpen) {    // adjacent left cell
          for (int i = 0; i < floodFillStack.count(); i++) { 
               if ( isInStack[x-1][y] == false) {
                  adjacentValues[arrayCounter] = left;
                  structCellValue[arrayCounter] = left.cellValue;
                  arrayCounter++;
                  
                  isInStack[x-1][y] = true;
              }
          }  
    }

    if ( (x+1 >= 0) || (x+1 <= 4) || (maze[x+1][y] != 0) && eastIsOpen) {    // adjacent right cell
        for (int i = 0; i < floodFillStack.count(); i++) {
               if ( isInStack[x+1][y] == false) {
                adjacentValues[arrayCounter] = right;
                structCellValue[arrayCounter] = right.cellValue;
                arrayCounter++;
                
                isInStack[x+1][y] = true;            
            }
        }
    }

    md = structCellValue[0]; // sets md to first vector value
    
    // find minimum value (distance) of adjacent cells
    for (int i = 0; i < arrayCounter; i++) {
      if (structCellValue[i] < md) {
        md = structCellValue[i];
      }
    }

          for(int i = 0; i < arrayCounter; i++) { // push all adjacent squares into stack
            if (structCellValue[i] != 0) {
            floodFillStack.push(adjacentValues[i]);
            }
        }

    // reset all array values for next recursion    
    for (int i = 0; i < arrayCounter; i++) {
        adjacentValues[i].clearData();    
    }

    for (int i = 0; i < 4; i++) {
        structCellValue[i] = 0;
    }
  
    arrayCounter = 0;
    
    stackIsEmpty = floodFillStack.isEmpty();
    floodFill(stackIsEmpty);

  }

    
    stackIsEmpty = floodFillStack.isEmpty();

    if(!stackIsEmpty) {
    floodFill(stackIsEmpty);
    }
  
} // end of floodFill function

//Variable to change the motor speed and direction
static int duty = 40;

// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************

void setup() {
  // put your setup code here, to run once://Serial port initialization
  Serial.begin(115200);

  //Establishing the communication with the motor shield
  if (controller.begin()) 
    {
      Serial.print("MKR Motor Shield connected, firmware version ");
      Serial.println(controller.getFWVersion());
    } 
  else 
    {
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
  
  //Take the battery status
  float batteryVoltage = (float)battery.getConverted();
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage);
  Serial.print("V, Raw ");
  Serial.println(battery.getRaw());
}

// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************
// **********************************************************************

void loop() {
  // put your main code here, to run repeatedly:
  //Take the battery status
  float batteryVoltage = (float)battery.getConverted();
  
  //Reset to the default values if the battery level is lower than 11V
  if (batteryVoltage < 11) 
  {
    Serial.println(" ");
    Serial.println("WARNING: LOW BATTERY");
    Serial.println("ALL SYSTEMS DOWN");
    M1.setDuty(0);
    M2.setDuty(0);
    M3.setDuty(0);
    M4.setDuty(0);
    while (batteryVoltage < 11) 
    {
      batteryVoltage = (float)battery.getConverted();
    }
  }
  else
  {

// initialize 5x5 array with center square as 0


//create empty stack to record movements from start position
  moveFwd(1, -30, -30); // 1 cell 
  moveNorth(); // update first movement


// all movements after the initial one
while(isAtCenter == false){

// get wall sensor data. update __IsOpen bool values

//if (hasMovedFwd) {
//  //check theta and determine direction of movement
//    switch(theta) {
//        case 0:
//            moveEast();
//            break;
//        case 90:
//            moveNorth();
//            break;
//        case 180:
//            moveWest();
//            break;
//        case 270:
//            moveSouth();
//            break;
//    }
//}

floodFill(true);

  // determine where cell with md is

j = 0;
    if ( (y-1 >= 0) || (y-1 <= 4) && northIsOpen) {
        currentLocAdj[j] = {x, y-1, maze[x][y-1]};
        currentLocAdjCellValue[j] = maze[x][y-1];
        j++;
    }

    if ( (y+1 >= 0) || (y+1 <= 4) && southIsOpen) {
        currentLocAdj[j] = {x, y+1, maze[x][y+1]};
        currentLocAdjCellValue[j] = maze[x][y+1];
        j++;
    }
     
    if ( (x-1 >= 0) || (x-1 <= 4) && westIsOpen) {
        currentLocAdj[j] = {x-1, y, maze[x-1][y]};
        currentLocAdjCellValue[j] = maze[x-1][y];
        j++;
    }

    if ( (x+1 >= 0) || (x+1 <= 4) && eastIsOpen) {
        currentLocAdj[j] = {x+1, y, maze[x+1][y]};
        currentLocAdjCellValue[j] = maze[x+1][y];
        j++;
    }
  
    currentLocMD = currentLocAdjCellValue[0]; // sets md to first array value
    
    // find minimum value (distance) of adjacent cells
    for (int i = 0; i < j; i++) {
      if (currentLocAdjCellValue[i] < currentLocMD) {
        currentLocMD = currentLocAdjCellValue[i];
        nextLocation = currentLocAdj[i];
        xNext = nextLocation.xStack;
        yNext = nextLocation.yStack;
      }
    }

md = currentLocMD; // fix md later is confusion

    switch(xNext) {
        case currentLocation.xStack - 1:    // left cell; move left
            switch(theta) {
              case 0:
              //rotate 180
              turnLeft();
              turnLeft();
              moveFwd(1, -30, -30);
              
              moveWest();
              break;
              case 90:
                  turnLeft();
                  moveFwd(1, -30, -30);
                  moveWest();
              break;
              case 180:
                  moveFwd(1, -30, -30);
                  moveWest();
              break;
              case 270:
                  turnRight(-30, -30);
                  moveFwd(1, -30, -30);
                  moveWest();
              break;
            }
            break;
        case currentLocation.xStack + 1:  // right cell; move right
          switch(theta){
              case 0:
              moveFwd(-30, -30);
              moveEast();
              break;
              case 90:
              turnRight(-30, -30);
              moveFwd(1, -30, -30);
              moveEast();
              break;
              case 180:
              turnLeft(-30, -30);
              turnLeft(-30, -30);
              moveFwd(1, -30, -30);
              moveEast();
              break;
              case 270:
              turnLeft(-30, -30);
              moveFwd(1, -30, -30);
              moveEast();
              break;
          }
            break;
        case currentLocation.xStack:    // x doesn't change; y changes
            switch(yNext) {
              case currentLocation.yStack - 1: // top cell, move up
                  switch(theta){
                  case 0:
                  turnLeft(-30, -30);
                  moveFwd(1, -30, -30);
                  moveNorth();
                  break;
                  case 90:
                  moveFwd(1, -30, -30);
                  moveNorth();
                  break;
                  case 180:
                  turnRight(-30, -30);
                  moveFwd(1, -30, -30);
                  moveNorth();
                  break;
                  case 270:
                  turnLeft(-30, -30);
                  turnLeft(-30, -30);
                  moveFwd(1, -30, -30);
                  moveNorth();
                  break;
          }
              break;
              case currentLocation.yStack + 1: // bottom cell, move down
                 switch(theta){
                    case 0:
                    turnRight(-30, -30);
                    moveFwd(1, -30, -30);
                    moveSouth();
                    break;
                    case 90:
                    turnLeft(-30, -30);
                    turnLeft(-30, -30);
                    moveFwd(1, -30, -30);
                    moveSouth();
                    break;
                    case 180:
                    turnLeft(-30, -30);
                    moveFwd(1, -30, -30);
                    moveSouth();
                    break;
                    case 270:
                    moveFwd(1, -30, -30);
                    moveSouth();
                    break;
              }
              break;
            }
        break;
    }

    if (currentLocation.cellValue == 0) {
      isAtCenter = true;
    }

// senseWalls() to read surrounding walls
// when sensing walls, need to make sure the IR sensor reading is for CLOSEST walls
// next, get the open neighboring cells


  //PROCESS:

  //move
  //update current cell location based on movement direction maze[x][y]
  //read sensors
  //update cell values
  //repeat until at center which is at [2][2]

  //find min of adjacent values
        }
    }
}
