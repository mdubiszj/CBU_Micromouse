//taken from Arduino Education website: https://create.arduino.cc/edu/courses/mod/book/view.php?id=83&chapterid=118

#include <MKRMotorCarrier.h>
#include "BNO055_support.h"
#include <Wire.h>
#define INTERRUPT_PIN 6

//Variable to store the battery voltage
static int batteryVoltage;

//Variable to change the motor speed and direction
int m1Duty = -60;   //for adjusting duty cycle, m1 stays constant & m2 adjusts to match
int m2Duty = -60;
int cellDistance = 18; // in cm
int targetEncoderCount;

  //This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

unsigned long lastTime = 0;

void setup() 
{
  //Serial port initialization
  Serial.begin(115200);
  while (!Serial);

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

  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);

}

// ***FUNCTIONS***///

void moveFwd(int numCells, int dutyCycleM1, int dutyCycleM2) 
{
  encoder1.resetCounter(0);
  encoder2.resetCounter(0); 
  targetEncoderCount = numCells * 8145;
  
  M1.setDuty(dutyCycleM1);
  M2.setDuty(dutyCycleM2);
  
  while (abs(encoder1.getRawCount()) < targetEncoderCount) 
  {
    delay(5);

        //Get encoder data
    double encoder1Count = encoder1.getRawCount();
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
}

void turnLeft(int dutyCycleM1, int dutyCycleM2) 
{
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw - 90;
  float adjustTargetYaw = targetYaw;

  if (targetYaw < 0)    // may not need this
  {
    adjustTargetYaw = targetYaw + 360;
  }
  
//  encoder1.resetCounter(0);
//  encoder2.resetCounter(0); 
  M1.setDuty(-dutyCycleM1);
  M2.setDuty(dutyCycleM2); 
    
//    while (float(myEulerData.h) / 16.00 + 360 > targetYaw + 360) {
//      delay(5); 
//      Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
//      Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees
//    }

if (adjustTargetYaw < 360 && adjustTargetYaw > 270) {     // i.e. 0 < initialYaw < 90
      while (float(myEulerData.h) / 16.00 < adjustTargetYaw)      // For when current yaw is 45 - 0 degrees
      {
        delay(100); 
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }

      while (float(myEulerData.h) / 16.00 > adjustTargetYaw)    // For when current yaw is 359.99 - adjustedTargetYaw
      {
        delay(100); 
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }
      
//     // while (float(myEulerData.h) / 16.00 < targetYaw + 360) {        // NOT SURE IF THIS IS RIGHT
//         
//        
//        delay(100); 
//
//        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
//        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees
//
//        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
//      }      
    }

    else
    {
      while (float(myEulerData.h) / 16.00 > targetYaw) {
        delay(100); 
        
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
       }      
    }

  M1.setDuty(0);
  M2.setDuty(0);
}

void turnRight(int dutyCycleM1, int dutyCycleM2) 
{
  bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw + 90;
  float adjustTargetYaw = targetYaw;

    if (targetYaw > 360)   
  {
    adjustTargetYaw = targetYaw - 360;
  }
  
//  encoder1.resetCounter(0);
//  encoder2.resetCounter(0); 
  M1.setDuty(dutyCycleM1);
  M2.setDuty(-dutyCycleM2); 

    if (adjustTargetYaw < 90) {     // i.e. 270 < initialYaw < 360
      while (float(myEulerData.h) / 16.00 - 360 < targetYaw - 360) {        // NOT SURE IF THIS IS RIGHT
        delay(100); 

        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }      
    }

    else
    {
      while (float(myEulerData.h) / 16.00 < targetYaw) {
        delay(100); 
        
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
       }      
    }
    
  M1.setDuty(0);
  M2.setDuty(0);
}

void turn180Deg(int dutyCycleM1, int dutyCycleM2) {    // this rotates 180 degrees clockwise 
  float initialYaw = float(myEulerData.h) / 16.00;
  float targetYaw = initialYaw + 180;
  float adjustTargetYaw = targetYaw;

    if (targetYaw > 360)    
  {
    adjustTargetYaw = targetYaw - 360;
  }
  
//  encoder1.resetCounter(0);
//  encoder2.resetCounter(0); 
  M1.setDuty(dutyCycleM1);
  M2.setDuty(-dutyCycleM2); 

    if (adjustTargetYaw < 180) {     // i.e. 180 < initialYaw < 270
      while (float(myEulerData.h) / 16.00 - 360 < targetYaw - 360) {        // NOT SURE IF THIS IS RIGHT
        delay(5); 
        
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }      
    }

    else
    {
      while (float(myEulerData.h) / 16.00 < targetYaw) {
        delay(5); 
        Serial.print("Heading(Yaw): ");        //To read out the Heading (Yaw)
        Serial.println(float(myEulerData.h) / 16.00);   //Convert to degrees

        bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
      }      
    }
    
  M1.setDuty(0);
  M2.setDuty(0);
}

// ***END OF FUNCTIONS***///


void loop() {
  
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
    
  bno055_read_euler_hrp(&myEulerData); //Update Euler data into the structure
  
    //Chose the motor to use:M1(default), M2, M3 or M4
//    Serial.print("M1 Duty: ");
//    Serial.println(m1Duty);
//    Serial.print("M2 Duty: ");
//    Serial.println(m2Duty);
    
//    M1.setDuty(m1Duty);
//    M2.setDuty(m2Duty);

    // TESTING FUNCTIONS
    //moveFwd(2, m1Duty, m2Duty);  // this works

    //turnRight(m1Duty, m2Duty);  // this works
    
   // turnLeft(m1Duty, m2Duty);     // this works

   turn180Deg(m1Duty, m2Duty);
    delay(1000);
  
//    //Get encoder data
//    double encoder1Count = encoder1.getRawCount();
//    double encoder2Count = encoder2.getRawCount();
//    double encoderDif = encoder1Count - encoder2Count;
//    
//    Serial.print("Encoder1 Pos [counts]: ");
//    Serial.println(encoder1Count);
//    Serial.print("Encoder2 Pos [counts]: ");
//    Serial.println(encoder2Count);
//    
//    Serial.print("Dif: ");
//    Serial.println(encoderDif);
//    Serial.println();

    


//    //if motor 1 has gone at least 5 counts farther than motor 2, increase magnitude of motor 2 duty cycle 
//    //if (encoder1Count/encoder2Count > 1.01)
//    if((abs(encoder1Count) - abs(encoder2Count)) > 5)
//      if (m2Duty > 0)
//        m2Duty++;
//      else if (m2Duty < 0)
//        m2Duty--;
//    // if motor 2 has gone at least 5 counts farther than motor 1, decrease magnitude of motor 2 duty cycle 
//    //else if (encoder1Count/encoder2Count < (0.99) )
//    if((abs(encoder1Count) - abs(encoder2Count)) < -5)
//      if (m2Duty > 0)
//        m2Duty--;
//      else if (m2Duty < 0)
//        m2Duty++;

    
  }
  
  //Keep active the communication MKR1000 & MKRMotorCarrier
  //Ping the samd11
  controller.ping();
  //wait
  delay(50);
}
