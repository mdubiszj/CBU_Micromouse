//taken from Arduino Education website: https://create.arduino.cc/edu/courses/mod/book/view.php?id=83&chapterid=118

#include <MKRMotorCarrier.h>
#define INTERRUPT_PIN 6

//Variable to store the battery voltage
static int batteryVoltage;

//Variable to change the motor speed and direction
int m1Duty = -60;   //for adjusting duty cycle, m1 stays constant & m2 adjusts to match
int m2Duty = -60;

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
}


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
    //Chose the motor to use:M1(default), M2, M3 or M4
    Serial.print("M1 Duty: ");
    Serial.println(m1Duty);
    Serial.print("M2 Duty: ");
    Serial.println(m2Duty);
    
    M1.setDuty(m1Duty);
    M2.setDuty(m2Duty);
  
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


    //if motor 1 has gone at least 5 counts farther than motor 2, increase magnitude of motor 2 duty cycle 
    //if (encoder1Count/encoder2Count > 1.01)
    if((abs(encoder1Count) - abs(encoder2Count)) > 5)
      if (m2Duty > 0)
        m2Duty++;
      else if (m2Duty < 0)
        m2Duty--;
    // if motor 2 has gone at least 5 counts farther than motor 1, decrease magnitude of motor 2 duty cycle 
    //else if (encoder1Count/encoder2Count < (0.99) )
    if((abs(encoder1Count) - abs(encoder2Count)) < -5)
      if (m2Duty > 0)
        m2Duty--;
      else if (m2Duty < 0)
        m2Duty++;

    
  }
  
  //Keep active the communication MKR1000 & MKRMotorCarrier
  //Ping the samd11
  controller.ping();
  //wait
  delay(50);
}
