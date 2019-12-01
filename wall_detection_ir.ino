//IR TEST CODE
//Inspired by this tutorial: https://www.instructables.com/id/How-to-setup-a-Pololu-Carrier-with-Sharp-GP2Y0A60S/


//Analog reading of about 11cm distance (largest distance from IR sensor to wall if there is a wall next to the mouse)
#define IR_THRESHOLD  350

#define IR_LEFT_PIN A6 //IN1 on Motor Carrier
#define IR_RIGHT_PIN A1 //IN2 on Motor Carrier
#define IR_FRONT_PIN A2 //IN4 on Motor Carrier


// The setup routine runs once when you press reset:
void setup() {
  // Sets pin 13 for output in order to blink the LED:
  pinMode(13, OUTPUT);
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// The loop routine runs over and over again forever:
void loop() {
  
  // Read the input on analog pin:
  int leftSensorValue = analogRead(IR_LEFT_PIN);
  int rightSensorValue = analogRead(IR_RIGHT_PIN);
  int frontSensorValue = analogRead(IR_FRONT_PIN);

  //Converts analog value to distance in centimeters (NOT CALIBRATED)
//  double IRdistance = 187754 * pow(sensorValue, -1.51);


//  Serial.print("Left IR Value (raw): ");
//  Serial.println(leftSensorValue);
//
//  Serial.print("Right IR Value (raw): ");
//  Serial.println(rightSensorValue);
//
  Serial.print("Front IR Value (raw): ");
  Serial.println(frontSensorValue);
//
//  Serial.println();


  Serial.println("------");
  if(leftSensorValue > IR_THRESHOLD)
    Serial.println("LEFT WALL");
  else
    Serial.println("NO LEFT WALL");
    
   if(rightSensorValue > IR_THRESHOLD)
    Serial.println("RIGHT WALL");
   else
    Serial.println("NO RIGHT WALL");

  
  
  //Serial.println(":)");

//  Serial.print("IR Value (cm): ");
//  Serial.println(IRdistance);


  
  
  delay(500);

}
