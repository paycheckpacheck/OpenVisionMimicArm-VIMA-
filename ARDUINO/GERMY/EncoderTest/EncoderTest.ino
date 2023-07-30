


#include <Encoder.h>

// Define the pins for the encoder
int encoderPinA = 50;
int encoderPinB = 51;

// Create an Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

// Define variables for tracking encoder position
float encoderPosition = 0.0;
float encoderResolution = 1024.0;
float wheelCircumference = 20.0;
float numRotations = 0.0;

void setup() {
  // Set up Serial communication
  Serial.begin(9600);

  pinMode(encoderPinA,INPUT_PULLUP);
  pinMode(encoderPinB,INPUT_PULLUP);


  
  }

void loop() {
  // Read the encoder position
  encoderPosition = myEncoder.read();
  
  // Calculate the number of rotations based on encoder position
  numRotations = encoderPosition / encoderResolution;
  
  // Calculate the distance traveled (in cm) based on wheel circumference
  
  // Print the number of rotations and distance traveled to the Serial Monitor
  Serial.println(numRotations/360);
 
  // Wait for a short period of time before reading encoder again
  delay(50);
}