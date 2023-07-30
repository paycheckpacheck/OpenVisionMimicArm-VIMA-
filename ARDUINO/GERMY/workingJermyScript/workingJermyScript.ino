


/*PIN CONFIG

This example shows how to drive 2 motors using the PWM and DIR pins with
 * 2-channel motor driver.
 * 
 * 
 * CONNECTIONS:
 * 
 * Arduino D3  - Motor Driver PWM 1 Input
 * Arduino D4  - Motor Driver DIR 1 Input
 * Arduino D9  - Motor Driver PWM 2 Input
 * Arduino D10 - Motor Driver DIR 2 Input
 * Arduino GND - Motor Driver GND

 */

 
#include "CytronMotorDriver.h"
#include <Arduino.h>
#include <Arduino.h>
#include <PIDController.h>
#include <Encoder.h>


/*


CRTL ARDUINO:
  FOR EACH ATTACHED MOTOR:
      ctrl arduino should calculate the number of rotations needed to get to that angle

      ctrl arduino  should write to the motor driver, telling it to go that many rotations

      After it hits that many rotations, it should call the encoder to verifiy those rotations

      If it overshoots or undershoots, it should make such correction
*/



// Configure the motor driver.
CytronMD motor1(PWM_DIR, 3, 4);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 9, 10); // PWM 2 = Pin 9, DIR 2 = Pin 10.


//Encoder encoder(6, 7);


//test string for GORA    090100110120130140


void driveAngle(float angle)
{

  // f(255) = 3.333 rot/sec
  //f(74) = 1 rot/sec
  float rotations = angle/360.0;

  float deltaTime =  rotations*1000;

  //Start motor
  motor2.setSpeed(75);  

  delay(deltaTime);

  motor2.setSpeed(0);

  delay(deltaTime);


}



void driveRotations(float rotations)
{

  // f(255) = 3.333 rot/sec
  //f(74) = 1 rot/sec

  float deltaTime =  rotations*1000;

  //Start motor
  motor2.setSpeed(75);  

  delay(deltaTime);

  motor2.setSpeed(0);

  delay(deltaTime);


}




void parseSerialInput(float (&theta_list)[2]) {
  // Read 6 characters from the serial monitor
  String input = Serial.readStringUntil('\n');
  input.trim(); // Remove any leading or trailing white space

  // Split the input into two three-digit numbers
  String sub_str = input.substring(0, 3);
  theta_list[0] = sub_str.toFloat();

  sub_str = input.substring(3, 6);
  theta_list[1] = sub_str.toFloat();
}





void setup() {

  Serial.begin(9600);

}




float thetaList[2];

void loop() {


  //Get 6 character from serial and parse into a 2 angle list bby reference
  parseSerialInput(thetaList);  

  //Print results of 2 angle list
  Serial.println(thetaList[0]);  
  Serial.println(thetaList[1]);  



  
  

  
}


