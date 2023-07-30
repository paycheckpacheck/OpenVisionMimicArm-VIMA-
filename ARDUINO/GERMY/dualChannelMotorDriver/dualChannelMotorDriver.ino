


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



// Configure the motor driver.
CytronMD motor1(PWM_DIR, 3, 4);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 9, 10); // PWM 2 = Pin 9, DIR 2 = Pin 10.


Encoder encoder(6, 7);

double targetAngle = 0;
double currentAngle = 0;
double outputSpeed = 0;
int delta_time = 0.2;

///PID 
double kp = 0.1, ki = 0.01, kd = 0.01;
PIDController pid;//(&currentAngle, &outputSpeed, &targetAngle, kp, ki, kd);

class MotorControl{
  public:
    MotorControl() {

      //My constructor def __init__(self):
      //pid.setSampleTime(100);
      //Set motor speed limits
      pid.limit(-255, 255);
      //PID Kp, Ki, Kd tunning
      pid.tune(1,1,1);
      //pid.setMode(AUTOMATIC);
    }

/*
//Function gets the target ange from serial monitor
    int getTargetAngle() {
      // Get target angle from serial monitor
      if (Serial.available()>0)
      {
        int targetAngle = Serial.readStringUntil('\n').toInt();
        pid.setpoint(targetAngle);
      }
      // TODO: add logic to read target angle from serial monitor
      return targetAngle;
    }
*/

// Function gets the current angle from feedback encoder
    int getCurrentAngle() {
      // Get current angle from encoder
      int currentAngle = encoder.read();
      Serial.write(currentAngle);      
      return currentAngle;
    }


// Function writes to the motor using the results from pid
    void writeSpeedToMotor(int delta_time, int encoder_feedback) {

      int pid_output = pid.compute(encoder_feedback);  //PID output is an angle trying to reach setpoint angle
      Serial.write(pid_output); 

      int pid_omega = pid_output * delta_time ;
      motor2.setSpeed(pid_omega);   // Motor 1 runs forward at 50% speed.


      //delta_time is in seconds

      delay(delta_time*1000);

      //motor2.setSpeed(0);


    }
};

MotorControl ctrl;

void setup() {
  // nothing to do in the setup
  Serial.begin(9600);

}

void loop() {




  if (Serial.available()>0)
        {
          int targetAngle = Serial.readStringUntil('\n').toInt();
          pid.setpoint(targetAngle);
          currentAngle = ctrl.getCurrentAngle();
          ctrl.writeSpeedToMotor(delta_time, currentAngle);

        }  
  
}



  
