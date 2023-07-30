#include "CytronMotorDriver.h"


/*


CRTL ARDUINO:
  FOR EACH ATTACHED MOTOR:
      ctrl arduino should calculate the number of rotations needed to get to that angle

      ctrl arduino  should write to the motor driver, telling it to go that many rotations

      After it hits that many rotations, it should call the encoder to verifiy those rotations

      If it overshoots or undershoots, it should make such correction
*/







class Motor : public CytronMD {

  private:
    float speed;
  public:
    Motor(int pwm_dir_pin, int pin1, int pin2, float speed) : CytronMD(pwm_dir_pin, pin1, pin2) {
      this->speed = speed;
      // Constructor to initialize the CytronMD object
    }

    void driveAngle(float angle) {
      float rotations = angle / 360.0;
      float deltaTime = rotations * 1000;
      setSpeed(this->speed);
      delay(deltaTime);
      setSpeed(0);
      delay(deltaTime);
    }

    void driveRotations(float rotations) {
      float deltaTime = rotations * 1000;
      setSpeed(this->speed);
      delay(deltaTime);
      setSpeed(0);
      delay(deltaTime);
    }
};








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


float speed = 77.8;
Motor motor1(PWM_DIR, 3, 4, speed);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
Motor motor2(PWM_DIR, 9, 10, speed); // PWM 2 = Pin 9, DIR 2 = Pin 10.

void setup() {
  Serial.begin(9600);
}

float thetaList[2];

void loop() {
  //Get a 6 character string from serial monitor and parse into 2 angle list by reference
  parseSerialInput(thetaList);
  //Print the referenced list
  Serial.println(thetaList[0]);
  Serial.println(thetaList[1]);

  //Write to the motors
  motor1.driveRotations(thetaList[0]);
  motor2.driveRotations(thetaList[1]);

  /*
  NOT IMPLEMENTED

  //Call encoder to check if it has reached desired angle
  float correction[2];

  //Make correction for the angle
  motor1.driveAngle(correction[0]);
  motor2.driveAngle(correction[1]);

  */







}
