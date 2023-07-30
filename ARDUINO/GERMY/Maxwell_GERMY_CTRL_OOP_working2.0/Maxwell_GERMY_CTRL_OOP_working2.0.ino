#include "CytronMotorDriver.h"


/*


CRTL ARDUINO:
  FOR EACH ATTACHED MOTOR:
      ctrl arduino should calculate the number of rotations needed to get to that angle

      ctrl arduino  should write to the motor driver, telling it to go that many rotations

      After it hits that many rotations, it should call the encoder to verifiy those rotations

      If it overshoots or undershoots, it should make such correction
*/



#include <SimpleEncoder.h>






/*
SimpleEncoder BaseEncoder(BUTTON_PIN_base, baseEncoderA, PINB, myValue, minValue, maxValue);
SimpleEncoder shoulderEncoder(BUTTON_PIN_shoulder, PINA, PINB, myValue, minValue, maxValue);
SimpleEncoder elbowEncoder(BUTTON_PIN, PINA, PINB, myValue, minValue, maxValue);
SimpleEncoder differAEncoder(BUTTON_PIN, PINA, PINB, myValue, minValue, maxValue);
SimpleEncoder differBEncoder(BUTTON_PIN, PINA, PINB, myValue, minValue, maxValue);

*/





const int BUTTON_PIN = 3;

class Motor : public CytronMD, public SimpleEncoder {
private:
  float speed;

public:
  Motor(int pwm_dir_pin, int pin1, int pin2, float speed)
    : CytronMD(pwm_dir_pin, pin1, pin2), SimpleEncoder(BUTTON_PIN, PINA, PINB, 0, -1000, 1000) {
    this->speed = speed;
    //CytronMD::begin();
  }

  void driveAngle(float angle) {
    float rotations = angle / 360.0;
    driveRotations(rotations);
  }

  void driveRotations(float rotations) {
    float deltaTime = rotations * 1000.0;
    setSpeed(speed);
    unsigned long startTime = millis();
    while (millis() - startTime < deltaTime) {
      setSpeed(speed);
    }
    setSpeed(0);
  }
};
float speed = 77.8;



//Gets 18 characters from serial monitor and parses into list by reference
//========================================== USED IN GORA IMPLEMENTATION
void parseSerialInput4GERMY(float (&theta_list)[6]) {
  // Read 18 characters from the serial monitor
  String input = Serial.readStringUntil('\n');
  input.trim(); // Remove any leading or trailing white space

  // Split the input into six three-digit numbers
  for (int i = 0; i < 6; i++) {
    String sub_str = input.substring(i*4, (i+1)*4);
    theta_list[i] = sub_str.toFloat();
  }
}
//==========================================================================







Motor differA(PWM_DIR, 13, 12, speed);
Motor differB(PWM_DIR, 11, 10, speed);
Motor elbow(PWM_DIR, 9, 8, speed);
Motor gripper(PWM_DIR, 7, 6, speed);
Motor shoulder(PWM_DIR, 5, 4, speed);
Motor base(PWM_DIR, 3, 2, speed);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
}

float thetaList[6];
float priorThetaList[6];
float driveThetaList[6];




void loop() {

  //Get
  parseSerialInput4GERMY(thetaList);


  
 
  for (int i = 0; i < 6; i++) {
    driveThetaList[i] = -priorThetaList[i] + thetaList[i];
    Serial.println(driveThetaList[i]);

  }
  


  
  

  base.setSpeed(thetaList[0]);
  shoulder.setSpeed(thetaList[1]);
  elbow.setSpeed(thetaList[2]);
  differA.setSpeed(thetaList[3]);
  differB.setSpeed(thetaList[4]);
  gripper.setSpeed(thetaList[5]);


  

  for (int i = 0; i < 6; i++) {
    priorThetaList[i] = driveThetaList[i];

    Serial.print(priorThetaList[i]);
  }

  delay(400);
}



  /*
  NOT IMPLEMENTED

  //Call encoder to check if it has reached desired angle
  float correction[2];

  //Make correction for the angle
  motor1.driveAngle(correction[0]);
  motor2.driveAngle(correction[1]);

  */







